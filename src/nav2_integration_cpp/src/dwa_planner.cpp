// Copyright 2026 a2_system_ws.
#include "nav2_integration_cpp/dwa_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#if defined(NAV2INT_HAVE_OPENMP)
#include <omp.h>
#endif

namespace nav2_integration_cpp
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

inline double normalize_angle(double a) noexcept
{
  while (a > kPi) {a -= 2.0 * kPi;}
  while (a < -kPi) {a += 2.0 * kPi;}
  return a;
}

inline bool finite_(double v) noexcept {return std::isfinite(v);}
}  // namespace

void validate_params(const DwaParams & p)
{
  auto pos = [](double v) {return std::isfinite(v) && v >= 0.0;};
  auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  if (!pos(p.max_linear_x) || !pos(p.max_linear_y) || !pos(p.max_yaw_rate)) {
    throw std::invalid_argument("DWA velocity limits must be >= 0 and finite");
  }
  if (p.min_linear_x > p.max_linear_x) {
    throw std::invalid_argument("DWA min_linear_x must be <= max_linear_x");
  }
  if (p.n_vx < 1 || p.n_vy < 1 || p.n_wz < 1) {
    throw std::invalid_argument("DWA sample counts must be >= 1");
  }
  if (p.n_vx * p.n_vy * p.n_wz > 4096) {
    throw std::invalid_argument("DWA sample grid too large (n_vx*n_vy*n_wz > 4096)");
  }
  if (!positive(p.sim_time) || !positive(p.sim_step) || p.sim_step > p.sim_time) {
    throw std::invalid_argument("DWA sim_time and sim_step must be > 0 and step<=time");
  }
  if (!pos(p.inflation_radius) || !pos(p.hard_clearance) ||
    p.hard_clearance > p.inflation_radius)
  {
    throw std::invalid_argument(
            "DWA clearance: 0 <= hard_clearance <= inflation_radius");
  }
  if (!pos(p.obstacle_cost_weight) || !pos(p.goal_dist_weight) ||
    !pos(p.goal_heading_weight) || !pos(p.forward_progress_weight) ||
    !pos(p.velocity_smoothness_weight))
  {
    throw std::invalid_argument("DWA cost weights must be >= 0 and finite");
  }
  if (!pos(p.goal_tolerance_xy) || !pos(p.goal_tolerance_yaw)) {
    throw std::invalid_argument("DWA tolerances must be >= 0");
  }
  if (!pos(p.block_grace_sec)) {
    throw std::invalid_argument("DWA block_grace_sec must be >= 0");
  }
}

DwaPlanner::DwaPlanner(const DwaParams & params)
{
  validate_params(params);
  p_ = params;
}

void DwaPlanner::reconfigure(const DwaParams & params)
{
  validate_params(params);
  p_ = params;
}

std::vector<Pose2D> DwaPlanner::simulate(
  const Pose2D & start, const Velocity & v,
  double sim_time, double sim_step) noexcept
{
  std::vector<Pose2D> traj;
  if (!(sim_step > 0.0) || !(sim_time > 0.0)) {
    return traj;
  }
  const int steps = std::max(1, static_cast<int>(std::ceil(sim_time / sim_step)));
  traj.reserve(static_cast<std::size_t>(steps + 1));
  Pose2D cur = start;
  traj.push_back(cur);
  for (int k = 0; k < steps; ++k) {
    const double cy = std::cos(cur.yaw);
    const double sy = std::sin(cur.yaw);
    const double dx = (cy * v.vx - sy * v.vy) * sim_step;
    const double dy = (sy * v.vx + cy * v.vy) * sim_step;
    cur.x += dx;
    cur.y += dy;
    cur.yaw = normalize_angle(cur.yaw + v.wz * sim_step);
    traj.push_back(cur);
  }
  return traj;
}

PlanResult DwaPlanner::plan(
  const Pose2D & current,
  const Pose2D & goal,
  const Eigen::Ref<const Eigen::MatrixX2f> & obstacles_xy,
  const Velocity & last_cmd) const noexcept
{
  PlanResult result{};
  result.min_clearance = std::numeric_limits<double>::infinity();

  if (!finite_(current.x) || !finite_(current.y) || !finite_(current.yaw) ||
    !finite_(goal.x) || !finite_(goal.y) || !finite_(goal.yaw))
  {
    result.reason = "nonfinite_pose_or_goal";
    return result;
  }

  // Build sample grid.
  const int Nvx = p_.n_vx;
  const int Nvy = p_.n_vy;
  const int Nwz = p_.n_wz;
  const int total = Nvx * Nvy * Nwz;

  auto vx_value = [this, Nvx](int i) noexcept {
      if (Nvx == 1) {return p_.min_linear_x;}
      const double t = static_cast<double>(i) / static_cast<double>(Nvx - 1);
      return p_.min_linear_x + t * (p_.max_linear_x - p_.min_linear_x);
    };
  auto vy_value = [this, Nvy](int j) noexcept {
      if (Nvy == 1) {return 0.0;}
      const double t = static_cast<double>(j) / static_cast<double>(Nvy - 1);
      return -p_.max_linear_y + 2.0 * t * p_.max_linear_y;
    };
  auto wz_value = [this, Nwz](int k) noexcept {
      if (Nwz == 1) {return 0.0;}
      const double t = static_cast<double>(k) / static_cast<double>(Nwz - 1);
      return -p_.max_yaw_rate + 2.0 * t * p_.max_yaw_rate;
    };

  // Pre-extract obstacles to plain arrays for fast nearest-neighbor.
  const auto M = static_cast<std::size_t>(obstacles_xy.rows());
  std::vector<float> ox(M);
  std::vector<float> oy(M);
  for (std::size_t i = 0; i < M; ++i) {
    ox[i] = obstacles_xy(static_cast<Eigen::Index>(i), 0);
    oy[i] = obstacles_xy(static_cast<Eigen::Index>(i), 1);
  }

  const double r_hard = p_.hard_clearance;
  const double r_inflate = p_.inflation_radius;
  const double r_hard_sq = r_hard * r_hard;
  const double r_band = std::max(1e-9, r_inflate - r_hard);

  // Per-sample evaluation results.
  std::vector<double> costs(total, std::numeric_limits<double>::infinity());
  std::vector<unsigned char> admissible(total, 0u);
  std::vector<double> clearances(total, std::numeric_limits<double>::infinity());

#if defined(NAV2INT_HAVE_OPENMP)
  #pragma omp parallel for collapse(3) schedule(static)
#endif
  for (int i = 0; i < Nvx; ++i) {
    for (int j = 0; j < Nvy; ++j) {
      for (int k = 0; k < Nwz; ++k) {
        const int idx = (i * Nvy + j) * Nwz + k;
        Velocity v{vx_value(i), vy_value(j), wz_value(k)};
        const auto traj = DwaPlanner::simulate(current, v, p_.sim_time, p_.sim_step);
        if (traj.empty()) {
          continue;
        }

        // Min clearance over rollout.
        double min_d2 = std::numeric_limits<double>::infinity();
        if (M > 0) {
          for (const auto & ps : traj) {
            for (std::size_t m = 0; m < M; ++m) {
              const double dx = static_cast<double>(ox[m]) - ps.x;
              const double dy = static_cast<double>(oy[m]) - ps.y;
              const double d2 = dx * dx + dy * dy;
              if (d2 < min_d2) {
                min_d2 = d2;
              }
            }
          }
        }

        if (min_d2 < r_hard_sq) {
          // Reject this trajectory.
          continue;
        }
        const double min_d = std::sqrt(min_d2);
        clearances[idx] = std::isfinite(min_d) ? min_d : 1e6;
        admissible[idx] = 1u;

        // Costs.
        const auto & end = traj.back();
        const double goal_dx = goal.x - end.x;
        const double goal_dy = goal.y - end.y;
        const double goal_dist = std::hypot(goal_dx, goal_dy);
        const double yaw_err = std::fabs(normalize_angle(goal.yaw - end.yaw));

        double obstacle_cost = 0.0;
        if (M > 0 && min_d < r_inflate) {
          // Linear ramp from 0 at radius -> 1 at hard.
          obstacle_cost = (r_inflate - min_d) / r_band;
        }

        // Forward progress: the larger forward_displacement (current_yaw aligned),
        // the smaller the cost.
        const double cy = std::cos(current.yaw);
        const double sy = std::sin(current.yaw);
        const double dx = end.x - current.x;
        const double dy = end.y - current.y;
        const double forward = cy * dx + sy * dy;
        const double progress_cost = std::max(0.0, 1.0 - std::max(0.0, forward));

        // Smoothness vs last command.
        const double sm =
          (v.vx - last_cmd.vx) * (v.vx - last_cmd.vx) +
          (v.vy - last_cmd.vy) * (v.vy - last_cmd.vy) +
          (v.wz - last_cmd.wz) * (v.wz - last_cmd.wz);

        const double cost =
          p_.goal_dist_weight * goal_dist +
          p_.goal_heading_weight * yaw_err +
          p_.obstacle_cost_weight * obstacle_cost +
          p_.forward_progress_weight * progress_cost +
          p_.velocity_smoothness_weight * sm;
        costs[idx] = cost;
      }
    }
  }

  // Pick the best admissible trajectory.
  int best_idx = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  std::size_t adm = 0;
  for (int idx = 0; idx < total; ++idx) {
    if (!admissible[idx]) {
      continue;
    }
    ++adm;
    if (costs[idx] < best_cost) {
      best_cost = costs[idx];
      best_idx = idx;
    }
  }
  result.admissible_count = adm;
  result.rejected_count = static_cast<std::size_t>(total) - adm;

  if (best_idx < 0) {
    result.reason = "no_admissible_trajectory";
    return result;
  }

  const int i = best_idx / (Nvy * Nwz);
  const int rem = best_idx - i * (Nvy * Nwz);
  const int j = rem / Nwz;
  const int k = rem - j * Nwz;
  result.success = true;
  result.cmd = Velocity{vx_value(i), vy_value(j), wz_value(k)};
  result.best_cost = best_cost;
  result.min_clearance = clearances[best_idx];
  result.reason = "ok";
  return result;
}

}  // namespace nav2_integration_cpp

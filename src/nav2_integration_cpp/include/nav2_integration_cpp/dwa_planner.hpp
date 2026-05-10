// Copyright 2026 a2_system_ws.
// DWA-Lite local planner core (ROS-free) for the A2 3D pipeline.
//
// World convention: positions and velocities are in the **map frame**.
//   - state: (x, y, yaw)
//   - control candidate: (vx, vy, wz) interpreted in the *current robot heading*
//   - obstacles: 2D points (x, y) in the same frame as the state
//
// Forward simulation uses constant body-frame velocity over `sim_time`,
// integrating yaw with wz (so yaw advances during the rollout).

#ifndef NAV2_INTEGRATION_CPP__DWA_PLANNER_HPP_
#define NAV2_INTEGRATION_CPP__DWA_PLANNER_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace nav2_integration_cpp
{

struct DwaParams
{
  // Velocity sampling envelope.
  double max_linear_x{0.18};
  double max_linear_y{0.12};
  double max_yaw_rate{0.3};
  double min_linear_x{0.0};      // allow forward-only by default
  // Number of samples per axis.
  int n_vx{7};
  int n_vy{5};
  int n_wz{9};

  // Forward simulation.
  double sim_time{1.2};
  double sim_step{0.1};

  // Obstacle clearance shaping.
  double inflation_radius{0.40};   // soft clearance start (cost grows < radius)
  double hard_clearance{0.20};     // any rollout point closer than this => trajectory rejected
  double obstacle_cost_weight{2.0};
  double goal_dist_weight{1.0};
  double goal_heading_weight{0.6};
  double forward_progress_weight{0.4};
  double velocity_smoothness_weight{0.3};

  // Goal tolerances.
  double goal_tolerance_xy{0.15};
  double goal_tolerance_yaw{0.18};

  // Block-state grace before reporting "no admissible trajectory".
  double block_grace_sec{3.0};
};

void validate_params(const DwaParams & p);   // throws std::invalid_argument

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Velocity
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct PlanResult
{
  bool success{false};       // false => no admissible trajectory found
  Velocity cmd{};            // body-frame command (vx, vy, wz)
  double best_cost{0.0};
  double min_clearance{0.0}; // m to nearest obstacle along chosen rollout
  std::size_t admissible_count{0};
  std::size_t rejected_count{0};
  std::string reason;        // diagnostic
};

class DwaPlanner
{
public:
  explicit DwaPlanner(const DwaParams & params);
  void reconfigure(const DwaParams & params);
  const DwaParams & params() const noexcept { return p_; }

  /// Plan one cycle.
  /// @param current pose in the planning frame (e.g. map)
  /// @param goal   pose in the planning frame
  /// @param obstacles_xy Nx2 obstacle points in the planning frame.
  ///                    May be empty (no obstacles seen).
  /// @param last_cmd previous command for smoothness term.
  /// @returns PlanResult; never throws.
  PlanResult plan(
    const Pose2D & current,
    const Pose2D & goal,
    const Eigen::Ref<const Eigen::MatrixX2f> & obstacles_xy,
    const Velocity & last_cmd) const noexcept;

  /// Pure helper: simulate a body-frame (vx, vy, wz) constant command from `start`
  /// over `sim_time` at `sim_step`. Returns trajectory of (x,y,yaw) including endpoint.
  static std::vector<Pose2D> simulate(
    const Pose2D & start, const Velocity & v,
    double sim_time, double sim_step) noexcept;

private:
  DwaParams p_{};
};

}  // namespace nav2_integration_cpp

#endif  // NAV2_INTEGRATION_CPP__DWA_PLANNER_HPP_

// Copyright 2026 a2_system_ws.
#include "a2_ground_segmentation_cpp/ground_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>

#if defined(A2_GS_HAVE_OPENMP)
#include <omp.h>
#endif

namespace a2_ground_segmentation_cpp
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr float kQuietNaN = std::numeric_limits<float>::quiet_NaN();

inline bool in_range(double v, double lo, double hi) noexcept
{
  return v >= lo && v <= hi && std::isfinite(v);
}

// V2 reason codes (OccupancyGrid encoding).
constexpr int8_t kReasonUnknown = -1;
constexpr int8_t kReasonFree = 0;
constexpr int8_t kReasonSlope = 10;
constexpr int8_t kReasonRoughness = 20;
constexpr int8_t kReasonStep = 30;
constexpr int8_t kReasonObstacleDensity = 40;
constexpr int8_t kReasonLowConfidence = 50;

// OccupancyGrid value ranges.
constexpr int8_t kUnknown = -1;
constexpr int8_t kFree = 0;
constexpr int8_t kSuspiciousBase = 31;
constexpr int8_t kLethalBase = 71;
constexpr int8_t kLethalMax = 100;
}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Parameter validation
// ─────────────────────────────────────────────────────────────────────────────

void validate_params(const GroundSegmenterParams & p)
{
  if (!in_range(p.radial_divider_angle_deg, 0.05, 30.0)) {
    throw std::invalid_argument("radial_divider_angle_deg must be in (0.05, 30]");
  }
  if (!in_range(p.general_max_slope_deg, 0.0, 60.0)) {
    throw std::invalid_argument("general_max_slope_deg must be in [0, 60]");
  }
  if (!in_range(p.local_max_slope_deg, 0.0, 60.0)) {
    throw std::invalid_argument("local_max_slope_deg must be in [0, 60]");
  }
  if (!in_range(p.initial_max_slope_deg, 0.0, 60.0)) {
    throw std::invalid_argument("initial_max_slope_deg must be in [0, 60]");
  }
  if (!std::isfinite(p.min_height_threshold) || p.min_height_threshold < 0.0) {
    throw std::invalid_argument("min_height_threshold must be >= 0");
  }
  if (!std::isfinite(p.concentric_divider_distance) || p.concentric_divider_distance < 0.0) {
    throw std::invalid_argument("concentric_divider_distance must be >= 0");
  }
  if (!std::isfinite(p.reclass_distance_threshold) || p.reclass_distance_threshold < 0.0) {
    throw std::invalid_argument("reclass_distance_threshold must be >= 0");
  }
  if (!(p.traversability_resolution > 0.0) || !std::isfinite(p.traversability_resolution)) {
    throw std::invalid_argument("traversability_resolution must be > 0");
  }
  if (p.traversability_width <= 0 || p.traversability_height <= 0) {
    throw std::invalid_argument("traversability_width/height must be > 0");
  }
  const std::int64_t cells =
    static_cast<std::int64_t>(p.traversability_width) *
    static_cast<std::int64_t>(p.traversability_height);
  if (cells > 4'000'000) {
    throw std::invalid_argument("traversability_width*height too large (>4M cells)");
  }
  if (!in_range(p.max_traversable_slope_deg, 0.0, 89.0)) {
    throw std::invalid_argument("max_traversable_slope_deg must be in [0, 89]");
  }
  if (!std::isfinite(p.traversability_publish_hz) || p.traversability_publish_hz <= 0.0) {
    throw std::invalid_argument("traversability_publish_hz must be > 0");
  }
  if (!in_range(p.traversability_height_ema_alpha, 0.0, 1.0)) {
    throw std::invalid_argument("traversability_height_ema_alpha must be in [0,1]");
  }
  if (p.traversability_min_count_known < 1) {
    throw std::invalid_argument("traversability_min_count_known must be >= 1");
  }
  // V2 params
  if (p.traversability_cell_timeout_sec <= 0.0) {
    throw std::invalid_argument("traversability_cell_timeout_sec must be > 0");
  }
  if (p.traversability_confidence_decay_per_sec < 0.0) {
    throw std::invalid_argument("traversability_confidence_decay_per_sec must be >= 0");
  }
  if (!in_range(p.traversability_min_confidence, 0.0, 1.0)) {
    throw std::invalid_argument("traversability_min_confidence must be in [0,1]");
  }
  if (p.traversability_unknown_policy != "ignore" &&
      p.traversability_unknown_policy != "lethal" &&
      p.traversability_unknown_policy != "soft_cost") {
    throw std::invalid_argument(
      "traversability_unknown_policy must be 'ignore', 'lethal', or 'soft_cost'");
  }
  if (!in_range(p.traversability_max_slope_deg, 0.0, 89.0)) {
    throw std::invalid_argument("traversability_max_slope_deg must be in [0, 89]");
  }
  if (!std::isfinite(p.traversability_max_roughness_m) || p.traversability_max_roughness_m < 0.0) {
    throw std::invalid_argument("traversability_max_roughness_m must be >= 0");
  }
  if (!std::isfinite(p.traversability_max_step_height_m) || p.traversability_max_step_height_m < 0.0) {
    throw std::invalid_argument("traversability_max_step_height_m must be >= 0");
  }
  if (!in_range(p.traversability_obstacle_density_threshold, 0.0, 1.0)) {
    throw std::invalid_argument("traversability_obstacle_density_threshold must be in [0,1]");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / reconfigure
// ─────────────────────────────────────────────────────────────────────────────

GroundSegmenter::GroundSegmenter(const GroundSegmenterParams & params)
{
  validate_params(params);
  p_ = params;
  recompute_cache();
  allocate_grid_if_needed(p_.traversability_width, p_.traversability_height);
}

void GroundSegmenter::reconfigure(const GroundSegmenterParams & params)
{
  validate_params(params);
  const bool geom_changed =
    params.traversability_width != p_.traversability_width ||
    params.traversability_height != p_.traversability_height ||
    params.traversability_resolution != p_.traversability_resolution ||
    params.traversability_origin_x != p_.traversability_origin_x ||
    params.traversability_origin_y != p_.traversability_origin_y;
  const bool v2_toggled = params.traversability_v2_enabled != p_.traversability_v2_enabled;
  p_ = params;
  recompute_cache();
  if (geom_changed || v2_toggled) {
    height_.clear();
    count_.clear();
    cells_.clear();
    v2_cells_initialized_ = false;
    known_cells_ = 0;
    allocate_grid_if_needed(p_.traversability_width, p_.traversability_height);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Cache / allocation helpers
// ─────────────────────────────────────────────────────────────────────────────

void GroundSegmenter::recompute_cache()
{
  tan_general_ = std::tan(p_.general_max_slope_deg * kPi / 180.0);
  tan_local_ = std::tan(p_.local_max_slope_deg * kPi / 180.0);
  tan_initial_ = std::tan(p_.initial_max_slope_deg * kPi / 180.0);
  const double divider = std::max(0.1, p_.radial_divider_angle_deg);
  num_sectors_ = static_cast<int>(std::ceil(360.0 / divider));
  if (num_sectors_ < 1) {
    num_sectors_ = 1;
  }
}

void GroundSegmenter::allocate_grid_if_needed(int w, int h)
{
  const std::size_t cells = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (height_.size() != cells) {
    height_.assign(cells, kQuietNaN);
    count_.assign(cells, 0);
    known_cells_ = 0;
  }
  if (p_.traversability_v2_enabled) {
    ensure_v2_cells(w, h);
  }
}

void GroundSegmenter::ensure_v2_cells(int w, int h)
{
  const std::size_t cells = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (cells_.size() != cells) {
    cells_.assign(cells, TraversabilityCell{});
    v2_cells_initialized_ = true;
  }
}

void GroundSegmenter::reset_traversability() noexcept
{
  std::fill(height_.begin(), height_.end(), kQuietNaN);
  std::fill(count_.begin(), count_.end(), 0);
  known_cells_ = 0;
  if (!cells_.empty()) {
    std::fill(cells_.begin(), cells_.end(), TraversabilityCell{});
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Classification (unchanged from V1)
// ─────────────────────────────────────────────────────────────────────────────

GroundClassification GroundSegmenter::classify(
  const Eigen::Ref<const Eigen::MatrixX3f> & points) const noexcept
{
  GroundClassification out;
  const auto N = static_cast<std::size_t>(points.rows());
  out.total_count = N;
  out.ground_mask.assign(N, 0u);

  if (N < 10) {
    return out;
  }

  const double divider = std::max(0.1, p_.radial_divider_angle_deg);
  const int num_sectors = num_sectors_;

  // Bucket point indices into sectors by theta (0..360).
  std::vector<std::vector<uint32_t>> buckets(static_cast<std::size_t>(num_sectors));
  std::vector<float> radius(N, 0.0f);

  // First pass: compute radius & sector index, drop NaNs.
  for (std::size_t i = 0; i < N; ++i) {
    const float x = points(static_cast<Eigen::Index>(i), 0);
    const float y = points(static_cast<Eigen::Index>(i), 1);
    const float z = points(static_cast<Eigen::Index>(i), 2);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const float r = std::hypot(x, y);
    radius[i] = r;
    double theta = std::atan2(static_cast<double>(y), static_cast<double>(x)) * kRadToDeg;
    if (theta < 0.0) {
      theta += 360.0;
    } else if (theta >= 360.0) {
      theta -= 360.0;
    }
    int idx = static_cast<int>(std::floor(theta / divider));
    if (idx < 0) {
      idx = 0;
    } else if (idx >= num_sectors) {
      idx = num_sectors - 1;
    }
    buckets[static_cast<std::size_t>(idx)].push_back(static_cast<uint32_t>(i));
  }

  // Second pass: per-sector sort by radius and sweep.
#if defined(A2_GS_HAVE_OPENMP)
  #pragma omp parallel for schedule(dynamic) default(none) \
    shared(buckets, radius, points, out) firstprivate(num_sectors)
#endif
  for (int s = 0; s < num_sectors; ++s) {
    auto & idxs = buckets[static_cast<std::size_t>(s)];
    if (idxs.size() < 2) {
      continue;
    }
    std::sort(
      idxs.begin(), idxs.end(),
      [&radius](uint32_t a, uint32_t b) {return radius[a] < radius[b];});

    double prev_radius = 0.0;
    double prev_height = 0.0;
    bool prev_ground = false;

    for (std::size_t j = 0; j < idxs.size(); ++j) {
      const uint32_t pi = idxs[j];
      const double r = static_cast<double>(radius[pi]);
      const double h = static_cast<double>(points(static_cast<Eigen::Index>(pi), 2));

      bool is_ground = false;

      if (j == 0) {
        double h_thresh = tan_initial_ * r;
        if (h_thresh < p_.min_height_threshold) {
          h_thresh = p_.min_height_threshold;
        }
        const double general_thresh = tan_general_ * r;
        if (std::fabs(h) <= h_thresh) {
          is_ground = true;
        } else if (std::fabs(h) <= general_thresh) {
          is_ground = true;
        }
      } else {
        const double dr = r - prev_radius;
        if (dr < p_.concentric_divider_distance) {
          is_ground = prev_ground;
        } else {
          double local_thresh = tan_local_ * dr;
          if (local_thresh < p_.min_height_threshold) {
            local_thresh = p_.min_height_threshold;
          }
          const double general_thresh = tan_general_ * r;
          const double dh = std::fabs(h - prev_height);
          if (dh <= local_thresh) {
            if (prev_ground) {
              is_ground = true;
            } else {
              is_ground = std::fabs(h) <= general_thresh;
            }
          } else {
            if (dr > p_.reclass_distance_threshold && std::fabs(h) <= general_thresh) {
              is_ground = true;
            } else {
              is_ground = false;
            }
          }
        }
      }

      out.ground_mask[pi] = is_ground ? 1u : 0u;
      prev_ground = is_ground;
      prev_radius = r;
      prev_height = h;
    }
  }

  // Final ground count.
  std::size_t gc = 0;
  for (auto v : out.ground_mask) {
    gc += (v ? 1u : 0u);
  }
  out.ground_count = gc;
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// V1 accumulation (unchanged, for backward compatibility)
// ─────────────────────────────────────────────────────────────────────────────

void GroundSegmenter::accumulate_traversability(
  const Eigen::Ref<const Eigen::MatrixX3f> & ground_points) noexcept
{
  const auto N = static_cast<std::size_t>(ground_points.rows());
  if (N < 20 || height_.empty()) {
    return;
  }
  const float res = static_cast<float>(p_.traversability_resolution);
  const float ox = static_cast<float>(p_.traversability_origin_x);
  const float oy = static_cast<float>(p_.traversability_origin_y);
  const int W = p_.traversability_width;
  const int H = p_.traversability_height;
  const float alpha = static_cast<float>(p_.traversability_height_ema_alpha);
  const float one_minus_alpha = 1.0f - alpha;

  for (std::size_t i = 0; i < N; ++i) {
    const float x = ground_points(static_cast<Eigen::Index>(i), 0);
    const float y = ground_points(static_cast<Eigen::Index>(i), 1);
    const float z = ground_points(static_cast<Eigen::Index>(i), 2);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const int c = static_cast<int>(std::floor((x - ox) / res));
    const int r = static_cast<int>(std::floor((y - oy) / res));
    if (c < 0 || c >= W || r < 0 || r >= H) {
      continue;
    }
    const std::size_t k = static_cast<std::size_t>(r) * static_cast<std::size_t>(W) +
      static_cast<std::size_t>(c);
    const int32_t cnt = count_[k];
    const float old = height_[k];
    if (cnt == 0 || !std::isfinite(old)) {
      height_[k] = z;
    } else {
      height_[k] = old * one_minus_alpha + z * alpha;
    }
    if (cnt < p_.traversability_min_count_known &&
      cnt + 1 >= p_.traversability_min_count_known)
    {
      ++known_cells_;
    }
    count_[k] = cnt + 1;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// V2 multi-metric traversability accumulation
// ─────────────────────────────────────────────────────────────────────────────

void GroundSegmenter::accumulate_traversability_v2(
  const Eigen::Ref<const Eigen::MatrixX3f> & ground_points,
  const Eigen::Ref<const Eigen::MatrixX3f> & obstacle_points,
  double now_sec) noexcept
{
  if (cells_.empty()) {
    ensure_v2_cells(p_.traversability_width, p_.traversability_height);
  }
  if (cells_.empty()) {
    return;
  }
  const float res = static_cast<float>(p_.traversability_resolution);
  const float ox = static_cast<float>(p_.traversability_origin_x);
  const float oy = static_cast<float>(p_.traversability_origin_y);
  const int W = p_.traversability_width;
  const int H = p_.traversability_height;

  auto accumulate_points =
    [&](const Eigen::Ref<const Eigen::MatrixX3f> & pts, bool is_ground) {
    const auto N = static_cast<std::size_t>(pts.rows());
    for (std::size_t i = 0; i < N; ++i) {
      const float x = pts(static_cast<Eigen::Index>(i), 0);
      const float y = pts(static_cast<Eigen::Index>(i), 1);
      const float z = pts(static_cast<Eigen::Index>(i), 2);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const int c = static_cast<int>(std::floor((x - ox) / res));
      const int r = static_cast<int>(std::floor((y - oy) / res));
      if (c < 0 || c >= W || r < 0 || r >= H) {
        continue;
      }
      const std::size_t k = static_cast<std::size_t>(r) * static_cast<std::size_t>(W) +
        static_cast<std::size_t>(c);
      auto & cell = cells_[k];

      // Per-frame accumulation (no per-point EMA — that happens in finalize).
      cell.frame_sum_z += z;
      cell.frame_sum_z2 += z * z;
      cell.frame_point_count++;

      // Per-frame min/max.
      if (!cell.frame_has_sample) {
        cell.frame_min_z = z;
        cell.frame_max_z = z;
        cell.frame_has_sample = true;
      } else {
        if (z < cell.frame_min_z) { cell.frame_min_z = z; }
        if (z > cell.frame_max_z) { cell.frame_max_z = z; }
      }

      // Cumulative counts.
      cell.point_count++;
      if (is_ground) {
        cell.ground_count++;
      } else {
        cell.obstacle_count++;
      }

      cell.last_seen_time = now_sec;
      // Boost confidence on fresh data (cap at 1.0).
      cell.confidence = std::min(1.0f, cell.confidence + 0.15f);
      if (cell.confidence > 1.0f) { cell.confidence = 1.0f; }
    }
  };

  // Accumulate ground first, then obstacle.
  accumulate_points(ground_points, true);
  accumulate_points(obstacle_points, false);

  // Apply per-frame EMA to mean_z, variance, min_z, max_z.
  finalize_v2_frame(now_sec);
}

void GroundSegmenter::finalize_v2_frame(double now_sec) noexcept
{
  const float ema_alpha = static_cast<float>(p_.traversability_height_ema_alpha);
  const float one_minus_alpha = 1.0f - ema_alpha;
  (void)now_sec;

  for (auto & cell : cells_) {
    if (!cell.frame_has_sample) {
      continue;
    }
    // Per-frame mean_z: EMA of frame observation mean.
    const float frame_count_f = static_cast<float>(cell.frame_point_count);
    const float frame_mean_z = cell.frame_sum_z / frame_count_f;

    // Per-frame variance: max(0, E[z^2] - E[z]^2).
    const float frame_var = std::max(0.0f,
      cell.frame_sum_z2 / frame_count_f - frame_mean_z * frame_mean_z);

    // EMA-blend mean_z across frames.
    const bool first_frame = (cell.point_count == cell.frame_point_count);
    if (first_frame) {
      cell.mean_z = frame_mean_z;
      cell.m2_z = frame_var;       // m2_z now stores EMA-smoothed variance.
      cell.min_z = cell.frame_min_z;
      cell.max_z = cell.frame_max_z;
    } else {
      cell.mean_z = cell.mean_z * one_minus_alpha + frame_mean_z * ema_alpha;
      cell.m2_z = cell.m2_z * one_minus_alpha + frame_var * ema_alpha;
      cell.min_z = cell.min_z * one_minus_alpha + cell.frame_min_z * ema_alpha;
      cell.max_z = cell.max_z * one_minus_alpha + cell.frame_max_z * ema_alpha;
    }

    // Reset per-frame accumulators.
    cell.frame_sum_z = 0.0f;
    cell.frame_sum_z2 = 0.0f;
    cell.frame_point_count = 0;
    cell.frame_min_z = 0.0f;
    cell.frame_max_z = 0.0f;
    cell.frame_has_sample = false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// V2 cost computation for a single cell
// ─────────────────────────────────────────────────────────────────────────────

void GroundSegmenter::compute_cell_costs(
  TraversabilityCell & cell,
  const TraversabilityCell * right,
  const TraversabilityCell * down,
  double now_sec) const noexcept
{
  // ── Effective confidence (with decay, does NOT mutate cell.confidence) ─
  const float dt = static_cast<float>(now_sec - cell.last_seen_time);
  float eff_conf = cell.confidence;
  if (dt > static_cast<float>(p_.traversability_cell_timeout_sec)) {
    const float excess = dt - static_cast<float>(p_.traversability_cell_timeout_sec);
    eff_conf -= static_cast<float>(p_.traversability_confidence_decay_per_sec) * excess;
    if (eff_conf < 0.0f) { eff_conf = 0.0f; }
  }
  cell.effective_confidence = eff_conf;

  // Reset costs.
  cell.slope_cost = 0.0f;
  cell.roughness_cost = 0.0f;
  cell.step_cost = 0.0f;
  cell.obstacle_density_cost = 0.0f;
  cell.confidence_cost = 0.0f;
  cell.final_cost = 0.0f;

  // ── Obstacle density cost ─────────────────────────────────────────────
  if (cell.point_count > 0) {
    const float density = static_cast<float>(cell.obstacle_count) /
      static_cast<float>(cell.point_count);
    const float thresh = static_cast<float>(p_.traversability_obstacle_density_threshold);
    if (density > thresh) {
      cell.obstacle_density_cost = std::min(100.0f, (density - thresh) / (1.0f - thresh + 0.01f) * 100.0f);
    }
  }

  // ── Roughness cost (from EMA-smoothed per-frame variance) ────────────
  if (cell.point_count >= static_cast<int32_t>(p_.traversability_min_count_known) &&
      cell.m2_z > 0.0f) {
    const float stddev = std::sqrt(cell.m2_z);
    const float max_r = static_cast<float>(p_.traversability_max_roughness_m);
    if (max_r > 0.0f && stddev > max_r) {
      cell.roughness_cost = std::min(100.0f, (stddev - max_r) / (max_r + 0.001f) * 100.0f);
    }
  }

  // ── Slope & step costs (from neighbors) ───────────────────────────────
  const float res = static_cast<float>(p_.traversability_resolution);
  float max_slope_deg = 0.0f;
  float max_step_m = 0.0f;

  auto check_neighbor = [&](const TraversabilityCell * nb) {
    if (!nb || nb->point_count < p_.traversability_min_count_known) { return; }
    const float dz = std::fabs(cell.mean_z - nb->mean_z);
    const float s = std::atan2(dz, res) * static_cast<float>(kRadToDeg);
    if (s > max_slope_deg) { max_slope_deg = s; }
    if (dz > max_step_m) { max_step_m = dz; }
  };

  check_neighbor(right);
  check_neighbor(down);

  const float max_slope_allowed = static_cast<float>(p_.traversability_max_slope_deg);
  if (max_slope_deg > max_slope_allowed) {
    cell.slope_cost = std::min(100.0f, (max_slope_deg - max_slope_allowed) /
      (max_slope_allowed + 0.1f) * 100.0f);
  }

  const float max_step_allowed = static_cast<float>(p_.traversability_max_step_height_m);
  if (max_step_m > max_step_allowed) {
    cell.step_cost = std::min(100.0f, (max_step_m - max_step_allowed) /
      (max_step_allowed + 0.001f) * 100.0f);
  }

  // ── Confidence cost (uses effective confidence, NOT persistent confidence) ─
  if (eff_conf < static_cast<float>(p_.traversability_min_confidence)) {
    cell.confidence_cost = static_cast<float>(p_.traversability_unknown_cost);
  }

  // ── Final cost = max of all components ───────────────────────────────
  cell.final_cost = std::max({
    cell.slope_cost,
    cell.roughness_cost,
    cell.step_cost,
    cell.obstacle_density_cost,
    cell.confidence_cost,
  });

  // ── Reason code: max-cost source with stable tie-breaking ────────────
  // Priority: step > obstacle_density > slope > roughness > low_confidence > free
  const float eps = 0.001f;
  if (cell.final_cost >= eps) {
    // Find which component(s) produced the max cost.
    if (cell.step_cost >= cell.final_cost - eps) {
      cell.reason_code = kReasonStep;
    } else if (cell.obstacle_density_cost >= cell.final_cost - eps) {
      cell.reason_code = kReasonObstacleDensity;
    } else if (cell.slope_cost >= cell.final_cost - eps) {
      cell.reason_code = kReasonSlope;
    } else if (cell.roughness_cost >= cell.final_cost - eps) {
      cell.reason_code = kReasonRoughness;
    } else if (cell.confidence_cost >= cell.final_cost - eps) {
      cell.reason_code = kReasonLowConfidence;
    } else {
      cell.reason_code = kReasonUnknown;
    }
  } else if (cell.point_count >= p_.traversability_min_count_known) {
    cell.reason_code = kReasonFree;
  } else {
    cell.reason_code = kReasonUnknown;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Unified unknown-policy application (stats + grid must use same logic)
// ─────────────────────────────────────────────────────────────────────────────

int8_t GroundSegmenter::apply_unknown_policy(const TraversabilityCell & cell) const noexcept
{
  if (cell.effective_confidence < static_cast<float>(p_.traversability_min_confidence)) {
    if (p_.traversability_unknown_policy == "ignore") {
      return kUnknown;
    } else if (p_.traversability_unknown_policy == "lethal") {
      return kLethalMax;
    } else if (p_.traversability_unknown_policy == "soft_cost") {
      return static_cast<int8_t>(p_.traversability_unknown_cost);
    }
  }
  return static_cast<int8_t>(std::clamp(static_cast<int>(cell.final_cost), 0, 100));
}

// ─────────────────────────────────────────────────────────────────────────────
// V1 render (binary 0/100)
// ─────────────────────────────────────────────────────────────────────────────

bool GroundSegmenter::render_traversability(std::vector<int8_t> & out_data) const noexcept
{
  if (p_.traversability_v2_enabled && !cells_.empty()) {
    // Delegate to V2 — caller must provide time; we use zero as sentinel
    // and the node layer provides the real timestamp.
    // Callers that go through render_cost_layer / render_traversability_v2
    // get proper timestamps. This path keeps the V1 API signature.
    out_data.clear();
    return false;
  }

  if (height_.empty()) {
    out_data.clear();
    return false;
  }
  const int W = p_.traversability_width;
  const int H = p_.traversability_height;
  const std::size_t cells = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);
  out_data.assign(cells, static_cast<int8_t>(-1));

  const float res = static_cast<float>(p_.traversability_resolution);
  const float max_slope = static_cast<float>(p_.max_traversable_slope_deg);
  const int min_count = p_.traversability_min_count_known;

#if defined(A2_GS_HAVE_OPENMP)
  #pragma omp parallel for schedule(static) default(none) \
    shared(out_data) firstprivate(W, H, res, max_slope, min_count)
#endif
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      const std::size_t k = static_cast<std::size_t>(r) * static_cast<std::size_t>(W) +
        static_cast<std::size_t>(c);
      if (count_[k] < min_count || !std::isfinite(height_[k])) {
        continue;  // remain unknown (-1)
      }
      out_data[k] = 0;  // default traversable
      float max_local_slope = 0.0f;
      const float h0 = height_[k];
      // right neighbor
      if (c + 1 < W) {
        const std::size_t kr = k + 1;
        if (count_[kr] >= min_count && std::isfinite(height_[kr])) {
          const float dz = std::fabs(h0 - height_[kr]);
          const float s = std::atan2(dz, res) * static_cast<float>(kRadToDeg);
          if (s > max_local_slope) {
            max_local_slope = s;
          }
        }
      }
      // down neighbor
      if (r + 1 < H) {
        const std::size_t kd = k + static_cast<std::size_t>(W);
        if (count_[kd] >= min_count && std::isfinite(height_[kd])) {
          const float dz = std::fabs(h0 - height_[kd]);
          const float s = std::atan2(dz, res) * static_cast<float>(kRadToDeg);
          if (s > max_local_slope) {
            max_local_slope = s;
          }
        }
      }
      if (max_local_slope > max_slope) {
        out_data[k] = static_cast<int8_t>(100);
      }
    }
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// V2 render — final traversability grid
// ─────────────────────────────────────────────────────────────────────────────

bool GroundSegmenter::render_traversability_v2(
  std::vector<int8_t> & out_data, double now_sec) const noexcept
{
  if (cells_.empty()) {
    out_data.clear();
    return false;
  }
  const int W = p_.traversability_width;
  const int H = p_.traversability_height;
  const std::size_t cells = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);
  out_data.assign(cells, kUnknown);

  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      const std::size_t k = static_cast<std::size_t>(r) * static_cast<std::size_t>(W) +
        static_cast<std::size_t>(c);
      auto & cell = cells_[k];
      if (cell.point_count < p_.traversability_min_count_known) {
        // Unknown cell — apply unknown policy.
        if (p_.traversability_unknown_policy == "lethal") {
          out_data[k] = kLethalMax;
        } else if (p_.traversability_unknown_policy == "soft_cost") {
          out_data[k] = static_cast<int8_t>(p_.traversability_unknown_cost);
        } else {
          out_data[k] = kUnknown;  // "ignore"
        }
        continue;
      }
      const TraversabilityCell * right =
        (c + 1 < W) ? &cells_[k + 1] : nullptr;
      const TraversabilityCell * down =
        (r + 1 < H) ? &cells_[k + static_cast<std::size_t>(W)] : nullptr;
      compute_cell_costs(cell, right, down, now_sec);
      out_data[k] = apply_unknown_policy(cell);
    }
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// V2 debug cost layer render
// ─────────────────────────────────────────────────────────────────────────────

bool GroundSegmenter::render_cost_layer(
  std::vector<int8_t> & out_data,
  const std::string & cost_type,
  double now_sec) const noexcept
{
  if (cells_.empty()) {
    out_data.clear();
    return false;
  }
  const int W = p_.traversability_width;
  const int H = p_.traversability_height;
  const std::size_t cells = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);
  out_data.assign(cells, kUnknown);

  if (cost_type == "reason") {
    for (int r = 0; r < H; ++r) {
      for (int c = 0; c < W; ++c) {
        const std::size_t k = static_cast<std::size_t>(r) * static_cast<std::size_t>(W) +
          static_cast<std::size_t>(c);
        auto & cell = cells_[k];
        if (cell.point_count < p_.traversability_min_count_known) {
          out_data[k] = kUnknown;
          continue;
        }
        const TraversabilityCell * right =
          (c + 1 < W) ? &cells_[k + 1] : nullptr;
        const TraversabilityCell * down =
          (r + 1 < H) ? &cells_[k + static_cast<std::size_t>(W)] : nullptr;
        compute_cell_costs(cell, right, down, now_sec);

        // Reason layer: use the same policy as the main grid, but encode
        // low-confidence reason when policy makes the cell lethal/soft_cost.
        if (cell.effective_confidence < static_cast<float>(p_.traversability_min_confidence)) {
          if (p_.traversability_unknown_policy == "ignore") {
            out_data[k] = kUnknown;
          } else {
            out_data[k] = kReasonLowConfidence;
          }
        } else {
          out_data[k] = cell.reason_code;
        }
      }
    }
    return true;
  }

  // For cost layers, compute cell costs first.
  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      const std::size_t k = static_cast<std::size_t>(r) * static_cast<std::size_t>(W) +
        static_cast<std::size_t>(c);
      auto & cell = cells_[k];
      if (cell.point_count < p_.traversability_min_count_known) {
        continue;
      }
      const TraversabilityCell * right =
        (c + 1 < W) ? &cells_[k + 1] : nullptr;
      const TraversabilityCell * down =
        (r + 1 < H) ? &cells_[k + static_cast<std::size_t>(W)] : nullptr;
      compute_cell_costs(cell, right, down, now_sec);

      float val = 0.0f;
      if (cost_type == "slope") {
        val = cell.slope_cost;
      } else if (cost_type == "roughness") {
        val = cell.roughness_cost;
      } else if (cost_type == "step") {
        val = cell.step_cost;
      } else if (cost_type == "obstacle_density") {
        val = cell.obstacle_density_cost;
      } else if (cost_type == "confidence") {
        val = (1.0f - cell.effective_confidence) * 100.0f;  // inverted: high confidence = low cost
      } else {
        continue;
      }
      out_data[k] = static_cast<int8_t>(std::clamp(static_cast<int>(val), 0, 100));
    }
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// V2 stats
// ─────────────────────────────────────────────────────────────────────────────

TraversabilityV2Stats GroundSegmenter::get_v2_stats(double now_sec) const noexcept
{
  TraversabilityV2Stats stats;
  if (cells_.empty()) {
    return stats;
  }
  const int W = p_.traversability_width;
  const int H = p_.traversability_height;
  float conf_sum = 0.0f;
  std::size_t conf_count = 0;

  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      const std::size_t k = static_cast<std::size_t>(r) * static_cast<std::size_t>(W) +
        static_cast<std::size_t>(c);
      auto & cell = cells_[k];
      if (cell.point_count < p_.traversability_min_count_known) {
        ++stats.unknown_cells;
        continue;
      }
      const TraversabilityCell * right =
        (c + 1 < W) ? &cells_[k + 1] : nullptr;
      const TraversabilityCell * down =
        (r + 1 < H) ? &cells_[k + static_cast<std::size_t>(W)] : nullptr;
      compute_cell_costs(cell, right, down, now_sec);

      // Count stale based on raw time delta (before policy classification).
      const float dt = static_cast<float>(now_sec - cell.last_seen_time);
      if (dt > static_cast<float>(p_.traversability_cell_timeout_sec)) {
        ++stats.stale_cells;
      }

      // Use the SAME policy-application logic as render_traversability_v2.
      const int8_t rendered = apply_unknown_policy(cell);

      if (rendered == kUnknown) {
        // ignore policy on low-confidence: cell is unknown.
        ++stats.unknown_cells;
        continue;
      }

      ++stats.known_cells;
      conf_sum += cell.effective_confidence;
      ++conf_count;

      // max_cost / high_cost_cells based on rendered value, not raw final_cost.
      const float rendered_f = static_cast<float>(static_cast<int>(rendered));
      if (rendered_f > stats.max_cost) {
        stats.max_cost = rendered_f;
      }
      if (rendered_f >= static_cast<float>(kLethalBase)) {
        ++stats.high_cost_cells;
      }
    }
  }
  if (conf_count > 0) {
    stats.mean_confidence = conf_sum / static_cast<float>(conf_count);
  }
  return stats;
}

}  // namespace a2_ground_segmentation_cpp

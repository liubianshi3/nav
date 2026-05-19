// Copyright 2026 a2_system_ws.
// Pure (ROS-free) ground segmenter and traversability accumulator.
//
// V1: Simple EMA height map + binary slope check.
// V2: Multi-layer cell statistics, online variance, time decay, graded cost output.

#ifndef A2_GROUND_SEGMENTATION_CPP__GROUND_SEGMENTER_HPP_
#define A2_GROUND_SEGMENTATION_CPP__GROUND_SEGMENTER_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace a2_ground_segmentation_cpp
{

struct GroundSegmenterParams
{
  double radial_divider_angle_deg{1.0};   // (0, 30]
  double general_max_slope_deg{8.0};      // [0, 60]
  double local_max_slope_deg{6.0};        // [0, 60]
  double initial_max_slope_deg{3.0};      // [0, 60]
  double min_height_threshold{0.15};      // >= 0
  double concentric_divider_distance{0.01};  // >= 0
  double reclass_distance_threshold{0.1};    // >= 0

  // Traversability grid
  double traversability_resolution{0.1};  // > 0
  int traversability_width{400};          // > 0, width*height <= 4_000_000
  int traversability_height{400};
  double traversability_origin_x{-20.0};
  double traversability_origin_y{-20.0};
  double max_traversable_slope_deg{20.0};
  double traversability_publish_hz{1.0};

  // EMA factor for height map update (matches Python alpha=0.3).
  double traversability_height_ema_alpha{0.3};

  // Minimum sample count per cell to be counted as "known" (matches Python "count >= 3").
  int traversability_min_count_known{3};

  // ── V2 multi-metric traversability ──────────────────────────────────

  /// Enable V2 cell statistics, time decay, and graded cost output.
  bool traversability_v2_enabled{false};

  /// Cells not refreshed within this window begin confidence decay.
  double traversability_cell_timeout_sec{1.5};

  /// Confidence lost per second when a cell is stale.
  double traversability_confidence_decay_per_sec{0.8};

  /// Cells with confidence below this threshold become unknown / soft cost.
  double traversability_min_confidence{0.35};

  /// Unknown cell policy: "ignore", "lethal", or "soft_cost".
  std::string traversability_unknown_policy{"ignore"};

  /// Cost assigned to unknown cells when policy=soft_cost.
  int traversability_unknown_cost{30};

  /// Maximum slope (deg) before a cell is considered lethal.
  double traversability_max_slope_deg{18.0};

  /// Maximum roughness (m stddev) for traversability.
  double traversability_max_roughness_m{0.06};

  /// Maximum step height (m) between adjacent cells.
  double traversability_max_step_height_m{0.10};

  /// Obstacle-point ratio threshold for obstacle_density_cost to trigger.
  double traversability_obstacle_density_threshold{0.25};

  /// Enable debug cost layer publishers (6x OccupancyGrid).
  bool traversability_debug_enabled{false};

  /// Maximum publish rate for debug layers (Hz).
  double traversability_debug_publish_hz{1.0};
};

void validate_params(const GroundSegmenterParams & p);  // throws std::invalid_argument

/// Result of one frame's classification.
struct GroundClassification
{
  // ground_mask[i] = true iff input point i was classified as ground.
  std::vector<uint8_t> ground_mask;
  // ground_count summary (also recoverable from the mask but cached for speed).
  std::size_t ground_count{0};
  std::size_t total_count{0};
};

/// Per-cell statistics for V2 multi-metric traversability.
struct TraversabilityCell
{
  float min_z = 0.0f;
  float max_z = 0.0f;
  float mean_z = 0.0f;
  float m2_z = 0.0f;          // Welford M2 for online variance
  int32_t point_count = 0;
  int32_t ground_count = 0;
  int32_t obstacle_count = 0;
  double last_seen_time = 0.0;  // ROS steady-clock seconds
  float confidence = 0.0f;

  // Effective confidence after time decay (computed during render, NOT persistent).
  mutable float effective_confidence = 0.0f;

  // Computed costs (populated during render).
  float slope_cost = 0.0f;
  float roughness_cost = 0.0f;
  float step_cost = 0.0f;
  float obstacle_density_cost = 0.0f;
  float confidence_cost = 0.0f;
  float final_cost = 0.0f;
  int8_t reason_code = -1;

  // Per-frame accumulators (reset in finalize_v2_frame after EMA application).
  float frame_sum_z = 0.0f;
  float frame_sum_z2 = 0.0f;
  int32_t frame_point_count = 0;
  float frame_min_z = 0.0f;
  float frame_max_z = 0.0f;
  bool frame_has_sample = false;
};

/// V2 statistics bundle.
struct TraversabilityV2Stats
{
  std::size_t known_cells{0};
  std::size_t unknown_cells{0};
  std::size_t stale_cells{0};
  std::size_t high_cost_cells{0};
  float max_cost{0.0f};
  float mean_confidence{0.0f};
};

/// Stateful segmenter (holds the traversability accumulator across frames).
class GroundSegmenter
{
public:
  explicit GroundSegmenter(const GroundSegmenterParams & params);

  // Re-validates and stores params. Will reset the traversability grid only if
  // the grid geometry changed.
  void reconfigure(const GroundSegmenterParams & params);

  const GroundSegmenterParams & params() const noexcept { return p_; }

  /// Classify points (Nx3, one XYZ point per row). Never throws.
  GroundClassification classify(const Eigen::Ref<const Eigen::MatrixX3f> & points) const noexcept;

  /// V1: Update traversability grid with ground points (Nx3). Never throws.
  void accumulate_traversability(const Eigen::Ref<const Eigen::MatrixX3f> & ground_points) noexcept;

  /// V2: Update traversability grid with ground + obstacle points, with timestamp.
  void accumulate_traversability_v2(
    const Eigen::Ref<const Eigen::MatrixX3f> & ground_points,
    const Eigen::Ref<const Eigen::MatrixX3f> & obstacle_points,
    double now_sec) noexcept;

  /// Render the V1 traversability grid into an int8 occupancy grid.
  /// V2 callers must use render_traversability_v2(double now_sec) instead;
  /// this method returns false when V2 is enabled and cells are populated.
  bool render_traversability(std::vector<int8_t> & out_data) const noexcept;

  /// Render the V2 final multi-metric traversability grid.
  bool render_traversability_v2(std::vector<int8_t> & out_data, double now_sec) const noexcept;

  /// Render a V2 debug cost layer at the current time.
  /// cost_type: "slope", "roughness", "step", "obstacle_density", "confidence", "reason"
  bool render_cost_layer(std::vector<int8_t> & out_data, const std::string & cost_type, double now_sec) const noexcept;

  /// Reset the traversability accumulator (used when grid geometry changes).
  void reset_traversability() noexcept;

  /// Number of valid (counted) cells, for diagnostics.
  std::size_t known_cell_count() const noexcept { return known_cells_; }

  /// V2 statistics query.
  TraversabilityV2Stats get_v2_stats(double now_sec) const noexcept;

private:
  GroundSegmenterParams p_{};

  // V1 accumulator (row-major). NaN in height means "no sample yet".
  std::vector<float> height_;
  std::vector<int32_t> count_;
  std::size_t known_cells_{0};

  // V2 accumulator (mutable: cost fields are computed on-demand during const renders).
  mutable std::vector<TraversabilityCell> cells_;
  bool v2_cells_initialized_{false};

  // Cached trig.
  double tan_general_{0.0};
  double tan_local_{0.0};
  double tan_initial_{0.0};
  int num_sectors_{360};

  void recompute_cache();
  void allocate_grid_if_needed(int w, int h);
  void ensure_v2_cells(int w, int h);
  void finalize_v2_frame(double now_sec) noexcept;

  /// Compute V2 costs for a single cell, given neighbors.
  void compute_cell_costs(
    TraversabilityCell & cell,
    const TraversabilityCell * right,
    const TraversabilityCell * down,
    double now_sec) const noexcept;

  /// Apply unknown_policy to a cell whose costs have been computed.
  /// Returns the OccupancyGrid value consistent with render_traversability_v2.
  int8_t apply_unknown_policy(const TraversabilityCell & cell) const noexcept;
};

}  // namespace a2_ground_segmentation_cpp

#endif  // A2_GROUND_SEGMENTATION_CPP__GROUND_SEGMENTER_HPP_

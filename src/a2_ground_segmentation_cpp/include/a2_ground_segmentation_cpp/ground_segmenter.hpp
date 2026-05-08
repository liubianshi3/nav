// Copyright 2026 a2_system_ws.
// Pure (ROS-free) ground segmenter and traversability accumulator.
//
// This is a faithful C++ port of the Python ray-ground filter implemented in
// `a2_ground_segmentation/ground_segmentation_node.py`. It is exercised by
// gtest unit tests against the same algorithm description.
//
// Defensive contract:
//   - All public methods are exception-safe; bad inputs produce empty outputs
//     (and counters), never throw.
//   - Configure() validates parameter ranges and throws std::invalid_argument
//     synchronously at construction/reconfigure time only.

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

/// Stateful segmenter (holds the EMA traversability accumulator across frames).
class GroundSegmenter
{
public:
  explicit GroundSegmenter(const GroundSegmenterParams & params);

  // Re-validates and stores params. Will reset the traversability grid only if
  // the grid geometry changed.
  void reconfigure(const GroundSegmenterParams & params);

  const GroundSegmenterParams & params() const noexcept { return p_; }

  /// Classify points (Nx3, row-major XYZ). Never throws.
  GroundClassification classify(const Eigen::Ref<const Eigen::MatrixX3f> & points) const noexcept;

  /// Update traversability grid with ground points (Nx3). Never throws.
  void accumulate_traversability(const Eigen::Ref<const Eigen::MatrixX3f> & ground_points) noexcept;

  /// Render the current traversability grid into an int8 occupancy grid.
  /// data layout: row-major, row r at offset r*width.
  ///  -1 = unknown, 0 = traversable, 100 = occupied (steep)
  /// Returns false if no data has been accumulated yet.
  bool render_traversability(std::vector<int8_t> & out_data) const noexcept;

  /// Reset the traversability accumulator (used when grid geometry changes).
  void reset_traversability() noexcept;

  /// Number of valid (counted) cells, for diagnostics.
  std::size_t known_cell_count() const noexcept { return known_cells_; }

private:
  GroundSegmenterParams p_{};

  // Traversability accumulator (row-major). NaN in height means "no sample yet".
  std::vector<float> height_;
  std::vector<int32_t> count_;
  std::size_t known_cells_{0};

  // Cached trig.
  double tan_general_{0.0};
  double tan_local_{0.0};
  double tan_initial_{0.0};
  int num_sectors_{360};

  void recompute_cache();
  void allocate_grid_if_needed(int w, int h);
};

}  // namespace a2_ground_segmentation_cpp

#endif  // A2_GROUND_SEGMENTATION_CPP__GROUND_SEGMENTER_HPP_

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
}  // namespace

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
}

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
  p_ = params;
  recompute_cache();
  if (geom_changed) {
    height_.clear();
    count_.clear();
    known_cells_ = 0;
    allocate_grid_if_needed(p_.traversability_width, p_.traversability_height);
  }
}

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
}

void GroundSegmenter::reset_traversability() noexcept
{
  std::fill(height_.begin(), height_.end(), kQuietNaN);
  std::fill(count_.begin(), count_.end(), 0);
  known_cells_ = 0;
}

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
  // Ground mask is written from independent index sets => safe to parallelize.
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
          // Inherit prev classification (matches Python).
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

  // Sequential update is safe & fast (typical N << 1e5 after ground filter).
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

bool GroundSegmenter::render_traversability(std::vector<int8_t> & out_data) const noexcept
{
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

  // Compute per-cell max slope from neighbors (right/down). Mirrors Python.
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

}  // namespace a2_ground_segmentation_cpp

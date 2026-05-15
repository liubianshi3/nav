// Copyright 2026 a2_system_ws.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "a2_ground_segmentation_cpp/ground_segmenter.hpp"

using a2_ground_segmentation_cpp::GroundSegmenter;
using a2_ground_segmentation_cpp::GroundSegmenterParams;

namespace
{
GroundSegmenterParams default_params()
{
  GroundSegmenterParams p;
  return p;
}
}  // namespace

TEST(GroundSegmenterParams, RejectsBadValues)
{
  GroundSegmenterParams p;
  p.radial_divider_angle_deg = 0.0;
  EXPECT_THROW(GroundSegmenter{p}, std::invalid_argument);

  p = GroundSegmenterParams{};
  p.traversability_resolution = 0.0;
  EXPECT_THROW(GroundSegmenter{p}, std::invalid_argument);

  p = GroundSegmenterParams{};
  p.traversability_width = -1;
  EXPECT_THROW(GroundSegmenter{p}, std::invalid_argument);

  p = GroundSegmenterParams{};
  p.traversability_width = 5000;
  p.traversability_height = 5000;  // 25M cells > 4M cap
  EXPECT_THROW(GroundSegmenter{p}, std::invalid_argument);
}

TEST(GroundSegmenterClassify, ShortInputReturnsAllNonGround)
{
  GroundSegmenter seg(default_params());
  Eigen::MatrixX3f pts(3, 3);
  pts << 1.0f, 0.0f, 0.0f,
    2.0f, 0.0f, 0.0f,
    3.0f, 0.0f, 0.0f;
  auto cls = seg.classify(pts);
  EXPECT_EQ(cls.total_count, 3u);
  EXPECT_EQ(cls.ground_count, 0u);
}

TEST(GroundSegmenterClassify, FlatGroundIsClassifiedGround)
{
  GroundSegmenter seg(default_params());
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> noise(-0.02f, 0.02f);
  std::vector<float> data;
  // 360 sectors * 12 ranges of flat ground at z ~ 0.
  const int sectors = 360;
  const int rings = 12;
  const int n = sectors * rings;
  Eigen::MatrixX3f pts(n, 3);
  int row = 0;
  for (int s = 0; s < sectors; ++s) {
    const float theta = static_cast<float>(s) * static_cast<float>(M_PI) / 180.0f;
    for (int rr = 0; rr < rings; ++rr) {
      const float r = 0.5f + 0.5f * static_cast<float>(rr);
      pts(row, 0) = r * std::cos(theta);
      pts(row, 1) = r * std::sin(theta);
      pts(row, 2) = noise(rng);
      ++row;
    }
  }
  auto cls = seg.classify(pts);
  // Expect overwhelming majority classified as ground (>=95%).
  EXPECT_GE(cls.ground_count * 100u, static_cast<unsigned>(n) * 95u);
}

TEST(GroundSegmenterClassify, ObstacleAboveGroundIsRejected)
{
  GroundSegmenter seg(default_params());
  // Build flat ring + a tall pillar at one bearing.
  const int sectors = 360;
  const int rings = 8;
  const int pillar_pts = 30;
  const int n = sectors * rings + pillar_pts;
  Eigen::MatrixX3f pts(n, 3);
  int row = 0;
  for (int s = 0; s < sectors; ++s) {
    const float theta = static_cast<float>(s) * static_cast<float>(M_PI) / 180.0f;
    for (int rr = 0; rr < rings; ++rr) {
      const float r = 0.5f + 0.5f * static_cast<float>(rr);
      pts(row, 0) = r * std::cos(theta);
      pts(row, 1) = r * std::sin(theta);
      pts(row, 2) = 0.0f;
      ++row;
    }
  }
  // Pillar at theta=0, r=2.0, z 0.5..2.0
  for (int p = 0; p < pillar_pts; ++p) {
    pts(row, 0) = 2.0f;
    pts(row, 1) = 0.0f;
    pts(row, 2) = 0.5f + 0.05f * static_cast<float>(p);
    ++row;
  }
  auto cls = seg.classify(pts);
  // Pillar points should be obstacles.
  std::size_t pillar_obstacle = 0;
  for (int i = sectors * rings; i < n; ++i) {
    if (!cls.ground_mask[static_cast<std::size_t>(i)]) {
      ++pillar_obstacle;
    }
  }
  EXPECT_EQ(pillar_obstacle, static_cast<std::size_t>(pillar_pts));
}

TEST(GroundSegmenterTraversability, AccumulatesAndRenders)
{
  GroundSegmenterParams p = default_params();
  p.traversability_width = 50;
  p.traversability_height = 50;
  p.traversability_resolution = 0.1;
  p.traversability_origin_x = -2.5;
  p.traversability_origin_y = -2.5;
  p.traversability_min_count_known = 1;
  GroundSegmenter seg(p);

  // 200 ground points clustered around origin.
  Eigen::MatrixX3f g(200, 3);
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  for (int i = 0; i < 200; ++i) {
    g(i, 0) = u(rng);
    g(i, 1) = u(rng);
    g(i, 2) = 0.0f;
  }
  seg.accumulate_traversability(g);
  EXPECT_GT(seg.known_cell_count(), 50u);
  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability(grid));
  EXPECT_EQ(grid.size(), 50u * 50u);
  std::size_t free_cells = 0;
  for (auto v : grid) {
    if (v == 0) {
      ++free_cells;
    }
  }
  EXPECT_GT(free_cells, 30u);
}

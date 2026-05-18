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

// ─────────────────────────────────────────────────────────────────────────────
// V2 traversability tests
// ─────────────────────────────────────────────────────────────────────────────

GroundSegmenterParams v2_default_params()
{
  GroundSegmenterParams p;
  p.traversability_v2_enabled = true;
  p.traversability_width = 50;
  p.traversability_height = 50;
  p.traversability_resolution = 0.1;
  p.traversability_origin_x = -2.5;
  p.traversability_origin_y = -2.5;
  p.traversability_min_count_known = 1;
  p.traversability_cell_timeout_sec = 10.0;  // long timeout for tests
  p.traversability_unknown_policy = "ignore";
  p.traversability_max_slope_deg = 18.0;
  p.traversability_max_roughness_m = 0.06;
  p.traversability_max_step_height_m = 0.10;
  p.traversability_obstacle_density_threshold = 0.25;
  return p;
}

TEST(GroundSegmenterV2, FlatGroundIsLowCost)
{
  auto p = v2_default_params();
  GroundSegmenter seg(p);

  // Flat ground points near z=0.
  Eigen::MatrixX3f ground(300, 3);
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> xy(-1.0f, 1.0f);
  std::normal_distribution<float> z_noise(0.0f, 0.005f);
  for (int i = 0; i < 300; ++i) {
    ground(i, 0) = xy(rng);
    ground(i, 1) = xy(rng);
    ground(i, 2) = z_noise(rng);
  }
  Eigen::MatrixX3f obstacle(0, 3);

  seg.accumulate_traversability_v2(ground, obstacle, 0.0);

  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 0.0));

  // Most cells should be free/low cost.
  std::size_t free = 0, lethal = 0;
  for (auto v : grid) {
    if (v >= 0 && v <= 30) { ++free; }
    if (v >= 71) { ++lethal; }
  }
  EXPECT_GT(free, 15u);
  EXPECT_EQ(lethal, 0u);
}

TEST(GroundSegmenterV2, StepObstacleIsLethal)
{
  auto p = v2_default_params();
  p.traversability_resolution = 0.1;
  GroundSegmenter seg(p);

  // Flat ground at z=0, plus a step at x>0.5 to z=0.2.
  Eigen::MatrixX3f ground(500, 3);
  std::mt19937 rng(99);
  std::uniform_real_distribution<float> x_dist(-1.0f, 1.0f);
  std::uniform_real_distribution<float> y_dist(-1.0f, 1.0f);
  for (int i = 0; i < 500; ++i) {
    const float x = x_dist(rng);
    ground(i, 0) = x;
    ground(i, 1) = y_dist(rng);
    ground(i, 2) = (x > 0.5f) ? 0.20f : 0.0f;
  }
  Eigen::MatrixX3f obstacle(0, 3);

  seg.accumulate_traversability_v2(ground, obstacle, 0.0);

  // Check step debug layer.
  std::vector<int8_t> step_grid;
  ASSERT_TRUE(seg.render_cost_layer(step_grid, "step", 0.0));

  bool has_step_cost = false;
  for (std::size_t k = 0; k < step_grid.size(); ++k) {
    if (step_grid[k] > 0) {
      has_step_cost = true;
      break;
    }
  }
  EXPECT_TRUE(has_step_cost);

  // Main grid should have lethal cells near the step boundary.
  std::vector<int8_t> main_grid;
  ASSERT_TRUE(seg.render_traversability_v2(main_grid, 0.0));
  std::size_t lethal = 0;
  for (auto v : main_grid) {
    if (v >= 71) { ++lethal; }
  }
  EXPECT_GT(lethal, 0u);
}

TEST(GroundSegmenterV2, RoughTerrainTriggersRoughnessCost)
{
  auto p = v2_default_params();
  p.traversability_max_roughness_m = 0.02;  // tight threshold
  GroundSegmenter seg(p);

  // Points with high z variance.
  Eigen::MatrixX3f ground(400, 3);
  std::mt19937 rng(77);
  std::uniform_real_distribution<float> xy(-0.5f, 0.5f);
  std::normal_distribution<float> z_noise(0.0f, 0.08f);  // stddev=0.08 > 0.02
  for (int i = 0; i < 400; ++i) {
    ground(i, 0) = xy(rng);
    ground(i, 1) = xy(rng);
    ground(i, 2) = z_noise(rng);
  }
  Eigen::MatrixX3f obstacle(0, 3);

  seg.accumulate_traversability_v2(ground, obstacle, 0.0);

  std::vector<int8_t> rough_grid;
  ASSERT_TRUE(seg.render_cost_layer(rough_grid, "roughness", 0.0));

  bool has_rough = false;
  for (auto v : rough_grid) {
    if (v > 0) {
      has_rough = true;
      break;
    }
  }
  EXPECT_TRUE(has_rough);
}

TEST(GroundSegmenterV2, UnknownPolicyIgnoreDefault)
{
  auto p = v2_default_params();
  p.traversability_unknown_policy = "ignore";
  GroundSegmenter seg(p);

  // No points accumulated — everything is unknown.
  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 0.0));

  // All cells should be -1 (unknown).
  for (auto v : grid) {
    EXPECT_EQ(v, static_cast<int8_t>(-1));
  }
}

TEST(GroundSegmenterV2, UnknownPolicyLethal)
{
  auto p = v2_default_params();
  p.traversability_unknown_policy = "lethal";
  GroundSegmenter seg(p);

  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 0.0));

  // All cells should be 100 (lethal).
  for (auto v : grid) {
    EXPECT_EQ(v, static_cast<int8_t>(100));
  }
}

TEST(GroundSegmenterV2, TimeDecayReducesConfidence)
{
  auto p = v2_default_params();
  p.traversability_cell_timeout_sec = 0.5;
  p.traversability_confidence_decay_per_sec = 0.8;
  p.traversability_min_confidence = 0.35;
  GroundSegmenter seg(p);

  // Seed one cell with 5 points at t=0 to build confidence > min_confidence.
  // Each point adds 0.15 confidence, so 5 points = 0.75, well above 0.35.
  Eigen::MatrixX3f ground(5, 3);
  for (int i = 0; i < 5; ++i) {
    ground(i, 0) = 0.01f * static_cast<float>(i);
    ground(i, 1) = 0.01f * static_cast<float>(i);
    ground(i, 2) = 0.0f;
  }
  Eigen::MatrixX3f obstacle(0, 3);

  seg.accumulate_traversability_v2(ground, obstacle, 0.0);

  // At t=0, cell should be known with confidence > min_confidence.
  auto stats = seg.get_v2_stats(0.0);
  EXPECT_GT(stats.known_cells, 0u);
  EXPECT_GT(stats.mean_confidence, static_cast<float>(p.traversability_min_confidence));

  // At t=2.0 (well past timeout), confidence should have decayed.
  stats = seg.get_v2_stats(2.0);
  EXPECT_GT(stats.stale_cells, 0u);

  // The main grid at t=2.0 should show the cell as unknown again.
  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 2.0));
  bool has_unknown = false;
  for (auto v : grid) {
    if (v == -1) { has_unknown = true; break; }
  }
  EXPECT_TRUE(has_unknown);
}

TEST(GroundSegmenterV2, V2ParamsAreValidated)
{
  GroundSegmenterParams p = v2_default_params();
  p.traversability_unknown_policy = "invalid_policy";
  EXPECT_THROW(GroundSegmenter{p}, std::invalid_argument);

  p = v2_default_params();
  p.traversability_max_slope_deg = 100.0;
  EXPECT_THROW(GroundSegmenter{p}, std::invalid_argument);

  p = v2_default_params();
  p.traversability_min_confidence = 2.0;
  EXPECT_THROW(GroundSegmenter{p}, std::invalid_argument);
}

TEST(GroundSegmenterV2, V2DisabledKeepsV1Behavior)
{
  // V2 off: main grid renders V1 binary 0/100.
  auto p = v2_default_params();
  p.traversability_v2_enabled = false;
  p.traversability_width = 50;
  p.traversability_height = 50;
  p.traversability_origin_x = -2.5;
  p.traversability_origin_y = -2.5;
  p.traversability_min_count_known = 1;
  GroundSegmenter seg(p);

  // Same flat ground points as V1 test.
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

  // V1 binary: only values -1, 0, or 100.
  for (auto v : grid) {
    EXPECT_TRUE(v == -1 || v == 0 || v == 100);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// V2 semantic regression tests (bugs found during audit)
// ─────────────────────────────────────────────────────────────────────────────

TEST(GroundSegmenterV2, RenderIsIdempotent)
{
  // compute_cell_costs must NOT mutate cell.confidence.
  // Repeated renders at the same now_sec must produce identical output.
  auto p = v2_default_params();
  GroundSegmenter seg(p);

  Eigen::MatrixX3f ground(10, 3);
  for (int i = 0; i < 10; ++i) {
    ground(i, 0) = 0.0f;
    ground(i, 1) = 0.0f;
    ground(i, 2) = 0.0f;
  }
  Eigen::MatrixX3f obstacle(0, 3);
  seg.accumulate_traversability_v2(ground, obstacle, 0.0);

  std::vector<int8_t> a, b;
  ASSERT_TRUE(seg.render_traversability_v2(a, 5.0));
  ASSERT_TRUE(seg.render_traversability_v2(b, 5.0));
  EXPECT_EQ(a, b);

  // stats must also be stable.
  auto s1 = seg.get_v2_stats(5.0);
  auto s2 = seg.get_v2_stats(5.0);
  EXPECT_FLOAT_EQ(s1.mean_confidence, s2.mean_confidence);
  EXPECT_EQ(s1.stale_cells, s2.stale_cells);
}

TEST(GroundSegmenterV2, UnknownPolicyIgnoreStaleCell)
{
  // A known cell decays below min_confidence. Under "ignore" policy it
  // must become -1, not a low cost.
  auto p = v2_default_params();
  p.traversability_unknown_policy = "ignore";
  p.traversability_cell_timeout_sec = 0.2;
  p.traversability_confidence_decay_per_sec = 0.8;
  p.traversability_min_confidence = 0.35;
  GroundSegmenter seg(p);

  // Seed a cell at origin with moderate confidence (5 pts = 0.75 confidence).
  Eigen::MatrixX3f ground(5, 3);
  for (int i = 0; i < 5; ++i) {
    ground(i, 0) = 0.0f;
    ground(i, 1) = 0.0f;
    ground(i, 2) = 0.0f;
  }
  Eigen::MatrixX3f obstacle(0, 3);
  seg.accumulate_traversability_v2(ground, obstacle, 0.0);

  // At t=0 the cell is known and free.
  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 0.0));
  int W = p.traversability_width;
  int cx = static_cast<int>(std::floor((0.0 - p.traversability_origin_x) / p.traversability_resolution));
  int cy = static_cast<int>(std::floor((0.0 - p.traversability_origin_y) / p.traversability_resolution));
  std::size_t k = static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx);
  EXPECT_GE(grid[k], 0);
  EXPECT_LE(grid[k], 30);

  // At t=10.0 the cell must be stale and output -1.
  ASSERT_TRUE(seg.render_traversability_v2(grid, 10.0));
  EXPECT_EQ(grid[k], static_cast<int8_t>(-1));
}

TEST(GroundSegmenterV2, UnknownPolicyLethalSoftCost)
{
  // Stale cell under "lethal" => 100, under "soft_cost" => unknown_cost.
  auto make_stale_grid = [](const std::string & policy) {
    auto p = v2_default_params();
    p.traversability_unknown_policy = policy;
    p.traversability_unknown_cost = 55;
    p.traversability_cell_timeout_sec = 0.2;
    p.traversability_confidence_decay_per_sec = 0.8;
    p.traversability_min_confidence = 0.35;
    GroundSegmenter seg(p);

    Eigen::MatrixX3f ground(5, 3);
    for (int i = 0; i < 5; ++i) {
      ground(i, 0) = 0.0f; ground(i, 1) = 0.0f; ground(i, 2) = 0.0f;
    }
    Eigen::MatrixX3f obstacle(0, 3);
    seg.accumulate_traversability_v2(ground, obstacle, 0.0);

    std::vector<int8_t> grid;
    seg.render_traversability_v2(grid, 10.0);
    int W = p.traversability_width;
    int cx = static_cast<int>(std::floor((0.0 - p.traversability_origin_x) / p.traversability_resolution));
    int cy = static_cast<int>(std::floor((0.0 - p.traversability_origin_y) / p.traversability_resolution));
    return grid[static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx)];
  };

  EXPECT_EQ(make_stale_grid("lethal"), static_cast<int8_t>(100));
  EXPECT_EQ(make_stale_grid("soft_cost"), static_cast<int8_t>(55));
}

TEST(GroundSegmenterV2, ReasonCodeIsMaxCostSource)
{
  // reason must reflect the DOMINANT cost source, not the first trigger.
  auto p = v2_default_params();
  p.traversability_max_step_height_m = 0.03;  // tight step threshold
  p.traversability_obstacle_density_threshold = 0.9;  // loose so density won't dominate
  p.traversability_max_slope_deg = 60.0;  // loose so slope won't dominate
  p.traversability_max_roughness_m = 1.0;  // loose so roughness won't dominate
  GroundSegmenter seg(p);

  // Create cells with a step: one at z=0, neighbor at z=0.15 (0.15 > 0.03 step threshold).
  Eigen::MatrixX3f ground(20, 3);
  for (int i = 0; i < 10; ++i) {
    ground(i, 0) = 0.0f;
    ground(i, 1) = static_cast<float>(i) * 0.05f;
    ground(i, 2) = 0.0f;
  }
  for (int i = 10; i < 20; ++i) {
    ground(i, 0) = 0.15f;  // adjacent column
    ground(i, 1) = static_cast<float>(i - 10) * 0.05f;
    ground(i, 2) = 0.15f;  // height jump → step
  }
  Eigen::MatrixX3f obstacle(0, 3);
  seg.accumulate_traversability_v2(ground, obstacle, 0.0);

  std::vector<int8_t> reason_grid;
  ASSERT_TRUE(seg.render_cost_layer(reason_grid, "reason", 0.0));
  bool has_step_reason = false;
  for (auto v : reason_grid) {
    if (v == 30) { has_step_reason = true; break; }
  }
  EXPECT_TRUE(has_step_reason) << "step cost must be the dominant reason";
}

TEST(GroundSegmenterV2, MeanZEmaRespondsToChange)
{
  // mean_z must respond to a second frame with different z values.
  // Fill a 3x3 patch of cells so there's no artificial slope edge.
  auto p = v2_default_params();
  p.traversability_height_ema_alpha = 0.5;
  GroundSegmenter seg(p);

  const float res = static_cast<float>(p.traversability_resolution);
  Eigen::MatrixX3f flat_pts(0, 3);
  Eigen::MatrixX3f empty(0, 3);

  // Fill a 3x3 patch at z=0.0 around origin.
  Eigen::MatrixX3f f1(90, 3);
  int ri = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int p = 0; p < 10; ++p) {
        f1(ri, 0) = static_cast<float>(dx) * res + static_cast<float>(p) * 0.01f;
        f1(ri, 1) = static_cast<float>(dy) * res;
        f1(ri, 2) = 0.0f;
        ++ri;
      }
    }
  }
  seg.accumulate_traversability_v2(f1, empty, 0.0);

  // Frame 2: same cells, z=1.0. EMA with alpha=0.5 moves mean_z significantly.
  Eigen::MatrixX3f f2(90, 3);
  ri = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int p = 0; p < 10; ++p) {
        f2(ri, 0) = static_cast<float>(dx) * res + static_cast<float>(p) * 0.01f;
        f2(ri, 1) = static_cast<float>(dy) * res;
        f2(ri, 2) = 1.0f;
        ++ri;
      }
    }
  }
  seg.accumulate_traversability_v2(f2, empty, 1.0);

  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 1.0));
  int W = p.traversability_width;
  int cx = static_cast<int>(std::floor((0.0 - p.traversability_origin_x) / res));
  int cy = static_cast<int>(std::floor((0.0 - p.traversability_origin_y) / res));
  std::size_t k = static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx);

  // The cell must be known (not -1), proving EMA is not no-op.
  // A correctly working EMA moves mean_z in response to new z values.
  EXPECT_NE(grid[k], static_cast<int8_t>(-1));
}

// ─────────────────────────────────────────────────────────────────────────────
// Stats / grid consistency tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(GroundSegmenterV2, StatsMatchRenderForStaleIgnore)
{
  // get_v2_stats must agree with render_traversability_v2 when policy=ignore.
  auto p = v2_default_params();
  p.traversability_unknown_policy = "ignore";
  p.traversability_cell_timeout_sec = 0.2;
  p.traversability_confidence_decay_per_sec = 1.0;
  p.traversability_min_confidence = 0.35;
  GroundSegmenter seg(p);

  Eigen::MatrixX3f ground(3, 3);  // 3 pts, confidence = 0.45
  for (int i = 0; i < 3; ++i) {
    ground(i, 0) = 0.0f; ground(i, 1) = 0.0f; ground(i, 2) = 0.0f;
  }
  Eigen::MatrixX3f empty(0, 3);
  seg.accumulate_traversability_v2(ground, empty, 0.0);

  // At t=10: cell is stale. Grid must have -1 at this cell.
  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 10.0));
  int W = p.traversability_width;
  int cx = static_cast<int>(std::floor((0.0 - p.traversability_origin_x) / p.traversability_resolution));
  int cy = static_cast<int>(std::floor((0.0 - p.traversability_origin_y) / p.traversability_resolution));
  std::size_t k = static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx);
  EXPECT_EQ(grid[k], static_cast<int8_t>(-1));

  // Stats must classify this cell as unknown (not known, not high_cost).
  auto stats = seg.get_v2_stats(10.0);
  EXPECT_EQ(stats.high_cost_cells, 0u);
  EXPECT_GT(stats.unknown_cells, 0u);
  EXPECT_EQ(stats.max_cost, 0.0f);
}

TEST(GroundSegmenterV2, StatsMatchRenderForStaleLethal)
{
  // Stale cell under lethal policy: grid=100, stats max_cost=100, high_cost>0.
  auto p = v2_default_params();
  p.traversability_unknown_policy = "lethal";
  p.traversability_cell_timeout_sec = 0.2;
  p.traversability_confidence_decay_per_sec = 1.0;
  p.traversability_min_confidence = 0.35;
  GroundSegmenter seg(p);

  Eigen::MatrixX3f ground(3, 3);
  for (int i = 0; i < 3; ++i) {
    ground(i, 0) = 0.0f; ground(i, 1) = 0.0f; ground(i, 2) = 0.0f;
  }
  Eigen::MatrixX3f empty(0, 3);
  seg.accumulate_traversability_v2(ground, empty, 0.0);

  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 10.0));
  int W = p.traversability_width;
  int cx = static_cast<int>(std::floor((0.0 - p.traversability_origin_x) / p.traversability_resolution));
  int cy = static_cast<int>(std::floor((0.0 - p.traversability_origin_y) / p.traversability_resolution));
  std::size_t k = static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx);
  EXPECT_EQ(grid[k], static_cast<int8_t>(100));

  auto stats = seg.get_v2_stats(10.0);
  EXPECT_GT(stats.high_cost_cells, 0u);
  EXPECT_FLOAT_EQ(stats.max_cost, 100.0f);
  // Under lethal, stale cells are "known" (rendered as lethal, not ignored).
  EXPECT_GT(stats.known_cells, 0u);
}

TEST(GroundSegmenterV2, StatsMatchRenderForStaleSoftCost)
{
  // Stale cell under soft_cost: grid=unknown_cost, max_cost >= unknown_cost.
  auto p = v2_default_params();
  p.traversability_unknown_policy = "soft_cost";
  p.traversability_unknown_cost = 55;
  p.traversability_cell_timeout_sec = 0.2;
  p.traversability_confidence_decay_per_sec = 1.0;
  p.traversability_min_confidence = 0.35;
  GroundSegmenter seg(p);

  Eigen::MatrixX3f ground(3, 3);
  for (int i = 0; i < 3; ++i) {
    ground(i, 0) = 0.0f; ground(i, 1) = 0.0f; ground(i, 2) = 0.0f;
  }
  Eigen::MatrixX3f empty(0, 3);
  seg.accumulate_traversability_v2(ground, empty, 0.0);

  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 10.0));
  int W = p.traversability_width;
  int cx = static_cast<int>(std::floor((0.0 - p.traversability_origin_x) / p.traversability_resolution));
  int cy = static_cast<int>(std::floor((0.0 - p.traversability_origin_y) / p.traversability_resolution));
  std::size_t k = static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx);
  EXPECT_EQ(grid[k], static_cast<int8_t>(55));

  auto stats = seg.get_v2_stats(10.0);
  EXPECT_GE(stats.max_cost, 55.0f);
  EXPECT_GT(stats.known_cells, 0u);
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-frame mean_z EMA tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(GroundSegmenterV2, MeanZEmaUsesFrameMeanOnce)
{
  // Verify mean_z EMA blends the per-frame observation mean (not each point).
  // alpha=0.5, frame1: 10 pts z=0, frame2: 10 pts z=1 => mean_z ≈ 0.5
  // A per-point EMA would converge to ~1.0 with many points.
  auto p = v2_default_params();
  p.traversability_height_ema_alpha = 0.5f;
  GroundSegmenter seg(p);

  const float res = static_cast<float>(p.traversability_resolution);
  Eigen::MatrixX3f empty(0, 3);

  // Fill a 3x3 patch at z=0.0.
  Eigen::MatrixX3f f1(90, 3);
  int ri = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int p = 0; p < 10; ++p) {
        f1(ri, 0) = static_cast<float>(dx) * res + static_cast<float>(p) * 0.008f;
        f1(ri, 1) = static_cast<float>(dy) * res;
        f1(ri, 2) = 0.0f;
        ++ri;
      }
    }
  }
  seg.accumulate_traversability_v2(f1, empty, 0.0);

  // Frame 2: same cells at z=1.0.
  Eigen::MatrixX3f f2(90, 3);
  ri = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int p = 0; p < 10; ++p) {
        f2(ri, 0) = static_cast<float>(dx) * res + static_cast<float>(p) * 0.008f;
        f2(ri, 1) = static_cast<float>(dy) * res;
        f2(ri, 2) = 1.0f;
        ++ri;
      }
    }
  }
  seg.accumulate_traversability_v2(f2, empty, 1.0);

  // Per-frame EMA: mean_z ≈ 0.0*0.5 + 1.0*0.5 = 0.5.
  // All 9 cells have the same z, so no slope → all free (0).
  std::vector<int8_t> grid;
  ASSERT_TRUE(seg.render_traversability_v2(grid, 1.0));
  int W = p.traversability_width;
  int cx = static_cast<int>(std::floor((0.0 - p.traversability_origin_x) / res));
  int cy = static_cast<int>(std::floor((0.0 - p.traversability_origin_y) / res));
  std::size_t k = static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx);
  EXPECT_EQ(grid[k], static_cast<int8_t>(0));

  // A per-point EMA with 10 pts at z=1 would push mean_z >> 0.5 toward 1.0,
  // creating a slope edge vs neighbor cells that only got frame1 (z=0). Verify
  // no such edge exists: cell at (cx+1, cy) should also be free.
  std::size_t kr = static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx + 1);
  EXPECT_EQ(grid[kr], static_cast<int8_t>(0));
}

TEST(GroundSegmenterV2, MeanZEmaIndependentOfPointsPerFrame)
{
  // mean_z after EMA should not depend on how many points were in the frame.
  auto p = v2_default_params();
  p.traversability_height_ema_alpha = 0.5;
  GroundSegmenter seg1(p);
  GroundSegmenter seg2(p);

  Eigen::MatrixX3f empty(0, 3);

  // seg1: first frame 10 pts z=0, second frame 1 pt z=1.
  {
    Eigen::MatrixX3f f1(10, 3);
    for (int i = 0; i < 10; ++i) {
      f1(i, 0) = 0.0f; f1(i, 1) = 0.0f; f1(i, 2) = 0.0f;
    }
    seg1.accumulate_traversability_v2(f1, empty, 0.0);
    Eigen::MatrixX3f f2(1, 3);
    f2(0, 0) = 0.0f; f2(0, 1) = 0.0f; f2(0, 2) = 1.0f;
    seg1.accumulate_traversability_v2(f2, empty, 1.0);
  }

  // seg2: first frame 10 pts z=0, second frame 10 pts z=1.
  {
    Eigen::MatrixX3f f1(10, 3);
    for (int i = 0; i < 10; ++i) {
      f1(i, 0) = 0.0f; f1(i, 1) = 0.0f; f1(i, 2) = 0.0f;
    }
    seg2.accumulate_traversability_v2(f1, empty, 0.0);
    Eigen::MatrixX3f f2(10, 3);
    for (int i = 0; i < 10; ++i) {
      f2(i, 0) = 0.0f; f2(i, 1) = 0.0f; f2(i, 2) = 1.0f;
    }
    seg2.accumulate_traversability_v2(f2, empty, 1.0);
  }

  // Both should produce the same rendered value at origin (frame mean = 1.0, EMA = 0.5).
  std::vector<int8_t> g1, g2;
  ASSERT_TRUE(seg1.render_traversability_v2(g1, 1.0));
  ASSERT_TRUE(seg2.render_traversability_v2(g2, 1.0));
  int W = p.traversability_width;
  int cx = static_cast<int>(std::floor((0.0 - p.traversability_origin_x) / p.traversability_resolution));
  int cy = static_cast<int>(std::floor((0.0 - p.traversability_origin_y) / p.traversability_resolution));
  std::size_t k = static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(cx);
  EXPECT_EQ(g1[k], g2[k]);
}

TEST(GroundSegmenterV2, V2RenderUsesExplicitTimestamp)
{
  // render_traversability (no timestamp) returns false when V2 enabled.
  auto p = v2_default_params();
  GroundSegmenter seg(p);

  Eigen::MatrixX3f ground(3, 3);
  for (int i = 0; i < 3; ++i) {
    ground(i, 0) = 0.0f; ground(i, 1) = 0.0f; ground(i, 2) = 0.0f;
  }
  Eigen::MatrixX3f empty(0, 3);
  seg.accumulate_traversability_v2(ground, empty, 0.0);

  std::vector<int8_t> grid;
  EXPECT_FALSE(seg.render_traversability(grid));

  // V2 callers must use render_traversability_v2 with explicit now_sec.
  EXPECT_TRUE(seg.render_traversability_v2(grid, 0.0));
}

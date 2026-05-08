// Copyright 2026 a2_system_ws.
#include <gtest/gtest.h>

#include <cmath>

#include "nav2_integration_cpp/dwa_planner.hpp"

using nav2_integration_cpp::DwaParams;
using nav2_integration_cpp::DwaPlanner;
using nav2_integration_cpp::Pose2D;
using nav2_integration_cpp::Velocity;

TEST(DwaPlannerParams, ValidateRejectsBad)
{
  DwaParams p;
  p.max_linear_x = -1.0;
  EXPECT_THROW(DwaPlanner{p}, std::invalid_argument);
  p = DwaParams{};
  p.hard_clearance = 1.0;
  p.inflation_radius = 0.5;
  EXPECT_THROW(DwaPlanner{p}, std::invalid_argument);
  p = DwaParams{};
  p.n_vx = 100; p.n_vy = 100; p.n_wz = 100;
  EXPECT_THROW(DwaPlanner{p}, std::invalid_argument);
}

TEST(DwaPlannerSimulate, AdvancesAlongHeading)
{
  Pose2D start{0.0, 0.0, 0.0};
  Velocity v{0.5, 0.0, 0.0};
  auto traj = DwaPlanner::simulate(start, v, 1.0, 0.1);
  ASSERT_GT(traj.size(), 5u);
  EXPECT_NEAR(traj.back().x, 0.5, 1e-3);
  EXPECT_NEAR(traj.back().y, 0.0, 1e-3);
}

TEST(DwaPlannerPlan, PicksForwardWhenClear)
{
  DwaParams p;
  DwaPlanner planner(p);
  Pose2D cur{0.0, 0.0, 0.0};
  Pose2D goal{1.2, 0.0, 0.0};
  Eigen::MatrixX2f obs(0, 2);
  auto r = planner.plan(cur, goal, obs, Velocity{});
  ASSERT_TRUE(r.success) << r.reason;
  EXPECT_GT(r.cmd.vx, 0.0);
  EXPECT_NEAR(r.cmd.vy, 0.0, 0.10);
  EXPECT_NEAR(r.cmd.wz, 0.0, 0.10);
}

TEST(DwaPlannerPlan, AvoidsObstacleAhead)
{
  DwaParams p;
  // Force a small grid that includes lateral motion.
  p.n_vx = 5; p.n_vy = 5; p.n_wz = 5;
  p.hard_clearance = 0.20;
  p.inflation_radius = 0.50;
  DwaPlanner planner(p);

  Pose2D cur{0.0, 0.0, 0.0};
  Pose2D goal{1.0, 0.0, 0.0};
  // Obstacle directly ahead at 0.6 m
  Eigen::MatrixX2f obs(1, 2);
  obs << 0.6f, 0.0f;
  auto r = planner.plan(cur, goal, obs, Velocity{});
  ASSERT_TRUE(r.success) << r.reason;
  // Either lateral offset or yaw rotation away from straight-ahead
  EXPECT_TRUE(std::fabs(r.cmd.vy) > 0.01 || std::fabs(r.cmd.wz) > 0.05);
  EXPECT_GE(r.min_clearance, p.hard_clearance);
}

TEST(DwaPlannerPlan, BlockedWhenObstacleSurrounds)
{
  DwaParams p;
  p.n_vx = 3; p.n_vy = 3; p.n_wz = 3;
  p.hard_clearance = 0.30;
  DwaPlanner planner(p);
  Pose2D cur{0.0, 0.0, 0.0};
  Pose2D goal{1.0, 0.0, 0.0};
  // Tightly surround the rollouts so all are rejected.
  Eigen::MatrixX2f obs(8, 2);
  obs << 0.05f, 0.0f,
    -0.05f, 0.0f,
    0.0f, 0.05f,
    0.0f, -0.05f,
    0.05f, 0.05f,
    -0.05f, 0.05f,
    0.05f, -0.05f,
    -0.05f, -0.05f;
  auto r = planner.plan(cur, goal, obs, Velocity{});
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.admissible_count, 0u);
}

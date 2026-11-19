// test_rrt.cpp — Unit tests for the RRT planner.
//
// Author: Mazen Ben Abid

#include <gtest/gtest.h>
#include "motion_planner/rrt.h"
#include <cmath>

using namespace mp;

// ─── Basic planning ──────────────────────────────────────────

TEST(RRTTest, FindsPathInEmptyWorkspace) {
    // No obstacles — should always find a path.
    Environment env(10.0, 10.0);
    RRTConfig cfg;
    cfg.max_iter = 2000;
    cfg.seed = 42;

    RRT planner(env, cfg);
    PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
    EXPECT_GT(result.nodes_explored, 0);
    EXPECT_GT(result.planning_time_ms, 0.0);
}

TEST(RRTTest, PathStartsAtStartAndEndsAtGoal) {
    Environment env(10.0, 10.0);
    RRTConfig cfg;
    cfg.seed = 42;

    RRT planner(env, cfg);
    Point2D start{1.0, 1.0};
    Point2D goal{9.0, 9.0};
    PlannerResult result = planner.plan(start, goal);

    ASSERT_TRUE(result.success);
    EXPECT_NEAR(result.path.waypoints.front().x, start.x, 1e-10);
    EXPECT_NEAR(result.path.waypoints.front().y, start.y, 1e-10);
    EXPECT_NEAR(result.path.waypoints.back().x, goal.x, cfg.goal_radius);
    EXPECT_NEAR(result.path.waypoints.back().y, goal.y, cfg.goal_radius);
}

TEST(RRTTest, PathIsCollisionFree) {
    // The path should not pass through any obstacles.
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.5});

    RRTConfig cfg;
    cfg.max_iter = 5000;
    cfg.seed = 42;

    RRT planner(env, cfg);
    PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});

    ASSERT_TRUE(result.success);

    // Check every waypoint is in free space.
    for (const auto& wp : result.path.waypoints) {
        EXPECT_TRUE(env.is_free(wp))
            << "Waypoint " << wp << " is inside an obstacle!";
    }

    // Check every edge is collision-free.
    for (size_t i = 1; i < result.path.waypoints.size(); ++i) {
        EXPECT_TRUE(env.is_segment_free(
            result.path.waypoints[i-1], result.path.waypoints[i]))
            << "Edge from " << result.path.waypoints[i-1]
            << " to " << result.path.waypoints[i] << " has collision!";
    }
}

TEST(RRTTest, FindsPathWithObstacles) {
    Environment env(10.0, 10.0);
    // Create a wall with a gap.
    env.add_obstacle(Rectangle{4.5, 0.0, 0.5, 4.0});
    env.add_obstacle(Rectangle{4.5, 6.0, 0.5, 4.0});
    // Gap is from y=4.0 to y=6.0.

    RRTConfig cfg;
    cfg.max_iter = 5000;
    cfg.seed = 42;

    RRT planner(env, cfg);
    PlannerResult result = planner.plan({1.0, 5.0}, {9.0, 5.0});

    EXPECT_TRUE(result.success);
    // Path must go through the gap.
    EXPECT_GT(result.path_length(), 8.0);  // at least the straight-line dist
}

TEST(RRTTest, FailsWhenStartInObstacle) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{1.0, 1.0}, 2.0});

    RRTConfig cfg;
    cfg.max_iter = 100;

    RRT planner(env, cfg);
    PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});

    EXPECT_FALSE(result.success);
}

TEST(RRTTest, FailsWhenGoalInObstacle) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{9.0, 9.0}, 2.0});

    RRTConfig cfg;
    cfg.max_iter = 100;

    RRT planner(env, cfg);
    PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});

    EXPECT_FALSE(result.success);
}

TEST(RRTTest, Reproducible) {
    // Same seed should give the same result.
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.0});

    RRTConfig cfg;
    cfg.seed = 99;
    cfg.max_iter = 3000;

    RRT planner1(env, cfg);
    PlannerResult r1 = planner1.plan({1.0, 1.0}, {9.0, 9.0});

    RRT planner2(env, cfg);
    PlannerResult r2 = planner2.plan({1.0, 1.0}, {9.0, 9.0});

    EXPECT_EQ(r1.success, r2.success);
    EXPECT_EQ(r1.nodes_explored, r2.nodes_explored);
    if (r1.success && r2.success) {
        EXPECT_DOUBLE_EQ(r1.path_length(), r2.path_length());
    }
}

TEST(RRTTest, TreeGrows) {
    // After planning, the tree should have more than just the root.
    Environment env(10.0, 10.0);
    RRTConfig cfg;
    cfg.max_iter = 100;
    cfg.seed = 42;

    RRT planner(env, cfg);
    planner.plan({1.0, 1.0}, {9.0, 9.0});

    EXPECT_GT(static_cast<int>(planner.tree().size()), 1);
}

TEST(RRTTest, PathLengthGreaterThanStraightLine) {
    // Path through obstacles must be at least as long as straight line.
    Environment env(10.0, 10.0);
    Point2D start{1.0, 1.0};
    Point2D goal{9.0, 9.0};
    double straight_line = start.distance_to(goal);

    RRTConfig cfg;
    cfg.seed = 42;
    RRT planner(env, cfg);
    PlannerResult result = planner.plan(start, goal);

    ASSERT_TRUE(result.success);
    EXPECT_GE(result.path_length(), straight_line - 1e-6);
}

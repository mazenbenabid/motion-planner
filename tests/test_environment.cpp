// test_environment.cpp — Unit tests for the Environment class.
//
// Tests collision checking, free-space queries, and sampling.
//
// Author: Mazen Ben Abid

#include <gtest/gtest.h>
#include "motion_planner/environment.h"
#include <cmath>

using namespace mp;

// ─── Construction ────────────────────────────────────────────

TEST(EnvironmentTest, ValidConstruction) {
    Environment env(10.0, 8.0);
    EXPECT_DOUBLE_EQ(env.width(), 10.0);
    EXPECT_DOUBLE_EQ(env.height(), 8.0);
    EXPECT_TRUE(env.obstacles().empty());
}

TEST(EnvironmentTest, InvalidDimensions) {
    EXPECT_THROW(Environment(0.0, 5.0), std::invalid_argument);
    EXPECT_THROW(Environment(5.0, -1.0), std::invalid_argument);
}

// ─── Point-in-free-space ─────────────────────────────────────

TEST(EnvironmentTest, EmptyWorkspaceAllFree) {
    Environment env(10.0, 10.0);
    EXPECT_TRUE(env.is_free({5.0, 5.0}));
    EXPECT_TRUE(env.is_free({0.0, 0.0}));   // corner
    EXPECT_TRUE(env.is_free({10.0, 10.0}));  // opposite corner
}

TEST(EnvironmentTest, OutsideWorkspace) {
    Environment env(10.0, 10.0);
    EXPECT_FALSE(env.is_free({-1.0, 5.0}));
    EXPECT_FALSE(env.is_free({5.0, -1.0}));
    EXPECT_FALSE(env.is_free({11.0, 5.0}));
    EXPECT_FALSE(env.is_free({5.0, 11.0}));
}

TEST(EnvironmentTest, InsideCircleObstacle) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.0});

    EXPECT_FALSE(env.is_free({5.0, 5.0}));  // center
    EXPECT_FALSE(env.is_free({5.5, 5.0}));  // inside
    EXPECT_TRUE(env.is_free({7.0, 5.0}));   // outside
}

TEST(EnvironmentTest, CircleBoundary) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.0});

    // Point exactly on the boundary is considered inside.
    EXPECT_FALSE(env.is_free({6.0, 5.0}));
    // Point just outside.
    EXPECT_TRUE(env.is_free({6.01, 5.0}));
}

TEST(EnvironmentTest, InsideRectObstacle) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Rectangle{2.0, 3.0, 4.0, 2.0});

    EXPECT_FALSE(env.is_free({4.0, 4.0}));  // center
    EXPECT_FALSE(env.is_free({2.0, 3.0}));  // corner (inside)
    EXPECT_TRUE(env.is_free({1.0, 4.0}));   // outside
    EXPECT_TRUE(env.is_free({7.0, 4.0}));   // outside
}

TEST(EnvironmentTest, MultipleObstacles) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{2.0, 2.0}, 0.5});
    env.add_obstacle(Rectangle{7.0, 7.0, 2.0, 2.0});

    EXPECT_FALSE(env.is_free({2.0, 2.0}));  // in circle
    EXPECT_FALSE(env.is_free({8.0, 8.0}));  // in rectangle
    EXPECT_TRUE(env.is_free({5.0, 5.0}));   // between obstacles
}

TEST(EnvironmentTest, ClearObstacles) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.0});
    EXPECT_FALSE(env.is_free({5.0, 5.0}));

    env.clear_obstacles();
    EXPECT_TRUE(env.is_free({5.0, 5.0}));
    EXPECT_TRUE(env.obstacles().empty());
}

// ─── Segment collision ───────────────────────────────────────

TEST(EnvironmentTest, SegmentFreeEmptyWorkspace) {
    Environment env(10.0, 10.0);
    EXPECT_TRUE(env.is_segment_free({0.0, 0.0}, {10.0, 10.0}));
}

TEST(EnvironmentTest, SegmentBlockedByCircle) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.0});

    // Segment passes through the circle.
    EXPECT_FALSE(env.is_segment_free({0.0, 5.0}, {10.0, 5.0}));

    // Segment goes around the circle.
    EXPECT_TRUE(env.is_segment_free({0.0, 0.0}, {10.0, 0.0}));
}

TEST(EnvironmentTest, SegmentBlockedByRect) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Rectangle{4.0, 0.0, 1.0, 10.0});  // vertical wall

    // Segment crosses the wall.
    EXPECT_FALSE(env.is_segment_free({0.0, 5.0}, {10.0, 5.0}));

    // Segment stays on one side.
    EXPECT_TRUE(env.is_segment_free({0.0, 5.0}, {3.0, 5.0}));
}

TEST(EnvironmentTest, DegenerateSegment) {
    Environment env(10.0, 10.0);
    // A segment from a point to itself should check that point.
    EXPECT_TRUE(env.is_segment_free({5.0, 5.0}, {5.0, 5.0}));

    env.add_obstacle(Circle{{5.0, 5.0}, 1.0});
    EXPECT_FALSE(env.is_segment_free({5.0, 5.0}, {5.0, 5.0}));
}

// ─── Random sampling ─────────────────────────────────────────

TEST(EnvironmentTest, SampleFreeIsActuallyFree) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 2.0});

    std::mt19937 rng(123);
    for (int i = 0; i < 100; ++i) {
        Point2D p = env.sample_free(rng);
        EXPECT_TRUE(env.is_free(p))
            << "Sample " << p << " is not in free space!";
    }
}

TEST(EnvironmentTest, SamplesInBounds) {
    Environment env(10.0, 8.0);
    std::mt19937 rng(456);
    for (int i = 0; i < 100; ++i) {
        Point2D p = env.sample_free(rng);
        EXPECT_GE(p.x, 0.0);
        EXPECT_LE(p.x, 10.0);
        EXPECT_GE(p.y, 0.0);
        EXPECT_LE(p.y, 8.0);
    }
}

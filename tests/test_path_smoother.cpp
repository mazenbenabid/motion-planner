// test_path_smoother.cpp — Unit tests for the PathSmoother.
//
// Author: Mazen Ben Abid

#include <gtest/gtest.h>
#include "motion_planner/path_smoother.h"

using namespace mp;

// ─── Basic smoothing tests ──────────────────────────────────

TEST(PathSmootherTest, SmoothEmptyPath) {
    Environment env(10.0, 10.0);
    PathSmoother smoother;
    Path empty;
    Path result = smoother.smooth(empty, env);
    EXPECT_TRUE(result.empty());
}

TEST(PathSmootherTest, SmoothSinglePoint) {
    Environment env(10.0, 10.0);
    PathSmoother smoother;
    Path single;
    single.waypoints = {{5.0, 5.0}};
    Path result = smoother.smooth(single, env);
    EXPECT_EQ(result.waypoints.size(), 1u);
}

TEST(PathSmootherTest, SmoothTwoPoints) {
    Environment env(10.0, 10.0);
    PathSmoother smoother;
    Path two;
    two.waypoints = {{1.0, 1.0}, {9.0, 9.0}};
    Path result = smoother.smooth(two, env);
    EXPECT_EQ(result.waypoints.size(), 2u);
}

TEST(PathSmootherTest, SmoothReducesLength) {
    // A zig-zag path in an empty workspace should become much shorter.
    Environment env(10.0, 10.0);
    Path zigzag;
    zigzag.waypoints = {
        {1.0, 1.0}, {3.0, 5.0}, {2.0, 3.0},
        {5.0, 7.0}, {4.0, 4.0}, {7.0, 6.0},
        {6.0, 2.0}, {9.0, 9.0}
    };

    double original_length = zigzag.length();

    PathSmoother smoother(500);  // generous budget
    Path smoothed = smoother.smooth(zigzag, env);

    EXPECT_LT(smoothed.length(), original_length)
        << "Smoothed path should be shorter than the zig-zag.";
    EXPECT_LE(smoothed.waypoints.size(), zigzag.waypoints.size());
}

TEST(PathSmootherTest, PreservesStartAndGoal) {
    Environment env(10.0, 10.0);
    Path path;
    path.waypoints = {{1.0, 1.0}, {5.0, 8.0}, {3.0, 3.0}, {9.0, 9.0}};

    PathSmoother smoother(100);
    Path smoothed = smoother.smooth(path, env);

    ASSERT_GE(smoothed.waypoints.size(), 2u);
    EXPECT_NEAR(smoothed.waypoints.front().x, 1.0, 1e-10);
    EXPECT_NEAR(smoothed.waypoints.front().y, 1.0, 1e-10);
    EXPECT_NEAR(smoothed.waypoints.back().x, 9.0, 1e-10);
    EXPECT_NEAR(smoothed.waypoints.back().y, 9.0, 1e-10);
}

TEST(PathSmootherTest, RespectsObstacles) {
    // The smoother should not create shortcuts through obstacles.
    Environment env(10.0, 10.0);
    // Big wall blocking the direct path.
    env.add_obstacle(Rectangle{4.5, 0.0, 0.5, 8.0});

    // A path that goes around the wall.
    Path path;
    path.waypoints = {
        {2.0, 5.0}, {2.0, 9.0}, {5.0, 9.0}, {8.0, 9.0}, {8.0, 5.0}
    };

    PathSmoother smoother(200);
    Path smoothed = smoother.smooth(path, env);

    // The smoothed path should still avoid the wall.
    for (size_t i = 1; i < smoothed.waypoints.size(); ++i) {
        EXPECT_TRUE(env.is_segment_free(
            smoothed.waypoints[i-1], smoothed.waypoints[i]))
            << "Smoothed edge " << (i-1) << " → " << i
            << " passes through an obstacle!";
    }
}

TEST(PathSmootherTest, StraightPathUnchanged) {
    // A straight path in empty space should stay straight.
    Environment env(10.0, 10.0);
    Path straight;
    straight.waypoints = {{1.0, 1.0}, {5.0, 5.0}, {9.0, 9.0}};

    PathSmoother smoother(100);
    Path smoothed = smoother.smooth(straight, env);

    // Should simplify to 2 points (start and end).
    EXPECT_LE(smoothed.waypoints.size(), 3u);
    EXPECT_NEAR(smoothed.length(), straight.waypoints.front().distance_to(
        straight.waypoints.back()), 0.1);
}

TEST(PathSmootherTest, Reproducible) {
    Environment env(10.0, 10.0);
    Path path;
    path.waypoints = {
        {1.0, 1.0}, {3.0, 7.0}, {5.0, 2.0}, {7.0, 8.0}, {9.0, 9.0}
    };

    PathSmoother smoother1(100, 42);
    Path s1 = smoother1.smooth(path, env);

    PathSmoother smoother2(100, 42);
    Path s2 = smoother2.smooth(path, env);

    EXPECT_DOUBLE_EQ(s1.length(), s2.length());
    EXPECT_EQ(s1.waypoints.size(), s2.waypoints.size());
}

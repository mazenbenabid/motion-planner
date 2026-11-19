// test_types.cpp — Unit tests for Point2D, Path, and other basic types.
//
// Author: Mazen Ben Abid

#include <gtest/gtest.h>
#include "motion_planner/types.h"
#include <cmath>

using namespace mp;

// ─── Point2D tests ───────────────────────────────────────────

TEST(Point2DTest, DistanceToSelf) {
    Point2D p{3.0, 4.0};
    EXPECT_DOUBLE_EQ(p.distance_to(p), 0.0);
}

TEST(Point2DTest, DistanceKnown) {
    // Classic 3-4-5 triangle.
    Point2D a{0.0, 0.0};
    Point2D b{3.0, 4.0};
    EXPECT_DOUBLE_EQ(a.distance_to(b), 5.0);
}

TEST(Point2DTest, DistanceSymmetric) {
    Point2D a{1.0, 2.0};
    Point2D b{4.0, 6.0};
    EXPECT_DOUBLE_EQ(a.distance_to(b), b.distance_to(a));
}

TEST(Point2DTest, StepToward) {
    Point2D a{0.0, 0.0};
    Point2D b{10.0, 0.0};
    Point2D stepped = a.step_toward(b, 3.0);
    EXPECT_NEAR(stepped.x, 3.0, 1e-10);
    EXPECT_NEAR(stepped.y, 0.0, 1e-10);
}

TEST(Point2DTest, StepTowardDiagonal) {
    Point2D a{0.0, 0.0};
    Point2D b{3.0, 4.0};  // distance = 5
    Point2D stepped = a.step_toward(b, 2.5);
    // Should be at half the distance.
    EXPECT_NEAR(stepped.x, 1.5, 1e-10);
    EXPECT_NEAR(stepped.y, 2.0, 1e-10);
}

TEST(Point2DTest, StepTowardSamePoint) {
    Point2D a{5.0, 5.0};
    Point2D stepped = a.step_toward(a, 1.0);
    EXPECT_NEAR(stepped.x, 5.0, 1e-10);
    EXPECT_NEAR(stepped.y, 5.0, 1e-10);
}

TEST(Point2DTest, Arithmetic) {
    Point2D a{1.0, 2.0};
    Point2D b{3.0, 4.0};

    Point2D sum = a + b;
    EXPECT_DOUBLE_EQ(sum.x, 4.0);
    EXPECT_DOUBLE_EQ(sum.y, 6.0);

    Point2D diff = b - a;
    EXPECT_DOUBLE_EQ(diff.x, 2.0);
    EXPECT_DOUBLE_EQ(diff.y, 2.0);

    Point2D scaled = a * 3.0;
    EXPECT_DOUBLE_EQ(scaled.x, 3.0);
    EXPECT_DOUBLE_EQ(scaled.y, 6.0);
}

TEST(Point2DTest, Equality) {
    Point2D a{1.0, 2.0};
    Point2D b{1.0, 2.0};
    Point2D c{1.0, 2.1};
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}

// ─── Path tests ──────────────────────────────────────────────

TEST(PathTest, EmptyPath) {
    Path p;
    EXPECT_TRUE(p.empty());
    EXPECT_DOUBLE_EQ(p.length(), 0.0);
}

TEST(PathTest, SinglePoint) {
    Path p;
    p.waypoints = {{1.0, 1.0}};
    EXPECT_FALSE(p.empty());
    EXPECT_DOUBLE_EQ(p.length(), 0.0);
}

TEST(PathTest, StraightLine) {
    Path p;
    p.waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    // 3 + 4 = 7
    EXPECT_DOUBLE_EQ(p.length(), 7.0);
}

// ─── Rectangle tests ────────────────────────────────────────

TEST(RectangleTest, Extents) {
    Rectangle r{1.0, 2.0, 3.0, 4.0};
    EXPECT_DOUBLE_EQ(r.x_max(), 4.0);
    EXPECT_DOUBLE_EQ(r.y_max(), 6.0);
}

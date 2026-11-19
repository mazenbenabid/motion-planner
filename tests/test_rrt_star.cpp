// test_rrt_star.cpp — Unit tests for the RRT* planner.
//
// RRT* should pass all the same correctness tests as RRT, plus
// it should tend to produce shorter paths (thanks to rewiring).
//
// Author: Mazen Ben Abid

#include <gtest/gtest.h>
#include "motion_planner/rrt.h"
#include "motion_planner/rrt_star.h"
#include <numeric>

using namespace mp;

// ─── Basic correctness ──────────────────────────────────────

TEST(RRTStarTest, FindsPathInEmptyWorkspace) {
    Environment env(10.0, 10.0);
    RRTConfig cfg;
    cfg.max_iter = 2000;
    cfg.seed = 42;

    RRTStar planner(env, cfg);
    PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.path.empty());
}

TEST(RRTStarTest, PathIsCollisionFree) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.5});
    env.add_obstacle(Rectangle{2.0, 7.0, 3.0, 0.5});

    RRTConfig cfg;
    cfg.max_iter = 5000;
    cfg.seed = 42;

    RRTStar planner(env, cfg);
    PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});

    ASSERT_TRUE(result.success);

    for (const auto& wp : result.path.waypoints) {
        EXPECT_TRUE(env.is_free(wp))
            << "Waypoint " << wp << " is inside an obstacle!";
    }

    for (size_t i = 1; i < result.path.waypoints.size(); ++i) {
        EXPECT_TRUE(env.is_segment_free(
            result.path.waypoints[i-1], result.path.waypoints[i]));
    }
}

TEST(RRTStarTest, PathStartsAndEndsCorrectly) {
    Environment env(10.0, 10.0);
    RRTConfig cfg;
    cfg.seed = 42;

    RRTStar planner(env, cfg);
    Point2D start{2.0, 2.0};
    Point2D goal{8.0, 8.0};
    PlannerResult result = planner.plan(start, goal);

    ASSERT_TRUE(result.success);
    EXPECT_NEAR(result.path.waypoints.front().x, start.x, 1e-10);
    EXPECT_NEAR(result.path.waypoints.front().y, start.y, 1e-10);
    EXPECT_NEAR(result.path.waypoints.back().x, goal.x, cfg.goal_radius);
    EXPECT_NEAR(result.path.waypoints.back().y, goal.y, cfg.goal_radius);
}

// ─── Optimality comparison ──────────────────────────────────

TEST(RRTStarTest, ProducesShorterPathsThanRRT) {
    // Run both planners multiple times and compare average path length.
    // RRT* should produce shorter paths on average.
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.5});

    const int num_trials = 10;
    std::vector<double> rrt_lengths, rrt_star_lengths;

    for (int trial = 0; trial < num_trials; ++trial) {
        RRTConfig cfg;
        cfg.max_iter = 3000;
        cfg.seed = trial * 7 + 1;

        {
            RRT planner(env, cfg);
            PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});
            if (result.success) {
                rrt_lengths.push_back(result.path_length());
            }
        }

        {
            RRTStar planner(env, cfg);
            PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});
            if (result.success) {
                rrt_star_lengths.push_back(result.path_length());
            }
        }
    }

    // Both should have found paths in most trials.
    ASSERT_GE(rrt_lengths.size(), 5u);
    ASSERT_GE(rrt_star_lengths.size(), 5u);

    double rrt_avg = std::accumulate(
        rrt_lengths.begin(), rrt_lengths.end(), 0.0) / rrt_lengths.size();
    double rrt_star_avg = std::accumulate(
        rrt_star_lengths.begin(), rrt_star_lengths.end(), 0.0)
        / rrt_star_lengths.size();

    // RRT* should be shorter on average (allow some tolerance).
    // This is a statistical test, not guaranteed per-run.
    EXPECT_LT(rrt_star_avg, rrt_avg * 1.05)
        << "RRT* avg=" << rrt_star_avg << " RRT avg=" << rrt_avg
        << " — RRT* should generally produce shorter paths.";
}

TEST(RRTStarTest, FailsGracefully) {
    // Start inside obstacle.
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{1.0, 1.0}, 2.0});

    RRTConfig cfg;
    cfg.max_iter = 100;

    RRTStar planner(env, cfg);
    PlannerResult result = planner.plan({1.0, 1.0}, {9.0, 9.0});
    EXPECT_FALSE(result.success);
}

TEST(RRTStarTest, Reproducible) {
    Environment env(10.0, 10.0);
    env.add_obstacle(Circle{{5.0, 5.0}, 1.0});

    RRTConfig cfg;
    cfg.seed = 77;
    cfg.max_iter = 2000;

    RRTStar p1(env, cfg);
    PlannerResult r1 = p1.plan({1.0, 1.0}, {9.0, 9.0});

    RRTStar p2(env, cfg);
    PlannerResult r2 = p2.plan({1.0, 1.0}, {9.0, 9.0});

    EXPECT_EQ(r1.success, r2.success);
    EXPECT_EQ(r1.nodes_explored, r2.nodes_explored);
}

TEST(RRTStarTest, NavigatesThroughNarrowPassage) {
    Environment env(10.0, 10.0);
    // Two walls with a narrow gap.
    env.add_obstacle(Rectangle{4.5, 0.0, 0.5, 4.5});
    env.add_obstacle(Rectangle{4.5, 5.5, 0.5, 4.5});

    RRTConfig cfg;
    cfg.max_iter = 8000;
    cfg.step_size = 0.2;   // smaller steps for narrow passage
    cfg.goal_radius = 0.3;
    cfg.goal_bias = 0.08;
    cfg.seed = 42;

    RRTStar planner(env, cfg);
    PlannerResult result = planner.plan({2.0, 5.0}, {8.0, 5.0});

    EXPECT_TRUE(result.success)
        << "RRT* should find a path through the narrow gap";
}

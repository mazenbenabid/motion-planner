// main.cpp — Demo program for the motion planning library.
//
// Creates a 2D environment with obstacles, runs both RRT and RRT*,
// exports the tree and path data to CSV files for visualization,
// and runs a benchmark comparing both planners.
//
// Usage:
//   ./motion_planner              # run everything
//   ./motion_planner --benchmark  # benchmark only (no tree export)
//
// Author: Mazen Ben Abid
// License: MIT

#include "motion_planner/types.h"
#include "motion_planner/environment.h"
#include "motion_planner/rrt.h"
#include "motion_planner/rrt_star.h"
#include "motion_planner/path_smoother.h"
#include "motion_planner/benchmark.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>

using namespace mp;

// ─── Create a standard test environment ──────────────────────

Environment create_demo_environment() {
    // 10×10 workspace with a mix of circular and rectangular obstacles.
    // The layout creates a challenging maze-like scenario that forces
    // the planner to find non-trivial paths.
    Environment env(10.0, 10.0);

    // Central wall — forces the path to go around.
    env.add_obstacle(Rectangle{4.0, 2.0, 0.5, 5.0});

    // Circular obstacles — scattered to create narrow passages.
    env.add_obstacle(Circle{{2.0, 5.0}, 0.8});
    env.add_obstacle(Circle{{6.5, 3.0}, 1.0});
    env.add_obstacle(Circle{{7.0, 7.0}, 0.7});
    env.add_obstacle(Circle{{3.0, 8.0}, 0.6});

    // Additional rectangular obstacles.
    env.add_obstacle(Rectangle{6.0, 5.5, 2.5, 0.4});
    env.add_obstacle(Rectangle{1.0, 1.0, 1.5, 0.4});

    return env;
}

// ─── Export tree and path to CSV ──────────────────────────────

void export_tree_csv(const std::vector<TreeNode>& tree,
                     const std::string& filename)
{
    std::ofstream f(filename);
    f << "node_x,node_y,parent_x,parent_y\n";
    for (size_t i = 1; i < tree.size(); ++i) {
        int p = tree[i].parent;
        if (p >= 0) {
            f << tree[i].position.x << "," << tree[i].position.y << ","
              << tree[p].position.x << "," << tree[p].position.y << "\n";
        }
    }
}

void export_path_csv(const Path& path, const std::string& filename) {
    std::ofstream f(filename);
    f << "x,y\n";
    for (const auto& wp : path.waypoints) {
        f << wp.x << "," << wp.y << "\n";
    }
}

void export_obstacles_csv(const Environment& env,
                          const std::string& filename)
{
    std::ofstream f(filename);
    f << "type,cx,cy,r,x_min,y_min,width,height\n";
    for (const auto& obs : env.obstacles()) {
        std::visit([&f](const auto& shape) {
            using T = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<T, Circle>) {
                f << "circle," << shape.center.x << ","
                  << shape.center.y << "," << shape.radius
                  << ",0,0,0,0\n";
            } else if constexpr (std::is_same_v<T, Rectangle>) {
                f << "rect,0,0,0," << shape.x_min << ","
                  << shape.y_min << "," << shape.width << ","
                  << shape.height << "\n";
            }
        }, obs);
    }
}

// ─── Main ────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    bool benchmark_only = false;
    if (argc > 1 && std::strcmp(argv[1], "--benchmark") == 0) {
        benchmark_only = true;
    }

    // Set up the environment and start/goal.
    Environment env = create_demo_environment();
    Point2D start = {1.0, 1.5};
    Point2D goal  = {9.0, 8.5};

    std::cout << "═══════════════════════════════════════════\n";
    std::cout << "  2D Sampling-Based Motion Planning Demo\n";
    std::cout << "═══════════════════════════════════════════\n";
    std::cout << "Workspace: " << env.width() << " x " << env.height() << "\n";
    std::cout << "Obstacles: " << env.obstacles().size() << "\n";
    std::cout << "Start:     " << start << "\n";
    std::cout << "Goal:      " << goal << "\n\n";

    if (!benchmark_only) {
        // ── Single run of each planner (for visualization) ──────
        PathSmoother smoother(300);

        // --- RRT ---
        {
            RRTConfig cfg;
            cfg.max_iter = 5000;
            cfg.seed = 42;
            RRT planner(env, cfg);
            PlannerResult result = planner.plan(start, goal);

            std::cout << "── RRT ────────────────────────────────\n";
            if (result.success) {
                Path smoothed = smoother.smooth(result.path, env);
                std::cout << "  Status:    SUCCESS\n";
                std::cout << "  Nodes:     " << result.nodes_explored << "\n";
                std::cout << "  Path len:  " << result.path_length() << "\n";
                std::cout << "  Smoothed:  " << smoothed.length() << "\n";
                std::cout << "  Time:      " << result.planning_time_ms
                          << " ms\n\n";

                export_tree_csv(planner.tree(), "results/rrt_tree.csv");
                export_path_csv(result.path, "results/rrt_path.csv");
                export_path_csv(smoothed, "results/rrt_smoothed.csv");
            } else {
                std::cout << "  FAILED (no path found)\n\n";
            }
        }

        // --- RRT* ---
        {
            RRTConfig cfg;
            cfg.max_iter = 5000;
            cfg.seed = 42;
            RRTStar planner(env, cfg);
            PlannerResult result = planner.plan(start, goal);

            std::cout << "── RRT* ───────────────────────────────\n";
            if (result.success) {
                Path smoothed = smoother.smooth(result.path, env);
                std::cout << "  Status:    SUCCESS\n";
                std::cout << "  Nodes:     " << result.nodes_explored << "\n";
                std::cout << "  Path len:  " << result.path_length() << "\n";
                std::cout << "  Smoothed:  " << smoothed.length() << "\n";
                std::cout << "  Time:      " << result.planning_time_ms
                          << " ms\n\n";

                export_tree_csv(planner.tree(), "results/rrt_star_tree.csv");
                export_path_csv(result.path, "results/rrt_star_path.csv");
                export_path_csv(smoothed, "results/rrt_star_smoothed.csv");
            } else {
                std::cout << "  FAILED (no path found)\n\n";
            }
        }

        export_obstacles_csv(env, "results/obstacles.csv");
        std::cout << "Tree and path data exported to results/\n\n";
    }

    // ── Benchmark ───────────────────────────────────────────────
    std::cout << "Running benchmark (" << 20 << " trials per planner)...\n";
    BenchmarkConfig bcfg;
    bcfg.num_trials = 20;
    auto results = run_benchmark(env, start, goal, bcfg);
    print_summary(results);
    export_csv(results, "results/benchmark.csv");

    return 0;
}

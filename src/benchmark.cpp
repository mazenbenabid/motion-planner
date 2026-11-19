// benchmark.cpp — Planner benchmarking and CSV export.
//
// Each trial uses a different RNG seed so results are statistically
// independent but fully reproducible.  We run both RRT and RRT*
// on the same set of seeds for a fair comparison.
//
// Author: Mazen Ben Abid
// License: MIT

#include "motion_planner/benchmark.h"
#include "motion_planner/rrt.h"
#include "motion_planner/rrt_star.h"
#include "motion_planner/path_smoother.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <algorithm>

namespace mp {

std::vector<TrialResult> run_benchmark(
    const Environment& env,
    const Point2D& start,
    const Point2D& goal,
    const BenchmarkConfig& config)
{
    std::vector<TrialResult> results;
    results.reserve(config.num_trials * 2);  // RRT + RRT*

    // Run trials for both planners.
    for (int planner_id = 0; planner_id < 2; ++planner_id) {
        std::string name = (planner_id == 0) ? "RRT" : "RRT*";

        for (int trial = 0; trial < config.num_trials; ++trial) {
            RRTConfig rrt_cfg;
            rrt_cfg.step_size   = config.step_size;
            rrt_cfg.goal_radius = config.goal_radius;
            rrt_cfg.goal_bias   = config.goal_bias;
            rrt_cfg.max_iter    = config.max_iter;
            rrt_cfg.seed        = trial * 7 + 13;  // different seed per trial

            PlannerResult pr;
            if (planner_id == 0) {
                RRT planner(env, rrt_cfg);
                pr = planner.plan(start, goal);
            } else {
                RRTStar planner(env, rrt_cfg);
                pr = planner.plan(start, goal);
            }

            TrialResult tr;
            tr.planner_name    = name;
            tr.trial_number    = trial;
            tr.success         = pr.success;
            tr.path_length     = pr.path_length();
            tr.nodes_explored  = pr.nodes_explored;
            tr.planning_time_ms = pr.planning_time_ms;

            // Smooth the path if planning succeeded.
            if (pr.success) {
                PathSmoother smoother(config.smooth_iters,
                                      trial * 11 + 17);
                Path smoothed = smoother.smooth(pr.path, env);
                tr.smoothed_length = smoothed.length();
            } else {
                tr.smoothed_length = 0.0;
            }

            results.push_back(tr);
        }
    }

    return results;
}

// ─── CSV export ──────────────────────────────────────────────

void export_csv(const std::vector<TrialResult>& results,
                const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open " << filename << " for writing\n";
        return;
    }

    // Header row.
    file << "planner,trial,success,path_length,smoothed_length,"
         << "nodes_explored,planning_time_ms\n";

    // Data rows.
    for (const auto& r : results) {
        file << r.planner_name << ","
             << r.trial_number << ","
             << (r.success ? 1 : 0) << ","
             << std::fixed << std::setprecision(4) << r.path_length << ","
             << r.smoothed_length << ","
             << r.nodes_explored << ","
             << std::setprecision(2) << r.planning_time_ms << "\n";
    }

    std::cout << "Results exported to " << filename << "\n";
}

// ─── Summary table ───────────────────────────────────────────

void print_summary(const std::vector<TrialResult>& results) {
    // Compute statistics for each planner.
    for (const auto& planner_name : {"RRT", "RRT*"}) {
        std::vector<double> lengths, smoothed, times;
        std::vector<int> nodes;
        int successes = 0;
        int total = 0;

        for (const auto& r : results) {
            if (r.planner_name != planner_name) continue;
            total++;
            if (r.success) {
                successes++;
                lengths.push_back(r.path_length);
                smoothed.push_back(r.smoothed_length);
                times.push_back(r.planning_time_ms);
                nodes.push_back(r.nodes_explored);
            }
        }

        std::cout << "\n══════════════════════════════════════\n";
        std::cout << "  " << planner_name << " Summary\n";
        std::cout << "══════════════════════════════════════\n";
        std::cout << "  Success rate:    " << successes << "/" << total
                  << " (" << std::fixed << std::setprecision(0)
                  << (100.0 * successes / total) << "%)\n";

        if (!lengths.empty()) {
            auto mean = [](const std::vector<double>& v) {
                return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
            };

            std::cout << std::setprecision(2);
            std::cout << "  Path length:     "
                      << mean(lengths) << " (mean), "
                      << *std::min_element(lengths.begin(), lengths.end())
                      << " (best)\n";
            std::cout << "  After smoothing: "
                      << mean(smoothed) << " (mean), "
                      << *std::min_element(smoothed.begin(), smoothed.end())
                      << " (best)\n";
            std::cout << "  Planning time:   "
                      << mean(times) << " ms (mean)\n";

            double mean_nodes = std::accumulate(
                nodes.begin(), nodes.end(), 0.0) / nodes.size();
            std::cout << "  Nodes explored:  "
                      << static_cast<int>(mean_nodes) << " (mean)\n";
        }
    }
    std::cout << "\n";
}

}  // namespace mp

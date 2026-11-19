// benchmark.h — Benchmarking utilities for comparing planners.
//
// Runs multiple trials of RRT and RRT* on the same environment,
// collects statistics (path length, planning time, nodes explored,
// success rate), and exports results to CSV for visualization.
//
// Author: Mazen Ben Abid
// License: MIT

#pragma once

#include "types.h"
#include "environment.h"
#include <string>
#include <vector>

namespace mp {

// A single benchmark trial result.
struct TrialResult {
    std::string planner_name;   // "RRT" or "RRT*"
    int         trial_number;
    bool        success;
    double      path_length;        // 0 if failed
    double      smoothed_length;    // 0 if failed
    int         nodes_explored;
    double      planning_time_ms;
};

// Benchmark configuration.
struct BenchmarkConfig {
    int num_trials       = 20;    // trials per planner
    int max_iter         = 5000;  // iteration budget per trial
    int smooth_iters     = 200;   // smoothing iterations
    double step_size     = 0.3;
    double goal_radius   = 0.3;
    double goal_bias     = 0.05;
};

// Run the full benchmark: RRT and RRT* on the given environment.
// Returns all trial results.
std::vector<TrialResult> run_benchmark(
    const Environment& env,
    const Point2D& start,
    const Point2D& goal,
    const BenchmarkConfig& config = {});

// Export results to a CSV file.
void export_csv(const std::vector<TrialResult>& results,
                const std::string& filename);

// Print a summary table to stdout.
void print_summary(const std::vector<TrialResult>& results);

}  // namespace mp

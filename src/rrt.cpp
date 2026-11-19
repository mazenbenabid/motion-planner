// rrt.cpp — Rapidly-exploring Random Tree implementation.
//
// The main loop follows the standard RRT algorithm:
//   for each iteration:
//     1. Sample a random configuration (with goal bias).
//     2. Find the nearest existing node.
//     3. Steer toward the sample by step_size.
//     4. If the edge is collision-free, add the new node.
//     5. If the new node is close to the goal, success!
//
// Author: Mazen Ben Abid
// License: MIT

#include "motion_planner/rrt.h"
#include <chrono>
#include <algorithm>
#include <limits>

namespace mp {

RRT::RRT(const Environment& env, const RRTConfig& config)
    : env_(env), config_(config), rng_(config.seed)
{}

// ─── Main planning loop ──────────────────────────────────────

PlannerResult RRT::plan(const Point2D& start, const Point2D& goal) {
    // Start the timer.
    auto t_start = std::chrono::steady_clock::now();

    // Initialize the tree with the start node.
    nodes_.clear();
    nodes_.push_back({start, -1, 0.0});

    // Validate that start and goal are in free space.
    if (!env_.is_free(start) || !env_.is_free(goal)) {
        return {Path{}, false, 0, 0.0};
    }

    // Distribution for goal biasing: with probability goal_bias,
    // we sample the goal directly instead of a random point.
    // This prevents the tree from wandering forever.
    std::uniform_real_distribution<double> coin(0.0, 1.0);

    for (int iter = 0; iter < config_.max_iter; ++iter) {
        // Step 1: Sample a random point (or the goal).
        Point2D sample;
        if (coin(rng_) < config_.goal_bias) {
            sample = goal;    // bias toward the goal
        } else {
            sample = env_.sample_free(rng_);
        }

        // Step 2: Find the nearest node in the tree.
        int near_idx = nearest(sample);

        // Step 3+4: Extend toward the sample.
        int new_idx = extend(near_idx, sample);
        if (new_idx < 0) {
            continue;  // extension failed (collision)
        }

        // Step 5: Check if we've reached the goal.
        if (nodes_[new_idx].position.distance_to(goal) <= config_.goal_radius) {
            // Connect directly to the goal if possible.
            if (env_.is_segment_free(nodes_[new_idx].position, goal)) {
                double edge_cost = nodes_[new_idx].position.distance_to(goal);
                nodes_.push_back({goal, new_idx,
                                  nodes_[new_idx].cost + edge_cost});
                int goal_idx = static_cast<int>(nodes_.size()) - 1;

                auto t_end = std::chrono::steady_clock::now();
                double elapsed_ms = std::chrono::duration<double, std::milli>(
                    t_end - t_start).count();

                return {extract_path(goal_idx), true,
                        static_cast<int>(nodes_.size()), elapsed_ms};
            }
        }
    }

    // Budget exhausted — no path found.
    auto t_end = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(
        t_end - t_start).count();
    return {Path{}, false, static_cast<int>(nodes_.size()), elapsed_ms};
}

// ─── Nearest neighbor (brute-force) ──────────────────────────

int RRT::nearest(const Point2D& point) const {
    // Linear scan over all nodes.  For trees with < 10 000 nodes
    // this is fast enough; a k-d tree would help for larger trees.
    int best = 0;
    double best_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        double d = nodes_[i].position.distance_to(point);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

// ─── Extend the tree ─────────────────────────────────────────

int RRT::extend(int nearest_idx, const Point2D& target) {
    const Point2D& near_pos = nodes_[nearest_idx].position;

    // Steer: move from nearest toward target, but no farther
    // than step_size.
    double dist = near_pos.distance_to(target);
    Point2D new_pos;
    if (dist <= config_.step_size) {
        new_pos = target;  // target is within reach
    } else {
        new_pos = near_pos.step_toward(target, config_.step_size);
    }

    // Check collision for the new edge.
    if (!env_.is_segment_free(near_pos, new_pos)) {
        return -1;  // collision — don't add this node
    }

    // Add the new node to the tree.
    double edge_cost = near_pos.distance_to(new_pos);
    nodes_.push_back({new_pos, nearest_idx,
                      nodes_[nearest_idx].cost + edge_cost});
    return static_cast<int>(nodes_.size()) - 1;
}

// ─── Path extraction ─────────────────────────────────────────

Path RRT::extract_path(int node_idx) const {
    // Walk backwards from the goal to the root, collecting waypoints.
    Path path;
    int idx = node_idx;
    while (idx >= 0) {
        path.waypoints.push_back(nodes_[idx].position);
        idx = nodes_[idx].parent;
    }
    // Reverse so the path goes start → goal.
    std::reverse(path.waypoints.begin(), path.waypoints.end());
    return path;
}

}  // namespace mp

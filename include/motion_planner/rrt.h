// rrt.h — Rapidly-exploring Random Tree (RRT) planner.
//
// RRT is the foundational sampling-based motion planning algorithm
// (LaValle, 1998).  It builds a tree of collision-free paths by:
//   1. Sampling a random point in the workspace.
//   2. Finding the nearest node in the tree.
//   3. Extending toward the sample by a fixed step size.
//   4. Adding the new node if the edge is collision-free.
//   5. Stopping when a node is close enough to the goal.
//
// Key properties:
//   • Probabilistically complete — will find a path if one exists
//     (given enough iterations).
//   • Fast exploration of the space (Voronoi bias).
//   • The path is NOT optimal — it has unnecessary detours.
//     → RRT* (rrt_star.h) fixes this with rewiring.
//
// Author: Mazen Ben Abid
// License: MIT

#pragma once

#include "types.h"
#include "environment.h"
#include <vector>
#include <random>
#include <optional>

namespace mp {

// ───────────────────────────────────────────────────────────────
// Configuration for the RRT planner
// ───────────────────────────────────────────────────────────────
struct RRTConfig {
    double step_size     = 0.3;    // max extension per iteration
    double goal_radius   = 0.3;    // how close to goal counts as "reached"
    double goal_bias     = 0.05;   // probability of sampling the goal directly
    int    max_iter      = 5000;   // iteration budget
    int    seed          = 42;     // RNG seed for reproducibility
};

// ───────────────────────────────────────────────────────────────
// Tree node — stored in a flat vector for cache efficiency
// ───────────────────────────────────────────────────────────────
struct TreeNode {
    Point2D position;
    int     parent = -1;    // index of parent in the nodes vector (-1 = root)
    double  cost   = 0.0;   // cost from root (used by RRT*, ignored by RRT)
};

// ───────────────────────────────────────────────────────────────
// RRT planner
// ───────────────────────────────────────────────────────────────
class RRT {
public:
    RRT(const Environment& env, const RRTConfig& config = {});

    // Plan a path from 'start' to 'goal'.
    // Returns a PlannerResult with the path (if found) and statistics.
    PlannerResult plan(const Point2D& start, const Point2D& goal);

    // Access the tree for visualization.
    const std::vector<TreeNode>& tree() const { return nodes_; }

protected:
    // --- Helpers (virtual so RRT* can override) --------------------

    // Find the index of the nearest node to 'point'.
    virtual int nearest(const Point2D& point) const;

    // Try to extend the tree from node 'nearest_idx' toward 'target'.
    // Returns the index of the new node, or -1 if the edge is blocked.
    virtual int extend(int nearest_idx, const Point2D& target);

    // Extract the path from root to 'node_idx' by following parent links.
    Path extract_path(int node_idx) const;

    // --- Data members (accessible to RRT*) -------------------------
    const Environment& env_;
    RRTConfig          config_;
    std::vector<TreeNode> nodes_;
    std::mt19937       rng_;
};

}  // namespace mp

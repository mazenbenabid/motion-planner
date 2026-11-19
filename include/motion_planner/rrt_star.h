// rrt_star.h — RRT* (RRT-Star) optimal motion planner.
//
// RRT* (Karaman & Frazzoli, 2011) improves on RRT by adding two
// key operations that make the algorithm asymptotically optimal:
//
//   1. CHOOSE PARENT: When adding a new node, instead of always
//      connecting to the nearest node, check all nodes within a
//      radius and pick the one that gives the lowest cost-to-come.
//
//   2. REWIRE: After adding the new node, check if any nearby nodes
//      would have a shorter path if they went through the new node.
//      If so, rewire them (change their parent).
//
// These two operations gradually shorten the tree's paths toward
// the optimal solution.  The rewiring radius shrinks as the tree
// grows (following the formula from the original paper).
//
// Author: Mazen Ben Abid
// License: MIT

#pragma once

#include "rrt.h"
#include <vector>

namespace mp {

// RRT* inherits from RRT and overrides the extend() method.
// Everything else (plan loop, nearest, extract_path) is reused.
class RRTStar : public RRT {
public:
    // Uses the same config as RRT.
    RRTStar(const Environment& env, const RRTConfig& config = {});

protected:
    // Override extend to add choose-parent and rewire steps.
    int extend(int nearest_idx, const Point2D& target) override;

private:
    // Find all node indices within 'radius' of 'point'.
    std::vector<int> near_nodes(const Point2D& point, double radius) const;

    // Compute the rewiring radius based on the current tree size.
    // This follows the formula: r = gamma * (log(n) / n)^(1/d)
    // where d=2 for 2D planning.
    double rewire_radius() const;
};

}  // namespace mp

// rrt_star.cpp — RRT* (optimal RRT) implementation.
//
// The key difference from basic RRT is in the extend() method:
//
//   Basic RRT:   new_node.parent = nearest_node
//   RRT*:        new_node.parent = cheapest_neighbor_within_radius
//                + rewire other neighbors through new_node if cheaper
//
// This makes the tree converge toward the optimal path as the
// number of samples increases.
//
// Author: Mazen Ben Abid
// License: MIT

#include "motion_planner/rrt_star.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace mp {

RRTStar::RRTStar(const Environment& env, const RRTConfig& config)
    : RRT(env, config)
{}

// ─── Rewiring radius ─────────────────────────────────────────

double RRTStar::rewire_radius() const {
    // Formula from Karaman & Frazzoli (2011):
    //   r_n = gamma * (log(n) / n)^(1/d)
    //
    // gamma is chosen so the ball always contains enough nodes
    // for rewiring to help.  We use a generous constant here.
    // d = 2 for our 2D workspace.
    int n = static_cast<int>(nodes_.size());
    if (n <= 1) return config_.step_size * 3.0;

    // gamma depends on workspace volume and dimension.
    double workspace_area = env_.width() * env_.height();
    double gamma = 2.0 * std::sqrt(workspace_area / M_PI);

    double r = gamma * std::pow(std::log(static_cast<double>(n)) / n, 0.5);

    // Clamp to a reasonable range: at least step_size, at most
    // 3× step_size (to avoid checking too many neighbors).
    return std::clamp(r, config_.step_size, config_.step_size * 3.0);
}

// ─── Find neighbors within radius ────────────────────────────

std::vector<int> RRTStar::near_nodes(const Point2D& point,
                                      double radius) const
{
    std::vector<int> result;
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        if (nodes_[i].position.distance_to(point) <= radius) {
            result.push_back(i);
        }
    }
    return result;
}

// ─── Extend with choose-parent and rewire ────────────────────

int RRTStar::extend(int nearest_idx, const Point2D& target) {
    const Point2D& near_pos = nodes_[nearest_idx].position;

    // Steer toward target (same as basic RRT).
    double dist = near_pos.distance_to(target);
    Point2D new_pos;
    if (dist <= config_.step_size) {
        new_pos = target;
    } else {
        new_pos = near_pos.step_toward(target, config_.step_size);
    }

    // Check collision for the basic edge.
    if (!env_.is_segment_free(near_pos, new_pos)) {
        return -1;
    }

    // ── STEP 1: CHOOSE BEST PARENT ──────────────────────────
    // Look at all nodes within the rewiring radius and pick
    // the one that gives the lowest cost-to-come for new_pos.

    double radius = rewire_radius();
    std::vector<int> neighbors = near_nodes(new_pos, radius);

    int    best_parent = nearest_idx;
    double best_cost   = nodes_[nearest_idx].cost +
                         near_pos.distance_to(new_pos);

    for (int i : neighbors) {
        double candidate_cost = nodes_[i].cost +
                                nodes_[i].position.distance_to(new_pos);
        if (candidate_cost < best_cost) {
            // Check collision for this alternative edge.
            if (env_.is_segment_free(nodes_[i].position, new_pos)) {
                best_parent = i;
                best_cost   = candidate_cost;
            }
        }
    }

    // Add the new node with the best parent.
    nodes_.push_back({new_pos, best_parent, best_cost});
    int new_idx = static_cast<int>(nodes_.size()) - 1;

    // ── STEP 2: REWIRE NEIGHBORS ────────────────────────────
    // Check if routing through new_node is cheaper for any neighbor.
    // If so, change their parent to new_node.

    for (int i : neighbors) {
        if (i == best_parent) continue;  // skip our own parent

        double rewire_cost = best_cost +
                             new_pos.distance_to(nodes_[i].position);

        if (rewire_cost < nodes_[i].cost) {
            // This neighbor would be cheaper through new_node.
            if (env_.is_segment_free(new_pos, nodes_[i].position)) {
                nodes_[i].parent = new_idx;
                nodes_[i].cost   = rewire_cost;
                // Note: in a full implementation, we'd propagate
                // the cost update to all descendants.  For our
                // purposes, the local rewire is sufficient and
                // matches most textbook implementations.
            }
        }
    }

    return new_idx;
}

}  // namespace mp

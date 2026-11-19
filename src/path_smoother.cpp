// path_smoother.cpp — Random shortcutting path smoother.
//
// This is a simple but effective technique used in many real
// motion planners (MoveIt!, OMPL).  The key insight is that
// RRT paths follow tree edges which are rarely straight, so
// there are usually many opportunities to shortcut.
//
// Author: Mazen Ben Abid
// License: MIT

#include "motion_planner/path_smoother.h"
#include <random>
#include <algorithm>

namespace mp {

PathSmoother::PathSmoother(int iterations, int seed)
    : iterations_(iterations), seed_(seed)
{}

Path PathSmoother::smooth(const Path& path, const Environment& env) const {
    // Can't smooth a path with fewer than 3 points.
    if (path.waypoints.size() < 3) {
        return path;
    }

    // Work on a copy so we don't modify the original.
    Path smoothed = path;
    std::mt19937 rng(seed_);

    for (int iter = 0; iter < iterations_; ++iter) {
        int n = static_cast<int>(smoothed.waypoints.size());
        if (n < 3) break;  // already minimal

        // Pick two random distinct indices, ensuring i < j.
        std::uniform_int_distribution<int> dist(0, n - 1);
        int i = dist(rng);
        int j = dist(rng);
        if (i == j) continue;         // same point, skip
        if (i > j) std::swap(i, j);
        if (j - i <= 1) continue;     // adjacent points, nothing to cut

        // Try the shortcut: can we go directly from i to j?
        const Point2D& a = smoothed.waypoints[i];
        const Point2D& b = smoothed.waypoints[j];

        if (env.is_segment_free(a, b)) {
            // Success! Remove all waypoints between i and j.
            // This replaces the sub-path [i, i+1, ..., j] with
            // just [i, j], removing (j - i - 1) waypoints.
            smoothed.waypoints.erase(
                smoothed.waypoints.begin() + i + 1,
                smoothed.waypoints.begin() + j);
        }
    }

    return smoothed;
}

}  // namespace mp

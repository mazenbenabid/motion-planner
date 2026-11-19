// path_smoother.h — Post-processing to shorten planned paths.
//
// Sampling-based planners (RRT, RRT*) produce paths with unnecessary
// detours because they follow tree edges.  Path smoothing removes
// these detours by trying to shortcut between non-adjacent waypoints.
//
// Algorithm (random shortcutting):
//   for N iterations:
//     1. Pick two random waypoints i and j on the path (i < j).
//     2. If the straight line from waypoint[i] to waypoint[j] is
//        collision-free, remove all waypoints between them.
//     3. Repeat until convergence or budget exhausted.
//
// Author: Mazen Ben Abid
// License: MIT

#pragma once

#include "types.h"
#include "environment.h"

namespace mp {

class PathSmoother {
public:
    // 'iterations' controls how many shortcutting attempts to try.
    // More iterations → smoother path, but diminishing returns
    // after ~200 iterations for typical paths.
    explicit PathSmoother(int iterations = 200, int seed = 42);

    // Smooth the given path.  Returns a new, shorter path.
    // The start and goal points are preserved.
    Path smooth(const Path& path, const Environment& env) const;

private:
    int iterations_;
    int seed_;
};

}  // namespace mp

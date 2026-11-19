// environment.h — 2D workspace with obstacles and collision checking.
//
// The Environment class holds the workspace bounds and a collection of
// obstacles (circles and rectangles).  It provides:
//   - point-in-free-space queries
//   - line-segment collision checks (used to validate edges in RRT)
//   - random free-space sampling
//
// Collision checking is the computational bottleneck of sampling-based
// planners, so the segment check uses early-exit logic.
//
// Author: Mazen Ben Abid
// License: MIT

#pragma once

#include "types.h"
#include <random>
#include <variant>
#include <vector>

namespace mp {

// An obstacle is either a circle or a rectangle.
using Obstacle = std::variant<Circle, Rectangle>;

class Environment {
public:
    // Create an environment with the given workspace bounds.
    // The workspace is the rectangle [0, width] × [0, height].
    Environment(double width, double height);

    // --- Obstacle management -------------------------------------------

    void add_obstacle(const Circle& c);
    void add_obstacle(const Rectangle& r);
    void clear_obstacles();

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }

    // --- Queries -------------------------------------------------------

    // Is 'p' inside the workspace and not inside any obstacle?
    bool is_free(const Point2D& p) const;

    // Is the straight-line segment from 'a' to 'b' collision-free?
    // Checks at fine resolution (step_size controls granularity).
    bool is_segment_free(const Point2D& a, const Point2D& b,
                         double step_size = 0.02) const;

    // Sample a random point in free space (rejection sampling).
    // Uses the provided RNG engine so results are reproducible.
    Point2D sample_free(std::mt19937& rng) const;

    // --- Accessors -----------------------------------------------------

    double width()  const { return width_;  }
    double height() const { return height_; }

private:
    double width_;
    double height_;
    std::vector<Obstacle> obstacles_;

    // Does point 'p' lie inside obstacle 'obs'?
    bool point_in_obstacle(const Point2D& p, const Obstacle& obs) const;
};

}  // namespace mp

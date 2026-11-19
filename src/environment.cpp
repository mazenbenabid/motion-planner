// environment.cpp — Implementation of 2D workspace collision checking.
//
// Key algorithms:
//   • Point-in-circle: compare distance to center with radius.
//   • Point-in-rectangle: simple AABB containment test.
//   • Segment collision: walk along the segment in small steps,
//     checking each sample point.  This is simple, correct, and
//     fast enough for the obstacle counts we use (< 50 obstacles).
//
// Author: Mazen Ben Abid
// License: MIT

#include "motion_planner/environment.h"
#include <algorithm>
#include <stdexcept>

namespace mp {

Environment::Environment(double width, double height)
    : width_(width), height_(height)
{
    if (width <= 0.0 || height <= 0.0) {
        throw std::invalid_argument("Workspace dimensions must be positive");
    }
}

// ─── Obstacle management ──────────────────────────────────────

void Environment::add_obstacle(const Circle& c) {
    obstacles_.emplace_back(c);
}

void Environment::add_obstacle(const Rectangle& r) {
    obstacles_.emplace_back(r);
}

void Environment::clear_obstacles() {
    obstacles_.clear();
}

// ─── Point-in-obstacle (private) ──────────────────────────────

bool Environment::point_in_obstacle(const Point2D& p,
                                     const Obstacle& obs) const
{
    // std::visit dispatches based on the actual obstacle type stored
    // in the variant — this is C++17's type-safe alternative to
    // inheritance + virtual functions for simple cases.
    return std::visit([&p](const auto& shape) -> bool {
        using T = std::decay_t<decltype(shape)>;

        if constexpr (std::is_same_v<T, Circle>) {
            // Point is inside the circle if its distance to the
            // center is less than the radius.
            double dx = p.x - shape.center.x;
            double dy = p.y - shape.center.y;
            return (dx * dx + dy * dy) <= (shape.radius * shape.radius);
        }
        else if constexpr (std::is_same_v<T, Rectangle>) {
            // Point is inside the axis-aligned rectangle if it falls
            // within all four edges.
            return p.x >= shape.x_min && p.x <= shape.x_max() &&
                   p.y >= shape.y_min && p.y <= shape.y_max();
        }
    }, obs);
}

// ─── Free-space queries ───────────────────────────────────────

bool Environment::is_free(const Point2D& p) const {
    // First check workspace bounds.
    if (p.x < 0.0 || p.x > width_ || p.y < 0.0 || p.y > height_) {
        return false;
    }
    // Then check every obstacle — exit early on first collision.
    for (const auto& obs : obstacles_) {
        if (point_in_obstacle(p, obs)) {
            return false;
        }
    }
    return true;
}

bool Environment::is_segment_free(const Point2D& a, const Point2D& b,
                                   double step_size) const
{
    // Walk along the segment from a to b, checking points at
    // intervals of 'step_size'.  This catches obstacles that the
    // segment passes through even if both endpoints are free.
    double seg_len = a.distance_to(b);
    if (seg_len < 1e-12) {
        return is_free(a);  // degenerate segment (a == b)
    }

    int num_steps = static_cast<int>(std::ceil(seg_len / step_size));
    for (int i = 0; i <= num_steps; ++i) {
        double t = static_cast<double>(i) / num_steps;  // 0.0 to 1.0
        Point2D sample = {a.x + t * (b.x - a.x),
                          a.y + t * (b.y - a.y)};
        if (!is_free(sample)) {
            return false;  // collision found — no need to continue
        }
    }
    return true;
}

// ─── Random free-space sampling ───────────────────────────────

Point2D Environment::sample_free(std::mt19937& rng) const {
    // Rejection sampling: draw uniformly in the workspace and
    // discard points that land inside obstacles.  This is efficient
    // as long as obstacles don't cover most of the workspace.
    std::uniform_real_distribution<double> dist_x(0.0, width_);
    std::uniform_real_distribution<double> dist_y(0.0, height_);

    // Safety limit to avoid infinite loops in pathological cases.
    constexpr int MAX_ATTEMPTS = 10000;
    for (int i = 0; i < MAX_ATTEMPTS; ++i) {
        Point2D p = {dist_x(rng), dist_y(rng)};
        if (is_free(p)) {
            return p;
        }
    }
    // If we get here, the workspace is almost entirely blocked.
    throw std::runtime_error(
        "Could not find a free sample after 10000 attempts — "
        "workspace may be too cluttered");
}

}  // namespace mp

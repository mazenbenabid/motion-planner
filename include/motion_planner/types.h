// types.h — Core geometric types for 2D motion planning.
//
// Defines the fundamental data structures used throughout the library:
// Point2D for positions, obstacle shapes (Circle, Rectangle), and
// the Path type that planners return.
//
// Author: Mazen Ben Abid
// License: MIT

#pragma once

#include <cmath>
#include <vector>
#include <ostream>
#include <limits>

namespace mp {

// ───────────────────────────────────────────────────────────────
// Point2D — a position in the 2D workspace
// ───────────────────────────────────────────────────────────────
struct Point2D {
    double x = 0.0;
    double y = 0.0;

    // Euclidean distance to another point.
    double distance_to(const Point2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Vector from this point toward 'target', scaled to 'length'.
    // If this == target, returns {0, 0}.
    Point2D step_toward(const Point2D& target, double length) const {
        double d = distance_to(target);
        if (d < 1e-12) return *this;          // already there
        double ratio = length / d;
        return {x + (target.x - x) * ratio,
                y + (target.y - y) * ratio};
    }

    // Arithmetic helpers (useful for interpolation and smoothing).
    Point2D operator+(const Point2D& o) const { return {x + o.x, y + o.y}; }
    Point2D operator-(const Point2D& o) const { return {x - o.x, y - o.y}; }
    Point2D operator*(double s)         const { return {x * s,   y * s};   }

    bool operator==(const Point2D& o) const {
        return std::abs(x - o.x) < 1e-12 && std::abs(y - o.y) < 1e-12;
    }
    bool operator!=(const Point2D& o) const { return !(*this == o); }
};

// Pretty-print for debugging and test output.
inline std::ostream& operator<<(std::ostream& os, const Point2D& p) {
    return os << "(" << p.x << ", " << p.y << ")";
}

// ───────────────────────────────────────────────────────────────
// Obstacle shapes
// ───────────────────────────────────────────────────────────────

// A circular obstacle defined by its center and radius.
struct Circle {
    Point2D center;
    double  radius = 0.0;
};

// An axis-aligned rectangular obstacle defined by its corner and size.
// (x_min, y_min) is the bottom-left corner.
struct Rectangle {
    double x_min = 0.0;
    double y_min = 0.0;
    double width = 0.0;    // extent along x
    double height = 0.0;   // extent along y

    double x_max() const { return x_min + width;  }
    double y_max() const { return y_min + height; }
};

// ───────────────────────────────────────────────────────────────
// Path — the output of a planner
// ───────────────────────────────────────────────────────────────

// An ordered sequence of waypoints from start to goal.
struct Path {
    std::vector<Point2D> waypoints;

    bool empty() const { return waypoints.empty(); }

    // Total Euclidean length of the path.
    double length() const {
        double len = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            len += waypoints[i - 1].distance_to(waypoints[i]);
        }
        return len;
    }
};

// ───────────────────────────────────────────────────────────────
// PlannerResult — everything a planner returns
// ───────────────────────────────────────────────────────────────

struct PlannerResult {
    Path   path;              // empty if planning failed
    bool   success = false;
    int    nodes_explored = 0;
    double planning_time_ms = 0.0;

    double path_length() const { return path.length(); }
};

}  // namespace mp

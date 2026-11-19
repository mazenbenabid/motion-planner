#!/usr/bin/env python3
"""
visualize.py — Plot motion planning results.

Reads the CSV files exported by the C++ demo and generates:
  1. planning_comparison.png — Side-by-side RRT vs RRT* trees and paths
  2. benchmark_results.png  — Box plots comparing planner performance

Usage:
    python3 scripts/visualize.py                    # from project root
    python3 scripts/visualize.py path/to/results/   # custom results dir

Author: Mazen Ben Abid
License: MIT
"""

import sys
import os
import csv
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from pathlib import Path


def load_csv(filepath):
    """Load a CSV file into a list of dicts."""
    with open(filepath, "r") as f:
        reader = csv.DictReader(f)
        return list(reader)


def parse_obstacles(filepath):
    """Parse obstacles CSV into circle and rectangle lists."""
    circles = []
    rects = []
    for row in load_csv(filepath):
        if row["type"] == "circle":
            circles.append({
                "cx": float(row["cx"]),
                "cy": float(row["cy"]),
                "r": float(row["r"]),
            })
        elif row["type"] == "rect":
            rects.append({
                "x": float(row["x_min"]),
                "y": float(row["y_min"]),
                "w": float(row["width"]),
                "h": float(row["height"]),
            })
    return circles, rects


def draw_obstacles(ax, circles, rects):
    """Draw obstacles on a matplotlib axis."""
    for c in circles:
        patch = mpatches.Circle(
            (c["cx"], c["cy"]), c["r"],
            color="#d44", alpha=0.6, zorder=2
        )
        ax.add_patch(patch)
    for r in rects:
        patch = mpatches.Rectangle(
            (r["x"], r["y"]), r["w"], r["h"],
            color="#d44", alpha=0.6, zorder=2
        )
        ax.add_patch(patch)


def draw_tree(ax, tree_file, color="#ccc", alpha=0.3):
    """Draw tree edges from CSV."""
    rows = load_csv(tree_file)
    for row in rows:
        x0, y0 = float(row["node_x"]), float(row["node_y"])
        x1, y1 = float(row["parent_x"]), float(row["parent_y"])
        ax.plot([x0, x1], [y0, y1], color=color, alpha=alpha,
                linewidth=0.5, zorder=1)


def draw_path(ax, path_file, color, label, linewidth=2.0, style="-"):
    """Draw a path from CSV."""
    rows = load_csv(path_file)
    xs = [float(r["x"]) for r in rows]
    ys = [float(r["y"]) for r in rows]
    ax.plot(xs, ys, color=color, linewidth=linewidth,
            linestyle=style, label=label, zorder=5)


def setup_axes(ax, title, workspace_size=10.0):
    """Configure axis limits and labels."""
    ax.set_xlim(-0.3, workspace_size + 0.3)
    ax.set_ylim(-0.3, workspace_size + 0.3)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.2)


def plot_planning_comparison(results_dir, output_dir):
    """Create side-by-side comparison of RRT vs RRT*."""
    circles, rects = parse_obstacles(results_dir / "obstacles.csv")

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    start = (1.0, 1.5)
    goal = (9.0, 8.5)

    for idx, (name, tree_file, path_file, smooth_file) in enumerate([
        ("RRT", "rrt_tree.csv", "rrt_path.csv", "rrt_smoothed.csv"),
        ("RRT*", "rrt_star_tree.csv", "rrt_star_path.csv",
         "rrt_star_smoothed.csv"),
    ]):
        ax = axes[idx]
        setup_axes(ax, name)
        draw_obstacles(ax, circles, rects)
        draw_tree(ax, results_dir / tree_file,
                  color="#8bb" if idx == 0 else "#b8b")
        draw_path(ax, results_dir / path_file,
                  "#2266cc" if idx == 0 else "#22aa44",
                  f"{name} path", linewidth=1.5, style="--")
        draw_path(ax, results_dir / smooth_file,
                  "#cc2222", "Smoothed", linewidth=2.5)

        # Mark start and goal.
        ax.plot(*start, "go", markersize=12, zorder=10, label="Start")
        ax.plot(*goal, "r*", markersize=15, zorder=10, label="Goal")
        ax.legend(loc="lower right", fontsize=9)

    plt.suptitle("Sampling-Based Motion Planning: RRT vs RRT*",
                 fontsize=16, fontweight="bold", y=1.02)
    plt.tight_layout()
    out_path = output_dir / "planning_comparison.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved {out_path}")
    plt.close()


def plot_benchmark(results_dir, output_dir):
    """Create benchmark comparison plots."""
    rows = load_csv(results_dir / "benchmark.csv")

    # Separate by planner.
    data = {"RRT": [], "RRT*": []}
    for r in rows:
        name = r["planner"]
        if int(r["success"]) == 1:
            data[name].append({
                "path_length": float(r["path_length"]),
                "smoothed_length": float(r["smoothed_length"]),
                "nodes": int(r["nodes_explored"]),
                "time_ms": float(r["planning_time_ms"]),
            })

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    metrics = [
        ("path_length", "Path Length (m)", axes[0, 0]),
        ("smoothed_length", "Smoothed Path Length (m)", axes[0, 1]),
        ("time_ms", "Planning Time (ms)", axes[1, 0]),
        ("nodes", "Nodes Explored", axes[1, 1]),
    ]

    colors = ["#2266cc", "#22aa44"]

    for metric, ylabel, ax in metrics:
        rrt_vals = [d[metric] for d in data["RRT"]]
        rrt_star_vals = [d[metric] for d in data["RRT*"]]

        bp = ax.boxplot(
            [rrt_vals, rrt_star_vals],
            tick_labels=["RRT", "RRT*"],
            patch_artist=True,
            widths=0.5,
        )
        for patch, color in zip(bp["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)

        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.2, axis="y")

        # Add mean markers.
        for i, vals in enumerate([rrt_vals, rrt_star_vals]):
            if vals:
                mean_val = sum(vals) / len(vals)
                ax.plot(i + 1, mean_val, "D", color="black",
                        markersize=6, zorder=5)

    plt.suptitle("Benchmark: RRT vs RRT* (20 trials)",
                 fontsize=16, fontweight="bold")
    plt.tight_layout()
    out_path = output_dir / "benchmark_results.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved {out_path}")
    plt.close()


def main():
    # Determine results directory.
    if len(sys.argv) > 1:
        results_dir = Path(sys.argv[1])
    else:
        # Try common locations.
        for candidate in [Path("results"), Path("build/results")]:
            if candidate.exists():
                results_dir = candidate
                break
        else:
            print("Error: Cannot find results directory.")
            print("Run the motion_planner executable first, or pass the path:")
            print("  python3 scripts/visualize.py path/to/results/")
            sys.exit(1)

    output_dir = Path("results")
    output_dir.mkdir(exist_ok=True)

    print(f"Reading results from: {results_dir}")
    print(f"Saving plots to:      {output_dir}")

    # Check required files exist.
    required = ["obstacles.csv", "rrt_tree.csv", "rrt_path.csv",
                "rrt_smoothed.csv", "rrt_star_tree.csv",
                "rrt_star_path.csv", "rrt_star_smoothed.csv",
                "benchmark.csv"]
    missing = [f for f in required if not (results_dir / f).exists()]
    if missing:
        print(f"Error: Missing files: {missing}")
        sys.exit(1)

    plot_planning_comparison(results_dir, output_dir)
    plot_benchmark(results_dir, output_dir)
    print("\nDone! Open results/planning_comparison.png and "
          "results/benchmark_results.png")


if __name__ == "__main__":
    main()

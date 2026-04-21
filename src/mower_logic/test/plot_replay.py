#!/usr/bin/env python3
"""Visualize the output of the blade_speed_adapter CSV-replay rostest.

Reads the trace dumped by test_csv_replay.py (default /tmp/csv_replay_trace.csv)
and the original sensor log, then writes two PNG files:

  time_series.png  — each metric (grass, rpm, current, target_speed) vs sim time
  map.png          — path in (x, y), a panel per metric, colored by metric value

Usage (after running the rostest at least once):
    python3 plot_replay.py \
        --trace /tmp/csv_replay_trace.csv \
        --csv   /home/alex/catkin_ws/src/data/sensorlog_20260420-1106_20260420-1115.csv \
        --out   /tmp/csv_replay_plots
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grass_map import GrassMap


def load_trace(path):
    rows = {"t": [], "s": [], "grass": [], "rpm": [], "current": [], "target_speed": []}
    with open(path) as f:
        for row in csv.DictReader(f):
            for k in rows:
                rows[k].append(float(row[k]))
    return rows


def plot_time_series(trace, out_path):
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    t = trace["t"]

    axes[0].plot(t, trace["grass"], color="tab:green", linewidth=0.8)
    axes[0].set_ylabel("grass_thickness")
    axes[0].set_ylim(-0.05, 1.1)
    axes[0].axhline(0.7, color="red", linestyle="--", linewidth=0.5, label="thick threshold")
    axes[0].legend(loc="upper right", fontsize=8)

    axes[1].plot(t, trace["rpm"], color="tab:blue", linewidth=0.6)
    axes[1].set_ylabel("blade rpm")
    axes[1].axhline(2000, color="red", linestyle="--", linewidth=0.5, label="rpm_max_load")
    axes[1].axhline(4400, color="gray", linestyle=":", linewidth=0.5, label="rpm_nominal")
    axes[1].legend(loc="lower right", fontsize=8)

    axes[2].plot(t, trace["current"], color="tab:orange", linewidth=0.6)
    axes[2].set_ylabel("blade current (A)")
    axes[2].axhline(1.5, color="red", linestyle="--", linewidth=0.5, label="peak")
    axes[2].axhline(0.4, color="gray", linestyle=":", linewidth=0.5, label="idle")
    axes[2].legend(loc="upper right", fontsize=8)

    axes[3].plot(t, trace["target_speed"], color="tab:purple", linewidth=0.8)
    axes[3].set_ylabel("target_speed (m/s)")
    axes[3].set_xlabel("sim time (s)")
    axes[3].axhline(0.4, color="gray", linestyle=":", linewidth=0.5, label="speed_max")
    axes[3].axhline(0.15, color="red", linestyle="--", linewidth=0.5, label="sustained threshold")
    axes[3].set_ylim(0, 0.45)
    axes[3].legend(loc="lower right", fontsize=8)

    fig.suptitle("blade_speed_adapter CSV replay — metrics vs time")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print("wrote %s" % out_path)


def plot_map(trace, grass_map, out_path):
    # Derive (x, y) per trace row from arclength via the same map the test used
    xs, ys = [], []
    for s in trace["s"]:
        x, y, _ = grass_map.sample(s)
        xs.append(x)
        ys.append(y)

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    panels = [
        ("grass_thickness", trace["grass"], "Greens", 0.0, 1.0),
        ("blade rpm", trace["rpm"], "viridis_r", 500, 4400),
        ("blade current (A)", trace["current"], "plasma", 0.4, 1.5),
        ("target_speed (m/s)", trace["target_speed"], "coolwarm_r", 0.05, 0.40),
    ]
    for ax, (title, values, cmap, vmin, vmax) in zip(axes.flat, panels):
        sc = ax.scatter(xs, ys, c=values, cmap=cmap, vmin=vmin, vmax=vmax, s=6)
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal")
        fig.colorbar(sc, ax=ax, shrink=0.8)

    fig.suptitle("blade_speed_adapter CSV replay — metrics on the lawn")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print("wrote %s" % out_path)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--trace", default="/tmp/csv_replay_trace.csv")
    p.add_argument("--csv",
                   default="/home/alex/catkin_ws/src/data/sensorlog_20260420-1106_20260420-1115.csv")
    _default_out = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", ".hypertask", "task", "blade_speed_adapter_plots"))
    p.add_argument("--out", default=_default_out)
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)
    trace = load_trace(args.trace)
    print("loaded trace: %d rows" % len(trace["t"]))

    grass_map = GrassMap(args.csv)
    print("loaded csv: %d rows  path=%.1fm" % (len(grass_map.x), grass_map.total_length))

    plot_time_series(trace, os.path.join(args.out, "time_series.png"))
    plot_map(trace, grass_map, os.path.join(args.out, "map.png"))


if __name__ == "__main__":
    main()

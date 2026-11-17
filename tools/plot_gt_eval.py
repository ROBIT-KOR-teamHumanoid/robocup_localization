#!/usr/bin/python3
"""Plot a gt_eval.py CSV: error over time, error CDF, and paths on the field.

Usage:
  /usr/bin/python3 tools/plot_gt_eval.py gt_eval.csv [--t0 10 --t1 200] [--out out.png]
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
MCL_COLOR = "#2a78d6"
MLP_COLOR = "#eb6834"

# (A, B) from field_coordinate.hpp
FIELDS = {"large": (22.0, 14.0), "middle": (14.0, 9.0), "small": (9.0, 6.0)}


def load(path, source, t0, t1):
    rows = []
    with open(path) as handle:
        for row in csv.DictReader(handle):
            if row["source"] != source:
                continue
            t = float(row["t_sec"])
            if t < t0 or t > t1:
                continue
            rows.append([t, float(row["gt_x_m"]), float(row["gt_y_m"]),
                         float(row["est_x_m"]), float(row["est_y_m"]), float(row["err_cm"])])
    return np.array(rows).reshape(-1, 6)


def describe(name, data):
    if len(data) == 0:
        return f"{name} (no samples)"
    err = data[:, 5]
    return (f"{name}  median {np.median(err):.1f} cm, "
            f"p90 {np.percentile(err, 90):.1f} cm, n={len(err)}")


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9, length=0)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)


def plot_time(ax, mcl, mlp):
    style(ax)
    ax.plot(mcl[:, 0], mcl[:, 5], color=MCL_COLOR, linewidth=1.5, label=describe("MCL", mcl))
    ax.scatter(mlp[:, 0], mlp[:, 5], s=22, color=MLP_COLOR, edgecolors=SURFACE, linewidths=0.8,
               zorder=3, label=describe("MLP", mlp))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position error (cm)")
    ax.set_ylim(bottom=0)
    ax.set_title("Error over time", loc="left", color=INK, fontsize=12)
    ax.legend(loc="upper right", frameon=False, labelcolor=INK_2, fontsize=9)


def plot_cdf(ax, mcl, mlp):
    style(ax)
    for name, data, color in (("MCL", mcl, MCL_COLOR), ("MLP", mlp, MLP_COLOR)):
        if len(data) == 0:
            continue
        err = np.sort(data[:, 5])
        share = np.arange(1, len(err) + 1) / len(err) * 100.0
        ax.plot(err, share, color=color, linewidth=2.0, label=name)
    for limit in (20, 50):
        ax.axvline(limit, color=AXIS, linewidth=1.0)
        ax.text(limit + 1, 3, f"{limit} cm", color=MUTED, fontsize=9)
    ax.set_xlabel("Position error (cm)")
    ax.set_ylabel("Samples at or below (%)")
    ax.set_xlim(left=0)
    ax.set_ylim(0, 102)
    ax.set_title("Error distribution (CDF)", loc="left", color=INK, fontsize=12)
    ax.legend(loc="lower right", frameon=False, labelcolor=INK_2, fontsize=9)


def plot_field(ax, mcl, mlp, field):
    style(ax)
    ax.grid(False)
    a, b = FIELDS[field]
    ax.add_patch(Rectangle((-a / 2, -b / 2), a, b, fill=False, edgecolor=AXIS, linewidth=1.2))
    ax.plot([0, 0], [-b / 2, b / 2], color=AXIS, linewidth=1.0)
    ax.plot(mcl[:, 1], mcl[:, 2], color=MUTED, linewidth=2.0, label="Ground truth")
    ax.plot(mcl[:, 3], mcl[:, 4], color=MCL_COLOR, linewidth=1.5, label="MCL")
    ax.scatter(mlp[:, 3], mlp[:, 4], s=18, color=MLP_COLOR, edgecolors=SURFACE, linewidths=0.6,
               zorder=3, label="MLP")
    ax.set_aspect("equal")
    ax.set_xlim(-a / 2 - 1, a / 2 + 1)
    ax.set_ylim(-b / 2 - 1, b / 2 + 1)
    ax.set_xlabel("Field x (m)")
    ax.set_ylabel("Field y (m)")
    ax.set_title("Paths on the field", loc="left", color=INK, fontsize=12)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=3, frameon=False,
              labelcolor=INK_2, fontsize=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=float("inf"))
    ap.add_argument("--field", default="large", choices=list(FIELDS))
    ap.add_argument("--title", default="")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    mcl = load(args.csv, "mcl", args.t0, args.t1)
    mlp = load(args.csv, "mlp", args.t0, args.t1)

    out = args.out
    if not out:
        out = os.path.splitext(args.csv)[0] + ".png"
    title = args.title
    if not title:
        title = f"MCL and MLP position error vs ground truth ({os.path.basename(args.csv)})"

    fig = plt.figure(figsize=(13, 9), facecolor=SURFACE, constrained_layout=True)
    grid = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.25])
    plot_time(fig.add_subplot(grid[0, :]), mcl, mlp)
    plot_cdf(fig.add_subplot(grid[1, 0]), mcl, mlp)
    plot_field(fig.add_subplot(grid[1, 1]), mcl, mlp, args.field)
    fig.suptitle(title, color=INK, fontsize=14, x=0.01, ha="left")
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main()

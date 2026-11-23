#!/usr/bin/env python3
"""
sonar_viewer.py - plot a rotating-sonar scan as a 2D map with wide-beam erosion and DBSCAN.

Reads a .txt file containing pasted serial output of the form:

    Distance buffer:  [349, 349, 353, ...]
    Theta buffer:  [0, 0.00214, 0.00429, ...]

Distances are in mm, angles in radians. Each (distance, theta) pair is
converted to x = r*cos(theta), y = r*sin(theta).

Usage:
    python sonar_viewer.py scan.txt
    python sonar_viewer.py scan.txt --offset 80          # sensor 80 mm from rotation center
    python sonar_viewer.py scan.txt --tol 32 --eps 110   # tune erosion tolerance & DBSCAN radius
    python sonar_viewer.py scan.txt --zoom 250           # add a near-field panel
    python sonar_viewer.py scan.txt --out map.png --no-show
    python sonar_viewer.py a.txt b.txt c.txt --overlay   # compare several scans
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

NO_RETURN_DEFAULT = 2500  # readings >= this are treated as "no echo"


def parse_buffer(text, name):
    """Return the list of numbers in '<name> buffer: [ ... ]' (may span lines)."""
    m = re.search(rf"{name}\s*buffer\s*:\s*\[([^\]]*)\]", text, re.IGNORECASE)
    if not m:
        raise ValueError(f"could not find a '{name} buffer: [...]' list in the file")
    items = [s for s in re.split(r"[,\s]+", m.group(1)) if s]
    return np.array([float(s) for s in items])


def load_scan(path, beam_deg=45.0, tol_mm=10.0, no_return=NO_RETURN_DEFAULT):
    text = Path(path).read_text()
    d = parse_buffer(text, "Distance")
    t = parse_buffer(text, "Theta")

    if len(d) != len(t):
        n = min(len(d), len(t))
        print(f"[{Path(path).name}] length mismatch: {len(d)} distances vs {len(t)} "
              f"angles; using the first {n} pairs", file=sys.stderr)
        d, t = d[:n], t[:n]

    d, t = wide_beam_min_erosion(d, t, beam_deg=beam_deg, tol_mm=tol_mm, no_return=no_return)
    return np.asarray(d, dtype=float), np.asarray(t, dtype=float)


def wide_beam_min_erosion(distance, angle, beam_deg=45.0, tol_mm=10.0, no_return=2500):
    """
    Pure-Python wide-beam minimum erosion using a true angular window (in degrees).
    """
    # 1. Pair valid readings and sort by angle
    pairs = [
        (t, d)
        for d, t in zip(distance, angle)
        if 0 <= d < no_return
    ]
    if not pairs:
        return [], []

    pairs.sort(key=lambda item: item[0])

    # 2. Collapse duplicate angles using median distance
    unique_t = []
    med_d = []
    i = 0
    total = len(pairs)
    while i < total:
        curr_t = pairs[i][0]
        group = [pairs[i][1]]
        i += 1
        while i < total and pairs[i][0] == curr_t:
            group.append(pairs[i][1])
            i += 1
        group.sort()
        mid = len(group) // 2
        median_val = group[mid] if len(group) % 2 == 1 else 0.5 * (group[mid - 1] + group[mid])
        unique_t.append(curr_t)
        med_d.append(median_val)

    # 3. Keep points within tol_mm of the local minimum inside +/- (beam_deg / 2)
    half_rad = math.radians(beam_deg) * 0.5
    two_pi = 2.0 * math.pi
    filtered_d = []
    filtered_t = []

    for idx, t_center in enumerate(unique_t):
        local_min = med_d[idx]
        for j, t_other in enumerate(unique_t):
            # Shortest circular angular distance on [0, 2*pi]
            diff = abs(t_center - t_other) % two_pi
            if diff > math.pi:
                diff = two_pi - diff
            if diff <= half_rad and med_d[j] < local_min:
                local_min = med_d[j]

        if med_d[idx] <= local_min + tol_mm:
            filtered_d.append(med_d[idx])
            filtered_t.append(t_center)

    return filtered_d, filtered_t


def dbscan_clusters(x, y, eps_mm=110.0, min_samples=3):
    """
    Pure-Python DBSCAN on (x, y) points.
    Returns (clusters, smallest_circle) where:
      - clusters: list of lists of (x, y) points (noise ignored)
      - smallest_circle: (cx, cy, radius) for the smallest cluster, or None
    """
    pts = list(zip(x, y))
    n = len(pts)
    eps_sq = eps_mm * eps_mm
    labels = [None] * n  # None = unvisited, -1 = noise, >= 0 = cluster id
    cluster_id = 0

    def region_query(idx):
        x0, y0 = pts[idx]
        return [
            j for j, (x1, y1) in enumerate(pts)
            if (x0 - x1) ** 2 + (y0 - y1) ** 2 <= eps_sq
        ]

    for i in range(n):
        if labels[i] is not None:
            continue
        neighbors = region_query(i)
        if len(neighbors) < min_samples:
            labels[i] = -1
            continue

        labels[i] = cluster_id
        seed_set = list(neighbors)
        k = 0
        while k < len(seed_set):
            curr = seed_set[k]
            k += 1
            if labels[curr] == -1:
                labels[curr] = cluster_id
            if labels[curr] is not None:
                continue
            labels[curr] = cluster_id
            curr_neighbors = region_query(curr)
            if len(curr_neighbors) >= min_samples:
                seed_set.extend(curr_neighbors)
        cluster_id += 1

    grouped = {}
    for idx, lbl in enumerate(labels):
        if lbl is not None and lbl >= 0:
            grouped.setdefault(lbl, []).append(pts[idx])

    if not grouped:
        return [], None

    clusters = list(grouped.values())
    smallest = min(clusters, key=len)
    cx = sum(p[0] for p in smallest) / len(smallest)
    cy = sum(p[1] for p in smallest) / len(smallest)
    radius = max(math.hypot(p[0] - cx, p[1] - cy) for p in smallest)

    return clusters, (cx, cy, radius)


def to_xy(d, t, offset=0.0):
    """Polar -> Cartesian. offset = sensor distance from rotation center (mm),
    assuming the sensor points radially outward."""
    r = d + offset
    return r * np.cos(t), r * np.sin(t)


def summarize(name, d, t, no_return, eps_mm=110.0, min_samples=3):
    print(f"{name}: {len(d)} eroded points, angle 0..{t.max():.2f} rad "
          f"({np.degrees(t.max()):.0f} deg)")
    print(f"  distance min/median/max: {d.min():.0f} / {np.median(d):.0f} / {d.max():.0f} mm")
    x, y = to_xy(d, t, 0.0)
    clusters, circle = dbscan_clusters(x, y, eps_mm=eps_mm, min_samples=min_samples)
    print(f"  DBSCAN clusters: {len(clusters)} (sizes: {sorted(len(c) for c in clusters)})")
    if circle:
        cx, cy, rad = circle
        print(f"  smallest segment center: ({cx:.0f}, {cy:.0f}) mm, radius: {rad:.0f} mm")


def draw_scan(ax, d, t, offset, no_return, eps_mm=110.0, min_samples=3, label=None, color=None, size=10):
    good = (d >= 0) & (d < no_return)
    x, y = to_xy(d[good], t[good], offset)

    clusters, circle = dbscan_clusters(x, y, eps_mm=eps_mm, min_samples=min_samples)
    cmap = plt.cm.tab10.colors

    if color is None:
        # Plot each non-noise DBSCAN cluster in a distinct color
        for ci, cluster in enumerate(clusters):
            cx_pts = [p[0] for p in cluster]
            cy_pts = [p[1] for p in cluster]
            ax.scatter(cx_pts, cy_pts, color=cmap[ci % 10], s=size, zorder=2,
                       label=f"seg {ci + 1} ({len(cluster)} pts)")
    else:
        all_pts = [p for c in clusters for p in c]
        if all_pts:
            ax.scatter([p[0] for p in all_pts], [p[1] for p in all_pts],
                       color=color, s=size, alpha=0.7, label=label, zorder=2)

    # Circle the smallest valid segment
    if circle:
        cx, cy, rad = circle
        ax.add_patch(plt.Circle((cx, cy), rad + 35, fill=False, ec="red", lw=2, ls="--",
                                zorder=5, label=f"smallest ({cx:.0f}, {cy:.0f})"))
        ax.scatter([cx], [cy], c="red", marker="+", s=60, zorder=5)


def style_axes(ax, offset, title):
    ax.scatter([0], [0], c="k", marker="^", s=80, zorder=4)
    if offset:
        ax.add_patch(plt.Circle((0, 0), offset, fill=False, ls="--", color="gray"))
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title(title)


                # beam_deg=40.0,
                # tol_mm=27.0,
                # eps_mm=35.0,
                # min_samples=8,
def main():
    p = argparse.ArgumentParser(description="Plot a rotating-sonar scan from a text file.")
    p.add_argument("files", nargs="+", help=".txt file(s) with Distance/Theta buffers")
    p.add_argument("--offset", type=float, default=0.0,
                   help="sensor distance from rotation center in mm (default 0 = raw)")
    p.add_argument("--no-return", type=float, default=NO_RETURN_DEFAULT,
                   help=f"readings >= this are marked as no-return (default {NO_RETURN_DEFAULT})")
    # p.add_argument("--beam", type=float, default=45.0,
    p.add_argument("--beam", type=float, default=60.0,
                   help="wide-beam erosion angular window in degrees (default 40)")
    p.add_argument("--tol", type=float, default=27.0,
                   help="local minimum distance tolerance in mm (default 27)")
    p.add_argument("--eps", type=float, default=200.0,
                   help="DBSCAN neighborhood radius in mm (default 200)")
    p.add_argument("--min-samples", type=int, default=12,
                   help="DBSCAN minimum points per cluster (default 12)")
    p.add_argument("--zoom", type=float, default=None,
                   help="add a near-field panel showing +/- this many mm")
    p.add_argument("--drop", type=float, nargs=2, metavar=("MIN", "MAX"), default=None,
                   help="drop readings between MIN and MAX mm (e.g. --drop 400 410)")
    p.add_argument("--overlay", action="store_true",
                   help="draw all files on one plot in different colors")
    p.add_argument("--out", default=None, help="save the figure to this path (png/pdf/svg)")
    p.add_argument("--no-show", action="store_true", help="do not open a window")
    args = p.parse_args()

    scans = []
    for f in args.files:
        try:
            d, t = load_scan(f, beam_deg=args.beam, tol_mm=args.tol, no_return=args.no_return)
        except (OSError, ValueError) as e:
            sys.exit(f"{f}: {e}")
        if args.drop:
            keep = ~((d >= args.drop[0]) & (d <= args.drop[1]))
            print(
                f"[{Path(f).name}] dropped {(~keep).sum()} readings in "
                f"{args.drop[0]:.0f}-{args.drop[1]:.0f} mm",
                file=sys.stderr,
            )
            d, t = d[keep], t[keep]

        summarize(
            Path(f).name,
            d,
            t,
            args.no_return,
            eps_mm=args.eps,
            min_samples=args.min_samples,
        )
        scans.append((Path(f).name, d, t))

    colors = plt.cm.tab10.colors
    if args.overlay or len(scans) == 1:
        panels = 2 if args.zoom else 1
        fig, axes = plt.subplots(
            1,
            panels,
            figsize=(7.5 * panels, 7.5),
            squeeze=False,
        )
        for panel_index, ax in enumerate(axes[0]):
            for scan_index, (name, d, t) in enumerate(scans):
                if len(scans) == 1:
                    draw_scan(
                        ax,
                        d,
                        t,
                        args.offset,
                        args.no_return,
                        eps_mm=args.eps,
                        min_samples=args.min_samples,
                    )
                else:
                    draw_scan(
                        ax,
                        d,
                        t,
                        args.offset,
                        args.no_return,
                        eps_mm=args.eps,
                        min_samples=args.min_samples,
                        label=name,
                        color=colors[scan_index % 10],
                    )

            title = scans[0][0] if len(scans) == 1 else "Overlay"
            title += f" ({args.beam:g}° erosion ±{args.tol:g}mm + DBSCAN)"
            if args.offset:
                title += f" [offset {args.offset:g} mm]"
            if panel_index == 1:
                ax.set_xlim(-args.zoom, args.zoom)
                ax.set_ylim(-args.zoom, args.zoom)
                title = f"Zoom +/-{args.zoom:g} mm"
            style_axes(ax, args.offset, title)
            if panel_index == 0:
                ax.legend(loc="upper right", fontsize=8)
    else:
        n = len(scans)
        cols = min(n, 3)
        rows = -(-n // cols)
        fig, axes = plt.subplots(
            rows,
            cols,
            figsize=(6 * cols, 6 * rows),
            squeeze=False,
        )
        for ax in axes.flat[n:]:
            ax.axis("off")
        for ax, (name, d, t) in zip(axes.flat, scans):
            draw_scan(
                ax,
                d,
                t,
                args.offset,
                args.no_return,
                eps_mm=args.eps,
                min_samples=args.min_samples,
            )
            if args.zoom:
                ax.set_xlim(-args.zoom, args.zoom)
                ax.set_ylim(-args.zoom, args.zoom)
            style_axes(ax, args.offset, name)

    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=140, bbox_inches="tight")
        print(f"saved {args.out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
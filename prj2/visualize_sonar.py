#!/usr/bin/env python3
"""
visualize_sonar.py - replay CandleSearch._findCandleByDip on logged scan data.

Reads a .txt file containing pasted serial output of the form:

    Distance buffer: [349, 349, 353, ...]
    Theta buffer: [0, 0.00214, 0.00429, ...]

Distances are in mm, angles in radians. The logged distances ALREADY include the
robot's +80 mm ultrasonic offset (it is added before buffering), so --offset
defaults to 0 and no_return defaults to 2500 + 80, exactly like the robot code.

Uses the same circular 1-degree binning and dip detection as
CandleSearch._findCandleByDip (candle_search.py):
  1. keep the minimum distance in each 1-degree bin
  2. compare each bin with 5-bin median samples 30 degrees to each side
  3. mark bins at least 100 mm below both background sides
  4. select the median-depth run within the 4-50 degree width limits

Usage:
    python visualize_sonar.py good_data.txt
    python visualize_sonar.py good_data.txt --side 30 --drop 100
    python visualize_sonar.py good_data.txt --min-width 4 --max-width 50
    python visualize_sonar.py good_data.txt --zoom 900
    python visualize_sonar.py good_data.txt --out map.png --no-show
    python visualize_sonar.py data.txt good_data.txt --overlay
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

SENSOR_OFFSET_MM = 80                        # robot's ultrasonic_distance_offset
NO_RETURN_DEFAULT = 2500 + SENSOR_OFFSET_MM  # robot passes no_return=2500 + offset


# --------------------------------------------------------------------------- #
# Parsing
# --------------------------------------------------------------------------- #
def parse_buffer(text, name):
    """Return the list of numbers in '<name> buffer: [ ... ]' (may span lines).
    If the buffer is printed more than once, the LAST occurrence is used."""
    matches = list(re.finditer(rf"{name}\s*buffer\s*:\s*\[([^\]]*)\]", text, re.IGNORECASE))
    if not matches:
        raise ValueError(f"could not find a '{name} buffer: [...]' list in the file")
    items = [s for s in re.split(r"[,\s]+", matches[-1].group(1)) if s]
    return [None if s.lower() == "none" else float(s) for s in items]


def load_scan(path):
    """Return raw (distance, theta) lists exactly as logged by the robot."""
    text = Path(path).read_text()
    d = parse_buffer(text, "Distance")
    t = parse_buffer(text, "Theta")
    if len(d) != len(t):
        n = min(len(d), len(t))
        print(f"[{Path(path).name}] WARNING: {len(d)} distances vs {len(t)} angles; "
              f"using the first {n} pairs", file=sys.stderr)
        d, t = d[:n], t[:n]
    return d, t


# --------------------------------------------------------------------------- #
# Port of CandleSearch._findSmallestSegmentTarget
# --------------------------------------------------------------------------- #
def find_candle(
    distance_buffer,
    theta_buffer,
    bin_deg=1.0,
    side_deg=30.0,
    drop_mm=100.0,
    min_width_deg=4.0,
    max_width_deg=50.0,
    no_return=NO_RETURN_DEFAULT,
):
    """
    Returns a dict:
      bins:       minimum distance for each angular bin
      dip_bins:   True for bins classified as the candle dip
      runs:       circular groups of adjacent dip bins
      target:     angle/range tuple matching _findCandleByDip
      result:     (theta, dist) or None
      raw:        [(distance, angle), ...] valid raw readings
    """
    bin_count = int(360 / bin_deg)
    range_bins = [no_return] * bin_count
    raw = []

    for distance, angle in zip(distance_buffer, theta_buffer):
        if distance is None or distance <= 0 or distance >= no_return:
            continue
        raw.append((distance, angle))
        bin_index = int((angle % (2 * math.pi)) / (2 * math.pi) * bin_count) % bin_count
        range_bins[bin_index] = min(range_bins[bin_index], distance)

    def median_range(bin_indices):
        range_values = sorted(range_bins[i % bin_count] for i in bin_indices)
        return range_values[len(range_values) // 2]

    side_bins = int(side_deg / bin_deg)
    dip_bins = []
    for bin_index in range(bin_count):
        left_range = median_range(range(bin_index - side_bins - 5, bin_index - side_bins))
        right_range = median_range(range(bin_index + side_bins, bin_index + side_bins + 5))
        background_range = min(left_range, right_range)
        dip_bins.append(
            range_bins[bin_index] < no_return
            and range_bins[bin_index] < background_range - drop_mm
        )

    first_valid_bin = next((i for i in range(bin_count) if not dip_bins[i]), None)
    runs = []
    best_run = None
    if first_valid_bin is not None:
        current_run = []
        for offset in range(1, bin_count + 1):
            bin_index = (first_valid_bin + offset) % bin_count
            if dip_bins[bin_index]:
                current_run.append(bin_index)
            elif current_run:
                runs.append(current_run)
                current_run = []

        for run_bins in runs:
            run_width_deg = len(run_bins) * bin_deg
            if not (min_width_deg <= run_width_deg <= max_width_deg):
                continue
            median_run_range = sorted(range_bins[i] for i in run_bins)[len(run_bins) // 2]
            if best_run is None or median_run_range < best_run[1]:
                best_run = (run_bins, median_run_range)

    if best_run is None:
        return {
            "bins": range_bins, "dip_bins": dip_bins, "runs": runs,
            "best_run": None, "target": None, "result": None,
            "raw": raw, "bin_deg": bin_deg, "no_return": no_return,
        }

    run_bins, median_run_range = best_run
    middle_bin = run_bins[len(run_bins) // 2]
    target_theta = (middle_bin + 0.5) * bin_deg * math.pi / 180.0
    return {
        "bins": range_bins, "dip_bins": dip_bins, "runs": runs,
        "best_run": best_run, "target": (target_theta, median_run_range),
        "result": (target_theta, median_run_range),
        "raw": raw, "bin_deg": bin_deg, "no_return": no_return,
    }


# --------------------------------------------------------------------------- #
# Reporting / drawing
# --------------------------------------------------------------------------- #
def summarize(name, raw_d, res):
    print(f"{name}: {len(raw_d)} raw -> {len(res['raw'])} valid readings "
          f"-> {len(res['bins'])} angle bins")
    if res["raw"]:
        distances = [distance for distance, _ in res["raw"]]
        print(f"  distance min/median/max: {min(distances):.0f} / "
              f"{float(np.median(distances)):.0f} / {max(distances):.0f} mm")
    print(f"  dip runs: {len(res['runs'])}; dip bins: {sum(res['dip_bins'])}")
    for index, run in enumerate(res["runs"], start=1):
        width = len(run) * res["bin_deg"]
        is_target = res["best_run"] is not None and run is res["best_run"][0]
        print(f"    run {index}: {len(run):3d} bins, width {width:.1f} deg"
              f"{'  <== TARGET RUN' if is_target else ''}")
    if res["result"]:
        theta, distance = res["result"]
        print(f"  candle candidate: dist {distance:.0f} mm, "
              f"theta {math.degrees(theta):.1f} deg")
    else:
        print("  no valid candle segment found")


def _shift(x, y, r, offset):
    """Optionally push points radially outward by an extra offset."""
    if not offset:
        return x, y
    s = (r + offset) / r if r > 0 else 1.0
    return x * s, y * s


def draw_scan(ax, res, offset, label=None, color=None, size=10, show_raw=True):
    if show_raw:
        rx = [distance * math.cos(angle) for distance, angle in res["raw"]]
        ry = [distance * math.sin(angle) for distance, angle in res["raw"]]
        ax.scatter(rx, ry, color="lightgray", s=max(size * 0.5, 3), zorder=1,
                   label="raw valid readings" if color is None else None)

    bin_deg = res["bin_deg"]
    bin_ranges = res["bins"]
    bx = []
    by = []
    for i, distance in enumerate(bin_ranges):
        if distance >= res["no_return"]:
            continue
        theta = math.radians((i + 0.5) * bin_deg)
        bx.append(distance * math.cos(theta))
        by.append(distance * math.sin(theta))
    ax.scatter(bx, by, color="tab:blue" if color is None else color,
               s=size, alpha=0.65, zorder=3,
               label="1-degree range bins" if color is None else label)

    dip_indices = [i for i, is_dip in enumerate(res["dip_bins"]) if is_dip]
    if dip_indices:
        dx = []
        dy = []
        for i in dip_indices:
            distance = bin_ranges[i]
            theta = math.radians((i + 0.5) * bin_deg)
            dx.append(distance * math.cos(theta))
            dy.append(distance * math.sin(theta))
        ax.scatter(dx, dy, color="orange", s=size * 1.8, zorder=4,
                   label="dip bins")

    if res["target"] is not None:
        theta, distance = res["result"]
        x = distance * math.cos(theta)
        y = distance * math.sin(theta)
        ax.scatter([x], [y], c="red", marker="+", s=100, zorder=6,
                   label=f"target {distance:.0f} mm @ {math.degrees(theta):.1f}°")
        reach = max(distance, 1) * 1.15
        ax.plot([0, reach * math.cos(theta)], [0, reach * math.sin(theta)],
                color="red", lw=1, alpha=0.5, zorder=4)


def style_axes(ax, offset, title):
    ax.scatter([0], [0], c="k", marker="^", s=80, zorder=6)
    if offset:
        ax.add_patch(plt.Circle((0, 0), offset, fill=False, ls="--", color="gray"))
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title(title)


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main():
    p = argparse.ArgumentParser(description="Replay the CandleSearch pipeline on a logged scan.")
    p.add_argument("files", nargs="+", help=".txt file(s) with Distance/Theta buffers")
    p.add_argument("--offset", type=float, default=0.0,
                   help="EXTRA radial offset in mm for plotting (logged data already includes "
                        f"the robot's +{SENSOR_OFFSET_MM} mm; default 0)")
    p.add_argument("--no-return", type=float, default=NO_RETURN_DEFAULT,
                   help=f"readings >= this are discarded (default {NO_RETURN_DEFAULT} = 2500 + offset)")
    p.add_argument("--bin", type=float, default=1.0,
                   help="angular bin width in degrees (default 1)")
    p.add_argument("--side", type=float, default=30.0,
                   help="distance from each side sample in degrees (default 30)")
    p.add_argument("--drop", type=float, default=100.0,
                   dest="dip_drop", help="required dip depth below both sides in mm (default 100)")
    p.add_argument("--min-width", type=float, default=4.0,
                   help="minimum dip run width in degrees (default 4)")
    p.add_argument("--max-width", type=float, default=50.0,
                   help="maximum dip run width in degrees (default 50)")
    p.add_argument("--zoom", type=float, default=None,
                   help="add a near-field panel showing +/- this many mm")
    p.add_argument("--remove-range", type=float, nargs=2, metavar=("MIN", "MAX"), default=None,
                   help="drop raw readings between MIN and MAX mm (e.g. --remove-range 400 410)")
    p.add_argument("--no-raw", action="store_true", help="hide the faint pre-erosion points")
    p.add_argument("--overlay", action="store_true",
                   help="draw all files on one plot in different colors")
    p.add_argument("--out", default=None, help="save the figure to this path (png/pdf/svg)")
    p.add_argument("--no-show", action="store_true", help="do not open a window")
    args = p.parse_args()

    scans = []
    for f in args.files:
        try:
            d, t = load_scan(f)
        except (OSError, ValueError) as e:
            sys.exit(f"{f}: {e}")

        if args.remove_range:
            keep = [not (dv is not None and args.remove_range[0] <= dv <= args.remove_range[1]) for dv in d]
            print(f"[{Path(f).name}] dropped {keep.count(False)} readings in "
                  f"{args.remove_range[0]:.0f}-{args.remove_range[1]:.0f} mm", file=sys.stderr)
            d = [v for v, k in zip(d, keep) if k]
            t = [v for v, k in zip(t, keep) if k]

        res = find_candle(
            d, t,
            bin_deg=args.bin, side_deg=args.side, drop_mm=args.dip_drop,
            min_width_deg=args.min_width, max_width_deg=args.max_width,
            no_return=args.no_return,
        )
        summarize(Path(f).name, d, res)
        scans.append((Path(f).name, res))

    title_suffix = (f" ({args.bin:g}° bins, sides ±{args.side:g}°, "
                    f"dip {args.dip_drop:g}mm, width {args.min_width:g}-{args.max_width:g}°)")
    colors = plt.cm.tab10.colors
    show_raw = not args.no_raw

    if args.overlay or len(scans) == 1:
        panels = 2 if args.zoom else 1
        fig, axes = plt.subplots(1, panels, figsize=(7.5 * panels, 7.5), squeeze=False)
        for panel_index, ax in enumerate(axes[0]):
            for si, (name, res) in enumerate(scans):
                if len(scans) == 1:
                    draw_scan(ax, res, args.offset, show_raw=show_raw)
                else:
                    draw_scan(ax, res, args.offset, label=name, color=colors[si % 10],
                              show_raw=False)
            title = (scans[0][0] if len(scans) == 1 else "Overlay") + title_suffix
            if args.offset:
                title += f" [extra offset {args.offset:g} mm]"
            if panel_index == 1:
                ax.set_xlim(-args.zoom, args.zoom)
                ax.set_ylim(-args.zoom, args.zoom)
                title = f"Zoom +/-{args.zoom:g} mm"
            style_axes(ax, args.offset, title)
            if panel_index == 0:
                ax.legend(loc="upper right", fontsize=7)
    else:
        n = len(scans)
        cols = min(n, 3)
        rows = -(-n // cols)
        fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 6 * rows), squeeze=False)
        for ax in axes.flat[n:]:
            ax.axis("off")
        for ax, (name, res) in zip(axes.flat, scans):
            draw_scan(ax, res, args.offset, show_raw=show_raw)
            if args.zoom:
                ax.set_xlim(-args.zoom, args.zoom)
                ax.set_ylim(-args.zoom, args.zoom)
            style_axes(ax, args.offset, name)
            ax.legend(loc="upper right", fontsize=7)

    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=140, bbox_inches="tight")
        print(f"saved {args.out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
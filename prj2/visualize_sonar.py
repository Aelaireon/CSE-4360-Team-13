#!/usr/bin/env python3
"""
sonar_viewer.py - replay the robot's CandleSearch pipeline on logged scan data.

Reads a .txt file containing pasted serial output of the form:

    Distance buffer: [349, 349, 353, ...]
    Theta buffer: [0, 0.00214, 0.00429, ...]

Distances are in mm, angles in radians. The logged distances ALREADY include the
robot's +80 mm ultrasonic offset (it is added before buffering), so --offset
defaults to 0 and no_return defaults to 2500 + 80, exactly like the robot code.

Port of CandleSearch._findSmallestSegmentTarget (candle_search.py):
  1. consecutive-duplicate-theta removal (keeps the FIRST reading)
  2. wide-beam minimum erosion (linear window, no wrap-around, scan order kept)
  3. sequential breakpoint segmentation: a new segment starts when the jump between
     consecutive points exceeds max(gap_mm, k_spread * r * d_theta); the last and
     first segments are joined across the 0 / 2*pi seam if they touch
  4. smallest VALID segment (enough points, optional width limits) -> centroid
     -> (target_theta, target_dist)

Usage:
    python sonar_viewer.py scan.txt
    python sonar_viewer.py scan.txt --beam 40 --tol 27 --gap 70 --min-samples 12
    python sonar_viewer.py scan.txt --min-width 30 --max-width 250
    python sonar_viewer.py scan.txt --zoom 900
    python sonar_viewer.py scan.txt --out map.png --no-show
    python sonar_viewer.py a.txt b.txt c.txt --overlay
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
    beam_deg=40.0,
    tol_mm=27.0,
    gap_mm=70.0,
    min_samples=12,
    no_return=NO_RETURN_DEFAULT,
    k_spread=3.0,
    ref_range_mm=500.0,
    min_samples_floor=3,
    min_width_mm=0.0,
    max_width_mm=1.0e9,
):
    """
    Returns a dict:
      unique:   [(t, d), ...]   after consecutive-theta dedupe
      pts:      [(x, y, r, t), ...] points surviving erosion, in scan order
      segments: list of dicts (idx, n, cx, cy, rc, need, width, radius, status)
                status: 'ok' | 'few' (too few points) | 'width' (width limit)
      target:   index into segments of the chosen one, or None
      result:   (theta, dist) or None   (what the robot would return)
      d_theta:  median angular step (rad)
    """
    # 1. consecutive dedupe
    unique_t, unique_d = [], []
    last_t = None
    for d, t in zip(distance_buffer, theta_buffer):
        if d is None or t is None or d < 0 or d >= no_return:
            continue
        if t != last_t:
            unique_t.append(t)
            unique_d.append(d)
            last_t = t

    out = {"unique": list(zip(unique_t, unique_d)), "pts": [], "segments": [],
           "target": None, "result": None, "d_theta": None}
    n = len(unique_t)
    if n == 0:
        return out

    if n > 1:
        diffs = [abs(unique_t[i] - unique_t[i - 1]) for i in range(1, n)
                 if unique_t[i] != unique_t[i - 1]]
        d_theta = sorted(diffs)[len(diffs) // 2] if diffs else math.radians(1.0)
    else:
        d_theta = math.radians(1.0)
    out["d_theta"] = d_theta

    # 2. wide-beam minimum erosion, O(N) monotonic queue, scan order kept
    half_rad = math.radians(beam_deg) * 0.5
    px, py, pr, pt = [], [], [], []
    q = []
    head = 0
    hi = 0
    for i in range(n):
        t_center = unique_t[i]
        upper = t_center + half_rad
        lower = t_center - half_rad

        while hi < n and unique_t[hi] <= upper:
            d_hi = unique_d[hi]
            while len(q) > head and unique_d[q[-1]] >= d_hi:
                q.pop()
            q.append(hi)
            hi += 1

        while head < len(q) and unique_t[q[head]] < lower:
            head += 1

        d_i = unique_d[i]
        local_min = unique_d[q[head]] if head < len(q) else d_i
        if d_i <= local_min + tol_mm:
            px.append(d_i * math.cos(t_center))
            py.append(d_i * math.sin(t_center))
            pr.append(d_i)
            pt.append(t_center)

    m = len(px)
    out["pts"] = list(zip(px, py, pr, pt))
    if m == 0:
        return out

    # 3. sequential breakpoint segmentation
    seg_idx = []          # list of lists of point indices
    cur = [0]
    for i in range(1, m):
        r = pr[i] if pr[i] > pr[i - 1] else pr[i - 1]
        thr = k_spread * r * d_theta
        if thr < gap_mm:
            thr = gap_mm
        dx = px[i] - px[i - 1]
        dy = py[i] - py[i - 1]
        if dx * dx + dy * dy <= thr * thr:
            cur.append(i)
        else:
            seg_idx.append(cur)
            cur = [i]
    seg_idx.append(cur)

    # wrap-around join of last and first segment
    if len(seg_idx) > 1:
        wrap_gap = pt[0] + 2.0 * math.pi - pt[m - 1]
        r = pr[0] if pr[0] > pr[m - 1] else pr[m - 1]
        thr = k_spread * r * d_theta
        if thr < gap_mm:
            thr = gap_mm
        dx = px[0] - px[m - 1]
        dy = py[0] - py[m - 1]
        if wrap_gap <= 10.0 * d_theta and dx * dx + dy * dy <= thr * thr:
            last = seg_idx.pop()
            seg_idx[0] = last + seg_idx[0]   # runs last_f ... (wrap) ... first_l

    # 4. smallest valid segment -> centroid
    best = None
    best_n = 0
    segments = []
    for idxs in seg_idx:
        cnt = len(idxs)
        cx = sum(px[i] for i in idxs) / cnt
        cy = sum(py[i] for i in idxs) / cnt
        rc = math.sqrt(cx * cx + cy * cy)
        need = int(round(min_samples * (ref_range_mm / max(ref_range_mm, rc))))
        if need < min_samples_floor:
            need = min_samples_floor
        wx = px[idxs[-1]] - px[idxs[0]]
        wy = py[idxs[-1]] - py[idxs[0]]
        width = math.sqrt(wx * wx + wy * wy)
        radius = max(math.hypot(px[i] - cx, py[i] - cy) for i in idxs)
        if cnt < need:
            status = "few"
        elif width < min_width_mm or width > max_width_mm:
            status = "width"
        else:
            status = "ok"
        segments.append({"idx": idxs, "n": cnt, "cx": cx, "cy": cy, "rc": rc,
                         "need": need, "width": width, "radius": radius, "status": status})
        if status == "ok" and (best is None or cnt < best_n):
            best = len(segments) - 1
            best_n = cnt

    out["segments"] = segments
    out["target"] = best
    if best is not None:
        s = segments[best]
        out["result"] = (math.atan2(s["cy"], s["cx"]), math.sqrt(s["cx"] ** 2 + s["cy"] ** 2))
    return out


# --------------------------------------------------------------------------- #
# Reporting / drawing
# --------------------------------------------------------------------------- #
STATUS_TEXT = {"ok": "valid", "few": "too few points", "width": "width limit"}


def summarize(name, raw_d, res):
    uniq = res["unique"]
    print(f"{name}: {len(raw_d)} raw -> {len(uniq)} after dedupe/range filter "
          f"-> {len(res['pts'])} after erosion")
    if uniq:
        ds = [d for _, d in uniq]
        print(f"  distance min/median/max: {min(ds):.0f} / {float(np.median(ds)):.0f} / {max(ds):.0f} mm"
              f"   (median d_theta {math.degrees(res['d_theta']):.3f} deg)")
    segs = res["segments"]
    print(f"  segments: {len(segs)}")
    for k, s in enumerate(segs):
        mark = "  <== TARGET" if k == res["target"] else ""
        print(f"    seg {k + 1}: {s['n']:4d} pts (need {s['need']:2d}), width {s['width']:5.0f} mm, "
              f"centroid ({s['cx']:.0f}, {s['cy']:.0f}) r={s['rc']:.0f} mm, "
              f"theta {math.degrees(math.atan2(s['cy'], s['cx'])):6.1f} deg  "
              f"[{STATUS_TEXT[s['status']]}]{mark}")
    if res["result"]:
        th, dist = res["result"]
        print(f"  candle candidate: dist {dist:.0f} mm, theta {math.degrees(th):.1f} deg")
    else:
        print("  no valid candle segment found (robot would clear buffers and rescan)")


def _shift(x, y, r, offset):
    """Optionally push points radially outward by an extra offset."""
    if not offset:
        return x, y
    s = (r + offset) / r if r > 0 else 1.0
    return x * s, y * s


def draw_scan(ax, res, offset, label=None, color=None, size=10, show_raw=True):
    cmap = plt.cm.tab10.colors
    pts = res["pts"]

    # Raw (post-dedupe, pre-erosion) readings, faint
    if show_raw and res["unique"]:
        rx = [d * math.cos(t) for t, d in res["unique"]]
        ry = [d * math.sin(t) for t, d in res["unique"]]
        ax.scatter(rx, ry, color="lightgray", s=max(size * 0.5, 3), zorder=1,
                   label="removed by erosion" if color is None else None)

    for k, s in enumerate(res["segments"]):
        xs, ys = zip(*[_shift(pts[i][0], pts[i][1], pts[i][2], offset) for i in s["idx"]])
        if color is not None:
            if s["status"] == "ok":
                ax.scatter(xs, ys, color=color, s=size, alpha=0.7, zorder=3,
                           label=label if k == 0 else None)
            continue
        if s["status"] == "ok":
            ax.scatter(xs, ys, color=cmap[k % 10], s=size, zorder=3,
                       label=f"seg {k + 1} ({s['n']} pts, {s['width']:.0f} mm)")
        else:
            ax.scatter(xs, ys, color="gray", marker="x", s=size, zorder=2,
                       label=f"seg {k + 1} rejected: {STATUS_TEXT[s['status']]} ({s['n']} pts)")

    # Target (smallest valid segment centroid) and heading the robot turns to
    if res["target"] is not None:
        s = res["segments"][res["target"]]
        th, dist = res["result"]
        ax.add_patch(plt.Circle((s["cx"], s["cy"]), s["radius"] + 35, fill=False, ec="red",
                                lw=2, ls="--", zorder=5,
                                label=f"target {dist:.0f} mm @ {math.degrees(th):.1f}°"))
        ax.scatter([s["cx"]], [s["cy"]], c="red", marker="+", s=80, zorder=5)
        reach = max(dist, 1) * 1.15
        ax.plot([0, reach * math.cos(th)], [0, reach * math.sin(th)],
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
    p.add_argument("--beam", type=float, default=40.0,
                   help="wide-beam erosion angular window in degrees (default 40)")
    p.add_argument("--tol", type=float, default=27.0,
                   help="local-minimum distance tolerance in mm (default 27)")
    p.add_argument("--gap", type=float, default=70.0,
                   help="max jump between consecutive points within one segment, mm (default 70)")
    p.add_argument("--min-samples", type=int, default=12,
                   help="min points per segment at the reference range (default 12)")
    p.add_argument("--k-spread", type=float, default=3.0,
                   help="gap range-scaling factor: thr = max(gap, k * r * d_theta) (default 3.0)")
    p.add_argument("--ref-range", type=float, default=500.0,
                   help="range in mm at which min_samples applies in full (default 500)")
    p.add_argument("--min-samples-floor", type=int, default=3,
                   help="lowest min_samples after range scaling (default 3)")
    p.add_argument("--min-width", type=float, default=0.0,
                   help="reject segments narrower than this many mm (default 0 = off)")
    p.add_argument("--max-width", type=float, default=1.0e9,
                   help="reject segments wider than this many mm (default off)")
    p.add_argument("--zoom", type=float, default=None,
                   help="add a near-field panel showing +/- this many mm")
    p.add_argument("--drop", type=float, nargs=2, metavar=("MIN", "MAX"), default=None,
                   help="drop raw readings between MIN and MAX mm (e.g. --drop 400 410)")
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

        if args.drop:
            keep = [not (dv is not None and args.drop[0] <= dv <= args.drop[1]) for dv in d]
            print(f"[{Path(f).name}] dropped {keep.count(False)} readings in "
                  f"{args.drop[0]:.0f}-{args.drop[1]:.0f} mm", file=sys.stderr)
            d = [v for v, k in zip(d, keep) if k]
            t = [v for v, k in zip(t, keep) if k]

        res = find_candle(
            d, t,
            beam_deg=args.beam, tol_mm=args.tol, gap_mm=args.gap,
            min_samples=args.min_samples, no_return=args.no_return,
            k_spread=args.k_spread, ref_range_mm=args.ref_range,
            min_samples_floor=args.min_samples_floor,
            min_width_mm=args.min_width, max_width_mm=args.max_width,
        )
        summarize(Path(f).name, d, res)
        scans.append((Path(f).name, res))

    title_suffix = (f" ({args.beam:g}° erosion ±{args.tol:g}mm + segments gap≥{args.gap:g}, "
                    f"n≥{args.min_samples})")
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
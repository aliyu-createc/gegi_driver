#!/usr/bin/env python
"""Position / heterogeneity sensitivity (DJR Assay Experiment E).

Quantifies how the reported activity varies with source position within the tray
-> the geometry/heterogeneity uncertainty term (usually the dominant one). Pure
offline analysis of activity CSVs. Works with Python 2.7 or 3.

  python tools/position_check.py \
      --runs center:c.csv corner:a.csv corner:b.csv corner:d.csv corner:e.csv \
      --nuclide Co60 [--tray 0.35 --standoff 0.33 --crystal-radius 0.045]

Labels may repeat (e.g. 4 'corner' runs). The 'center' label (if present) is the
reference; otherwise the max-activity run is used. If --tray and --standoff are
given, the geometric center/corner prediction is shown for comparison.
"""
from __future__ import print_function

import argparse
import csv
import math
from collections import defaultdict


def base(label):
    return label.split('_')[0]


def median(xs):
    ys = sorted(xs); n = len(ys)
    return 0.0 if n == 0 else (ys[n // 2] if n % 2 else 0.5 * (ys[n // 2 - 1] + ys[n // 2]))


def run_activity(path, want, drop=0.75):
    """Combined activity (mean of that nuclide's lines) for one run, partial
    windows dropped (net < drop*median)."""
    per_line = defaultdict(list)
    for row in csv.DictReader(open(path)):
        iso = row['isotope']
        if base(iso) != want:
            continue
        valid = str(row.get('valid', '')).strip().lower() in ('true', '1')
        try:
            a = float(row['activity_MBq']); n = float(row['net_corrected'])
        except (ValueError, KeyError):
            continue
        if valid and a > 0:
            per_line[iso].append((a, n))
    line_means = []
    for iso, rows in per_line.items():
        med = median([n for _, n in rows])
        kept = [a for a, n in rows if med <= 0 or n >= drop * med]
        if kept:
            line_means.append(sum(kept) / len(kept))
    return sum(line_means) / len(line_means) if line_means else None


def mean_sd(xs):
    n = len(xs)
    if n == 0:
        return 0.0, 0.0
    mu = sum(xs) / n
    if n == 1:
        return mu, 0.0
    return mu, math.sqrt(sum((x - mu) ** 2 for x in xs) / (n - 1))


def omega(d, r):
    return 0.5 * (1.0 - d / math.sqrt(d * d + r * r)) if d > 0 else 0.0


def main():
    p = argparse.ArgumentParser(description="GeGI position sensitivity (Exp E)")
    p.add_argument("--runs", nargs='+', required=True, help="label:csv pairs")
    p.add_argument("--nuclide", default="Co60")
    p.add_argument("--tray", type=float, default=None, help="tray side length (m)")
    p.add_argument("--standoff", type=float, default=0.33, help="on-axis standoff (m)")
    p.add_argument("--crystal-radius", type=float, default=0.045)
    args = p.parse_args()
    want = base(args.nuclide)

    runs = []
    for item in args.runs:
        label, path = item.split(':', 1)
        a = run_activity(path, want)
        if a is not None:
            runs.append((label.strip(), path.strip(), a))

    if not runs:
        raise SystemExit("No valid activity found.")

    centers = [a for lbl, _, a in runs if lbl.lower() == 'center']
    ref = centers[0] if centers else max(a for _, _, a in runs)
    ref_name = 'center' if centers else 'max'

    print("=" * 70)
    print("GeGI position sensitivity  (DJR Assay Experiment E)  nuclide=%s" % want)
    print("=" * 70)
    print("%-10s %12s %12s" % ("position", "activity_MBq", "vs %s" % ref_name))
    for lbl, _, a in runs:
        print("%-10s %12.4f %11.1f%%" % (lbl, a, 100.0 * a / ref))

    allvals = [a for _, _, a in runs]
    mu, sd = mean_sd(allvals)
    print("-" * 70)
    print("across all %d positions : mean %.4f MBq, SD %.4f (%.1f%% RSD)"
          % (len(allvals), mu, sd, 100.0 * sd / mu if mu else 0))
    corners = [a for lbl, _, a in runs if lbl.lower() == 'corner']
    if centers and corners:
        cmu, csd = mean_sd(corners)
        print("center=%.4f  corner mean=%.4f (%.1f%% of center, spread %.1f%%)"
              % (ref, cmu, 100.0 * cmu / ref, 100.0 * csd / cmu if cmu else 0))
        print("=> a corner source reads %.0f%% LOW vs center (uncorrected)."
              % (100.0 * (1 - cmu / ref)))

    if args.tray and centers and corners:
        half_diag = (args.tray / 2.0) * math.sqrt(2.0)
        d_corner = math.sqrt(args.standoff ** 2 + half_diag ** 2)
        ang = math.degrees(math.atan2(half_diag, args.standoff))
        # inverse-square-only and solid-angle predictions of corner/center
        inv_sq = (args.standoff / d_corner) ** 2
        sa = omega(d_corner, args.crystal_radius) / omega(args.standoff, args.crystal_radius)
        print("-" * 70)
        print("geometry: corner offset %.3f m, true dist %.3f m, angle %.0f deg"
              % (half_diag, d_corner, ang))
        print("predicted corner/center: inverse-square %.1f%%, solid-angle %.1f%%"
              % (100 * inv_sq, 100 * sa))
    print("=" * 70)


if __name__ == '__main__':
    main()

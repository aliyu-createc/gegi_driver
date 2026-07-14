#!/usr/bin/env python
"""Repeatability analysis (DJR Assay Experiment D).

Aggregates the per-run reported activity across a set of repeat measurements to
give the Type A (repeatability) uncertainty, and compares the mean to a
certificate. Works with Python 2.7 or 3.

Partial/ramp-up counting windows are excluded: the pipeline labels the first
window of a run with a full 60 s live time even when it only captured a few
seconds, so it reads spuriously low. Any window whose net counts fall below
--drop-threshold x the run-median (default 0.75) is dropped and reported.

Example:
  python tools/repeatability.py --dir "data/Experiment D" \
      --nuclide Co60 --cert-activity 1.130 --cert-date 2026-06-01
"""
from __future__ import print_function

import argparse
import csv
import glob
import math
import os
from datetime import datetime

HALF_LIFE_DAYS = {
    "Cs137": 30.08 * 365.25,
    "Co60":  5.2711 * 365.25,
    "Co57":  271.74,
    "Ba133": 10.551 * 365.25,
    "Eu152": 13.517 * 365.25,
    "Na22":  2.6018 * 365.25,
    "Mn54":  312.20,
    "Am241": 432.6 * 365.25,
    "Cd109": 461.9,
}


def base_nuclide(label):
    return label.split("_")[0]


def median(xs):
    ys = sorted(xs)
    n = len(ys)
    if n == 0:
        return 0.0
    m = n // 2
    return ys[m] if n % 2 else 0.5 * (ys[m - 1] + ys[m])


def mean_sd(xs):
    n = len(xs)
    if n == 0:
        return 0.0, 0.0
    mu = sum(xs) / n
    if n == 1:
        return mu, 0.0
    var = sum((x - mu) ** 2 for x in xs) / (n - 1)
    return mu, math.sqrt(var)


def run_line_activity(csv_path, want_base, drop_thr):
    """Return {line_label: mean_activity} for one run, dropping partial windows."""
    per_line = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            label = (row.get("isotope") or "").strip()
            if not label or base_nuclide(label) != want_base:
                continue
            try:
                act = float(row.get("activity_MBq", "0") or 0)
                net = float(row.get("net_corrected", "0") or 0)
            except ValueError:
                continue
            valid = str(row.get("valid", "")).strip().lower() in ("true", "1")
            if valid and act > 0:
                per_line.setdefault(label, []).append({"act": act, "net": net})
    out = {}
    dropped = {}
    for label, rows in per_line.items():
        med = median([r["net"] for r in rows])
        kept = [r for r in rows if med <= 0 or r["net"] >= drop_thr * med]
        dropped[label] = len(rows) - len(kept)
        if kept:
            out[label] = sum(r["act"] for r in kept) / len(kept)
    return out, dropped


def main():
    p = argparse.ArgumentParser(description="GeGI repeatability (Exp D)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--dir", help="folder of *_activity.csv repeat runs")
    g.add_argument("--csvs", nargs="+", help="explicit list of activity CSVs")
    p.add_argument("--nuclide", required=True, help="base nuclide, e.g. Co60")
    p.add_argument("--cert-activity", type=float, default=None,
                   help="certificate activity in MBq (optional)")
    p.add_argument("--cert-date", default=None, help="certificate date YYYY-MM-DD")
    p.add_argument("--meas-date", default=None, help="measurement date (default today)")
    p.add_argument("--drop-threshold", type=float, default=0.75,
                   help="drop windows with net < this x run-median (default 0.75)")
    args = p.parse_args()

    want = base_nuclide(args.nuclide)
    if args.dir:
        files = sorted(glob.glob(os.path.join(args.dir, "*activity*.csv")))
    else:
        files = args.csvs
    if not files:
        raise SystemExit("No activity CSVs found.")

    a_ref = None
    if args.cert_activity is not None and args.cert_date:
        t_half = HALF_LIFE_DAYS[want]
        cert_date = datetime.strptime(args.cert_date, "%Y-%m-%d")
        meas_date = (datetime.strptime(args.meas_date, "%Y-%m-%d")
                     if args.meas_date else datetime.now())
        dt_days = (meas_date - cert_date).total_seconds() / 86400.0
        a_ref = args.cert_activity * math.exp(-math.log(2.0) * dt_days / t_half)

    print("=" * 72)
    print("GeGI repeatability analysis  (DJR Assay Experiment D)")
    print("=" * 72)
    print("Nuclide : %s     runs : %d     partial-window cut : net < %.2f x median"
          % (want, len(files), args.drop_threshold))
    if a_ref is not None:
        print("Reference (decay-corrected) : %.5g MBq" % a_ref)
    print("-" * 72)

    line_series = {}    # label -> [per-run activity]
    combined = []       # per-run mean-of-lines
    total_dropped = 0
    hdr = "%-28s %12s %12s %12s" % ("run", "1173", "1332", "combined")
    print(hdr)
    for path in files:
        acts, dropped = run_line_activity(path, want, args.drop_threshold)
        total_dropped += sum(dropped.values())
        if not acts:
            print("%-28s   (no valid rows)" % os.path.basename(path)[:28])
            continue
        for label, a in acts.items():
            line_series.setdefault(label, []).append(a)
        comb = sum(acts.values()) / len(acts)
        combined.append(comb)
        labels = sorted(acts)
        v1173 = "%.4f" % acts.get(want + "_1173", float("nan")) if any("1173" in l for l in labels) else "-"
        v1332 = "%.4f" % acts.get(want + "_1332", float("nan")) if any("1332" in l for l in labels) else "-"
        # generic single-line nuclides (e.g. Cs137) just show combined
        if want + "_1173" not in acts and want + "_1332" not in acts:
            v1173 = v1332 = "-"
        print("%-28s %12s %12s %12.4f"
              % (os.path.basename(path)[:28], v1173, v1332, comb))

    print("-" * 72)
    for label in sorted(line_series):
        mu, sd = mean_sd(line_series[label])
        rsd = 100.0 * sd / mu if mu else 0.0
        line = "%-11s : mean %.5g MBq  SD %.4g  RSD %.2f%%  (n=%d)" % (
            label, mu, sd, rsd, len(line_series[label]))
        if a_ref:
            line += "  bias %+.1f%%" % ((mu / a_ref - 1) * 100)
        print(line)

    if combined:
        mu, sd = mean_sd(combined)
        rsd = 100.0 * sd / mu if mu else 0.0
        sem = sd / math.sqrt(len(combined)) if combined else 0.0
        print("-" * 72)
        print("COMBINED per-run activity:")
        print("  mean        : %.5g MBq" % mu)
        print("  SD (Type A) : %.4g MBq   ->  repeatability RSD = %.2f%%" % (sd, rsd))
        print("  SEM         : %.4g MBq   (SD/sqrt(%d))" % (sem, len(combined)))
        print("  range       : %.5g - %.5g MBq" % (min(combined), max(combined)))
        if a_ref:
            print("  accuracy    : mean bias %+.1f%% vs certificate" %
                  ((mu / a_ref - 1) * 100))
    if total_dropped:
        print("  (%d partial/ramp windows excluded across all runs)" % total_dropped)
    print("=" * 72)


if __name__ == "__main__":
    main()

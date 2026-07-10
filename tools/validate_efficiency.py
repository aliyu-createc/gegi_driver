#!/usr/bin/env python
"""Efficiency validation (DJR Assay Experiment B).

Reads an assay activity CSV produced by the data_recorder, decay-corrects the
source certificate activity to the measurement date, and compares the GeGI's
reported activity against it -> bias %, pass/fail. Works with Python 2.7 or 3.

Example:
  python tools/validate_efficiency.py \
      --csv data/GEGI_20260709_120000_activity.csv \
      --nuclide Cs137 --cert-activity 0.370 --cert-date 2015-06-01 \
      --meas-date 2026-07-09 --tolerance 10

For Co-60 both photopeak lines (1173 & 1332 keV) are compared to the same
certificate activity and to each other (an energy-dependent split hints at an
efficiency-shape or mu problem rather than a simple geometry offset).
"""
from __future__ import print_function

import argparse
import csv
import math
from datetime import datetime

# Half-lives in DAYS (from DDEP / NNDC evaluated data). Add sources as needed.
HALF_LIFE_DAYS = {
    "Cs137": 30.08 * 365.25,     # 30.08 y
    "Co60":  5.2711 * 365.25,    # 5.2711 y
    "Co57":  271.74,
    "Ba133": 10.551 * 365.25,
    "Eu152": 13.517 * 365.25,
    "Na22":  2.6018 * 365.25,
    "Mn54":  312.20,
    "Am241": 432.6 * 365.25,
    "Cd109": 461.9,
}


def base_nuclide(label):
    """'Co60_1332' -> 'Co60', 'Cs137' -> 'Cs137'."""
    return label.split("_")[0]


def decay_factor(nuclide, cert_date, meas_date):
    """exp(-ln2 * dt / T_half) from certificate date to measurement date."""
    t_half = HALF_LIFE_DAYS.get(base_nuclide(nuclide))
    if t_half is None:
        raise SystemExit("Unknown nuclide '%s' - add its half-life to "
                         "HALF_LIFE_DAYS." % nuclide)
    dt_days = (meas_date - cert_date).total_seconds() / 86400.0
    return math.exp(-math.log(2.0) * dt_days / t_half), dt_days


def read_rows(csv_path, want_base):
    """Return the valid activity rows for the requested base nuclide, keyed by
    the per-line label (so Co-60 yields its 1173 and 1332 rows separately)."""
    lines = {}
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            label = (row.get("isotope") or "").strip()
            if not label or base_nuclide(label) != want_base:
                continue
            try:
                act = float(row.get("activity_MBq", "0") or 0)
                sig = float(row.get("sigma_activity_MBq", "0") or 0)
                net = float(row.get("net_corrected", "0") or 0)
                lt = float(row.get("live_time_s", "0") or 0)
                dt = float(row.get("detector_run_dead_time_percent", "0") or 0)
            except ValueError:
                continue
            valid = str(row.get("valid", "")).strip().lower() in ("true", "1")
            lines.setdefault(label, []).append(
                {"act": act, "sig": sig, "net": net, "lt": lt, "dt": dt,
                 "valid": valid})
    return lines


def summarise(samples):
    """Mean activity over valid rows (fallback: all rows), with spread."""
    vals = [s for s in samples if s["valid"] and s["act"] > 0]
    if not vals:
        vals = [s for s in samples if s["act"] > 0]
    if not vals:
        return None
    acts = [s["act"] for s in vals]
    mean = sum(acts) / len(acts)
    if len(acts) > 1:
        var = sum((a - mean) ** 2 for a in acts) / (len(acts) - 1)
        sd = math.sqrt(var)
    else:
        sd = vals[0]["sig"]
    return {
        "mean": mean, "sd": sd, "n": len(acts),
        "net": sum(s["net"] for s in vals) / len(vals),
        "lt": sum(s["lt"] for s in vals) / len(vals),
        "dt": sum(s["dt"] for s in vals) / len(vals),
    }


def main():
    p = argparse.ArgumentParser(description="GeGI efficiency validation (Exp B)")
    p.add_argument("--csv", required=True, help="assay *_activity.csv from a run")
    p.add_argument("--nuclide", required=True,
                   help="base nuclide, e.g. Cs137 or Co60")
    p.add_argument("--cert-activity", type=float, required=True,
                   help="certificate reference activity in MBq")
    p.add_argument("--cert-date", required=True,
                   help="certificate reference date, YYYY-MM-DD")
    p.add_argument("--meas-date", default=None,
                   help="measurement date YYYY-MM-DD (default: today)")
    p.add_argument("--tolerance", type=float, default=10.0,
                   help="pass/fail band in percent (default 10)")
    args = p.parse_args()

    want = base_nuclide(args.nuclide)
    cert_date = datetime.strptime(args.cert_date, "%Y-%m-%d")
    if args.meas_date:
        meas_date = datetime.strptime(args.meas_date, "%Y-%m-%d")
    else:
        meas_date = datetime.now()

    f_decay, dt_days = decay_factor(want, cert_date, meas_date)
    a_ref = args.cert_activity * f_decay

    print("=" * 64)
    print("GeGI efficiency validation  (DJR Assay Experiment B)")
    print("=" * 64)
    print("Nuclide            : %s" % want)
    print("Certificate        : %.6g MBq on %s" %
          (args.cert_activity, args.cert_date))
    print("Elapsed / half-life: %.1f d  (T1/2 = %.1f d)" %
          (dt_days, HALF_LIFE_DAYS[want]))
    print("Decay factor       : %.5f" % f_decay)
    print("Reference activity @ %s : %.6g MBq  <-- truth" %
          (meas_date.strftime("%Y-%m-%d"), a_ref))
    print("-" * 64)

    lines = read_rows(args.csv, want)
    if not lines:
        raise SystemExit("No %s rows found in %s" % (want, args.csv))

    all_means = []
    for label in sorted(lines):
        s = summarise(lines[label])
        if s is None:
            print("%-11s : no valid activity rows" % label)
            continue
        bias = s["mean"] / a_ref if a_ref > 0 else float("nan")
        pct = (bias - 1.0) * 100.0
        verdict = "PASS" if abs(pct) <= args.tolerance else "FAIL"
        all_means.append(s["mean"])
        print("%-11s : measured %.6g MBq (n=%d, SD %.2g)  "
              "net=%.0f  live=%.0fs  DT=%.1f%%"
              % (label, s["mean"], s["n"], s["sd"], s["net"], s["lt"], s["dt"]))
        print("%-11s   bias %+.1f%%  -> %s (tol +-%.0f%%)"
              % ("", pct, verdict, args.tolerance))

    if len(all_means) > 1:
        combo = sum(all_means) / len(all_means)
        pct = (combo / a_ref - 1.0) * 100.0
        spread = (max(all_means) - min(all_means)) / combo * 100.0
        print("-" * 64)
        print("Combined (mean of lines): %.6g MBq  bias %+.1f%%  "
              "line-to-line spread %.1f%%" % (combo, pct, spread))
        if spread > args.tolerance:
            print("  NOTE: line-to-line spread exceeds tolerance -> suspect "
                  "efficiency shape / mu, not a simple geometry offset.")
    print("=" * 64)


if __name__ == "__main__":
    main()

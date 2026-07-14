#!/usr/bin/env python
"""Limits of detection (DJR Assay Experiment A).

Computes Currie critical level, detection limit (MDA) and a quantification
limit per nuclide, from a source-free background activity CSV plus the
efficiency/geometry in isotopes.yaml. Works with Python 2.7 or 3.

  python tools/mda.py --csv "data/Experiment A/<bg>_activity.csv" \
      --config config/isotopes.yaml --count-time 300 [--plates 0]

Method (Currie 1968, 95% confidence, counts):
  B    = background counts in the peak ROI over the assay count time T
  L_C  = 2.33*sqrt(B)                    (critical level)
  L_D  = 2.71 + 4.65*sqrt(B)            (detection limit)
  MDA  = L_D / (eps_abs * I_gamma * T * T_shield)      [Bq]
The efficiency chain mirrors the activity node: eps_intrinsic is derived from
the calibration_factor, scaled by the operational solid angle and shielding.
"""
from __future__ import print_function

import argparse
import csv
import math
from collections import defaultdict

import yaml


def solid_angle_fraction(d, r):
    if d <= 0:
        return 0.0
    return 0.5 * (1.0 - d / math.sqrt(d * d + r * r))


def base(label):
    return label.split('_')[0]


def currie_limits(background_counts):
    """Currie (1968) critical level and detection limit, in COUNTS.

      L_C = 2.33*sqrt(B)          (decide detected / not detected)
      L_D = 2.71 + 4.65*sqrt(B)   (detection limit; 2.71 floor at zero background)

    Pure function so the detection-limit maths is unit-testable.
    """
    b = max(0.0, background_counts)
    root = math.sqrt(b)
    return 2.33 * root, 2.71 + 4.65 * root


def main():
    p = argparse.ArgumentParser(description="GeGI limits of detection (Exp A)")
    p.add_argument("--csv", required=True, help="source-free background activity CSV")
    p.add_argument("--config", default="config/isotopes.yaml")
    p.add_argument("--count-time", type=float, default=300.0,
                   help="assay live time in seconds for the quoted MDA")
    p.add_argument("--plates", type=int, default=0,
                   help="EXTRA shielding plates for the operational config "
                        "(base_shield_plates from the yaml is added automatically)")
    args = p.parse_args()

    cfg = yaml.safe_load(open(args.config))
    isos = cfg['isotopes']
    cal_d = cfg.get('calibration_distance_m', 0.5)
    r = cfg.get('crystal_radius_m', 0.045)
    shield = cfg.get('shielding', {}) or {}
    t_plate = shield.get('plate_thickness_m', 0.0) or 0.0
    base_stand = shield.get('base_standoff_m', cfg.get('source_distance_m', 0.5))
    base_plates = int(shield.get('base_shield_plates', 0) or 0)
    total_plates = base_plates + args.plates
    d_op = base_stand + total_plates * t_plate
    omega_cal = solid_angle_fraction(cal_d, r)
    omega_op = solid_angle_fraction(d_op, r)

    # background counts per peak ROI from the source-free run
    gross = defaultdict(float)
    live = defaultdict(float)
    for row in csv.DictReader(open(args.csv)):
        iso = row['isotope']
        try:
            gross[iso] += float(row['gross_counts'])
            live[iso] += float(row['live_time_s'])
        except (ValueError, KeyError):
            continue

    T = args.count_time
    print("=" * 74)
    print("GeGI limits of detection  (DJR Assay Experiment A)")
    print("=" * 74)
    print("Background CSV : %s" % args.csv)
    print("Geometry      : d=%.3f m (base %.3f + %d plate(s) x %.3f), "
          "count time T=%.0f s" % (d_op, base_stand, total_plates, t_plate, T))
    print("Solid angle   : Omega_op/4pi=%.6f  (cal %.6f at %.2f m)"
          % (omega_op, omega_cal, cal_d))
    print("-" * 74)
    hdr = "%-11s %9s %8s %8s %8s %12s %14s"
    print(hdr % ("line", "bg_cps", "B(T)", "L_C", "L_D", "MDA", "LoQ(400ct)"))
    for name in sorted(isos):
        c = isos[name]
        CF = c.get('calibration_factor', 0.0) or 0.0
        I = c['emission_probability']
        mu = c.get('mu_shield_per_m', 0.0) or 0.0
        if CF <= 0 or omega_cal <= 0:
            continue
        eps_int = 1.0 / (CF * omega_cal * I)
        eps_abs = eps_int * omega_op
        t_shield = math.exp(-mu * total_plates * t_plate) if mu > 0 else 1.0
        bg_cps = gross[name] / live[name] if live.get(name) else 0.0
        B = bg_cps * T
        L_C, L_D = currie_limits(B)
        denom = eps_abs * I * T * t_shield
        mda_bq = L_D / denom if denom > 0 else float('inf')
        loq_bq = 400.0 / denom if denom > 0 else float('inf')   # 400 net counts
        print(hdr % (name, "%.5f" % bg_cps, "%.1f" % B, "%.1f" % L_C,
                     "%.1f" % L_D,
                     "%.4g Bq" % mda_bq,
                     "%.4g Bq" % loq_bq))
    print("-" * 74)
    print("MDA = detection limit (Currie 95%). LoQ(400ct) = activity giving the")
    print("driver's 400-net-count validity gate (~5%% counting precision).")
    print("=" * 74)


if __name__ == '__main__':
    main()

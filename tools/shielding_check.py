#!/usr/bin/env python
"""Shielding transmission check (DJR Assay Experiment F).

Confirms the steel linear-attenuation coefficients (mu_shield_per_m) in
isotopes.yaml by comparing the measured net-count-rate ratio between plate
configurations against the model exp(-mu * n * t). Pure offline analysis of
activity CSVs - does NOT touch the driver. Works with Python 2.7 or 3.

Take one run per plate count with the SAME source, then:
  python tools/shielding_check.py --config config/isotopes.yaml \
      --runs 0:"data/Experiment F/p0.csv" 1:"data/Experiment F/p1.csv" \
             2:"data/Experiment F/p2.csv" 3:"data/Experiment F/p3.csv"

The number before each ':' is the count of EXTRA plates inserted for that run
(0 = baseline). By default the source is assumed to stay at a FIXED distance
(plates inserted into the gap); pass --moving-source if each plate pushed the
source away (then the solid-angle change is divided out using the yaml geometry).
"""
from __future__ import print_function

import argparse
import csv
import math
from collections import defaultdict


def solid_angle_fraction(d, r):
    if d <= 0:
        return 0.0
    return 0.5 * (1.0 - d / math.sqrt(d * d + r * r))


def net_rate_per_line(path):
    """Sum net_corrected / live_time over valid rows -> net cps per line."""
    net = defaultdict(float)
    live = defaultdict(float)
    for row in csv.DictReader(open(path)):
        iso = row['isotope']
        valid = str(row.get('valid', '')).strip().lower() in ('true', '1')
        try:
            n = float(row['net_corrected'])
            lt = float(row['live_time_s'])
        except (ValueError, KeyError):
            continue
        if valid and n > 0:
            net[iso] += n
            live[iso] += lt
    return {k: net[k] / live[k] for k in net if live[k] > 0}


def linfit(xs, ys):
    """Least-squares slope/intercept of ys = a + b*xs (b returned)."""
    n = len(xs)
    if n < 2:
        return 0.0
    sx = sum(xs); sy = sum(ys)
    sxx = sum(x * x for x in xs); sxy = sum(x * y for x, y in zip(xs, ys))
    denom = n * sxx - sx * sx
    if denom == 0:
        return 0.0
    return (n * sxy - sx * sy) / denom


def main():
    import yaml
    p = argparse.ArgumentParser(description="GeGI shielding mu check (Exp F)")
    p.add_argument("--config", default="config/isotopes.yaml")
    p.add_argument("--runs", nargs='+', required=True,
                   help='EXTRA_plates:csv pairs, e.g. 0:p0.csv 1:p1.csv')
    p.add_argument("--moving-source", action='store_true',
                   help="each plate pushed the source away (divide out solid angle)")
    args = p.parse_args()

    cfg = yaml.safe_load(open(args.config))
    isos = cfg['isotopes']
    r = cfg.get('crystal_radius_m', 0.045)
    shield = cfg.get('shielding', {}) or {}
    t = shield.get('plate_thickness_m', 0.005) or 0.005
    base_stand = shield.get('base_standoff_m', cfg.get('source_distance_m', 0.5))
    base_plates = int(shield.get('base_shield_plates', 0) or 0)

    runs = []
    for item in args.runs:
        k, path = item.split(':', 1)
        runs.append((int(k), path.strip().strip('"')))
    runs.sort()
    if len(runs) < 2:
        raise SystemExit("Need at least 2 plate configurations.")
    base_extra = runs[0][0]

    rates = {}   # extra_plates -> {line: net_cps}
    for extra, path in runs:
        rates[extra] = net_rate_per_line(path)

    print("=" * 74)
    print("GeGI shielding transmission check  (DJR Assay Experiment F)")
    print("=" * 74)
    print("plate thickness t=%.4f m, base_shield_plates=%d, baseline=%d extra"
          % (t, base_plates, base_extra))
    print("source assumed %s"
          % ("MOVING (solid angle divided out)" if args.moving_source
             else "FIXED distance (plates inserted in gap)"))

    for line in sorted(set().union(*[set(v) for v in rates.values()])):
        mu_cfg = isos.get(line, {}).get('mu_shield_per_m', 0.0) or 0.0
        print("-" * 74)
        print("%s   (config mu = %.1f /m)" % (line, mu_cfg))
        print("  %-7s %11s %13s %13s %9s" %
              ("extra", "net_cps", "T_measured", "T_model", "ratio"))
        base_rate = rates[base_extra].get(line)
        xs, ys = [], []
        for extra, _ in runs:
            rate = rates[extra].get(line)
            if not rate or not base_rate:
                continue
            n_rel = extra - base_extra
            geom = 1.0
            if args.moving_source:
                d0 = base_stand + (base_plates + base_extra) * t
                dn = base_stand + (base_plates + extra) * t
                geom = solid_angle_fraction(d0, r) / solid_angle_fraction(dn, r)
            t_meas = (rate / base_rate) * geom
            t_model = math.exp(-mu_cfg * n_rel * t)
            ratio = t_meas / t_model if t_model > 0 else float('nan')
            print("  %-7d %11.3f %13.4f %13.4f %9.3f" %
                  (extra, rate, t_meas, t_model, ratio))
            # for the mu fit: ln(geom-corrected rate) vs plate count
            xs.append(n_rel)
            ys.append(math.log(rate * (geom if args.moving_source else 1.0)))
        if len(xs) >= 2:
            slope = linfit(xs, ys)
            mu_fit = -slope / t if t > 0 else 0.0
            diff = (mu_fit / mu_cfg - 1.0) * 100.0 if mu_cfg else float('nan')
            print("  fitted mu = %.1f /m   (config %.1f /m, %+.1f%%)"
                  % (mu_fit, mu_cfg, diff))
    print("=" * 74)
    print("T_measured = net-rate ratio vs baseline; T_model = exp(-mu*n*t).")
    print("ratio near 1.0 and fitted mu near config mu => mu values confirmed.")
    print("=" * 74)


if __name__ == '__main__':
    main()

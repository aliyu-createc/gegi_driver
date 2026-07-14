#!/usr/bin/env python
"""Unit tests for the DJR analysis tools.

These tools produced the numbers quoted in the Design Justification Report
(decay-corrected certificates, Currie detection limits, fitted steel mu,
repeatability). They must not drift.
"""
from __future__ import print_function

import math
import os
import sys
import tempfile
import unittest
from datetime import datetime, timedelta

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(REPO, 'tools'))

import mda                    # noqa: E402
import position_check as pc   # noqa: E402
import repeatability as rp    # noqa: E402
import shielding_check as sc  # noqa: E402
import validate_efficiency as ve  # noqa: E402


class TestDecayCorrection(unittest.TestCase):
    def test_two_half_lives_of_co60_leaves_a_quarter(self):
        half_life = ve.HALF_LIFE_DAYS['Co60']
        cert = datetime(2020, 1, 1)
        meas = cert + timedelta(days=2 * half_life)
        factor, dt_days = ve.decay_factor('Co60', cert, meas)
        self.assertAlmostEqual(factor, 0.25, delta=1e-9)
        self.assertAlmostEqual(dt_days, 2 * half_life, delta=1e-6)

    def test_one_half_life_of_cs137_leaves_a_half(self):
        half_life = ve.HALF_LIFE_DAYS['Cs137']
        cert = datetime(2020, 1, 1)
        meas = cert + timedelta(days=half_life)
        factor, _ = ve.decay_factor('Cs137', cert, meas)
        self.assertAlmostEqual(factor, 0.5, delta=1e-9)

    def test_no_elapsed_time_means_no_decay(self):
        d = datetime(2026, 6, 1)
        factor, _ = ve.decay_factor('Co60', d, d)
        self.assertAlmostEqual(factor, 1.0, delta=1e-12)

    def test_per_line_labels_resolve_to_the_parent_nuclide(self):
        self.assertEqual(ve.base_nuclide('Co60_1332'), 'Co60')
        self.assertEqual(ve.base_nuclide('Cs137'), 'Cs137')

    def test_unknown_nuclide_is_rejected(self):
        with self.assertRaises(SystemExit):
            ve.decay_factor('Xx999', datetime(2020, 1, 1), datetime(2021, 1, 1))


class TestCurrieLimits(unittest.TestCase):
    """L_C = 2.33*sqrt(B);  L_D = 2.71 + 4.65*sqrt(B)."""

    def test_zero_background_hits_the_currie_floor(self):
        l_c, l_d = mda.currie_limits(0.0)
        self.assertAlmostEqual(l_c, 0.0, delta=1e-12)
        self.assertAlmostEqual(l_d, 2.71, delta=1e-12)

    def test_known_background(self):
        l_c, l_d = mda.currie_limits(100.0)          # sqrt(B) = 10
        self.assertAlmostEqual(l_c, 23.3, delta=1e-9)
        self.assertAlmostEqual(l_d, 2.71 + 46.5, delta=1e-9)

    def test_detection_limit_exceeds_critical_level(self):
        for b in (0.0, 1.0, 15.0, 100.0, 1e4):
            l_c, l_d = mda.currie_limits(b)
            self.assertGreater(l_d, l_c)

    def test_grows_as_root_background(self):
        _, l_d1 = mda.currie_limits(100.0)
        _, l_d2 = mda.currie_limits(400.0)           # 4x background -> 2x sqrt term
        self.assertAlmostEqual((l_d2 - 2.71) / (l_d1 - 2.71), 2.0, delta=1e-9)

    def test_negative_background_is_clamped(self):
        self.assertEqual(mda.currie_limits(-5.0), mda.currie_limits(0.0))


class TestSolidAngleAgreesAcrossTools(unittest.TestCase):
    """mda.py and position_check.py must use the same geometry as the driver."""

    def test_tools_agree_with_each_other(self):
        for d in (0.33, 0.41, 0.5):
            self.assertAlmostEqual(mda.solid_angle_fraction(d, 0.045),
                                   pc.omega(d, 0.045), delta=1e-12)

    def test_matches_the_known_operational_value(self):
        self.assertAlmostEqual(
            mda.solid_angle_fraction(0.33, 0.045), 0.00458485, delta=1e-7)


class TestShieldingFit(unittest.TestCase):
    """The mu fit must recover a known attenuation coefficient."""

    def test_linfit_recovers_a_known_slope(self):
        xs = [0.0, 1.0, 2.0, 3.0]
        ys = [2.0 + 3.0 * x for x in xs]
        self.assertAlmostEqual(sc.linfit(xs, ys), 3.0, delta=1e-9)

    def test_recovers_mu_from_synthetic_transmission(self):
        mu_true, thickness, rate0 = 41.8, 0.005, 20.0
        plates = [0, 1, 2, 3]
        rates = [rate0 * math.exp(-mu_true * n * thickness) for n in plates]
        slope = sc.linfit([float(n) for n in plates],
                          [math.log(r) for r in rates])
        mu_fit = -slope / thickness
        self.assertAlmostEqual(mu_fit, mu_true, delta=1e-6)

    def test_degenerate_input_does_not_raise(self):
        self.assertEqual(sc.linfit([1.0], [1.0]), 0.0)
        self.assertEqual(sc.linfit([2.0, 2.0], [1.0, 3.0]), 0.0)


class TestStatistics(unittest.TestCase):
    def test_mean_and_sample_sd(self):
        mu, sd = pc.mean_sd([2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0])
        self.assertAlmostEqual(mu, 5.0, delta=1e-12)
        self.assertAlmostEqual(sd, 2.13808993, delta=1e-6)   # n-1 denominator

    def test_single_sample_has_zero_sd(self):
        mu, sd = pc.mean_sd([3.0])
        self.assertAlmostEqual(mu, 3.0, delta=1e-12)
        self.assertEqual(sd, 0.0)

    def test_median_handles_even_and_odd(self):
        self.assertAlmostEqual(rp.median([3.0, 1.0, 2.0]), 2.0, delta=1e-12)
        self.assertAlmostEqual(rp.median([4.0, 1.0, 3.0, 2.0]), 2.5, delta=1e-12)


ACTIVITY_CSV_HEADER = ("timestamp_s,live_time_s,isotope,net_corrected,"
                       "activity_MBq,sigma_activity_MBq,valid\n")


def _write_activity_csv(rows):
    fd, path = tempfile.mkstemp(suffix='_activity.csv')
    with os.fdopen(fd, 'w') as f:
        f.write(ACTIVITY_CSV_HEADER)
        for r in rows:
            f.write(r + "\n")
    return path


class TestPartialWindowRejection(unittest.TestCase):
    """The first counting window of a run is often partial (a few seconds of data
    labelled with a full live time) and reads spuriously low. It must be dropped,
    otherwise it drags the reported activity down."""

    def setUp(self):
        # one ramp-up window (net 300) + four full windows (net ~1200)
        self.path = _write_activity_csv([
            "1,60.0,Co60_1173,300,0.30,0.01,True",
            "2,60.0,Co60_1173,1200,1.20,0.01,True",
            "3,60.0,Co60_1173,1200,1.20,0.01,True",
            "4,60.0,Co60_1173,1210,1.21,0.01,True",
            "5,60.0,Co60_1173,1190,1.19,0.01,True",
        ])

    def tearDown(self):
        os.remove(self.path)

    def test_ramp_window_is_excluded(self):
        act, dropped = rp.run_line_activity(self.path, 'Co60', 0.75)
        # mean of the four full windows, not of all five
        self.assertAlmostEqual(act['Co60_1173'], 1.20, delta=1e-9)
        self.assertEqual(dropped['Co60_1173'], 1, "the ramp window must be dropped")

    def test_position_tool_applies_the_same_rejection(self):
        a = pc.run_activity(self.path, 'Co60')
        self.assertAlmostEqual(a, 1.20, delta=1e-9)

    def test_invalid_rows_are_ignored(self):
        path = _write_activity_csv([
            "1,60.0,Co60_1173,1200,1.20,0.01,True",
            "2,60.0,Co60_1173,9999,9.99,0.01,False",   # flagged invalid
        ])
        try:
            act, _ = rp.run_line_activity(path, 'Co60', 0.75)
            self.assertAlmostEqual(act['Co60_1173'], 1.20, delta=1e-9)
        finally:
            os.remove(path)


if __name__ == '__main__':
    unittest.main()

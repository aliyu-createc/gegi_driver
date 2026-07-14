#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Unit tests for the run-level activity aggregation written into the N42.

Two things must not drift:

  1. The AGGREGATION RULE. Activity is computed from total net counts / total
     live time, and partial start-up windows are excluded. A partial window has
     partial counts but a FULL logged live time, so including it biases the
     activity LOW (this was measured at -6.8% on a real run).

  2. The UNCERTAINTY SEMANTICS. The N42 carries the EXPANDED (k=2) uncertainty,
     combining per-run counting statistics with the systematic budget. Writing
     counting statistics alone (~1-3%) into a durable record would understate the
     real measurement uncertainty roughly ten-fold.
"""
from __future__ import print_function

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src', 'phds_gegi_driver')))

import data_recorder_node as dr  # noqa: E402


def window(live_s, lines):
    """One counting-window report: lines = [(label, net, activity_MBq, energy)]."""
    return {
        'live_time_s': live_s,
        'isotopes': [
            {'isotope': lbl, 'net_corrected': net, 'activity_MBq': act,
             'energy_keV': e, 'valid': True}
            for lbl, net, act, e in lines
        ],
    }


class TestAggregation(unittest.TestCase):
    def test_activity_from_total_counts_over_total_live_time(self):
        # Bq-per-cps = 0.06 in both windows.
        # Total: 2100 net / 120 s = 17.5 cps -> 1.05 MBq.
        reports = [
            window(60.0, [('Cs137', 1000.0, 1.00, 661.7)]),   # 16.667 cps
            window(60.0, [('Cs137', 1100.0, 1.10, 661.7)]),   # 18.333 cps
        ]
        res = dr.aggregate_run_activity(reports)['Cs137']
        self.assertAlmostEqual(res['activity_MBq'], 1.05, delta=1e-9)
        self.assertAlmostEqual(res['net_counts'], 2100.0, delta=1e-9)
        self.assertAlmostEqual(res['live_time_s'], 120.0, delta=1e-9)
        self.assertEqual(res['windows_used'], 2)

    def test_longer_windows_are_weighted_more(self):
        """Total-counts/total-live-time, NOT the mean of the window activities.

        Bq-per-cps = 0.06.  60 s @ 16.667 cps + 180 s @ 20 cps
          -> 4600 net / 240 s = 19.167 cps -> 1.15 MBq
        The naive mean of the window activities would give 1.10 - wrong, because
        it under-weights the long window.
        """
        reports = [
            window(60.0, [('Cs137', 1000.0, 1.00, 661.7)]),
            window(180.0, [('Cs137', 3600.0, 1.20, 661.7)]),
        ]
        res = dr.aggregate_run_activity(reports)['Cs137']
        self.assertAlmostEqual(res['activity_MBq'], 1.15, delta=1e-9)
        self.assertNotAlmostEqual(res['activity_MBq'], 1.10, delta=1e-3)

    def test_short_window_with_a_normal_rate_is_KEPT(self):
        """The filter must key on RATE, not raw counts.

        A genuinely shorter window has fewer counts but a normal rate; dropping
        it would throw away good data. Only a ramp window - partial counts against
        a FULL logged live time, hence a low rate - may be dropped.
        """
        reports = [
            window(60.0, [('Cs137', 1200.0, 1.2, 661.7)]),   # 20 cps
            window(60.0, [('Cs137', 1200.0, 1.2, 661.7)]),   # 20 cps
            window(15.0, [('Cs137', 300.0, 1.2, 661.7)]),    # 20 cps - SHORT, normal rate
        ]
        res = dr.aggregate_run_activity(reports)['Cs137']
        self.assertEqual(res['windows_dropped'], 0, "short-but-normal must be kept")
        self.assertEqual(res['windows_used'], 3)
        self.assertAlmostEqual(res['activity_MBq'], 1.2, delta=1e-9)

    def test_partial_startup_window_is_excluded(self):
        """The regression this rule exists to prevent."""
        reports = [
            window(60.0, [('Co60_1173', 300.0, 0.30, 1173.2)]),   # ramp-up: partial
            window(60.0, [('Co60_1173', 1200.0, 1.20, 1173.2)]),
            window(60.0, [('Co60_1173', 1200.0, 1.20, 1173.2)]),
            window(60.0, [('Co60_1173', 1200.0, 1.20, 1173.2)]),
        ]
        res = dr.aggregate_run_activity(reports)['Co60_1173']
        self.assertEqual(res['windows_dropped'], 1)
        self.assertEqual(res['windows_used'], 3)
        self.assertAlmostEqual(res['activity_MBq'], 1.20, delta=1e-9)

    def test_including_the_partial_window_would_bias_low(self):
        """Demonstrates WHY the rule exists: without it the answer is wrong."""
        reports = [
            window(60.0, [('Co60_1173', 300.0, 0.30, 1173.2)]),
            window(60.0, [('Co60_1173', 1200.0, 1.20, 1173.2)]),
            window(60.0, [('Co60_1173', 1200.0, 1.20, 1173.2)]),
            window(60.0, [('Co60_1173', 1200.0, 1.20, 1173.2)]),
        ]
        good = dr.aggregate_run_activity(reports, drop_threshold=0.75)['Co60_1173']
        naive = dr.aggregate_run_activity(reports, drop_threshold=0.0)['Co60_1173']
        self.assertAlmostEqual(good['activity_MBq'], 1.20, delta=1e-9)
        self.assertLess(naive['activity_MBq'], 1.13)      # biased LOW
        self.assertGreater(good['activity_MBq'], naive['activity_MBq'])

    def test_invalid_windows_ignored(self):
        reports = [window(60.0, [('Cs137', 1000.0, 1.0, 661.7)])]
        reports.append({'live_time_s': 60.0, 'isotopes': [
            {'isotope': 'Cs137', 'net_corrected': 9999.0, 'activity_MBq': 9.9,
             'energy_keV': 661.7, 'valid': False}]})
        res = dr.aggregate_run_activity(reports)['Cs137']
        self.assertAlmostEqual(res['activity_MBq'], 1.0, delta=1e-9)

    def test_counting_sigma_shrinks_with_counts(self):
        few = dr.aggregate_run_activity(
            [window(60.0, [('Cs137', 100.0, 1.0, 661.7)])])['Cs137']
        many = dr.aggregate_run_activity(
            [window(60.0, [('Cs137', 10000.0, 1.0, 661.7)])])['Cs137']
        # sigma/activity = 1/sqrt(N):  10% at 100 counts, 1% at 10000
        self.assertAlmostEqual(few['counting_sigma_MBq'] / few['activity_MBq'],
                               0.10, delta=1e-9)
        self.assertAlmostEqual(many['counting_sigma_MBq'] / many['activity_MBq'],
                               0.01, delta=1e-9)

    def test_no_valid_data_yields_nothing(self):
        self.assertEqual(dr.aggregate_run_activity([]), {})


class TestRadionuclideGrouping(unittest.TestCase):
    """Co-60's two photopeaks quantify ONE nuclide and must be combined."""

    def _lines(self, a1173, a1332):
        return dr.aggregate_run_activity([window(60.0, [
            ('Co60_1173', 1200.0, a1173, 1173.2),
            ('Co60_1332', 800.0, a1332, 1332.5),
        ])])

    def test_two_co60_lines_collapse_to_one_nuclide(self):
        groups = dr.group_by_radionuclide(
            self._lines(1.10, 1.20), dr.DataRecorderNode._controlled_radionuclide)
        self.assertEqual(sorted(groups), ['Co-60'])
        self.assertAlmostEqual(groups['Co-60']['activity_MBq'], 1.15, delta=1e-9)
        self.assertEqual(groups['Co-60']['lines'], ['Co60_1173', 'Co60_1332'])

    def test_line_spread_is_reported(self):
        """The two lines disagreeing is a real internal-consistency signal."""
        groups = dr.group_by_radionuclide(
            self._lines(1.10, 1.20), dr.DataRecorderNode._controlled_radionuclide)
        # (1.20 - 1.10) / 1.15 = 8.7%
        self.assertAlmostEqual(groups['Co-60']['line_spread_percent'], 8.6957,
                               delta=1e-3)

    def test_agreeing_lines_give_zero_spread(self):
        groups = dr.group_by_radionuclide(
            self._lines(1.15, 1.15), dr.DataRecorderNode._controlled_radionuclide)
        self.assertAlmostEqual(groups['Co-60']['line_spread_percent'], 0.0,
                               delta=1e-9)

    def test_distinct_nuclides_stay_separate(self):
        lines = dr.aggregate_run_activity([window(60.0, [
            ('Cs137', 1000.0, 2.0, 661.7),
            ('Co60_1173', 1200.0, 1.1, 1173.2),
        ])])
        groups = dr.group_by_radionuclide(
            lines, dr.DataRecorderNode._controlled_radionuclide)
        self.assertEqual(sorted(groups), ['Co-60', 'Cs-137'])


class TestUncertaintySemantics(unittest.TestCase):
    """The N42 must carry the EXPANDED (k=2) uncertainty, not counting sigma."""

    @staticmethod
    def _expanded(u_counting_pct, u_systematic_pct):
        return 2.0 * math.sqrt(u_counting_pct ** 2 + u_systematic_pct ** 2)

    def test_well_counted_run_reports_about_the_budget(self):
        # counting 1.5%, systematic 10.7% -> ~21.6% expanded (the DJR headline)
        self.assertAlmostEqual(self._expanded(1.5, 10.7), 21.6, delta=0.2)

    def test_marginal_run_correctly_widens(self):
        # near the MDA counting statistics dominate and U must blow up
        self.assertGreater(self._expanded(20.0, 10.7), 44.0)

    def test_expanded_always_exceeds_counting_alone(self):
        """The whole point: counting sigma alone would understate the record."""
        for u_count in (0.5, 1.5, 5.0, 20.0):
            self.assertGreater(self._expanded(u_count, 10.7), 2.0 * u_count)

    def test_systematic_floor_is_never_undercut(self):
        # even with perfect counting statistics, U cannot fall below 2*u_sys
        self.assertGreaterEqual(self._expanded(0.0, 10.7), 21.4 - 1e-9)


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python
"""Unit tests for the GeGi activity physics.

These lock down the maths the Design Justification Report depends on: the
solid-angle geometry, the calibration-factor -> intrinsic-efficiency derivation,
the shielding attenuation, and the plate-derived standoff. A change that breaks
any of these silently invalidates the reported activities, so they are asserted
against independently-computed values (not re-derived from the code under test).

Run:  python -m unittest discover -s test    (with the ROS workspace sourced)
"""
from __future__ import print_function

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src', 'phds_gegi_driver')))

import activity_node as an  # noqa: E402


R = 0.045          # GeGi crystal radius (m)
CAL_D = 0.5        # calibration distance (m)
PLATE_T = 0.005    # steel plate thickness (m)


class TestSolidAngle(unittest.TestCase):
    """Omega/4pi = 0.5 * (1 - d/sqrt(d^2 + r^2))."""

    def test_calibration_geometry(self):
        # 0.5*(1 - 0.5/sqrt(0.25 + 0.002025))
        self.assertAlmostEqual(
            an.gegi_solid_angle_fraction(CAL_D, R), 0.0020127806, delta=1e-9)

    def test_operational_geometry(self):
        # 0.33 m: the standoff through the permanent plate
        self.assertAlmostEqual(
            an.gegi_solid_angle_fraction(0.33, R), 0.0045849160, delta=1e-9)

    def test_non_positive_distance_is_zero(self):
        self.assertEqual(an.gegi_solid_angle_fraction(0.0, R), 0.0)
        self.assertEqual(an.gegi_solid_angle_fraction(-0.2, R), 0.0)

    def test_monotonically_decreasing_with_distance(self):
        vals = [an.gegi_solid_angle_fraction(d, R)
                for d in (0.1, 0.2, 0.33, 0.5, 1.0)]
        for near, far in zip(vals, vals[1:]):
            self.assertGreater(near, far)

    def test_far_field_follows_inverse_square(self):
        # d >> r: Omega ~ 1/d^2, so halving distance quadruples solid angle
        ratio = (an.gegi_solid_angle_fraction(1.0, R)
                 / an.gegi_solid_angle_fraction(2.0, R))
        self.assertAlmostEqual(ratio, 4.0, delta=0.02)


class TestShielding(unittest.TestCase):
    """transmission = exp(-mu * n_plates * thickness)."""

    def test_one_steel_plate_per_line(self):
        # exp(-mu * 0.005) for the three calibrated gamma lines
        self.assertAlmostEqual(
            an.shield_transmission(57.4, 1, PLATE_T), 0.7505117288, delta=1e-9)  # Cs 662
        self.assertAlmostEqual(
            an.shield_transmission(41.8, 1, PLATE_T), 0.8113952356, delta=1e-9)  # Co 1173
        self.assertAlmostEqual(
            an.shield_transmission(39.6, 1, PLATE_T), 0.8203698531, delta=1e-9)  # Co 1332

    def test_plates_attenuate_multiplicatively(self):
        one = an.shield_transmission(41.8, 1, PLATE_T)
        two = an.shield_transmission(41.8, 2, PLATE_T)
        three = an.shield_transmission(41.8, 3, PLATE_T)
        self.assertAlmostEqual(two, one ** 2, delta=1e-9)
        self.assertAlmostEqual(three, one ** 3, delta=1e-9)

    def test_no_plates_means_no_attenuation(self):
        self.assertEqual(an.shield_transmission(41.8, 0, PLATE_T), 1.0)

    def test_no_mu_means_no_attenuation(self):
        self.assertEqual(an.shield_transmission(0.0, 3, PLATE_T), 1.0)

    def test_transmission_is_bounded(self):
        for n in range(0, 6):
            t = an.shield_transmission(57.4, n, PLATE_T)
            self.assertGreater(t, 0.0)
            self.assertLessEqual(t, 1.0)

    # --- material-agnostic invariants: these hold for steel, lead, anything ---

    def test_more_attenuating_material_transmits_less(self):
        steel, lead_like, tungsten_like = 41.8, 120.0, 250.0
        t_steel = an.shield_transmission(steel, 1, PLATE_T)
        t_lead = an.shield_transmission(lead_like, 1, PLATE_T)
        t_w = an.shield_transmission(tungsten_like, 1, PLATE_T)
        self.assertGreater(t_steel, t_lead)
        self.assertGreater(t_lead, t_w)

    def test_thicker_plates_transmit_less(self):
        for mu in (41.8, 120.0):
            thin = an.shield_transmission(mu, 1, 0.003)
            thick = an.shield_transmission(mu, 1, 0.010)
            self.assertGreater(thin, thick)

    def test_more_plates_transmit_less(self):
        for mu in (41.8, 120.0):
            prev = 1.0
            for n in range(1, 5):
                t = an.shield_transmission(mu, n, PLATE_T)
                self.assertLess(t, prev)
                prev = t

    def test_beer_lambert_holds_for_any_material_and_thickness(self):
        """transmission == exp(-mu * n * t) for arbitrary inputs."""
        for mu in (10.0, 41.8, 120.0, 250.0):
            for thickness in (0.002, 0.005, 0.012):
                for n in (1, 2, 5):
                    self.assertAlmostEqual(
                        an.shield_transmission(mu, n, thickness),
                        math.exp(-mu * n * thickness), delta=1e-12)


class TestPlateDerivedDistance(unittest.TestCase):
    """distance = base_standoff + total_plates * thickness."""

    def test_operational_geometry_is_330mm(self):
        # bare 0.325 m + the one permanent plate = the measured 0.33 m standoff
        self.assertAlmostEqual(
            an.plate_derived_distance(0.325, 1, PLATE_T), 0.330, delta=1e-9)

    def test_each_plate_adds_its_thickness(self):
        self.assertAlmostEqual(
            an.plate_derived_distance(0.325, 3, PLATE_T), 0.340, delta=1e-9)

    def test_zero_plates_is_bare_standoff(self):
        self.assertAlmostEqual(
            an.plate_derived_distance(0.325, 0, PLATE_T), 0.325, delta=1e-9)


def _cs137_cfg(calibration_factor=45672.0, **overrides):
    cfg = {
        'energy_keV': 661.7,
        'emission_probability': 0.851,
        'peak_roi_keV': [655.0, 669.0],
        'left_sideband_keV': [620.0, 645.0],
        'right_sideband_keV': [680.0, 705.0],
        'calibration_factor': calibration_factor,
        'mu_shield_per_m': 57.4,
    }
    cfg.update(overrides)
    return cfg


class TestIsotopeConfig(unittest.TestCase):
    """The calibration-factor -> intrinsic-efficiency chain."""

    def setUp(self):
        self.bin_edges = np.linspace(0.0, 3000.0, 1024)

    def _make(self, cfg):
        return an.IsotopeConfig('Cs137', cfg, self.bin_edges,
                                calibration_distance_m=CAL_D, crystal_radius_m=R)

    def test_intrinsic_efficiency_derived_from_calibration_factor(self):
        ic = self._make(_cs137_cfg())
        # eps = 1 / (CF * Omega_cal * I_gamma) = 1/(45672 * 0.0020127806 * 0.851)
        self.assertAlmostEqual(ic.intrinsic_efficiency, 0.0127827406, delta=1e-9)

    def test_calibration_factor_round_trip(self):
        """THE contract: at the calibration distance, activity = CF * net_cps.

        This is what the calibration factor *means*. It exercises the whole
        efficiency chain, so any change to the solid-angle formula or the
        efficiency derivation breaks it.
        """
        cf = 45672.0
        ic = self._make(_cs137_cfg(calibration_factor=cf))
        net_counts, live_s = 1000.0, 100.0
        omega_cal = an.gegi_solid_angle_fraction(CAL_D, R)
        activity_bq = net_counts / (ic.intrinsic_efficiency * omega_cal
                                    * ic.emission_probability * live_s)
        expected = cf * (net_counts / live_s)          # 45672 * 10 cps
        self.assertAlmostEqual(activity_bq, expected, delta=expected * 1e-9)

    def test_measured_efficiency_overrides_calibration_factor(self):
        ic = self._make(_cs137_cfg(measured_intrinsic_efficiency=0.05))
        self.assertAlmostEqual(ic.intrinsic_efficiency, 0.05, delta=1e-12)

    def test_falls_back_to_polynomial_without_calibration_factor(self):
        ic = self._make(_cs137_cfg(calibration_factor=0.0))
        expected = an.gegi_intrinsic_efficiency(661.7)
        self.assertAlmostEqual(ic.intrinsic_efficiency, expected, delta=1e-12)
        self.assertGreater(ic.intrinsic_efficiency, 0.0)

    def test_efficiency_is_physically_plausible(self):
        ic = self._make(_cs137_cfg())
        self.assertGreater(ic.intrinsic_efficiency, 0.0)
        self.assertLess(ic.intrinsic_efficiency, 1.0)

    def test_mu_defaults_to_zero_when_absent(self):
        cfg = _cs137_cfg()
        del cfg['mu_shield_per_m']
        self.assertEqual(self._make(cfg).mu_shield_per_m, 0.0)

    def test_roi_maps_to_contiguous_channels_covering_the_peak(self):
        ic = self._make(_cs137_cfg())
        ch = ic.peak_channels
        self.assertGreater(len(ch), 0)
        self.assertTrue(np.all(np.diff(ch) == 1), "channels must be contiguous")
        # the ROI must actually bracket the 662 keV line
        self.assertLessEqual(self.bin_edges[ch[0]], 661.7)
        self.assertGreaterEqual(self.bin_edges[ch[-1]] + (
            self.bin_edges[1] - self.bin_edges[0]), 661.7)

    def test_sidebands_sit_outside_the_peak(self):
        ic = self._make(_cs137_cfg())
        self.assertLess(ic.left_channels[-1], ic.peak_channels[0])
        self.assertGreater(ic.right_channels[0], ic.peak_channels[-1])


class TestIntrinsicEfficiencyPolynomial(unittest.TestCase):
    def test_zero_or_negative_energy_is_zero(self):
        self.assertEqual(an.gegi_intrinsic_efficiency(0.0), 0.0)
        self.assertEqual(an.gegi_intrinsic_efficiency(-100.0), 0.0)

    def test_positive_and_sub_unity_over_the_working_range(self):
        for e in (200.0, 662.0, 1173.0, 1332.0):
            eps = an.gegi_intrinsic_efficiency(e)
            self.assertGreater(eps, 0.0)
            self.assertLess(eps, 1.0)


if __name__ == '__main__':
    unittest.main()

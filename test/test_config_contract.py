#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Contract tests for config/isotopes.yaml.

Two kinds of test here, kept deliberately separate:

  * INVARIANTS — must hold for ANY rig, ANY shielding material, ANY standoff.
    (structure, physical sanity, monotonicity). These never need editing.

  * COMMISSIONED SNAPSHOT — asserts the yaml still matches the rig as built.
    These read test/commissioned.py; if the rig changes, edit THAT file only.

Every reported activity, MDA and maximum-activity figure derives from this yaml,
and a bad edit would corrupt the assay rather than crash — hence the snapshot.
"""
from __future__ import print_function

import os
import sys
import unittest

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'phds_gegi_driver'))
sys.path.insert(0, os.path.dirname(__file__))

import activity_node as an  # noqa: E402
import commissioned as C    # noqa: E402

CONFIG = os.path.join(REPO, 'config', 'isotopes.yaml')


def _load():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# INVARIANTS — independent of geometry, material and thickness
# ---------------------------------------------------------------------------
class TestStructuralInvariants(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = _load()

    def test_file_parses(self):
        self.assertIsInstance(self.cfg, dict)

    def test_required_top_level_keys(self):
        for key in ('gamma_constants', 'shielding', 'isotopes',
                    'crystal_radius_m', 'calibration_distance_m'):
            self.assertIn(key, self.cfg)

    def test_geometry_values_are_positive(self):
        self.assertGreater(self.cfg['crystal_radius_m'], 0.0)
        self.assertGreater(self.cfg['calibration_distance_m'], 0.0)

    def test_shielding_block_is_complete_and_positive(self):
        sh = self.cfg['shielding']
        self.assertGreater(sh['plate_thickness_m'], 0.0)
        self.assertGreater(sh['base_standoff_m'], 0.0)
        self.assertGreaterEqual(sh['base_shield_plates'], 0)
        self.assertTrue(sh.get('material'), "shielding material must be named")

    def test_gamma_constants_are_positive(self):
        for name, value in self.cfg['gamma_constants'].items():
            self.assertGreater(value, 0.0, name)

    def test_every_line_is_physically_sane(self):
        for name, iso in self.cfg['isotopes'].items():
            self.assertGreater(iso['energy_keV'], 0.0, name)
            self.assertGreater(iso['emission_probability'], 0.0, name)
            self.assertLessEqual(iso['emission_probability'], 1.0, name)
            self.assertGreater(iso['calibration_factor'], 0.0,
                               "%s: calibration_factor drives the activity" % name)
            self.assertGreater(iso['mu_shield_per_m'], 0.0,
                               "%s: mu drives the shielding correction" % name)

    def test_roi_brackets_the_photopeak(self):
        for name, iso in self.cfg['isotopes'].items():
            lo, hi = iso['peak_roi_keV']
            self.assertLess(lo, iso['energy_keV'], "%s: ROI low edge" % name)
            self.assertGreater(hi, iso['energy_keV'], "%s: ROI high edge" % name)

    def test_sidebands_flank_the_roi_without_overlapping(self):
        for name, iso in self.cfg['isotopes'].items():
            lo, hi = iso['peak_roi_keV']
            left, right = iso['left_sideband_keV'], iso['right_sideband_keV']
            self.assertLess(left[1], lo, "%s: left sideband overlaps ROI" % name)
            self.assertGreater(right[0], hi, "%s: right sideband overlaps ROI" % name)
            self.assertLess(left[0], left[1], name)
            self.assertLess(right[0], right[1], name)

    def test_derived_efficiencies_are_plausible(self):
        omega = an.gegi_solid_angle_fraction(
            self.cfg['calibration_distance_m'], self.cfg['crystal_radius_m'])
        for name, iso in self.cfg['isotopes'].items():
            eps = 1.0 / (iso['calibration_factor'] * omega
                         * iso['emission_probability'])
            self.assertGreater(eps, 1e-4, "%s: efficiency implausibly low" % name)
            self.assertLess(eps, 1.0, "%s: efficiency must be < 1" % name)

    def test_efficiency_falls_with_energy(self):
        """Thin HPGe: full-energy efficiency decreases with photon energy.
        True regardless of geometry."""
        omega = an.gegi_solid_angle_fraction(
            self.cfg['calibration_distance_m'], self.cfg['crystal_radius_m'])

        def eps(name):
            iso = self.cfg['isotopes'][name]
            return 1.0 / (iso['calibration_factor'] * omega
                          * iso['emission_probability'])

        by_energy = sorted(self.cfg['isotopes'],
                           key=lambda n: self.cfg['isotopes'][n]['energy_keV'])
        for lower, higher in zip(by_energy, by_energy[1:]):
            self.assertGreater(eps(lower), eps(higher),
                               "efficiency must fall from %s to %s" % (lower, higher))

    def test_mu_falls_with_energy(self):
        """Attenuation decreases with photon energy — true for ANY material."""
        isos = self.cfg['isotopes']
        by_energy = sorted(isos, key=lambda n: isos[n]['energy_keV'])
        for lower, higher in zip(by_energy, by_energy[1:]):
            self.assertGreater(isos[lower]['mu_shield_per_m'],
                               isos[higher]['mu_shield_per_m'],
                               "mu must fall from %s to %s" % (lower, higher))


# ---------------------------------------------------------------------------
# COMMISSIONED SNAPSHOT — if the rig changes, edit test/commissioned.py
# ---------------------------------------------------------------------------
class TestCommissionedConfiguration(unittest.TestCase):
    """These assert the yaml still describes the rig as built.

    A failure here means EITHER the yaml was edited by mistake, OR the rig
    genuinely changed and test/commissioned.py needs updating to match.
    """

    @classmethod
    def setUpClass(cls):
        cls.cfg = _load()

    def test_geometry_matches_commissioned(self):
        sh = self.cfg['shielding']
        self.assertAlmostEqual(sh['base_standoff_m'], C.BARE_STANDOFF_M, delta=1e-9)
        self.assertAlmostEqual(sh['plate_thickness_m'], C.PLATE_THICKNESS_M, delta=1e-9)
        self.assertEqual(sh['base_shield_plates'], C.BASE_SHIELD_PLATES)
        self.assertAlmostEqual(self.cfg['crystal_radius_m'], C.CRYSTAL_RADIUS_M,
                               delta=1e-9)
        self.assertAlmostEqual(self.cfg['calibration_distance_m'],
                               C.CALIBRATION_DISTANCE_M, delta=1e-9)

    def test_operational_standoff_matches_commissioned(self):
        sh = self.cfg['shielding']
        d = an.plate_derived_distance(
            sh['base_standoff_m'], sh['base_shield_plates'], sh['plate_thickness_m'])
        self.assertAlmostEqual(d, C.OPERATIONAL_STANDOFF_M, delta=1e-9)

    def test_shield_material_matches_commissioned(self):
        self.assertEqual(self.cfg['shielding']['material'], C.SHIELD_MATERIAL)

    def test_mu_values_match_commissioned(self):
        for line, mu in C.MU_PER_M.items():
            self.assertAlmostEqual(
                self.cfg['isotopes'][line]['mu_shield_per_m'], mu, delta=1e-9,
                msg="%s: mu changed — new shielding material/data?" % line)

    def test_expected_lines_present(self):
        for line in C.EXPECTED_LINES:
            self.assertIn(line, self.cfg['isotopes'])

    def test_gamma_constants_match_commissioned(self):
        for name, value in C.GAMMA_CONSTANTS.items():
            self.assertAlmostEqual(self.cfg['gamma_constants'][name], value,
                                   delta=1e-9, msg=name)


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python
"""Unit tests for the dose-rate gamma-constant helpers.

The constants were de-duplicated so that config/isotopes.yaml is the single
source of truth for both the data recorder and the heatmap. These tests assert
that both consumers load the same values and that the name-normalising lookup
tolerates every naming form in use ('Cs137', 'Cs-137', 'Co60_1173', ...).
"""
from __future__ import print_function

import os
import sys
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'phds_gegi_driver'))

import data_recorder_node as dr          # noqa: E402
import spherical_heatmap_node as hm      # noqa: E402

CONFIG = os.path.join(REPO, 'config', 'isotopes.yaml')

# Both modules carry an identical copy of the helpers; test them side by side so
# they cannot silently drift apart.
MODULES = (('data_recorder', dr), ('heatmap', hm))


class TestNormalisation(unittest.TestCase):
    def test_strips_case_and_punctuation(self):
        for _, mod in MODULES:
            self.assertEqual(mod._norm_iso('Cs-137'), 'CS137')
            self.assertEqual(mod._norm_iso('cs_137'), 'CS137')
            self.assertEqual(mod._norm_iso('Cs137'), 'CS137')
            self.assertEqual(mod._norm_iso('Co-60'), 'CO60')
            self.assertEqual(mod._norm_iso('Co60_1173'), 'CO601173')


class TestLoadFromYaml(unittest.TestCase):
    def test_loads_values_from_the_config(self):
        for name, mod in MODULES:
            table = mod.load_gamma_constants(CONFIG)
            self.assertAlmostEqual(table['CS137'], 0.0771, delta=1e-9,
                                   msg='%s Cs-137' % name)
            self.assertAlmostEqual(table['CO60'], 0.3059, delta=1e-9,
                                   msg='%s Co-60' % name)

    def test_both_consumers_agree(self):
        """The whole point of the de-duplication."""
        self.assertEqual(dr.load_gamma_constants(CONFIG),
                         hm.load_gamma_constants(CONFIG))

    def test_falls_back_to_defaults_when_file_missing(self):
        for name, mod in MODULES:
            table = mod.load_gamma_constants('/nonexistent/isotopes.yaml')
            # must not raise, and must still yield usable constants
            self.assertAlmostEqual(
                mod.gamma_constant_for(table, 'Cs-137'), 0.0771, delta=1e-9,
                msg='%s fallback' % name)


class TestTolerantLookup(unittest.TestCase):
    def setUp(self):
        self.tables = [(n, m, m.load_gamma_constants(CONFIG)) for n, m in MODULES]

    def test_all_naming_forms_resolve(self):
        cases = {
            'Cs-137': 0.0771, 'Cs137': 0.0771,
            'Co-60': 0.3059, 'Co60': 0.3059,
            'Co60_1173': 0.3059,     # per-line label falls back to the nuclide
            'Co60_1332': 0.3059,
        }
        for name, mod, table in self.tables:
            for label, expected in cases.items():
                self.assertAlmostEqual(
                    mod.gamma_constant_for(table, label), expected, delta=1e-9,
                    msg='%s: %s' % (name, label))

    def test_unknown_nuclide_returns_the_default(self):
        for name, mod, table in self.tables:
            self.assertAlmostEqual(
                mod.gamma_constant_for(table, 'Eu152', default=0.077),
                0.077, delta=1e-9, msg=name)


if __name__ == '__main__':
    unittest.main()

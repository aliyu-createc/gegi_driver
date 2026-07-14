#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Unit tests for the Compton-imaging heatmap.

Covers the pure imaging pieces: the Fibonacci sphere the back-projection is
evaluated on, and the spectral isotope identification that labels each detected
hotspot. These underpin the source-localisation results (~1 deg) reported in the
DJR, and the Co-60 / Cs-137 discrimination in mixed scenes.
"""
from __future__ import print_function

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src', 'phds_gegi_driver')))

import spherical_heatmap_node as hm  # noqa: E402


class TestFibonacciSphere(unittest.TestCase):
    """The directions the back-projection is scored on."""

    def test_returns_the_requested_number_of_points(self):
        for n in (10, 100, 2000):
            pts = hm.fibonacci_sphere(n)
            self.assertEqual(len(pts), n)

    def test_all_points_are_unit_vectors(self):
        pts = hm.fibonacci_sphere(500)
        norms = np.linalg.norm(pts, axis=1)
        self.assertTrue(np.allclose(norms, 1.0, atol=1e-9),
                        "every direction must be a unit vector")

    def test_points_are_three_dimensional(self):
        self.assertEqual(hm.fibonacci_sphere(50).shape, (50, 3))

    def test_points_are_distinct(self):
        pts = hm.fibonacci_sphere(200)
        # no two directions should coincide
        self.assertEqual(len(np.unique(np.round(pts, 6), axis=0)), 200)

    def test_covers_both_hemispheres(self):
        z = hm.fibonacci_sphere(1000)[:, 2]
        self.assertGreater(np.sum(z > 0), 100, "must cover +z")
        self.assertGreater(np.sum(z < 0), 100, "must cover -z")

    def test_distribution_is_roughly_uniform(self):
        """Mean of a uniform sphere sits at the origin."""
        pts = hm.fibonacci_sphere(4000)
        centroid = np.mean(pts, axis=0)
        self.assertTrue(np.allclose(centroid, 0.0, atol=0.05),
                        "centroid %s should be ~origin" % centroid)


class TestIsotopeIdentification(unittest.TestCase):
    """Spectral labelling of a hotspot's events."""

    def test_identifies_cs137_from_662kev_events(self):
        energies = np.full(50, 661.7)
        name, confidence = hm.identify_isotope(energies)
        self.assertEqual(name, 'Cs-137')
        self.assertGreater(confidence, 0.9)

    def test_identifies_co60_from_its_two_lines(self):
        energies = np.concatenate([np.full(25, 1173.2), np.full(25, 1332.5)])
        name, confidence = hm.identify_isotope(energies)
        self.assertEqual(name, 'Co-60')
        self.assertGreater(confidence, 0.9)

    def test_discriminates_between_the_two_nuclides(self):
        """The mixed-scene capability: Cs and Co must not be confused."""
        cs, _ = hm.identify_isotope(np.full(40, 661.7))
        co, _ = hm.identify_isotope(np.full(40, 1332.5))
        self.assertEqual(cs, 'Cs-137')
        self.assertEqual(co, 'Co-60')
        self.assertNotEqual(cs, co)

    def test_too_few_events_yields_no_identification(self):
        name, confidence = hm.identify_isotope(np.full(3, 661.7))
        self.assertIsNone(name)
        self.assertEqual(confidence, 0.0)

    def test_off_peak_continuum_is_not_identified(self):
        """Events far from any known line must not be labelled."""
        energies = np.full(50, 300.0)   # in no ROI window
        name, _ = hm.identify_isotope(energies)
        self.assertIsNone(name)

    def test_majority_nuclide_wins_in_a_mixture(self):
        energies = np.concatenate([np.full(45, 661.7), np.full(5, 1332.5)])
        name, _ = hm.identify_isotope(energies)
        self.assertEqual(name, 'Cs-137')

    def test_confidence_is_a_fraction(self):
        _, confidence = hm.identify_isotope(np.full(30, 661.7))
        self.assertGreaterEqual(confidence, 0.0)
        self.assertLessEqual(confidence, 1.0)


class TestIsotopePeakTable(unittest.TestCase):
    def test_known_lines_are_configured(self):
        self.assertIn('Cs-137', hm.ISOTOPE_PEAKS)
        self.assertIn('Co-60', hm.ISOTOPE_PEAKS)

    def test_windows_are_positive_and_do_not_overlap(self):
        cs = hm.ISOTOPE_PEAKS['Cs-137']
        co = hm.ISOTOPE_PEAKS['Co-60']
        for iso in (cs, co):
            self.assertGreater(iso['energy'], 0)
            self.assertGreater(iso['window'], 0)
        cs_hi = cs['energy'] + cs['window']
        co_lo = co['energy'] - co['window']
        self.assertLess(cs_hi, co_lo,
                        "Cs-137 and Co-60 identification windows must not overlap")


if __name__ == '__main__':
    unittest.main()

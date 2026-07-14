# -*- coding: utf-8 -*-
"""The CURRENTLY COMMISSIONED rig configuration.

>>> IF THE RIG CHANGES, EDIT THIS FILE — AND ONLY THIS FILE. <<<

A new standoff, a different shielding material, or a different plate thickness
changes these numbers. The *physics* tests are parameter-driven (they pass mu,
thickness and distance in as arguments) and are unaffected by any of it — only
the configuration-snapshot assertions in test_config_contract.py read this file.

Keeping the snapshot is deliberate: it means an accidental edit to
config/isotopes.yaml (a zeroed calibration factor, a mistyped standoff) fails a
test instead of silently corrupting every reported activity.
"""

# --- Geometry -------------------------------------------------------------
BARE_STANDOFF_M = 0.325       # source -> crystal face with ZERO plates
PLATE_THICKNESS_M = 0.005     # thickness of ONE plate (5 mm)
BASE_SHIELD_PLATES = 1        # permanently mounted plates, always in the beam
OPERATIONAL_STANDOFF_M = 0.330  # = BARE + BASE_SHIELD_PLATES * PLATE_THICKNESS

CRYSTAL_RADIUS_M = 0.045      # GeGi 90 mm diameter crystal
CALIBRATION_DISTANCE_M = 0.5  # where the calibration factors were measured

# --- Shielding ------------------------------------------------------------
SHIELD_MATERIAL = 'steel'
# Linear attenuation coefficient (1/m) per gamma line, for SHIELD_MATERIAL.
# Change the whole dict if the material changes.
MU_PER_M = {
    'Cs137': 57.4,        # 662 keV
    'Co60_1173': 41.8,    # 1173 keV
    'Co60_1332': 39.6,    # 1332 keV
}

# --- Nuclides expected in the assay --------------------------------------
EXPECTED_LINES = ('Cs137', 'Co60_1173', 'Co60_1332')

# --- Dose-rate gamma constants (uSv*m^2 / MBq*h) --------------------------
# Physical constants, not rig-dependent — they only change if a nuclide is added.
GAMMA_CONSTANTS = {'Cs137': 0.0771, 'Co60': 0.3059}

#!/usr/bin/env python
"""
Activity Node - Computes net peak area and activity (Bq) per isotope from
accumulated spectra.

Subscribes to /spectrum (radiation_detector_msgs/Spectrum) published by the
spectrum node, accumulates counts over a configurable counting window, then
computes per-isotope net peak areas using linear sideband background
subtraction, applies dead-time correction, and derives activity.

Physics:
  Net Peak Area = Gross(ROI) - Background(linear sideband interpolation)
  Dead-time corrected counts = N_net * realTime / liveTime
  Activity (Bq) = N_net_corrected / (efficiency * emission_probability * t_live)
  Or equivalently: Activity = N_net_corrected * calibration_factor

Published Topics:
  ~/total_activity (std_msgs/Float64): Sum of all isotope activities in MBq.
  ~/results (std_msgs/String): JSON with per-isotope breakdown (MBq).

Parameters:
  ~isotopes_config (str): Path to isotopes.yaml configuration.
  ~calibration_file (str): Path to EnergyCal.csv (energy bin edges in keV).
  ~counting_window_s (float): Override counting window (default from config).
  ~spectrum_topic (str): Input topic (default: /spectrum).
  ~publish_on_window (bool): Publish only when window completes (default: true).
  ~source_distance_m (float): Source-detector distance in metres. When > 0,
      computes solid angle for absolute activity. Updated dynamically via
      ~/source_distance topic (std_msgs/Float64, in metres).
  ~calibration_distance_m (float): Distance at which empirical calibration
      factors were measured (default: 0.5 m). Only used if calibration_factor
      fallback is active.
  ~position_correction (bool): When true, subscribe to the Compton imager's
      /source_directions + /source_isotopes hotspots and correct each
      nuclide's solid angle (and slant shield path) for its imaged off-axis
      position instead of assuming an on-axis source. Default false
      (pending Experiment E validation with the correction enabled).
  ~position_max_age_s (float): Hotspots older than this fall back to the
      on-axis assumption (default 120 s).
  ~position_geometry_exponent (float): n in the off-axis factor (d0/d')^n.
      Default 2.0 = inverse-square only (measured best model: foreshortening
      is cancelled by the oblique crystal chord); 3.0 = naive flat-disk.
"""
from __future__ import print_function

import collections
import json
import math
import os
import threading

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64, Int32, String
from std_srvs.srv import Trigger, TriggerResponse
from radiation_detector_msgs.msg import Spectrum


# PHDS GeGI Intrinsic Detection Efficiency polynomial coefficients.
# log10(eps_intrinsic) = a0 + a1*(log10 E) + a2*(log10 E)^2 + ... + a5*(log10 E)^5
# where E is gamma-ray energy in keV.
GEGI_EFFICIENCY_COEFFS = [-1.7696, -9.4708, 18.7567, -11.7681, 3.0472, -0.2854]


# GeGI crystal radius in metres (90 mm diameter)
GEGI_CRYSTAL_RADIUS_M = 0.045


def gegi_solid_angle_fraction(distance_m, crystal_radius_m=GEGI_CRYSTAL_RADIUS_M):
    """Compute geometric solid angle fraction Omega/(4*pi) for a point source
    at distance d from a circular detector of radius r.

    Formula: 0.5 * (1 - d / sqrt(d^2 + r^2))
    """
    if distance_m <= 0:
        return 0.0
    d2 = distance_m * distance_m
    r2 = crystal_radius_m * crystal_radius_m
    return 0.5 * (1.0 - distance_m / math.sqrt(d2 + r2))


def shield_transmission(mu_shield_per_m, total_plates, plate_thickness_m,
                        plate_transmission=None):
    """Fraction of gammas transmitted through `total_plates` shielding plates.

    plate_transmission: optional {n_plates: measured_T} table of BROAD-BEAM
    transmissions. Build-up grows with thickness, so a single-mu exponential
    cannot fit all plate counts (measured 2026-09-04, clean Co triplet with
    the source untouched: the FIRST plate transmits at ~narrow-beam while
    the second adds ~+9% build-up; forcing one mu costs +-3% across plate
    counts). A tabulated count uses its exact measured value; counts beyond
    the table extrapolate from the highest tabulated count with the
    exponential; no table (or untabulated gaps below it) -> pure
    exponential exp(-mu * n * t). Returns 1.0 for no shielding.
    Pure function so the shielding physics is unit-testable.
    """
    if total_plates <= 0:
        return 1.0
    if plate_transmission:
        table = {int(k): float(v) for k, v in plate_transmission.items()
                 if float(v) > 0.0}
        if total_plates in table:
            return table[total_plates]
        lower = [n for n in table if 0 < n < total_plates]
        if lower:
            n0 = max(lower)
            extra_m = (total_plates - n0) * plate_thickness_m
            if extra_m > 0 and mu_shield_per_m > 0:
                return table[n0] * math.exp(-mu_shield_per_m * extra_m)
            return table[n0]
    total_thickness = total_plates * plate_thickness_m
    if total_thickness <= 0 or mu_shield_per_m <= 0:
        return 1.0
    return math.exp(-mu_shield_per_m * total_thickness)


def plate_derived_distance(base_standoff_m, total_plates, plate_thickness_m):
    """Source-to-detector distance with `total_plates` in the beam.

    Each plate displaces the source by its thickness from the bare standoff.
    """
    return base_standoff_m + total_plates * plate_thickness_m


def off_axis_solid_angle_ratio(on_axis_distance_m, lateral_offset_m,
                               exponent=2.0):
    """Efficiency ratio (off-axis)/(on-axis) for a source displaced laterally
    by rho from the detector axis at perpendicular standoff d0:

        ratio = (d0 / d')^exponent,   d' = sqrt(d0^2 + rho^2)

    exponent selects the detector-response model:
      2.0 (DEFAULT) - inverse-square only. MEASURED best model for the GeGI
          (corner proof 2026-09-03, runs 20260903_141055/141737): publishing
          the slant distance alone recovered the Co-60 certificate to +1.6%
          (within counting statistics), i.e. the flat-disk foreshortening
          (cos theta) is cancelled by the longer oblique chord through the
          11-mm planar crystal. Also consistent with Exp E deficits
          (-8..-17%) being far smaller than the flat-disk -25% prediction.
      3.0 - naive far-field flat-disk (cos theta foreshortening included,
          no chord compensation); OVER-corrects a corner by ~15%.
    Returns 1.0 on-axis or for degenerate input. Pure function (unit-tested).
    """
    if on_axis_distance_m <= 0 or lateral_offset_m <= 0:
        return 1.0
    d_slant = math.sqrt(on_axis_distance_m ** 2 + lateral_offset_m ** 2)
    return (on_axis_distance_m / d_slant) ** exponent


def slant_shield_factor(mu_shield_per_m, total_plates, plate_thickness_m,
                        on_axis_distance_m, lateral_offset_m):
    """EXTRA shield transmission from the slant path through the plates.

    An off-axis ray crosses each plate at angle theta to the normal, so the
    steel path grows from t to t/cos(theta). The on-axis transmission is
    already corrected elsewhere (shield_transmission); this returns only the
    additional factor exp(-mu * n * t * (1/cos(theta) - 1)).
    Returns 1.0 on-axis or with no plates / no attenuation coefficient.
    """
    if on_axis_distance_m <= 0 or lateral_offset_m <= 0:
        return 1.0
    d_slant = math.sqrt(on_axis_distance_m ** 2 + lateral_offset_m ** 2)
    cos_theta = on_axis_distance_m / d_slant
    extra_path_m = total_plates * plate_thickness_m * (1.0 / cos_theta - 1.0)
    if extra_path_m <= 0 or mu_shield_per_m <= 0:
        return 1.0
    return math.exp(-mu_shield_per_m * extra_path_m)


def normalize_nuclide_name(name):
    """Canonical key for matching assay isotope names to imaging hotspot labels.

    'Cs-137' and 'Cs137' -> 'cs137'; 'Co60_1173', 'Co60_1332' and 'Co-60' all
    -> 'co60' (the assay-line energy suffix after '_' is dropped, so both Co-60
    photopeaks match the single imaged Co-60 hotspot).
    """
    base = str(name).split('_')[0]
    return ''.join(ch for ch in base if ch.isalnum()).lower()


def parse_hotspots(isotope_labels, yz_offsets_m):
    """Pair the heatmap node's index-aligned outputs into per-nuclide hotspots.

    isotope_labels: '/source_isotopes' payload, e.g. 'Cs-137:142|Co-60:98'
                    ('none' or '' = no hotspots).
    yz_offsets_m:   [(y, z), ...] source-plane offsets in metres from the
                    '/source_directions' poses, same order.

    Returns {normalized_name: {'name', 'count', 'y_m', 'z_m', 'offset_m'}},
    keeping the highest-count hotspot per nuclide - when the same nuclide
    images at several positions the dominant source drives the correction.
    Pure function (unit-tested); zip() truncates a length mismatch, callers
    should reject mismatched pairs before calling.
    """
    out = {}
    if not isotope_labels or isotope_labels == 'none':
        return out
    for label, yz in zip(isotope_labels.split('|'), yz_offsets_m):
        parts = label.rsplit(':', 1)
        name = parts[0].strip()
        if not name:
            continue
        try:
            count = int(parts[1]) if len(parts) > 1 else 0
        except ValueError:
            count = 0
        y = float(yz[0])
        z = float(yz[1])
        entry = {'name': name, 'count': count, 'y_m': y, 'z_m': z,
                 'offset_m': math.sqrt(y * y + z * z)}
        key = normalize_nuclide_name(name)
        if key not in out or count > out[key]['count']:
            out[key] = entry
    return out


def average_hotspot(entries):
    """Vector-mean position of one nuclide's hotspot samples.

    entries: list of hotspot dicts ({'y_m','z_m',...}) collected over a
    counting window. The imager's per-frame localisation jitters a few cm
    around the true position (reconstruction noise + grid quantisation),
    which maps to several % in the off-axis factor; the VECTOR mean (average
    y and z, then take the norm) is an unbiased position estimate and damps
    that jitter ~1/sqrt(n). Averaging |offset| directly would carry the
    positive noise bias instead. Returns None for no samples.
    Pure function (unit-tested).
    """
    if not entries:
        return None
    n = float(len(entries))
    y = sum(e['y_m'] for e in entries) / n
    z = sum(e['z_m'] for e in entries) / n
    return {'y_m': y, 'z_m': z, 'offset_m': math.sqrt(y * y + z * z),
            'n_samples': len(entries)}


def gegi_intrinsic_efficiency(energy_keV):
    """Compute GeGI intrinsic FEP efficiency at a given energy (keV).

    Uses the manufacturer-supplied 5th-order polynomial in log-log space.
    Valid range approximately 50 - 1500 keV.
    """
    if energy_keV <= 0:
        return 0.0
    log_e = math.log10(energy_keV)
    log_eff = sum(c * log_e**i for i, c in enumerate(GEGI_EFFICIENCY_COEFFS))
    return 10.0 ** log_eff


class IsotopeConfig(object):
    """Holds ROI channel indices and calibration data for one isotope."""

    def __init__(self, name, cfg, bin_edges, calibration_distance_m=0.5,
                 crystal_radius_m=GEGI_CRYSTAL_RADIUS_M):
        self.name = name
        self.energy_keV = cfg['energy_keV']
        self.emission_probability = cfg['emission_probability']
        self.calibration_factor = cfg.get('calibration_factor', 0.0) or 0.0
        self.efficiency = cfg.get('efficiency', 0.0) or 0.0
        # Linear attenuation coefficient (1/m) of the shielding-plate material at
        # this gamma line, for the in-line shielding correction (0 = none).
        self.mu_shield_per_m = cfg.get('mu_shield_per_m', 0.0) or 0.0
        # Optional measured per-plate-count broad-beam transmissions
        # ({n: T}); exact values beat the exponential (build-up grows with
        # thickness). See shield_transmission().
        self.plate_transmission = cfg.get('plate_transmission') or {}

        # Intrinsic efficiency: the geometry-independent detector constant.
        # Priority:
        #   1) Explicitly provided measured_intrinsic_efficiency (from prior calibration)
        #   2) Derived from calibration_factor + calibration_distance (auto-computed)
        #   3) PHDS polynomial estimate (theoretical, less accurate)
        measured = cfg.get('measured_intrinsic_efficiency', 0.0) or 0.0
        if measured > 0:
            self.intrinsic_efficiency = measured
        elif self.calibration_factor > 0 and calibration_distance_m > 0:
            # Derive from empirical calibration: eps = 1/(CF * omega_cal * I_gamma)
            omega_cal = gegi_solid_angle_fraction(calibration_distance_m, crystal_radius_m)
            if omega_cal > 0 and self.emission_probability > 0:
                self.intrinsic_efficiency = 1.0 / (
                    self.calibration_factor * omega_cal * self.emission_probability)
            else:
                self.intrinsic_efficiency = gegi_intrinsic_efficiency(self.energy_keV)
        else:
            # Fall back to PHDS polynomial (theoretical estimate)
            self.intrinsic_efficiency = gegi_intrinsic_efficiency(self.energy_keV)

        # Convert energy ranges to channel indices
        self.peak_channels = self._energy_to_channels(cfg['peak_roi_keV'], bin_edges)
        self.left_channels = self._energy_to_channels(cfg['left_sideband_keV'], bin_edges)
        self.right_channels = self._energy_to_channels(cfg['right_sideband_keV'], bin_edges)

    @staticmethod
    def _energy_to_channels(energy_range, bin_edges):
        """Map [E_low, E_high] in keV to array of channel indices."""
        lo = np.searchsorted(bin_edges, energy_range[0], side='right') - 1
        hi = np.searchsorted(bin_edges, energy_range[1], side='right') - 1
        lo = max(0, lo)
        hi = min(len(bin_edges) - 1, hi)
        return np.arange(lo, hi + 1, dtype=int)


class ActivityNode(object):
    def __init__(self):
        # Load isotope configuration
        config_path = rospy.get_param("~isotopes_config", "")
        cal_path = rospy.get_param("~calibration_file", "")
        spectrum_topic = rospy.get_param("~spectrum_topic", "/spectrum")
        self.publish_on_window = rospy.get_param("~publish_on_window", True)

        # Source distance in metres. Dynamically updatable via ~/source_distance topic.
        # When > 0, the node uses first-principles (intrinsic efficiency + solid angle)
        # for geometry-independent activity measurement.
        self.source_distance_m = rospy.get_param("~source_distance_m", 0.0)
        # Default 0.0 (NOT 0.5) so an unset param falls through to the config's
        # calibration_distance_m below. A non-zero default here would shadow the
        # config value (line ~197 only overrides when <= 0) and derive the
        # intrinsic efficiency at the wrong distance: e.g. anchoring at 0.5 m
        # while activity is computed at 0.58 m inflates every result by
        # Omega(0.5)/Omega(0.58) = 1.34x. Mirrors ~source_distance_m above.
        self.calibration_distance_m = rospy.get_param("~calibration_distance_m", 0.0)
        # Detector crystal radius (loaded from config below; param overrides).
        self.crystal_radius_m = rospy.get_param("~crystal_radius_m", 0.0)

        # In-line shielding: operator sets the number of identical plates; the
        # driver derives the standoff and the per-line attenuation from it.
        self.n_shielding_plates = int(rospy.get_param("~n_shielding_plates", 0))
        self.plate_thickness_m = 0.0
        self.base_standoff_m = 0.0

        # Position-aware correction: use the Compton imager's hotspot position
        # to evaluate the solid angle (and slant shield path) OFF-AXIS instead
        # of assuming the source sits on the detector axis. A corner source at
        # rho ~0.25 m otherwise reads ~-15% (inverse square) - 8% (disk
        # foreshortening) low. Default OFF until validated (Experiment E repeat
        # with the correction enabled).
        self.position_correction = bool(
            rospy.get_param("~position_correction", False))
        # Hotspots older than this fall back to on-axis (imaging needs events;
        # 120 s covers a quiet start-of-window without going stale mid-run).
        self.position_max_age_s = float(
            rospy.get_param("~position_max_age_s", 120.0))
        # Geometry-factor exponent: (d0/d')^n. 2.0 = inverse-square only
        # (measured best model - see off_axis_solid_angle_ratio); 3.0 = naive
        # flat-disk with foreshortening (over-corrects on this detector).
        self.position_geometry_exponent = float(
            rospy.get_param("~position_geometry_exponent", 2.0))
        self._hotspots = {}
        self._hotspot_stamp = None
        self._pose_offsets = None
        self._pose_stamp = None
        # Rolling (stamp, hotspots) samples; the correction averages each
        # nuclide's position over the counting window to damp the imager's
        # few-cm per-frame localisation jitter (heatmap publishes every few
        # seconds -> a 300 s window holds ~100 samples; maxlen is a bound).
        self._hotspot_samples = collections.deque(maxlen=1000)

        # Load energy calibration
        self.bin_edges = self._load_energy_cal(cal_path)
        n_bins = len(self.bin_edges)
        rospy.loginfo("Activity node: %d energy bins loaded", n_bins)

        # Load isotope config
        iso_cfg = self._load_isotope_config(config_path)
        self.counting_window_s = rospy.get_param(
            "~counting_window_s", iso_cfg.get('counting_window_s', 60.0))
        self.min_net_counts = iso_cfg.get('min_net_counts', 400)

        # Crystal radius from config if not overridden by param.
        if self.crystal_radius_m <= 0:
            self.crystal_radius_m = iso_cfg.get('crystal_radius_m', GEGI_CRYSTAL_RADIUS_M) \
                or GEGI_CRYSTAL_RADIUS_M

        # Allow source_distance_m from config file if not set via param
        if self.source_distance_m <= 0:
            self.source_distance_m = iso_cfg.get('source_distance_m', 0.0) or 0.0
        if self.calibration_distance_m <= 0:
            self.calibration_distance_m = iso_cfg.get('calibration_distance_m', 0.5) or 0.5

        # Shielding geometry (plate thickness + base standoff). base_standoff_m
        # defaults to the configured source distance (i.e. the 0-plate distance).
        shield_cfg = iso_cfg.get('shielding', {}) or {}
        self.plate_thickness_m = float(shield_cfg.get('plate_thickness_m', 0.0) or 0.0)
        self.base_standoff_m = float(
            shield_cfg.get('base_standoff_m', self.source_distance_m) or 0.0)
        # Permanently-mounted plates always in the beam (e.g. the fixed shield on
        # an upward-facing frame). base_standoff_m is the BARE (0-steel) distance;
        # these plates are added to whatever the operator sets via n_shielding_plates.
        self.base_shield_plates = int(shield_cfg.get('base_shield_plates', 0) or 0)

        # If shielding geometry is configured, derive the standoff from n_plates;
        # otherwise keep the static source_distance_m.
        self._recompute_distance_from_plates()
        self._update_solid_angle()

        # Fallback solid_angle_fraction from config (used only if distance not set)
        if self.solid_angle_fraction <= 0:
            self.solid_angle_fraction = iso_cfg.get('solid_angle_fraction', 0.0) or 0.0

        # Build isotope objects
        self.isotopes = []
        for name, cfg in iso_cfg.get('isotopes', {}).items():
            try:
                ic = IsotopeConfig(name, cfg, self.bin_edges,
                                   self.calibration_distance_m, self.crystal_radius_m)
                self.isotopes.append(ic)
                rospy.loginfo("  Isotope %s: peak channels %d-%d, E=%.1f keV, "
                              "eps_intrinsic=%.6f",
                              name, ic.peak_channels[0], ic.peak_channels[-1],
                              ic.energy_keV, ic.intrinsic_efficiency)
            except Exception as e:
                rospy.logwarn("Failed to configure isotope %s: %s", name, str(e))

        # Accumulation state
        self.lock = threading.Lock()
        self.accumulated_spectrum = np.zeros(n_bins, dtype=np.float64)
        self.accumulated_real_time_ms = 0
        self.accumulated_dead_time_ms = 0
        self.window_start_time = rospy.Time.now()

        # Publishers
        self.pub_total = rospy.Publisher("~total_activity", Float64, queue_size=2)
        self.pub_results = rospy.Publisher("~results", String, queue_size=2)
        # Effective source distance (base_standoff + n_plates * thickness), latched
        # so dose/imaging nodes (data_recorder, spherical_heatmap) track shielding.
        self.pub_distance = rospy.Publisher("~effective_source_distance", Float64,
                                            queue_size=1, latch=True)

        # Services
        self.clear_srv = rospy.Service("~clear", Trigger, self._handle_clear)
        # Publish the final, not-yet-full counting window on demand (called by the
        # data recorder at end-of-run) so no tail data is lost when the window is
        # long - e.g. counting_window_s == run length -> one window per run.
        self.flush_srv = rospy.Service("~flush", Trigger, self._handle_flush)

        # Subscribers
        self.sub = rospy.Subscriber(spectrum_topic, Spectrum,
                                    self._on_spectrum, queue_size=10)
        # Dynamic distance input - allows real-time distance updates from
        # range sensor, operator input, or localisation system.
        self.sub_distance = rospy.Subscriber("~source_distance", Float64,
                                             self._on_distance, queue_size=2)
        # Operator sets the number of in-line shielding plates for hot trays;
        # the driver re-derives the standoff and attenuation from it.
        self.sub_plates = rospy.Subscriber("~n_shielding_plates", Int32,
                                           self._on_n_plates, queue_size=2)
        # Imaging hotspots for the position-aware correction. The heatmap node
        # publishes /source_directions (PoseArray; pose.position.y/z = source-
        # plane offsets in metres) and /source_isotopes ('Cs-137:n|Co-60:n')
        # back-to-back and index-aligned; the String callback pairs them.
        if self.position_correction:
            self.sub_hotspot_poses = rospy.Subscriber(
                "/source_directions", PoseArray, self._on_source_directions,
                queue_size=2)
            self.sub_hotspot_names = rospy.Subscriber(
                "/source_isotopes", String, self._on_source_isotopes,
                queue_size=2)

        rospy.loginfo("Activity node ready. Window=%.1fs, %d isotopes configured. "
                      "distance=%.3fm, solid_angle=%.6f, shielding_plates=%d",
                      self.counting_window_s, len(self.isotopes),
                      self.source_distance_m, self.solid_angle_fraction,
                      self.n_shielding_plates)
        self._publish_distance()

    def _load_energy_cal(self, path):
        if not path or not os.path.exists(path):
            rospy.logwarn("No calibration file for activity node, using default 1024 bins")
            return np.linspace(0.0, 3000.0, 1024)
        values = []
        with open(path, 'r') as f:
            for line in f:
                text = line.strip()
                if text:
                    values.append(float(text))
        return np.array(values, dtype=np.float64)

    def _load_isotope_config(self, path):
        if not path or not os.path.exists(path):
            rospy.logwarn("No isotopes config file specified, using defaults")
            return {'counting_window_s': 60.0, 'min_net_counts': 400, 'isotopes': {}}
        with open(path, 'r') as f:
            return yaml.safe_load(f)

    def _update_solid_angle(self):
        """Recompute solid angle fraction from current source_distance_m."""
        if self.source_distance_m > 0:
            self.solid_angle_fraction = gegi_solid_angle_fraction(
                self.source_distance_m, self.crystal_radius_m)
        else:
            self.solid_angle_fraction = 0.0

    def _total_plates(self):
        """Total steel plates in the beam = permanent mounted plates + operator
        plates. Distance and attenuation both scale with this."""
        return self.base_shield_plates + self.n_shielding_plates

    def _recompute_distance_from_plates(self):
        """Derive standoff from the number of in-line shielding plates.

        source_distance_m = base_standoff_m + total_plates * plate_thickness_m,
        where total_plates includes the permanently-mounted base_shield_plates.
        Only applies when shielding geometry is configured (plate thickness and
        base standoff > 0); otherwise the static source_distance_m is kept.
        """
        if self.plate_thickness_m > 0 and self.base_standoff_m > 0:
            self.source_distance_m = plate_derived_distance(
                self.base_standoff_m, self._total_plates(), self.plate_thickness_m)

    def _publish_distance(self):
        """Broadcast the effective source distance to dose/imaging consumers."""
        try:
            self.pub_distance.publish(Float64(data=self.source_distance_m))
        except Exception:
            pass

    def _shield_transmission(self, iso):
        """Fraction of this isotope's gammas transmitted through the plates."""
        return shield_transmission(
            iso.mu_shield_per_m, self._total_plates(), self.plate_thickness_m,
            iso.plate_transmission)

    def _on_distance(self, msg):
        """Callback for dynamic distance updates (std_msgs/Float64, metres)."""
        new_dist = msg.data
        if new_dist > 0 and abs(new_dist - self.source_distance_m) > 0.001:
            self.source_distance_m = new_dist
            self._update_solid_angle()
            self._publish_distance()
            rospy.loginfo("Activity node: distance updated to %.3fm -> solid_angle=%.6f",
                          self.source_distance_m, self.solid_angle_fraction)

    def _on_n_plates(self, msg):
        """Callback: operator sets the number of in-line shielding plates."""
        n = int(msg.data)
        if n < 0:
            return
        if n != self.n_shielding_plates:
            self.n_shielding_plates = n
            self._recompute_distance_from_plates()
            self._update_solid_angle()
            self._publish_distance()
            rospy.loginfo("Activity node: %d shielding plate(s) -> distance=%.3fm, "
                          "solid_angle=%.6f", self.n_shielding_plates,
                          self.source_distance_m, self.solid_angle_fraction)

    def _on_source_directions(self, msg):
        """Cache the heatmap's hotspot source-plane offsets (metres)."""
        with self.lock:
            self._pose_offsets = [(p.position.y, p.position.z)
                                  for p in msg.poses]
            self._pose_stamp = rospy.Time.now()

    def _on_source_isotopes(self, msg):
        """Pair hotspot labels with the cached offsets into per-nuclide entries.

        The heatmap publishes the PoseArray immediately before this String from
        the same update cycle; a stale or length-mismatched pairing (messages
        from different cycles) is discarded rather than misassigned.
        """
        with self.lock:
            offsets = self._pose_offsets
            pose_stamp = self._pose_stamp
        if offsets is None or pose_stamp is None:
            return
        if (rospy.Time.now() - pose_stamp).to_sec() > 5.0:
            return
        labels = msg.data or ''
        n_labels = 0 if labels in ('', 'none') else len(labels.split('|'))
        if n_labels != len(offsets):
            return
        hotspots = parse_hotspots(labels, offsets)
        with self.lock:
            self._hotspots = hotspots
            self._hotspot_stamp = rospy.Time.now()
            if hotspots:
                self._hotspot_samples.append((self._hotspot_stamp, hotspots))

    def _position_correction_for(self, iso):
        """(factor, info) for this line's position-aware correction.

        factor multiplies the on-axis efficiency product K: the imaged
        off-axis solid-angle ratio times the extra slant-path shield
        transmission. Returns (1.0, None) when the correction is disabled, no
        fresh hotspot matches this line's nuclide, or geometry is unset - the
        assay then falls back to the on-axis assumption unchanged.
        """
        if not self.position_correction or self.source_distance_m <= 0:
            return 1.0, None
        now = rospy.Time.now()
        with self.lock:
            hotspots = dict(self._hotspots)
            stamp = self._hotspot_stamp
            samples = list(self._hotspot_samples)
        if not hotspots or stamp is None:
            return 1.0, None
        if (now - stamp).to_sec() > self.position_max_age_s:
            return 1.0, None
        key = normalize_nuclide_name(iso.name)
        hs = hotspots.get(key)
        if hs is None:
            return 1.0, None
        # Average this nuclide's imaged position over the counting window
        # (single-frame localisation jitters a few cm ~ several % in the
        # factor); the latest frame is the fallback when only it exists.
        window_entries = [h[key] for (t, h) in samples
                          if key in h
                          and (now - t).to_sec() <= self.counting_window_s]
        mean_hs = average_hotspot(window_entries)
        if mean_hs is not None:
            hs = mean_hs
        d0 = self.source_distance_m
        rho = hs['offset_m']
        geom = off_axis_solid_angle_ratio(d0, rho,
                                          self.position_geometry_exponent)
        shield = slant_shield_factor(iso.mu_shield_per_m, self._total_plates(),
                                     self.plate_thickness_m, d0, rho)
        info = {
            'hotspot_offset_m': rho,
            'slant_distance_m': math.sqrt(d0 * d0 + rho * rho),
            'geometry_factor': geom,
            'slant_shield_factor': shield,
            'hotspot_samples': hs.get('n_samples', 1),
        }
        return geom * shield, info

    def _on_spectrum(self, msg):
        """Accumulate incoming spectrum snapshots into the counting window.

        IDLE GUARD: the spectrum node stamps WALL-CLOCK time on every snapshot
        (realTime_ms = its publish period) whether or not the detector is
        acquiring. A zero-count snapshot means the detector is not streaming
        (even ambient background yields counts every interval on an HPGe), so
        counting its time would dilute the window rate - live activities read
        ~x0.6 low when the detector streamed for only ~60% of a window
        (observed 2026-08-26). Idle snapshots are skipped entirely: the live
        display freezes while the detector is idle instead of decaying, and
        the window's live time counts only genuinely active periods.
        """
        spectrum_arr = np.array(msg.spectrum, dtype=np.float64)
        if spectrum_arr.sum() <= 0:
            return

        with self.lock:
            n = min(len(spectrum_arr), len(self.accumulated_spectrum))
            self.accumulated_spectrum[:n] += spectrum_arr[:n]
            self.accumulated_real_time_ms += msg.realTime_ms
            self.accumulated_dead_time_ms += msg.deadTime_ms

        # Check if counting window has elapsed
        elapsed = (rospy.Time.now() - self.window_start_time).to_sec()
        if elapsed >= self.counting_window_s:
            self._compute_and_publish()

    def _compute_and_publish(self):
        """Compute net peak areas and activities, then publish and reset."""
        with self.lock:
            spectrum = self.accumulated_spectrum.copy()
            real_time_ms = self.accumulated_real_time_ms
            dead_time_ms = self.accumulated_dead_time_ms
            # Reset accumulator
            self.accumulated_spectrum[:] = 0
            self.accumulated_real_time_ms = 0
            self.accumulated_dead_time_ms = 0
            self.window_start_time = rospy.Time.now()

        # Live time in seconds
        real_time_s = real_time_ms / 1000.0
        live_time_s = (real_time_ms - dead_time_ms) / 1000.0
        if live_time_s <= 0:
            rospy.logwarn("Activity node: live time <= 0, skipping computation")
            return

        # Dead-time correction factor
        dt_correction = real_time_s / live_time_s if live_time_s > 0 else 1.0

        # Dead-time status is useful for downstream QA when detector run-info may
        # be unavailable or unresolved (e.g., all-zero dead-time path).
        if real_time_ms <= 0:
            dead_time_status = 'invalid_no_realtime'
        elif dead_time_ms < 0 or dead_time_ms > real_time_ms:
            dead_time_status = 'invalid_range'
        elif dead_time_ms == 0:
            dead_time_status = 'zero_or_unavailable'
        else:
            dead_time_status = 'valid'

        results = []
        total_activity_Bq = 0.0

        for iso in self.isotopes:
            result = self._compute_isotope_activity(
                iso, spectrum, live_time_s, dt_correction)
            results.append(result)

        # Compute total activity, combining same-source isotope peaks.
        # Co-60 emits two gammas per decay (1173 + 1332 keV); each peak
        # independently measures the same source activity. Average them
        # (inverse-variance weighted) rather than summing.
        source_activities = {}  # source_name -> (weighted_sum, weight_sum)
        for r in results:
            if not r['valid']:
                continue
            # Group by source (strip peak suffix like "_1173", "_1332")
            name = r['isotope']
            # Identify Co-60 peaks as same source
            if 'Co60' in name or 'Co-60' in name:
                source_key = 'Co-60'
            elif 'Cs137' in name or 'Cs-137' in name:
                source_key = 'Cs-137'
            else:
                source_key = name

            a = r['activity_Bq']
            sigma = r['sigma_activity_Bq']
            if sigma > 0:
                w = 1.0 / (sigma * sigma)
            else:
                w = 1.0
            if source_key in source_activities:
                ws, wt = source_activities[source_key]
                source_activities[source_key] = (ws + a * w, wt + w)
            else:
                source_activities[source_key] = (a * w, w)

        for _key, (ws, wt) in source_activities.items():
            total_activity_Bq += ws / wt if wt > 0 else 0.0

        # Convert to MBq
        total_activity_MBq = total_activity_Bq / 1.0e6

        # Publish total in MBq
        self.pub_total.publish(Float64(data=total_activity_MBq))

        # Publish detailed JSON results (all activities in MBq)
        results_MBq = []
        for r in results:
            r_copy = dict(r)
            r_copy['activity_MBq'] = r_copy.pop('activity_Bq') / 1.0e6
            r_copy['sigma_activity_MBq'] = r_copy.pop('sigma_activity_Bq') / 1.0e6
            results_MBq.append(r_copy)

        report = {
            'timestamp': rospy.Time.now().to_sec(),
            'real_time_s': real_time_s,
            'live_time_s': live_time_s,
            'dead_time_fraction': dead_time_ms / float(real_time_ms) if real_time_ms > 0 else 0.0,
            'dead_time_status': dead_time_status,
            'dt_correction_factor': dt_correction,
            'source_distance_m': self.source_distance_m,
            'solid_angle_fraction': self.solid_angle_fraction,
            'total_activity_MBq': total_activity_MBq,
            'isotopes': results_MBq
        }
        self.pub_results.publish(String(data=json.dumps(report, indent=2)))

        rospy.loginfo("Activity: total=%.4f MBq (%.1fs window, DT=%.3f%%)",
                      total_activity_MBq, real_time_s,
                      100.0 * dead_time_ms / max(real_time_ms, 1))

    def _compute_isotope_activity(self, iso, spectrum, live_time_s, dt_correction):
        """
        Compute net peak area and activity for a single isotope.

        Background subtraction: linear interpolation between left and right
        sideband means, evaluated at each peak channel.
        """
        n_channels = len(spectrum)

        # Gross counts in peak ROI
        peak_ch = iso.peak_channels[iso.peak_channels < n_channels]
        gross = float(np.sum(spectrum[peak_ch]))

        # Sideband background estimation (linear interpolation)
        left_ch = iso.left_channels[iso.left_channels < n_channels]
        right_ch = iso.right_channels[iso.right_channels < n_channels]

        if len(left_ch) == 0 or len(right_ch) == 0:
            background = 0.0
        else:
            left_mean_counts = np.mean(spectrum[left_ch])
            right_mean_counts = np.mean(spectrum[right_ch])
            x_left = np.mean(left_ch)
            x_right = np.mean(right_ch)

            if x_right <= x_left:
                background = 0.0
            else:
                # Interpolate background under each peak channel
                t = (peak_ch.astype(np.float64) - x_left) / (x_right - x_left)
                bg_per_channel = left_mean_counts + (right_mean_counts - left_mean_counts) * t
                background = float(np.sum(bg_per_channel))

        # Net peak area
        net_peak_area = gross - background
        if net_peak_area < 0:
            net_peak_area = 0.0

        # Dead-time corrected net counts
        net_corrected = net_peak_area * dt_correction

        # Statistical uncertainty (counting statistics)
        # sigma_net = sqrt(gross + background) for Poisson statistics
        sigma_counts = math.sqrt(gross + background) if (gross + background) > 0 else 0.0
        sigma_corrected = sigma_counts * dt_correction

        # Determine if measurement meets minimum counts threshold
        valid = net_peak_area >= self.min_net_counts

        # Activity calculation
        # Uses intrinsic efficiency (geometry-independent detector constant) combined
        # with solid angle from source distance. The intrinsic efficiency is either:
        #   - Derived from empirical calibration at calibration_distance (most accurate)
        #   - From PHDS polynomial (theoretical fallback)
        # This approach works at ANY distance without recalibration.
        # In-line shielding attenuates the measured signal; divide by the
        # transmission to recover the true activity (energy-dependent).
        transmission = self._shield_transmission(iso)

        # Position-aware correction (optional, ~position_correction): scale the
        # ON-AXIS efficiency product by the imaged hotspot's off-axis
        # solid-angle ratio and slant shield path so an off-centre source is
        # not under-reported. factor <= 1, so the corrected activity >= on-axis.
        position_factor, position_info = self._position_correction_for(iso)

        # Efficiency product K such that A_Bq = net_corrected / (K * live_time_s).
        # Computed UNCONDITIONALLY (it depends only on geometry/efficiency/emission
        # /shielding, never on counts), so a NON-detected line still carries the
        # sensitivity a Currie MDA needs. Mirrors the activity paths below.
        if iso.intrinsic_efficiency > 0 and self.solid_angle_fraction > 0:
            _eps_abs = iso.intrinsic_efficiency * self.solid_angle_fraction
        elif iso.efficiency > 0:
            _eps_abs = iso.efficiency
        else:
            _eps_abs = 0.0
        efficiency_product = (_eps_abs * iso.emission_probability
                              * transmission * position_factor)

        activity_Bq = 0.0
        epsilon_used = 0.0
        method_used = 'none'
        if net_corrected > 0 and valid:
            if iso.intrinsic_efficiency > 0 and self.solid_angle_fraction > 0:
                # First principles: A = N_net / (eps_intrinsic * Omega/(4pi) * I_gamma * t_live * T)
                epsilon_abs = iso.intrinsic_efficiency * self.solid_angle_fraction
                epsilon_used = epsilon_abs
                activity_Bq = net_corrected / (
                    epsilon_abs * iso.emission_probability * live_time_s
                    * transmission * position_factor)
                method_used = 'intrinsic_efficiency'
            elif iso.efficiency > 0 and iso.emission_probability > 0:
                # Manual absolute efficiency override (legacy)
                epsilon_used = iso.efficiency
                activity_Bq = net_corrected / (
                    iso.efficiency * iso.emission_probability * live_time_s
                    * transmission * position_factor)
                method_used = 'manual_efficiency'
            else:
                activity_Bq = 0.0
                rospy.logwarn_throttle(30,
                    "Isotope %s: cannot compute activity. Set source_distance_m "
                    "or provide calibration_factor for efficiency derivation.",
                    iso.name)

        # Uncertainty on activity (propagated from counting statistics)
        sigma_activity_Bq = 0.0
        if activity_Bq > 0 and net_corrected > 0:
            sigma_activity_Bq = activity_Bq * (sigma_corrected / net_corrected)

        if position_info is not None and position_factor < 0.999:
            rospy.loginfo_throttle(
                30, "Activity %s: position correction (hotspot offset %.3f m "
                "over %d sample(s), slant %.3f m, factor %.4f = geom %.4f x "
                "shield %.4f)",
                iso.name, position_info['hotspot_offset_m'],
                position_info['hotspot_samples'],
                position_info['slant_distance_m'], position_factor,
                position_info['geometry_factor'],
                position_info['slant_shield_factor'])

        return {
            'isotope': iso.name,
            'energy_keV': iso.energy_keV,
            'intrinsic_efficiency': iso.intrinsic_efficiency,
            'solid_angle_fraction': self.solid_angle_fraction,
            'absolute_efficiency': iso.intrinsic_efficiency * self.solid_angle_fraction,
            'source_distance_m': self.source_distance_m,
            'n_shielding_plates': self.n_shielding_plates,
            'total_shield_plates': self._total_plates(),
            'shield_transmission': transmission,
            'position_corrected': position_info is not None,
            'position_factor': position_factor,
            'hotspot_offset_m': (position_info['hotspot_offset_m']
                                 if position_info else 0.0),
            'slant_distance_m': (position_info['slant_distance_m']
                                 if position_info else self.source_distance_m),
            'method': method_used,
            'gross_counts': gross,
            'background_counts': background,
            'net_peak_area': net_peak_area,
            'net_corrected': net_corrected,
            'efficiency_product': efficiency_product,
            'sigma_counts': sigma_corrected,
            'activity_Bq': activity_Bq,
            'sigma_activity_Bq': sigma_activity_Bq,
            'count_rate_cps': net_corrected / live_time_s if live_time_s > 0 else 0.0,
            'valid': valid,
            'below_min_counts': not valid and net_peak_area > 0
        }

    def _handle_clear(self, req):
        with self.lock:
            self.accumulated_spectrum[:] = 0
            self.accumulated_real_time_ms = 0
            self.accumulated_dead_time_ms = 0
            self.window_start_time = rospy.Time.now()
        rospy.loginfo("Activity node accumulator cleared")
        return TriggerResponse(success=True, message="Activity accumulator cleared")

    def _handle_flush(self, req):
        """Publish whatever is accumulated NOW as a final (partial) window.

        Lets the data recorder capture the last, not-yet-full counting window at
        end-of-run so no counts are lost when counting_window_s is long. No-op if
        nothing has accumulated since the last publish.
        """
        with self.lock:
            has_data = (self.accumulated_real_time_ms > 0
                        or bool(np.any(self.accumulated_spectrum)))
        if has_data:
            self._compute_and_publish()
            return TriggerResponse(success=True, message="Flushed final window")
        return TriggerResponse(success=True, message="Nothing to flush")


def main():
    rospy.init_node("activity_node")
    node = ActivityNode()
    rospy.spin()


if __name__ == "__main__":
    main()

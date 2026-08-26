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
"""
from __future__ import print_function

import json
import math
import os
import threading

import numpy as np
import rospy
import yaml
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


def shield_transmission(mu_shield_per_m, total_plates, plate_thickness_m):
    """Fraction of gammas transmitted through `total_plates` shielding plates.

    transmission = exp(-mu * total_plates * plate_thickness). Returns 1.0 (no
    attenuation) when there is no shielding or no attenuation coefficient.
    Pure function so the shielding physics is unit-testable.
    """
    total_thickness = total_plates * plate_thickness_m
    if total_thickness <= 0 or mu_shield_per_m <= 0:
        return 1.0
    return math.exp(-mu_shield_per_m * total_thickness)


def plate_derived_distance(base_standoff_m, total_plates, plate_thickness_m):
    """Source-to-detector distance with `total_plates` in the beam.

    Each plate displaces the source by its thickness from the bare standoff.
    """
    return base_standoff_m + total_plates * plate_thickness_m


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
            iso.mu_shield_per_m, self._total_plates(), self.plate_thickness_m)

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
        efficiency_product = _eps_abs * iso.emission_probability * transmission

        activity_Bq = 0.0
        epsilon_used = 0.0
        method_used = 'none'
        if net_corrected > 0 and valid:
            if iso.intrinsic_efficiency > 0 and self.solid_angle_fraction > 0:
                # First principles: A = N_net / (eps_intrinsic * Omega/(4pi) * I_gamma * t_live * T)
                epsilon_abs = iso.intrinsic_efficiency * self.solid_angle_fraction
                epsilon_used = epsilon_abs
                activity_Bq = net_corrected / (
                    epsilon_abs * iso.emission_probability * live_time_s * transmission)
                method_used = 'intrinsic_efficiency'
            elif iso.efficiency > 0 and iso.emission_probability > 0:
                # Manual absolute efficiency override (legacy)
                epsilon_used = iso.efficiency
                activity_Bq = net_corrected / (
                    iso.efficiency * iso.emission_probability * live_time_s * transmission)
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

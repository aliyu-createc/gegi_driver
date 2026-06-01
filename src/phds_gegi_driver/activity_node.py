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
"""
from __future__ import print_function

import json
import math
import os
import threading

import numpy as np
import rospy
import yaml
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, TriggerResponse
from radiation_detector_msgs.msg import Spectrum


# PHDS GeGI Intrinsic Detection Efficiency polynomial coefficients.
# log10(eps_intrinsic) = a0 + a1*(log10 E) + a2*(log10 E)^2 + ... + a5*(log10 E)^5
# where E is gamma-ray energy in keV.
GEGI_EFFICIENCY_COEFFS = [-1.7696, -9.4708, 18.7567, -11.7681, 3.0472, -0.2854]


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

    def __init__(self, name, cfg, bin_edges):
        self.name = name
        self.energy_keV = cfg['energy_keV']
        self.emission_probability = cfg['emission_probability']
        self.calibration_factor = cfg.get('calibration_factor', 0.0) or 0.0
        self.efficiency = cfg.get('efficiency', 0.0) or 0.0

        # Compute intrinsic efficiency from PHDS polynomial
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

        # Geometric solid angle fraction: Omega / (4*pi)
        # Set from calibration: depends on source-detector distance and crystal area.
        # For a point source at distance d from a circular detector of radius r:
        #   Omega/(4pi) = 0.5 * (1 - d / sqrt(d^2 + r^2))
        # This must be determined during efficiency calibration for the tray geometry.
        self.solid_angle_fraction = rospy.get_param("~solid_angle_fraction", 0.0)

        # Load energy calibration
        self.bin_edges = self._load_energy_cal(cal_path)
        n_bins = len(self.bin_edges)
        rospy.loginfo("Activity node: %d energy bins loaded", n_bins)

        # Load isotope config
        iso_cfg = self._load_isotope_config(config_path)
        self.counting_window_s = rospy.get_param(
            "~counting_window_s", iso_cfg.get('counting_window_s', 60.0))
        self.min_net_counts = iso_cfg.get('min_net_counts', 400)
        # Allow solid_angle_fraction override from config file
        if self.solid_angle_fraction <= 0:
            self.solid_angle_fraction = iso_cfg.get('solid_angle_fraction', 0.0) or 0.0

        # Build isotope objects
        self.isotopes = []
        for name, cfg in iso_cfg.get('isotopes', {}).items():
            try:
                ic = IsotopeConfig(name, cfg, self.bin_edges)
                self.isotopes.append(ic)
                rospy.loginfo("  Isotope %s: peak channels %d-%d, E=%.1f keV",
                              name, ic.peak_channels[0], ic.peak_channels[-1],
                              ic.energy_keV)
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

        # Services
        self.clear_srv = rospy.Service("~clear", Trigger, self._handle_clear)

        # Subscriber
        self.sub = rospy.Subscriber(spectrum_topic, Spectrum,
                                    self._on_spectrum, queue_size=10)

        rospy.loginfo("Activity node ready. Window=%.1fs, %d isotopes configured.",
                      self.counting_window_s, len(self.isotopes))

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

    def _on_spectrum(self, msg):
        """Accumulate incoming spectrum snapshots into the counting window."""
        spectrum_arr = np.array(msg.spectrum, dtype=np.float64)

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

        results = []
        total_activity_Bq = 0.0

        for iso in self.isotopes:
            result = self._compute_isotope_activity(
                iso, spectrum, live_time_s, dt_correction)
            results.append(result)
            if result['valid']:
                total_activity_Bq += result['activity_Bq']

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
            'dt_correction_factor': dt_correction,
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
        # Priority: 1) empirical calibration factor, 2) intrinsic eff + solid angle,
        #           3) manual efficiency override
        activity_Bq = 0.0
        epsilon_used = 0.0
        if net_corrected > 0 and valid:
            if iso.calibration_factor > 0:
                # Use empirical calibration factor (Bq per net count per second)
                count_rate = net_corrected / live_time_s if live_time_s > 0 else 0.0
                activity_Bq = count_rate * iso.calibration_factor
                epsilon_used = iso.calibration_factor
            elif iso.intrinsic_efficiency > 0 and self.solid_angle_fraction > 0:
                # First principles using PHDS intrinsic efficiency curve:
                # A = N_net / (eps_intrinsic * Omega/(4pi) * I_gamma * t_live)
                epsilon_abs = iso.intrinsic_efficiency * self.solid_angle_fraction
                epsilon_used = epsilon_abs
                activity_Bq = net_corrected / (
                    epsilon_abs * iso.emission_probability * live_time_s)
            elif iso.efficiency > 0 and iso.emission_probability > 0:
                # Manual absolute efficiency override
                epsilon_used = iso.efficiency
                activity_Bq = net_corrected / (
                    iso.efficiency * iso.emission_probability * live_time_s)
            else:
                # No calibration available - report count rate only
                activity_Bq = 0.0
                rospy.logwarn_throttle(30,
                    "Isotope %s: set solid_angle_fraction or calibration_factor",
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
            'gross_counts': gross,
            'background_counts': background,
            'net_peak_area': net_peak_area,
            'net_corrected': net_corrected,
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


def main():
    rospy.init_node("activity_node")
    node = ActivityNode()
    rospy.spin()


if __name__ == "__main__":
    main()

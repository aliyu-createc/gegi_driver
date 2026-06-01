#!/usr/bin/env python
"""
Spherical Heatmap Node - Projects Compton cone scores onto a sphere.

Maps back-projection scores onto a sphere of configurable radius centered at
the detector origin. Publishes a PointCloud2 (colored by score) for RViz and
a PoseStamped for the peak direction.

This avoids depth ambiguity by only estimating source direction.
"""
from __future__ import print_function

import json
import threading
from collections import deque

import numpy as np
import rospy
import struct
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from radiation_detector_msgs.msg import ComptonEvent
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

# Known isotope photo-peak energies (keV) and identification windows
ISOTOPE_PEAKS = {
    'Cs-137': {'energy': 662, 'window': 80},
    'Co-60':  {'energy': 1252, 'window': 200},  # avg of 1173+1332, wide window covers both
}

# Specific gamma-ray dose rate constants (uSv*m^2 / MBq*h)
GAMMA_CONSTANTS = {
    'Cs-137': 0.0771,
    'Co-60':  0.3059,
}


def identify_isotope(energies_kev):
    """Identify the best-matching isotope from event energies near a peak.
    Returns (isotope_name, confidence) or (None, 0) if no clear match."""
    if len(energies_kev) < 5:
        return None, 0.0
    best_name = None
    best_fraction = 0.0
    for name, info in ISOTOPE_PEAKS.items():
        center = info['energy']
        window = info['window']
        in_window = np.sum((energies_kev > center - window) & (energies_kev < center + window))
        fraction = in_window / float(len(energies_kev))
        if fraction > best_fraction:
            best_fraction = fraction
            best_name = name
    if best_fraction >= 0.25:
        return best_name, best_fraction
    return None, 0.0


def fibonacci_sphere(n_points):
    """Generate approximately uniform points on a unit sphere using Fibonacci spiral."""
    indices = np.arange(0, n_points, dtype=np.float64)
    phi = np.arccos(1 - 2.0 * (indices + 0.5) / n_points)
    theta = np.pi * (1 + np.sqrt(5)) * indices
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)
    return np.stack([x, y, z], axis=1)


class SphericalHeatmapNode(object):
    def __init__(self):
        self.radius = rospy.get_param("~radius", 0.5)  # 1m diameter
        self.n_points = int(rospy.get_param("~n_points", 8000))
        self.window_s = rospy.get_param("~window_s", 120.0)
        self.update_period_s = rospy.get_param("~update_period_s", 2.0)
        self.min_events = int(rospy.get_param("~min_events", 50))
        self.max_events = int(rospy.get_param("~max_events", 3000))
        self.sigma_floor = rospy.get_param("~sigma_floor", 0.12)
        self.max_uncertainty = rospy.get_param("~max_uncertainty", 0.15)
        self.hemisphere_only = rospy.get_param("~hemisphere_only", True)
        self.max_peaks = int(rospy.get_param("~max_peaks", 5))
        self.peak_min_separation_deg = rospy.get_param("~peak_min_separation_deg", 15.0)
        self.peak_threshold = rospy.get_param("~peak_threshold", 0.4)  # fraction of max score
        self.refine_peaks = rospy.get_param("~refine_peaks", True)

        # Build sphere grid
        all_pts = fibonacci_sphere(self.n_points * (1 if not self.hemisphere_only else 2))
        if self.hemisphere_only:
            # Keep only +X hemisphere (forward-looking)
            all_pts = all_pts[all_pts[:, 0] > 0]
        self.directions = all_pts / np.linalg.norm(all_pts, axis=1, keepdims=True)
        self.sphere_points = self.directions * self.radius
        rospy.loginfo("Spherical heatmap: %d points on %.1fm radius sphere",
                      self.sphere_points.shape[0], self.radius)

        self.events = deque()
        self.lock = threading.Lock()

        self.sub = rospy.Subscriber("/compton_event", ComptonEvent, self.on_event, queue_size=600)
        self.pub_cloud = rospy.Publisher("/sphere_heatmap", PointCloud2, queue_size=2)
        self.pub_peak = rospy.Publisher("/source_direction", PoseStamped, queue_size=2)
        self.pub_peaks = rospy.Publisher("/source_directions", PoseArray, queue_size=2)
        self.pub_isotopes = rospy.Publisher("/source_isotopes", String, queue_size=2)

        self.clear_srv = rospy.Service("~clear", Trigger, self._handle_clear)

        # Subscribe to activity results for absolute dose rate scaling
        self._dose_rate_uSv_h = {}  # isotope display name -> dose rate in uSv/h
        self.sub_activity = rospy.Subscriber(
            "/activity/results", String, self._on_activity, queue_size=5)

        self.timer = rospy.Timer(rospy.Duration(self.update_period_s), self.on_timer)

    def _handle_clear(self, req):
        with self.lock:
            n = len(self.events)
            self.events.clear()
        rospy.loginfo("Cleared %d events from sphere heatmap buffer", n)
        return TriggerResponse(success=True, message="Cleared {} events".format(n))

    def _on_activity(self, msg):
        """Parse activity results and compute dose rate per isotope."""
        try:
            data = json.loads(msg.data)
            d = self.radius  # source distance = sphere radius
            dose_rates = {}
            activities = {}
            for iso in data.get('isotopes', []):
                name = iso.get('isotope', '')
                activity_mbq = iso.get('activity_MBq', 0.0)
                if 'Cs137' in name:
                    display = 'Cs-137'
                elif 'Co60' in name:
                    display = 'Co-60'
                else:
                    display = name
                # For Co-60 take max of two peaks (same source activity)
                if display in activities:
                    activities[display] = max(activities[display], activity_mbq)
                else:
                    activities[display] = activity_mbq
            for display, a_mbq in activities.items():
                gamma = GAMMA_CONSTANTS.get(display, 0.077)
                dose_rates[display] = gamma * a_mbq / (d * d) if d > 0 else 0.0
            with self.lock:
                self._dose_rate_uSv_h = dose_rates
        except (ValueError, TypeError):
            pass

    def on_event(self, msg):
        if msg.cone_angle_uncertainty <= 0.0 or msg.cone_angle_uncertainty > self.max_uncertainty:
            return

        p1 = np.array([msg.reading_location_1.x, msg.reading_location_1.y, msg.reading_location_1.z], dtype=np.float64)
        p2 = np.array([msg.reading_location_2.x, msg.reading_location_2.y, msg.reading_location_2.z], dtype=np.float64)

        axis = p1 - p2
        n = np.linalg.norm(axis)
        if n < 1e-9:
            return
        axis = axis / n
        total_energy = msg.energy_kev_1 + msg.energy_kev_2

        with self.lock:
            self.events.append((rospy.Time.now(), p1, axis, float(msg.cone_angle), total_energy))

    def _prune(self, now):
        cutoff = now - rospy.Duration(self.window_s)
        while self.events and self.events[0][0] < cutoff:
            self.events.popleft()
        if self.max_events > 0 and len(self.events) > self.max_events:
            for _ in range(len(self.events) - self.max_events):
                self.events.popleft()

    def _solve(self, events):
        points = self.sphere_points
        scores = np.zeros(points.shape[0], dtype=np.float64)

        for _, apex, axis, angle, _energy in events:
            ap = points - apex
            ap_norm = np.linalg.norm(ap, axis=1)
            cos_actual = np.dot(ap, axis) / np.maximum(ap_norm, 1e-9)
            cos_actual = np.clip(cos_actual, -1.0, 1.0)
            actual_angle = np.arccos(cos_actual)
            ang_dev = np.abs(actual_angle - angle)
            scores += np.exp(-0.5 * (ang_dev / self.sigma_floor) ** 2)

        return scores

    def _solve_energy_filtered(self, events, energy_center, energy_window):
        """Solve using only events within a specific energy band."""
        points = self.sphere_points
        scores = np.zeros(points.shape[0], dtype=np.float64)
        count = 0

        for _, apex, axis, angle, energy in events:
            if not (energy_center - energy_window < energy < energy_center + energy_window):
                continue
            ap = points - apex
            ap_norm = np.linalg.norm(ap, axis=1)
            cos_actual = np.dot(ap, axis) / np.maximum(ap_norm, 1e-9)
            cos_actual = np.clip(cos_actual, -1.0, 1.0)
            actual_angle = np.arccos(cos_actual)
            ang_dev = np.abs(actual_angle - angle)
            scores += np.exp(-0.5 * (ang_dev / self.sigma_floor) ** 2)
            count += 1

        return scores, count

    def _events_near_direction(self, events, direction, cone_half_angle_rad=0.10):
        """Return energies of events whose cones pass near the given direction.
        Uses tighter cone matching to better attribute events to this peak."""
        energies = []
        for _, apex, axis, angle, energy in events:
            # Check if this event's Compton cone intersects the direction
            vec = self.radius * direction - apex
            vec_norm = np.linalg.norm(vec)
            if vec_norm < 1e-9:
                continue
            cos_actual = np.dot(vec / vec_norm, axis)
            cos_actual = np.clip(cos_actual, -1.0, 1.0)
            actual_angle = np.arccos(cos_actual)
            if abs(actual_angle - angle) < cone_half_angle_rad:
                energies.append(energy)
        return np.array(energies)

    def _identify_peak_isotope(self, events, direction):
        """Score each isotope's events separately at this direction.
        Uses average score per event to avoid bias from source activity differences."""
        scores_by_isotope = {}
        for name, info in ISOTOPE_PEAKS.items():
            center = info['energy']
            window = info['window']
            score_sum = 0.0
            count = 0
            for _, apex, axis, angle, energy in events:
                total_e = energy
                if not (center - window < total_e < center + window):
                    continue
                vec = self.radius * direction - apex
                vec_norm = np.linalg.norm(vec)
                if vec_norm < 1e-9:
                    continue
                cos_actual = np.dot(vec / vec_norm, axis)
                cos_actual = np.clip(cos_actual, -1.0, 1.0)
                actual_angle = np.arccos(cos_actual)
                ang_dev = abs(actual_angle - angle)
                score_sum += np.exp(-0.5 * (ang_dev / self.sigma_floor) ** 2)
                count += 1
            avg_score = score_sum / max(count, 1)
            scores_by_isotope[name] = (avg_score, count)

        # Pick the isotope whose events fit best (highest average score) at this direction
        best_name = None
        best_avg = 0.0
        for name, (avg, count) in scores_by_isotope.items():
            if count >= 3 and avg > best_avg:
                best_avg = avg
                best_name = name
        if best_name is None:
            return None, 0.0
        total_avg = sum(a for a, _ in scores_by_isotope.values())
        confidence = best_avg / max(total_avg, 1e-9)
        return best_name, confidence

    def _is_local_maximum(self, idx, scores, radius_rad=0.10):
        """Check if point idx is a local maximum within the given angular radius."""
        center_dir = self.directions[idx]
        cos_angles = self.directions.dot(center_dir)
        neighbors = np.where((cos_angles > np.cos(radius_rad)) & (cos_angles < 1.0 - 1e-9))[0]
        if len(neighbors) == 0:
            return True
        return scores[idx] >= scores[neighbors].max()

    def _find_peaks(self, scores):
        """Find multiple local maxima on the sphere with minimum angular separation."""
        min_sep_rad = np.radians(self.peak_min_separation_deg)
        max_score = scores.max()
        min_score = scores.min()
        score_range = max_score - min_score
        if score_range < 1e-12:
            return []

        sorted_indices = np.argsort(scores)[::-1]
        peaks = []

        for idx in sorted_indices:
            if len(peaks) >= self.max_peaks:
                break

            # Must be a local maximum (filters out Compton ring ridge points)
            if not self._is_local_maximum(int(idx), scores):
                continue

            # Secondary peaks must score at least threshold * primary
            if peaks:
                primary_norm = (scores[peaks[0]] - min_score) / score_range
                this_norm = (scores[idx] - min_score) / score_range
                if this_norm < self.peak_threshold * primary_norm:
                    break

            # Check angular separation from existing peaks
            too_close = False
            for peak_idx in peaks:
                cos_sep = np.dot(self.directions[idx], self.directions[peak_idx])
                cos_sep = np.clip(cos_sep, -1.0, 1.0)
                sep = np.arccos(cos_sep)
                if sep < min_sep_rad:
                    too_close = True
                    break

            if not too_close:
                peaks.append(int(idx))

        return peaks

    def _refine_peak_centroid(self, peak_idx, scores, radius_rad=0.06):
        """Refine peak location using score-weighted centroid of nearby points.

        Instead of just taking the grid point with the highest score, compute
        a weighted average direction using all points within radius_rad.
        Uses a tight radius (0.06 rad ~ 3.4 deg) and high weighting exponent
        to avoid bias from neighboring sources.
        """
        center_dir = self.directions[peak_idx]
        cos_angles = self.directions.dot(center_dir)
        neighbors = np.where(cos_angles > np.cos(radius_rad))[0]

        if len(neighbors) < 3:
            return self.sphere_points[peak_idx]

        # Use scores raised to a power to sharpen the weighting
        neighbor_scores = scores[neighbors]
        # Shift so minimum in neighborhood is 0
        shifted = neighbor_scores - neighbor_scores.min()
        max_shifted = shifted.max()
        if max_shifted < 1e-12:
            return self.sphere_points[peak_idx]

        # Cube the weights for very sharp centroid (minimize pull from neighbors)
        weights = (shifted / max_shifted) ** 3

        # Weighted average of unit direction vectors
        weighted_dirs = self.directions[neighbors] * weights[:, np.newaxis]
        centroid_dir = weighted_dirs.sum(axis=0)
        norm = np.linalg.norm(centroid_dir)
        if norm < 1e-9:
            return self.sphere_points[peak_idx]

        centroid_dir /= norm
        return centroid_dir * self.radius

    def on_timer(self, _event):
        now = rospy.Time.now()
        with self.lock:
            self._prune(now)
            events = list(self.events)

        if len(events) < self.min_events:
            rospy.loginfo_throttle(10.0, "sphere heatmap waiting: %d/%d events",
                                   len(events), self.min_events)
            return

        scores = self._solve(events)

        # Normalize scores to [0, 1] for visualization
        smin = scores.min()
        smax = scores.max()
        if smax - smin < 1e-12:
            norm_scores = np.zeros_like(scores)
        else:
            norm_scores = (scores - smin) / (smax - smin)

        # Publish peak direction (strongest)
        idx_peak = int(np.argmax(scores))
        peak_dir = self.directions[idx_peak]

        peak_msg = PoseStamped()
        peak_msg.header.stamp = now
        peak_msg.header.frame_id = "detector"
        peak_msg.pose.position.x = float(self.sphere_points[idx_peak, 0])
        peak_msg.pose.position.y = float(self.sphere_points[idx_peak, 1])
        peak_msg.pose.position.z = float(self.sphere_points[idx_peak, 2])
        peak_msg.pose.orientation.w = 1.0
        self.pub_peak.publish(peak_msg)

        # Publish all detected peaks as PoseArray - find peaks per isotope energy band
        peaks_msg = PoseArray()
        peaks_msg.header.stamp = now
        peaks_msg.header.frame_id = "detector"
        isotope_labels = []

        for iso_name, iso_info in ISOTOPE_PEAKS.items():
            iso_scores, iso_count = self._solve_energy_filtered(
                events, iso_info['energy'], iso_info['window'])
            if iso_count < 5:
                continue
            iso_peaks = self._find_peaks(iso_scores)
            if not iso_peaks:
                continue
            # Take only the strongest peak for this isotope
            pidx = iso_peaks[0]
            # Refine peak location using score-weighted centroid
            if self.refine_peaks:
                refined_pos = self._refine_peak_centroid(pidx, iso_scores)
            else:
                refined_pos = self.sphere_points[pidx]
            # Convert sphere position to real-world coordinate at source plane
            # Y_real = distance * Y_sphere / X_sphere (ray-plane intersection)
            x_s = float(refined_pos[0])
            y_s = float(refined_pos[1])
            z_s = float(refined_pos[2])
            if abs(x_s) > 1e-6:
                y_real = self.radius * y_s / x_s
                z_real = self.radius * z_s / x_s
            else:
                y_real = y_s
                z_real = z_s
            p = Pose()
            p.position.x = self.radius  # source distance along X
            p.position.y = y_real
            p.position.z = z_real
            p.orientation.w = 1.0
            peaks_msg.poses.append(p)
            # Include event count so plotter can scale by activity
            isotope_labels.append("{}:{}".format(iso_name, iso_count))

        self.pub_peaks.publish(peaks_msg)

        # Publish isotope identification with counts
        iso_msg = String()
        iso_msg.data = "|".join(isotope_labels) if isotope_labels else "none"
        self.pub_isotopes.publish(iso_msg)

        # Publish PointCloud2 with per-isotope scores scaled to dose rate (uSv/h)
        with self.lock:
            dose_rates = dict(self._dose_rate_uSv_h)

        iso_norm_scores = {}
        for iso_name, iso_info in ISOTOPE_PEAKS.items():
            iso_scores, iso_count = self._solve_energy_filtered(
                events, iso_info['energy'], iso_info['window'])
            if iso_count >= 5:
                iso_min = iso_scores.min()
                iso_max = iso_scores.max()
                if iso_max - iso_min > 1e-12:
                    normed = (iso_scores - iso_min) / (iso_max - iso_min)
                else:
                    normed = np.zeros_like(iso_scores)
                # Scale peak to actual dose rate (uSv/h) from activity measurement
                dr = dose_rates.get(iso_name, 0.0)
                if dr > 0:
                    iso_norm_scores[iso_name] = normed * dr
                else:
                    # Fallback: use gamma constant as relative weight
                    gamma = GAMMA_CONSTANTS.get(iso_name, 0.077)
                    iso_norm_scores[iso_name] = normed * gamma
            else:
                iso_norm_scores[iso_name] = np.zeros_like(scores)

        # Combine per-isotope scores (take max at each point)
        # Do NOT renormalize - values represent dose rate in uSv/h
        combined = np.zeros_like(scores)
        for v in iso_norm_scores.values():
            combined = np.maximum(combined, v)
        norm_scores = combined

        self._publish_cloud(now, norm_scores, iso_norm_scores)

        rospy.loginfo_throttle(5.0,
                               "sphere heatmap: %d events, %d sources [%s], peak dose=%.3f uSv/h, dose_rates=%s",
                               len(events), len(peaks_msg.poses),
                               iso_msg.data,
                               float(norm_scores.max()),
                               dose_rates)

    def _publish_cloud(self, stamp, norm_scores, iso_norm_scores):
        """Publish colored PointCloud2 with per-isotope intensity fields.
        
        The intensity field contains dose rate in uSv/h.
        RGB is normalized for visual coloring only.
        """
        points = self.sphere_points
        n = points.shape[0]

        # Fields: x, y, z, rgb, intensity, cs137, co60  (7 floats = 28 bytes)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='cs137', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='co60', offset=24, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 28
        cs137_scores = iso_norm_scores.get('Cs-137', np.zeros(n))
        co60_scores = iso_norm_scores.get('Co-60', np.zeros(n))

        # Normalize scores to [0,1] for RGB coloring only
        vmax = norm_scores.max()
        if vmax > 1e-12:
            # Suppress background noise: zero out below 10% of peak
            threshold = vmax * 0.10
            suppressed = np.where(norm_scores >= threshold, norm_scores, 0.0)
            color_scores = suppressed / vmax
        else:
            color_scores = np.zeros_like(norm_scores)

        buf = bytearray(n * point_step)
        for i in range(n):
            struct.pack_into('fff', buf, i * point_step, points[i, 0], points[i, 1], points[i, 2])
            # Color: blue (cold) -> red (hot) using normalized values
            v = color_scores[i]
            r = int(min(255, v * 2 * 255))
            g = int(min(255, max(0, (v - 0.25) * 2) * 255)) if v > 0.25 else 0
            b = int(max(0, (1.0 - v * 2) * 255)) if v < 0.5 else 0
            rgb_int = (r << 16) | (g << 8) | b
            struct.pack_into('f', buf, i * point_step + 12, struct.unpack('f', struct.pack('I', rgb_int))[0])
            # Intensity field: dose rate, zeroed for background points
            intensity = norm_scores[i] if color_scores[i] > 0 else 0.0
            struct.pack_into('f', buf, i * point_step + 16, intensity)
            struct.pack_into('f', buf, i * point_step + 20, cs137_scores[i] if color_scores[i] > 0 else 0.0)
            struct.pack_into('f', buf, i * point_step + 24, co60_scores[i] if color_scores[i] > 0 else 0.0)

        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id="detector")
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = n * point_step
        msg.data = bytes(buf)
        msg.is_dense = True
        self.pub_cloud.publish(msg)


if __name__ == "__main__":
    rospy.init_node("spherical_heatmap")
    node = SphericalHeatmapNode()
    rospy.spin()

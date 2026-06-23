#!/usr/bin/env python
"""
Data Recorder Node - Saves acquisition data at the end of a timed acquisition.

Wraps the /detector/start_timed_acquisition service with a proxy that:
  1. Starts rosbag recording of /compton_event
  2. Accumulates spectrum data for N42 export
  3. Collects heatmap peak directions and isotope IDs for CSV export
  4. Collects activity results for CSV export
  5. When the timer expires, stops recording and writes all files

Output files (in ~output_dir, default /opt/phds_gegi_driver/data):
  - <timestamp>_compton_events.bag
  - <timestamp>_spectrum.n42
  - <timestamp>_heatmap.csv
  - <timestamp>_activity.csv

Parameters:
  ~output_dir (str): Directory for saved files (default: /opt/phds_gegi_driver/data)

Services:
  ~/start_timed_recording (phds_gegi_driver/StartTimedAcquisition):
    Starts timed acquisition AND recording.
"""
from __future__ import print_function

import csv
import os
import threading
import time
from datetime import datetime

import numpy as np
import rospy
import rosbag
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from radiation_detector_msgs.msg import ComptonEvent, Spectrum
from phds_gegi_driver.srv import StartTimedAcquisition, StartTimedAcquisitionRequest, GetRunInfo


class DataRecorderNode(object):
    def __init__(self):
        self.output_dir = rospy.get_param("~output_dir", "/opt/phds_gegi_driver/data")
        cal_path = rospy.get_param("~calibration_file", "")
        self.source_distance_m = rospy.get_param("~source_distance_m", 0.5)
        self.heatmap_grid_res = rospy.get_param("~heatmap_grid_res", 200)
        self.run_info_settle_s = rospy.get_param("~run_info_settle_s", 15.0)

        # Ensure output dir exists
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # Load energy calibration for N42 export
        self.bin_edges = self._load_energy_cal(cal_path)

        # Recording state
        self.lock = threading.Lock()
        self.recording = False
        self.bag = None
        self.accumulated_spectrum = np.zeros(len(self.bin_edges), dtype=np.uint32)
        self.total_real_time_ms = 0
        self.total_live_time_ms = 0
        self.heatmap_peaks = []  # list of (x, y, z, isotope, activity)
        self.heatmap_cloud = None  # latest full PointCloud2 (x, y, z, score)
        self.activity_results = []  # list of activity JSON dicts
        self.recording_start = None
        self.duration_minutes = 0
        self.timer = None

        # Proxy service: user calls this instead of /detector/start_timed_acquisition
        self.record_srv = rospy.Service(
            "~start_timed_recording", StartTimedAcquisition, self._handle_start)

        # Stop recording early (saves all data collected so far)
        from std_srvs.srv import Trigger, TriggerResponse
        self.stop_srv = rospy.Service("~stop_recording", Trigger, self._handle_stop)

        # One-shot "clear everything" coordinator. /detector/clear_data only
        # clears the hardware; the ROS nodes keep independent accumulators (the
        # heatmap holds a rolling ~120 s event buffer), so a hardware clear alone
        # leaves the live image populated until those buffers age out. This
        # service fans out to the hardware clear plus every node clear so a single
        # call resets the whole pipeline.
        self.node_clear_services = rospy.get_param(
            "~node_clear_services",
            ["/spherical_heatmap/clear", "/spectrum/clear",
             "/spectrum_singles/clear", "/activity/clear"])
        self.clear_all_srv = rospy.Service("~clear_all", Trigger, self._handle_clear_all)

        # Subscribers (always active, but only record when self.recording=True)
        self.sub_compton = rospy.Subscriber(
            "/compton_event", ComptonEvent, self._on_compton, queue_size=500)
        self.sub_spectrum = rospy.Subscriber(
            "/spectrum", Spectrum, self._on_spectrum, queue_size=10)
        self.sub_peaks = rospy.Subscriber(
            "/source_directions", PoseArray, self._on_peaks, queue_size=5)
        self.sub_isotopes = rospy.Subscriber(
            "/source_isotopes", String, self._on_isotopes, queue_size=5)
        self.sub_activity = rospy.Subscriber(
            "/activity/results", String, self._on_activity, queue_size=5)
        self.sub_cloud = rospy.Subscriber(
            "/sphere_heatmap", PointCloud2, self._on_cloud, queue_size=2)

        # Latest isotope text for pairing with peaks
        self._latest_isotope_text = ""
        # Latest activity per isotope (name -> MBq)
        self._latest_activity_MBq = {}

        rospy.loginfo("Data recorder node ready. Output dir: %s", self.output_dir)

    def _load_energy_cal(self, path):
        if not path or not os.path.exists(path):
            return np.linspace(0.0, 3000.0, 1024)
        values = []
        with open(path, 'r') as f:
            for line in f:
                text = line.strip()
                if text:
                    values.append(float(text))
        return np.array(values, dtype=np.float64)

    def _handle_start(self, req):
        """Proxy: start timed acquisition on detector + begin recording."""
        from phds_gegi_driver.srv import StartTimedAcquisitionResponse
        resp = StartTimedAcquisitionResponse()

        with self.lock:
            if self.recording:
                resp.success = False
                resp.message = "Already recording. Wait for current acquisition to finish."
                return resp

        # Forward to the real detector service
        try:
            rospy.wait_for_service("/detector/start_timed_acquisition", timeout=5.0)
            proxy = rospy.ServiceProxy("/detector/start_timed_acquisition", StartTimedAcquisition)
            det_resp = proxy(req)
            if not det_resp.success:
                resp.success = False
                resp.message = "Detector refused: " + det_resp.message
                return resp
        except Exception as e:
            resp.success = False
            resp.message = "Failed to call detector service: " + str(e)
            return resp

        # Start recording
        self._start_recording(req.duration_minutes)

        resp.success = True
        resp.message = "Recording started for {} minutes. Files will be saved to {}".format(
            req.duration_minutes, self.output_dir)
        return resp

    def _handle_stop(self, req):
        """Stop recording early and save all data collected so far."""
        from std_srvs.srv import TriggerResponse
        with self.lock:
            if not self.recording:
                return TriggerResponse(success=False, message="Not currently recording.")
        # Cancel the timer
        if self.timer:
            self.timer.cancel()
        # Also stop the detector acquisition
        try:
            from phds_gegi_driver.srv import StopAcquisition
            rospy.wait_for_service("/detector/stop_acquisition", timeout=5.0)
            stop_proxy = rospy.ServiceProxy("/detector/stop_acquisition", StopAcquisition)
            stop_proxy()
        except Exception:
            rospy.logwarn("Could not stop detector acquisition (may already be stopped)")
        # Save data
        self._stop_recording()
        return TriggerResponse(success=True, message="Recording stopped early. Data saved.")

    def _handle_clear_all(self, req):
        """Clear the detector hardware AND every ROS-side accumulator buffer."""
        from std_srvs.srv import Trigger, TriggerResponse
        from phds_gegi_driver.srv import ClearData

        results = []
        all_ok = True

        # 1) Hardware data buffer
        try:
            rospy.wait_for_service("/detector/clear_data", timeout=3.0)
            r = rospy.ServiceProxy("/detector/clear_data", ClearData)()
            ok = bool(r.success)
            results.append("/detector/clear_data:{}".format("ok" if ok else "fail"))
            all_ok = all_ok and ok
        except Exception as e:
            results.append("/detector/clear_data:err({})".format(e))
            all_ok = False

        # 2) ROS node buffers (heatmap event window, spectra, activity window)
        for name in self.node_clear_services:
            try:
                rospy.wait_for_service(name, timeout=3.0)
                r = rospy.ServiceProxy(name, Trigger)()
                ok = bool(r.success)
                results.append("{}:{}".format(name, "ok" if ok else "fail"))
                all_ok = all_ok and ok
            except Exception as e:
                results.append("{}:err({})".format(name, e))
                all_ok = False

        message = "; ".join(results)
        rospy.loginfo("clear_all: %s", message)
        return TriggerResponse(success=all_ok, message=message)

    def _start_recording(self, duration_minutes):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(self.output_dir, "{}_compton_events.bag".format(timestamp))

        with self.lock:
            self.recording = True
            self.duration_minutes = duration_minutes
            self.recording_start = rospy.Time.now()
            self.accumulated_spectrum[:] = 0
            self.total_real_time_ms = 0
            self.total_live_time_ms = 0
            self.heatmap_peaks = []
            self.heatmap_cloud = None
            self.activity_results = []
            self._timestamp_prefix = timestamp
            self.bag = rosbag.Bag(bag_path, 'w')

        rospy.loginfo("Recording started: %d min, bag=%s", duration_minutes, bag_path)

        # Set a timer to stop recording after the duration
        duration_secs = duration_minutes * 60.0
        self.timer = threading.Timer(duration_secs, self._stop_recording)
        self.timer.daemon = True
        self.timer.start()

    def _stop_recording(self):
        """Called when the timed acquisition ends. Save all files."""
        rospy.loginfo("Timed acquisition complete. Saving data files...")

        # Keep recording flag on briefly to capture any final heatmap/activity publishes
        # The heatmap node publishes every ~2s; wait one cycle to get final state.
        rospy.sleep(3.0)

        # Get detector-reported run info once the stream settles. This is the
        # most reliable path for the true dead-time value when run-info parsing
        # may be intermittent during active streaming.
        detector_run_info = self._fetch_detector_run_info_post_stop()

        with self.lock:
            self.recording = False
            bag = self.bag
            self.bag = None
            spectrum = self.accumulated_spectrum.copy()
            real_time_ms = self.total_real_time_ms
            live_time_ms = self.total_live_time_ms
            peaks = list(self.heatmap_peaks)
            cloud = self.heatmap_cloud
            activities = list(self.activity_results)
            prefix = self._timestamp_prefix

        # Close bag file
        if bag:
            bag.close()
            rospy.loginfo("  Bag saved: %s_compton_events.bag", prefix)

        # Save spectrum as N42
        self._save_n42(prefix, spectrum, real_time_ms, live_time_ms)

        # Save heatmap raw points CSV (irregular, per-isotope scores)
        try:
            self._save_heatmap_csv(prefix, cloud)
        except Exception as e:
            rospy.logerr("Failed to save heatmap raw CSV: %s", e)

        # Save heatmap rasterised CSV (regular 5mm Y-Z grid)
        try:
            self._save_heatmap_raster(prefix, cloud, peaks)
        except Exception as e:
            rospy.logerr("Failed to save heatmap raster CSV: %s", e)

        # Save 3D heatmap grid (Y-Z projection with Gaussian blobs at peaks)
        try:
            self._save_heatmap_3d(prefix, cloud, peaks)
        except Exception as e:
            rospy.logerr("Failed to save heatmap 3D CSV: %s", e)

        # Save activity results as CSV
        try:
            self._save_activity_csv(prefix, activities, detector_run_info,
                                    real_time_ms, live_time_ms)
        except Exception as e:
            rospy.logerr("Failed to save activity CSV: %s", e)

        rospy.loginfo("All data files saved with prefix: %s", prefix)

    def _fetch_detector_run_info_post_stop(self):
        """Fetch detector run info after acquisition stop with settle delay."""
        info = {
            'valid': False,
            'dead_time_percent': 0.0,
            'real_time_sec': 0.0,
            'live_time_sec': 0.0,
            'message': 'not_queried'
        }

        settle = max(0.0, float(self.run_info_settle_s))
        if settle > 0:
            rospy.sleep(settle)

        try:
            rospy.wait_for_service('/detector/get_run_info', timeout=5.0)
            proxy = rospy.ServiceProxy('/detector/get_run_info', GetRunInfo)
            for _ in range(3):
                resp = proxy()
                if resp.success:
                    info['valid'] = True
                    info['dead_time_percent'] = float(resp.dead_time_percent)
                    info['real_time_sec'] = float(resp.real_time_sec)
                    info['live_time_sec'] = float(resp.live_time_sec)
                    info['message'] = resp.message
                    break
                info['message'] = resp.message
                rospy.sleep(1.0)
        except Exception as e:
            info['message'] = str(e)

        if info['valid']:
            rospy.loginfo("  Detector run info: real=%.3fs live=%.3fs dead=%.3f%%",
                          info['real_time_sec'], info['live_time_sec'],
                          info['dead_time_percent'])
        else:
            rospy.logwarn("  Detector run info unavailable: %s", info['message'])

        return info

    def _on_compton(self, msg):
        with self.lock:
            if not self.recording or self.bag is None:
                return
            try:
                self.bag.write("/compton_event", msg, rospy.Time.now())
            except Exception:
                pass

    def _on_spectrum(self, msg):
        with self.lock:
            if not self.recording:
                return
            arr = np.array(msg.spectrum, dtype=np.uint32)
            n = min(len(arr), len(self.accumulated_spectrum))
            self.accumulated_spectrum[:n] += arr[:n]
            self.total_real_time_ms += msg.realTime_ms
            self.total_live_time_ms += (msg.realTime_ms - msg.deadTime_ms)

    def _on_peaks(self, msg):
        with self.lock:
            if not self.recording:
                return
            iso_text = self._latest_isotope_text
            # Parse isotope labels (pipe-separated "Cs-137:count|Co-60:count")
            isotope_names = []
            if iso_text and iso_text != 'none':
                for part in iso_text.split('|'):
                    part = part.strip()
                    if ':' in part:
                        name = part.rsplit(':', 1)[0].strip()
                    else:
                        name = part
                    isotope_names.append(name)

            for i, pose in enumerate(msg.poses):
                pos = pose.position
                iso_name = isotope_names[i] if i < len(isotope_names) else "unknown"
                activity = self._latest_activity_MBq.get(iso_name, 0.0)
                self.heatmap_peaks.append((pos.x, pos.y, pos.z, iso_name, activity))

    def _on_cloud(self, msg):
        """Store the latest sphere heatmap PointCloud2 (overwrite each update)."""
        with self.lock:
            if not self.recording:
                return
            # Parse PointCloud2 into numpy arrays
            import struct as st
            fields = {f.name: f.offset for f in msg.fields}
            x_off = fields.get('x', 0)
            y_off = fields.get('y', 4)
            z_off = fields.get('z', 8)
            intensity_off = fields.get('intensity', None)
            cs137_off = fields.get('cs137', None)
            co60_off = fields.get('co60', None)

            step = msg.point_step
            raw = msg.data
            n_points = len(raw) // step if step > 0 else 0

            # Columns: x, y, z, intensity, cs137, co60
            points = np.zeros((n_points, 6), dtype=np.float64)
            for idx in range(n_points):
                offset = idx * step
                x = st.unpack_from('<f', raw, offset + x_off)[0]
                y = st.unpack_from('<f', raw, offset + y_off)[0]
                z = st.unpack_from('<f', raw, offset + z_off)[0]
                intensity = st.unpack_from('<f', raw, offset + intensity_off)[0] if intensity_off is not None else 0.0
                cs137 = st.unpack_from('<f', raw, offset + cs137_off)[0] if cs137_off is not None else 0.0
                co60 = st.unpack_from('<f', raw, offset + co60_off)[0] if co60_off is not None else 0.0
                points[idx] = [x, y, z, intensity, cs137, co60]

            self.heatmap_cloud = points

    def _on_isotopes(self, msg):
        with self.lock:
            self._latest_isotope_text = msg.data

    def _on_activity(self, msg):
        import json
        with self.lock:
            if not self.recording:
                return
            try:
                data = json.loads(msg.data)
                self.activity_results.append(data)
                # Update latest activity per isotope for heatmap tagging
                for iso in data.get('isotopes', []):
                    name = iso.get('isotope', '')
                    # Map internal names to display names
                    if 'Cs137' in name:
                        display = 'Cs-137'
                    elif 'Co60' in name:
                        display = 'Co-60'
                    else:
                        display = name
                    mbq = iso.get('activity_MBq', 0.0)
                    if mbq > 0:
                        # For Co-60 take the max of its two peaks
                        if display in self._latest_activity_MBq:
                            self._latest_activity_MBq[display] = max(
                                self._latest_activity_MBq[display], mbq)
                        else:
                            self._latest_activity_MBq[display] = mbq
            except (ValueError, TypeError):
                pass

    def _save_n42(self, prefix, spectrum, real_time_ms, live_time_ms):
        """Save spectrum in ANSI N42.42 XML format."""
        filepath = os.path.join(self.output_dir, "{}_spectrum.n42".format(prefix))

        real_time_s = real_time_ms / 1000.0
        live_time_s = live_time_ms / 1000.0

        # Energy calibration coefficients (linear fit to bin edges)
        # Channel to energy: E = offset + gain * channel
        if len(self.bin_edges) >= 2:
            offset = self.bin_edges[0]
            gain = (self.bin_edges[-1] - self.bin_edges[0]) / (len(self.bin_edges) - 1)
        else:
            offset = 0.0
            gain = 3.0

        # N42 spectrum data as space-separated counts
        spectrum_text = " ".join(str(int(c)) for c in spectrum)

        n42_xml = """<?xml version="1.0" encoding="UTF-8"?>
<RadInstrumentData xmlns="http://physics.nist.gov/N42/2011/N42"
                   xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                   xsi:schemaLocation="http://physics.nist.gov/N42/2011/N42 http://physics.nist.gov/N42/2011/n42.xsd">
  <RadInstrumentInformation>
    <RadInstrumentManufacturerName>PHDS</RadInstrumentManufacturerName>
    <RadInstrumentModelName>GeGI</RadInstrumentModelName>
    <RadInstrumentClassCode>Spectroscopic Personal Radiation Detector</RadInstrumentClassCode>
    <RadInstrumentVersion>
      <RadInstrumentComponentName>Software</RadInstrumentComponentName>
      <RadInstrumentComponentVersion>ros_phds_gegi_driver</RadInstrumentComponentVersion>
    </RadInstrumentVersion>
  </RadInstrumentInformation>
  <RadDetectorInformation>
    <RadDetectorName>GeGI_HPGe</RadDetectorName>
    <RadDetectorCategoryCode>Gamma</RadDetectorCategoryCode>
    <RadDetectorKindCode>HPGe</RadDetectorKindCode>
    <RadDetectorDescription>PHDS GeGI HPGe detector, 90mm dia, 11mm thick</RadDetectorDescription>
  </RadDetectorInformation>
  <EnergyCalibration>
    <CoefficientValues>{offset:.6f} {gain:.6f} 0.0</CoefficientValues>
  </EnergyCalibration>
  <RadMeasurement>
    <MeasurementClassCode>Foreground</MeasurementClassCode>
    <RealTimeDuration>PT{real_time:.3f}S</RealTimeDuration>
    <Spectrum>
      <LiveTimeDuration>PT{live_time:.3f}S</LiveTimeDuration>
      <ChannelData compressionCode="None">{spectrum_data}</ChannelData>
    </Spectrum>
  </RadMeasurement>
</RadInstrumentData>
""".format(
            offset=offset,
            gain=gain,
            real_time=real_time_s,
            live_time=live_time_s,
            spectrum_data=spectrum_text
        )

        with open(filepath, 'w') as f:
            f.write(n42_xml)
        rospy.loginfo("  N42 saved: %s", filepath)

    def _save_heatmap_csv(self, prefix, cloud):
        """Save full sphere heatmap as CSV (raw irregular points, background-subtracted)."""
        filepath = os.path.join(self.output_dir, "{}_heatmap_raw.csv".format(prefix))

        with open(filepath, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "z", "intensity_uSv_h", "cs137_uSv_h", "co60_uSv_h"])
            if cloud is not None and len(cloud) > 0:
                # Background subtraction: remove the floor (5th percentile)
                intensity = cloud[:, 3].copy()
                cs137 = cloud[:, 4].copy()
                co60 = cloud[:, 5].copy()

                floor_i = np.percentile(intensity, 5) if len(intensity) > 0 else 0.0
                floor_c = np.percentile(cs137, 5) if len(cs137) > 0 else 0.0
                floor_o = np.percentile(co60, 5) if len(co60) > 0 else 0.0

                intensity = np.maximum(intensity - floor_i, 0.0)
                cs137 = np.maximum(cs137 - floor_c, 0.0)
                co60 = np.maximum(co60 - floor_o, 0.0)

                # Only write points above 5% of max (skip the zero-background majority)
                max_val = intensity.max() if intensity.max() > 0 else 1.0
                threshold = max_val * 0.05

                count = 0
                for i in range(len(cloud)):
                    if intensity[i] >= threshold:
                        writer.writerow(["{:.6f}".format(cloud[i, 0]),
                                         "{:.6f}".format(cloud[i, 1]),
                                         "{:.6f}".format(cloud[i, 2]),
                                         "{:.6f}".format(intensity[i]),
                                         "{:.6f}".format(cs137[i]),
                                         "{:.6f}".format(co60[i])])
                        count += 1
                rospy.loginfo("  Heatmap raw CSV saved: %s (%d/%d points above threshold)",
                              filepath, count, len(cloud))
            else:
                rospy.logwarn("  Heatmap raw CSV saved (empty - no cloud data received)")

    @staticmethod
    def _gaussian_smooth(grid, sigma=3.0):
        """2D Gaussian smoothing - same as live 2D heatmap."""
        from scipy.ndimage import gaussian_filter
        return gaussian_filter(grid, sigma=sigma)

    def _save_heatmap_raster(self, prefix, cloud, peaks=None):
        """Save rasterised heatmap on regular 5mm Y-Z grid.

        Uses the same algorithm as the live 2D heatmap: per-isotope grid
        projection with tight Gaussian blobs around detected peaks for
        clean source separation.
        """
        filepath = os.path.join(self.output_dir, "{}_heatmap_raster.csv".format(prefix))
        cell = 0.005  # 5mm cells
        radius = self.source_distance_m  # sphere radius
        fov = radius * 1.1  # slightly larger than sphere radius

        if cloud is None or len(cloud) == 0:
            with open(filepath, 'w') as f:
                f.write("x,y,z,intensity_uSv_h,cs137_uSv_h,co60_uSv_h\n")
            rospy.logwarn("  Heatmap raster CSV saved (empty)")
            return

        from scipy.ndimage import gaussian_filter

        res = 200  # same as live heatmap
        extent = 0.2  # +/-200mm (400mm x 400mm)
        y_edges = np.linspace(-extent, extent, res + 1)
        z_edges = np.linspace(-extent, extent, res + 1)
        yc = np.linspace(-extent, extent, res)
        zc = np.linspace(-extent, extent, res)
        YY, ZZ = np.meshgrid(yc, zc)
        r2 = YY**2 + ZZ**2
        R2 = radius**2

        # Extract sphere coordinates and scores
        points_yz = cloud[:, 1:3]  # Y, Z
        intensity = cloud[:, 3]
        cs137_scores = cloud[:, 4]
        co60_scores = cloud[:, 5]

        # Background subtraction
        valid_mask = intensity > 0
        if valid_mask.any():
            floor_i = np.median(intensity[valid_mask])
            floor_c = np.median(cs137_scores[valid_mask])
            floor_o = np.median(co60_scores[valid_mask])
        else:
            floor_i = floor_c = floor_o = 0.0

        intensity_sub = np.maximum(intensity - floor_i, 0.0)
        cs137_sub = np.maximum(cs137_scores - floor_c, 0.0)
        co60_sub = np.maximum(co60_scores - floor_o, 0.0)

        def _project_to_2d_grid(scores):
            """Identical to plot_live_2d_heatmap._project_to_2d_grid."""
            grid = np.zeros((res, res), dtype=np.float64)
            counts = np.zeros((res, res), dtype=np.float64)
            y_idx = np.digitize(cloud[:, 1], y_edges) - 1
            z_idx = np.digitize(cloud[:, 2], z_edges) - 1
            valid = (y_idx >= 0) & (y_idx < res) & (z_idx >= 0) & (z_idx < res)
            for i in range(len(cloud)):
                if valid[i]:
                    grid[z_idx[i], y_idx[i]] += scores[i]
                    counts[z_idx[i], y_idx[i]] += 1.0
            mask = counts > 0
            grid[mask] /= counts[mask]
            grid = gaussian_filter(grid, sigma=5.0)
            cos_factor = np.sqrt(np.clip(1.0 - r2 / R2, 0.0, 1.0))
            grid *= cos_factor
            grid[r2 > R2] = 0.0
            return grid

        grid_cs137 = _project_to_2d_grid(cs137_sub)
        grid_co60 = _project_to_2d_grid(co60_sub)

        # Per-peak pure Gaussian blobs scaled by smoothed grid intensity
        if peaks and len(peaks) > 0:
            sigma_blob = 0.05  # 50mm - smooth blob matching live heatmap appearance

            grid_intensity = np.zeros((res, res), dtype=np.float64)

            for peak in peaks:
                py, pz = peak[1], peak[2]
                iso_name = peak[3] if len(peak) > 3 else ''

                # Get amplitude from the smoothed grid at peak pixel position
                peak_yi = int(np.clip((py + extent) / (2.0 * extent) * res, 0, res - 1))
                peak_zi = int(np.clip((pz + extent) / (2.0 * extent) * res, 0, res - 1))

                if 'Cs-137' in iso_name or 'Cs137' in iso_name:
                    amp = float(grid_cs137[peak_zi, peak_yi])
                elif 'Co-60' in iso_name or 'Co60' in iso_name:
                    amp = float(grid_co60[peak_zi, peak_yi])
                else:
                    amp = float(max(grid_cs137[peak_zi, peak_yi],
                                    grid_co60[peak_zi, peak_yi]))
                if amp < 1e-12:
                    continue

                # Pure Gaussian blob - no grid multiplication
                dist = np.sqrt((YY - py)**2 + (ZZ - pz)**2)
                blob = amp * np.exp(-0.5 * (dist / sigma_blob)**2)

                grid_intensity = np.maximum(grid_intensity, blob)
        else:
            grid_intensity = _project_to_2d_grid(intensity_sub)

        # Write CSV
        count = 0
        x_coord = radius
        with open(filepath, 'w') as f:
            f.write("x,y,z,intensity\n")
            for zi in range(res):
                for yi in range(res):
                    f.write("{:.4f},{:.4f},{:.4f},{:.6f}\n".format(
                        x_coord, yc[yi], zc[zi],
                        grid_intensity[zi, yi]))
                    count += 1

        rospy.loginfo("  Heatmap raster CSV saved: %s (%d cells, res=%d)",
                      filepath, count, res)

    def _save_heatmap_3d(self, prefix, cloud, peaks):
        """Save 3D heatmap: localized blobs on a flat Y-Z plane at x=distance.

        Projects sphere back-projection scores onto a 0.5m x 0.5m plane using
        ray-plane intersection (Y_real = distance * Y_sphere / X_sphere), then
        applies Gaussian-weighted blobs around detected peaks.

        Output: x, y, z, dose_rate_uSv_h (only points above 10% of peak)
        Suitable for machine vision / robot picking.
        """
        filepath = os.path.join(self.output_dir, "{}_heatmap_3d.csv".format(prefix))
        distance = self.source_distance_m
        res = self.heatmap_grid_res
        blob_radius = 0.10  # metres - tight blob for good peak separation

        if cloud is None or len(cloud) == 0:
            with open(filepath, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "z", "dose_rate_uSv_h"])
            rospy.logwarn("  Heatmap 3D CSV saved (empty - no cloud data)")
            return

        # Fixed 0.5m x 0.5m plane: Y in [-0.5, 0.5], Z in [-0.5, 0.5]
        extent = 0.5
        y_edges = np.linspace(-extent, extent, res + 1)
        z_edges = np.linspace(-extent, extent, res + 1)
        y_centers = 0.5 * (y_edges[:-1] + y_edges[1:])
        z_centers = 0.5 * (z_edges[:-1] + z_edges[1:])
        YY, ZZ = np.meshgrid(y_centers, z_centers)

        # Step 1: Convert sphere points to real-world Y-Z via ray-plane intersection
        # Y_real = distance * Y_sphere / X_sphere
        pts_x = cloud[:, 0]
        pts_y = cloud[:, 1]
        pts_z = cloud[:, 2]
        scores = cloud[:, 3]  # intensity column

        # Only use points with positive X (forward hemisphere)
        valid_x = pts_x > 1e-6
        real_y = np.where(valid_x, distance * pts_y / pts_x, 0.0)
        real_z = np.where(valid_x, distance * pts_z / pts_x, 0.0)

        # Bin onto Y-Z grid
        grid_sum = np.zeros((res, res), dtype=np.float64)
        grid_count = np.zeros((res, res), dtype=np.float64)

        y_idx = np.digitize(real_y, y_edges) - 1
        z_idx = np.digitize(real_z, z_edges) - 1
        valid = valid_x & (y_idx >= 0) & (y_idx < res) & (z_idx >= 0) & (z_idx < res)

        for i in range(len(cloud)):
            if valid[i]:
                grid_sum[z_idx[i], y_idx[i]] += scores[i]
                grid_count[z_idx[i], y_idx[i]] += 1.0

        base_grid = np.zeros((res, res), dtype=np.float64)
        mask = grid_count > 0
        base_grid[mask] = grid_sum[mask] / grid_count[mask]

        # Smooth the base grid (3x3 average, 3 passes)
        for _ in range(3):
            padded = np.pad(base_grid, 1, mode='constant')
            base_grid = (padded[:-2, :-2] + padded[:-2, 1:-1] + padded[:-2, 2:] +
                         padded[1:-1, :-2] + padded[1:-1, 1:-1] + padded[1:-1, 2:] +
                         padded[2:, :-2] + padded[2:, 1:-1] + padded[2:, 2:]) / 9.0

        # Step 2: Extract unique peaks (already in real-world coordinates)
        unique_peaks = {}
        for entry in peaks:
            # entry = (x, y, z, isotope_name, activity_MBq)
            iso_name = entry[3]
            unique_peaks[iso_name] = (entry[1], entry[2])  # (py, pz) real-world

        if not unique_peaks:
            rospy.logwarn("  Heatmap 3D: no peaks detected, saving empty file")
            with open(filepath, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "z", "dose_rate_uSv_h"])
            return

        # Step 3: For each peak, create Gaussian-weighted blob from base grid
        grid = np.zeros((res, res), dtype=np.float64)
        sigma = blob_radius * 0.4

        # Use dose rate for amplitude weighting: D_dot = Gamma * A / d^2
        gamma_constants = {'Cs-137': 0.0771, 'Co-60': 0.3059}
        d = self.source_distance_m
        dose_rates = {}
        for entry in peaks:
            iso_name = entry[3]
            activity_mbq = entry[4] if len(entry) > 4 else 0.0
            gamma = gamma_constants.get(iso_name, 0.077)
            dr = gamma * activity_mbq / (d * d) if d > 0 else 0.0
            if iso_name in dose_rates:
                dose_rates[iso_name] = max(dose_rates[iso_name], dr)
            else:
                dose_rates[iso_name] = dr
        max_rate = max(dose_rates.values()) if dose_rates else 1.0
        if max_rate < 1e-9:
            max_rate = 1.0

        for iso_name, (py, pz) in unique_peaks.items():
            amp = dose_rates.get(iso_name, 0.0) / max_rate
            if amp < 0.1:
                amp = 0.5  # fallback if no activity data
            dist = np.sqrt((YY - py)**2 + (ZZ - pz)**2)
            falloff = np.exp(-0.5 * (dist / sigma)**2)
            blob = falloff * base_grid * amp
            grid = np.maximum(grid, blob)

        # Intensity values are dose-rate-weighted (not normalized)

        # Write only hot points (above 10% of peak value)
        grid_max = grid.max()
        threshold = grid_max * 0.10 if grid_max > 1e-12 else 0.0
        count = 0
        with open(filepath, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "z", "dose_rate_uSv_h"])
            for zi in range(res):
                for yi in range(res):
                    val = grid[zi, yi]
                    if val >= threshold:
                        writer.writerow([
                            "{:.4f}".format(distance),
                            "{:.6f}".format(y_centers[yi]),
                            "{:.6f}".format(z_centers[zi]),
                            "{:.6f}".format(val)
                        ])
                        count += 1

        rospy.loginfo("  Heatmap 3D CSV saved: %s (%d hot points of %dx%d grid, %d peaks, x=%.2fm)",
                      filepath, count, res, res, len(unique_peaks), distance)

    def _save_activity_csv(self, prefix, activities, detector_run_info,
                           run_real_time_ms=0, run_live_time_ms=0):
        """Save activity measurement results as CSV.

        Note on time columns:
          - real_time_s / live_time_s are PER counting window (~60 s each); they
            are the integration time for that window's counts and must stay
            per-window for the activity (count-rate) maths.
          - run_real_time_s / run_live_time_s are the WHOLE-RUN totals
            accumulated across the recording (e.g. ~300 s for a 5-minute scan),
            so the full scan duration is visible in every row.
        """
        import json
        filepath = os.path.join(self.output_dir, "{}_activity.csv".format(prefix))

        run_real_time_s = run_real_time_ms / 1000.0
        run_live_time_s = run_live_time_ms / 1000.0

        with open(filepath, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp_s", "real_time_s", "live_time_s", "dead_time_fraction", "dead_time_status",
                "detector_run_info_valid", "detector_run_dead_time_percent",
                "detector_run_real_time_s", "detector_run_live_time_s",
                "isotope", "energy_keV", "gross_counts", "background_counts",
                "net_peak_area", "net_corrected", "count_rate_cps",
                "activity_MBq", "sigma_activity_MBq", "valid",
                "run_real_time_s", "run_live_time_s"
            ])

            for report in activities:
                ts = report.get('timestamp', 0)
                rt = report.get('real_time_s', 0)
                lt = report.get('live_time_s', 0)
                dtf = report.get('dead_time_fraction', 0)
                dts = report.get('dead_time_status', 'unknown')
                drv = detector_run_info.get('valid', False)
                drdt = detector_run_info.get('dead_time_percent', 0.0)
                drrt = detector_run_info.get('real_time_sec', 0.0)
                drlt = detector_run_info.get('live_time_sec', 0.0)

                for iso in report.get('isotopes', []):
                    writer.writerow([
                        "{:.3f}".format(ts),
                        "{:.3f}".format(rt),
                        "{:.3f}".format(lt),
                        "{:.6f}".format(dtf),
                        dts,
                        drv,
                        "{:.6f}".format(drdt),
                        "{:.3f}".format(drrt),
                        "{:.3f}".format(drlt),
                        iso.get('isotope', ''),
                        "{:.1f}".format(iso.get('energy_keV', 0)),
                        "{:.0f}".format(iso.get('gross_counts', 0)),
                        "{:.1f}".format(iso.get('background_counts', 0)),
                        "{:.1f}".format(iso.get('net_peak_area', 0)),
                        "{:.1f}".format(iso.get('net_corrected', 0)),
                        "{:.4f}".format(iso.get('count_rate_cps', 0)),
                        "{:.6f}".format(iso.get('activity_MBq', 0)),
                        "{:.6f}".format(iso.get('sigma_activity_MBq', 0)),
                        iso.get('valid', False),
                        "{:.3f}".format(run_real_time_s),
                        "{:.3f}".format(run_live_time_s)
                    ])

        rospy.loginfo("  Activity CSV saved: %s (%d measurement windows)",
                      filepath, len(activities))


def main():
    rospy.init_node("data_recorder_node")
    node = DataRecorderNode()
    rospy.spin()


if __name__ == "__main__":
    main()

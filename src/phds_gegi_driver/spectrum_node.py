#!/usr/bin/env python
"""
Spectrum Node - Standalone raw energy spectrum accumulator.

Subscribes to /energy_deposit (Float64) published by the GeGI driver for every
single-site and two-site interaction energy. Accumulates into a configurable
histogram and publishes /spectrum (radiation_detector_msgs/Spectrum) periodically.

Parameters:
  ~calibration_file (str): Path to EnergyCal.csv (energy bin edges in keV)
  ~publish_rate_hz (float): Spectrum publish rate (default: 4.0 Hz)
  ~energy_topic (str): Input topic (default: /energy_deposit)
  ~spectrum_topic (str): Output topic (default: /spectrum)
  ~detector_frame (str): Frame ID for spectrum header (default: detector)
  ~use_run_info_deadtime (bool): Populate deadTime_ms from detector run-info
      dead_time_percent (default: true)
  ~run_info_service (str): Service name for detector run info (default:
      /detector/get_run_info)
  ~dead_time_poll_hz (float): Maximum rate for polling run-info dead-time
      when ~use_run_info_deadtime is true (default: 1.0)
  ~use_dead_time_topic (bool): Use a shared dead-time percentage topic instead
      of polling run-info in this node (default: false)
  ~dead_time_topic (str): Topic carrying dead-time percentage as Float64
      (default: /detector/dead_time_percent)
  ~publish_dead_time_topic (bool): Publish this node's dead-time percentage to
      ~dead_time_topic for other nodes to consume (default: false)
"""
from __future__ import print_function

import threading
import os

import numpy as np
import rospy
from std_msgs.msg import Float64
from radiation_detector_msgs.msg import Spectrum
from std_srvs.srv import Trigger, TriggerResponse
from phds_gegi_driver.srv import GetRunInfo


class SpectrumNode(object):
    def __init__(self):
        cal_path = rospy.get_param("~calibration_file", "")
        self.publish_rate = rospy.get_param("~publish_rate_hz", 4.0)
        energy_topic = rospy.get_param("~energy_topic", "/energy_deposit")
        spectrum_topic = rospy.get_param("~spectrum_topic", "/spectrum")
        self.detector_frame = rospy.get_param("~detector_frame", "detector")
        # IMPORTANT: run-info polling shares the same detector TCP channel as
        # event streaming. Enabling it continuously during acquisition can cause
        # command/stream contention on some firmware versions.
        self.use_run_info_deadtime = rospy.get_param("~use_run_info_deadtime", False)
        self.run_info_service = rospy.get_param("~run_info_service", "/detector/get_run_info")
        self.dead_time_poll_hz = float(rospy.get_param("~dead_time_poll_hz", 1.0))
        self.use_dead_time_topic = rospy.get_param("~use_dead_time_topic", False)
        self.dead_time_topic = rospy.get_param("~dead_time_topic", "/detector/dead_time_percent")
        self.publish_dead_time_topic = rospy.get_param("~publish_dead_time_topic", False)
        self._last_dead_time_percent = 0.0
        self._run_info_proxy = None
        self._dead_time_poll_period_s = 1.0 / max(self.dead_time_poll_hz, 0.1)
        self._last_run_info_query_wall_s = 0.0
        self._dead_time_lock = threading.Lock()
        self._dead_time_sub = None
        self._dead_time_pub = None

        # Load energy calibration (bin edges in keV)
        self.bin_edges = self._load_calibration(cal_path)
        self.n_bins = len(self.bin_edges)
        rospy.loginfo("Spectrum node: %d bins, publish rate %.1f Hz", self.n_bins, self.publish_rate)

        self.spectrum = np.zeros(self.n_bins, dtype=np.uint32)
        self.lock = threading.Lock()
        self.seq = 0

        self.sub = rospy.Subscriber(energy_topic, Float64, self._on_energy, queue_size=10000)
        self.pub = rospy.Publisher(spectrum_topic, Spectrum, queue_size=2)
        self.clear_srv = rospy.Service("~clear", Trigger, self._handle_clear)

        if self.use_dead_time_topic:
            self._dead_time_sub = rospy.Subscriber(
                self.dead_time_topic, Float64, self._on_dead_time_percent, queue_size=5
            )
            rospy.loginfo("Spectrum node: using shared dead-time topic %s", self.dead_time_topic)

        if self.publish_dead_time_topic:
            self._dead_time_pub = rospy.Publisher(self.dead_time_topic, Float64, queue_size=5, latch=True)
            rospy.loginfo("Spectrum node: publishing dead-time percentage to %s", self.dead_time_topic)

        if self.use_run_info_deadtime:
            try:
                rospy.wait_for_service(self.run_info_service, timeout=5.0)
                self._run_info_proxy = rospy.ServiceProxy(self.run_info_service, GetRunInfo)
                rospy.loginfo("Spectrum node: using %s for dead-time", self.run_info_service)
            except Exception as e:
                rospy.logwarn("Spectrum node: dead-time service unavailable (%s). "
                              "Using zero dead-time until service is available.", str(e))

        period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = rospy.Timer(rospy.Duration(period), self._publish)

        # Dead-time polling runs on its OWN timer thread so that a slow or
        # blocking run-info service call (the detector multiplexes commands and
        # the event stream on a single TCP link) can never stall the spectrum
        # publish loop. The publish hot path only reads the cached value.
        self.dead_time_timer = None
        if self.use_run_info_deadtime:
            self.dead_time_timer = rospy.Timer(
                rospy.Duration(self._dead_time_poll_period_s), self._poll_run_info)

    def _load_calibration(self, path):
        if not path or not os.path.exists(path):
            rospy.logwarn("No calibration file, using default 1024 bins (0-3000 keV)")
            return np.linspace(0.0, 3000.0, 1024)
        values = []
        with open(path, 'r') as f:
            for line in f:
                text = line.strip()
                if text:
                    values.append(float(text))
        return np.array(values, dtype=np.float64)

    def _on_energy(self, msg):
        energy = msg.data
        idx = np.searchsorted(self.bin_edges, energy, side='right') - 1
        if idx < 0 or idx >= self.n_bins:
            return
        with self.lock:
            self.spectrum[idx] += 1

    def _publish(self, event):
        msg = Spectrum()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.detector_frame
        msg.header.seq = self.seq
        self.seq += 1

        period_ms = int(1000.0 / max(self.publish_rate, 0.1))
        msg.realTime_ms = period_ms
        msg.deadTime_ms = self._get_dead_time_ms(period_ms)
        msg.endTime = rospy.Time.now()

        with self.lock:
            msg.spectrum = self.spectrum.tolist()
            msg.totalCount = int(self.spectrum.sum())
            self.spectrum[:] = 0

        self.pub.publish(msg)

    def _get_dead_time_ms(self, period_ms):
        """Non-blocking: convert the latest cached dead-time % to milliseconds.

        Runs on the publish hot path, so it MUST NOT make blocking service
        calls. The actual run-info query happens on a separate timer thread
        (`_poll_run_info`); here we only read the most recent cached value.
        """
        if not self.use_run_info_deadtime and not self.use_dead_time_topic:
            return 0

        with self._dead_time_lock:
            dead_time_percent = self._last_dead_time_percent

        # Re-publish the shared dead-time topic for downstream nodes.
        if self._dead_time_pub is not None:
            self._dead_time_pub.publish(Float64(data=dead_time_percent))

        return int(round(period_ms * dead_time_percent / 100.0))

    def _poll_run_info(self, event):
        """Background poll of the detector run-info service for dead-time.

        Runs on a dedicated rospy.Timer thread so a slow/blocking run-info
        response can never stall spectrum publishing. Polls at most once per
        ``_dead_time_poll_period_s`` (the timer period).
        """
        if self._run_info_proxy is None:
            try:
                rospy.wait_for_service(self.run_info_service, timeout=0.2)
                self._run_info_proxy = rospy.ServiceProxy(self.run_info_service, GetRunInfo)
            except Exception:
                return

        try:
            resp = self._run_info_proxy()
            if resp.success and np.isfinite(resp.dead_time_percent):
                with self._dead_time_lock:
                    self._last_dead_time_percent = max(0.0, min(100.0, float(resp.dead_time_percent)))
            else:
                rospy.logwarn_throttle(10.0, "Spectrum node: invalid run-info dead-time response")
        except Exception as e:
            rospy.logwarn_throttle(10.0, "Spectrum node: failed calling %s (%s)",
                                   self.run_info_service, str(e))

    def _on_dead_time_percent(self, msg):
        if not np.isfinite(msg.data):
            return
        with self._dead_time_lock:
            self._last_dead_time_percent = max(0.0, min(100.0, float(msg.data)))

    def _handle_clear(self, req):
        with self.lock:
            total = int(self.spectrum.sum())
            self.spectrum[:] = 0
        rospy.loginfo("Spectrum cleared (%d counts discarded)", total)
        return TriggerResponse(success=True, message="Cleared {} counts".format(total))


def main():
    rospy.init_node("spectrum_node")
    node = SpectrumNode()
    rospy.spin()


if __name__ == "__main__":
    main()

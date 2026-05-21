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
"""
from __future__ import print_function

import threading
import os

import numpy as np
import rospy
from std_msgs.msg import Float64
from radiation_detector_msgs.msg import Spectrum
from std_srvs.srv import Trigger, TriggerResponse


class SpectrumNode(object):
    def __init__(self):
        cal_path = rospy.get_param("~calibration_file", "")
        self.publish_rate = rospy.get_param("~publish_rate_hz", 4.0)
        energy_topic = rospy.get_param("~energy_topic", "/energy_deposit")
        spectrum_topic = rospy.get_param("~spectrum_topic", "/spectrum")
        self.detector_frame = rospy.get_param("~detector_frame", "detector")

        # Load energy calibration (bin edges in keV)
        self.bin_edges = self._load_calibration(cal_path)
        self.n_bins = len(self.bin_edges)
        rospy.loginfo("Spectrum node: %d bins, publish rate %.1f Hz", self.n_bins, self.publish_rate)

        self.spectrum = np.zeros(self.n_bins, dtype=np.uint32)
        self.lock = threading.Lock()
        self.seq = 0

        self.sub = rospy.Subscriber(energy_topic, Float64, self._on_energy, queue_size=500)
        self.pub = rospy.Publisher(spectrum_topic, Spectrum, queue_size=2)
        self.clear_srv = rospy.Service("~clear", Trigger, self._handle_clear)

        period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = rospy.Timer(rospy.Duration(period), self._publish)

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
        msg.deadTime_ms = 0
        msg.endTime = rospy.Time.now()

        with self.lock:
            msg.spectrum = self.spectrum.tolist()
            msg.totalCount = int(self.spectrum.sum())
            self.spectrum[:] = 0

        self.pub.publish(msg)

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

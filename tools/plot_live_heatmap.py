#!/usr/bin/env python
"""
Live Spherical Heatmap Plotter - Runs natively on Windows via rosbridge websocket.

Connects to rosbridge_server in the Docker container and subscribes to
/sphere_heatmap (PointCloud2), /source_directions, and /source_isotopes.
Displays a 3D scatter plot colored by back-projection score.

Prerequisites:
  - rosbridge running in container (started automatically by full pipeline)
  - pip install roslibpy matplotlib numpy

Usage:
  python plot_live_heatmap.py [--host localhost] [--port 9090]
"""
import argparse
import base64
import struct
import threading

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import matplotlib.animation as animation
import roslibpy


def parse_pointcloud2_rosbridge(msg):
    """Extract xyz and score from a rosbridge PointCloud2 JSON message."""
    fields = msg.get('fields', [])
    field_map = {f['name']: f for f in fields}
    x_off = field_map['x']['offset']
    y_off = field_map['y']['offset']
    z_off = field_map['z']['offset']
    has_rgb = 'rgb' in field_map
    rgb_off = field_map['rgb']['offset'] if has_rgb else None

    step = msg['point_step']
    # rosbridge sends data as base64-encoded string
    raw = base64.b64decode(msg.get('data', ''))

    points = []
    scores = []
    for i in range(0, len(raw), step):
        if i + z_off + 4 > len(raw):
            break
        x = struct.unpack_from('<f', raw, i + x_off)[0]
        y = struct.unpack_from('<f', raw, i + y_off)[0]
        z = struct.unpack_from('<f', raw, i + z_off)[0]
        points.append([x, y, z])

        if has_rgb and i + rgb_off + 4 <= len(raw):
            rgb_bytes = struct.unpack_from('<I', raw, i + rgb_off)[0]
            r = (rgb_bytes >> 16) & 0xFF
            scores.append(r / 255.0)
        else:
            scores.append(0.5)

    return np.array(points) if points else np.zeros((0, 3)), np.array(scores)


class LiveHeatmapPlotter(object):
    def __init__(self, host, port, cloud_topic, peak_topic, isotope_topic):
        self.points = None
        self.scores = None
        self.peak_dirs = []
        self.isotope_text = ""
        self.lock = threading.Lock()
        self.new_data = False

        self.client = roslibpy.Ros(host=host, port=port)

        self.cloud_listener = roslibpy.Topic(
            self.client, cloud_topic, 'sensor_msgs/PointCloud2')
        self.cloud_listener.subscribe(self._cloud_cb)

        self.peaks_listener = roslibpy.Topic(
            self.client, peak_topic, 'geometry_msgs/PoseArray')
        self.peaks_listener.subscribe(self._peaks_cb)

        self.isotope_listener = roslibpy.Topic(
            self.client, isotope_topic, 'std_msgs/String')
        self.isotope_listener.subscribe(self._isotope_cb)

    def _cloud_cb(self, msg):
        pts, scores = parse_pointcloud2_rosbridge(msg)
        with self.lock:
            self.points = pts
            self.scores = scores
            self.new_data = True

    def _peaks_cb(self, msg):
        dirs = []
        for pose in msg.get('poses', []):
            pos = pose.get('position', {})
            dirs.append([pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)])
        with self.lock:
            self.peak_dirs = dirs

    def _isotope_cb(self, msg):
        with self.lock:
            self.isotope_text = msg.get('data', '')

    def run(self):
        self.client.run()
        print("Connected to rosbridge. Waiting for heatmap data...")

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("GeGI Spherical Heatmap")

        def update(frame):
            with self.lock:
                if not self.new_data or self.points is None:
                    return []
                pts = self.points.copy()
                scores = self.scores.copy()
                peaks = list(self.peak_dirs)
                iso_text = self.isotope_text
                self.new_data = False

            if len(pts) == 0:
                return []

            ax.cla()
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")

            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                       c=scores, cmap='hot', s=2, alpha=0.6)

            if peaks:
                peak_arr = np.array(peaks)
                ax.scatter(peak_arr[:, 0], peak_arr[:, 1], peak_arr[:, 2],
                           c='gold', s=200, marker='*', edgecolors='black',
                           linewidths=0.8, zorder=10)

            ax.set_title("GeGI Spherical Heatmap")
            if iso_text:
                ax.text2D(0.5, 0.92, iso_text, transform=ax.transAxes,
                          fontsize=10, color='black', fontweight='bold',
                          ha='center', va='top')

            max_range = abs(pts).max() * 1.2 if len(pts) > 0 else 0.6
            ax.set_xlim(-max_range, max_range)
            ax.set_ylim(-max_range, max_range)
            ax.set_zlim(-max_range, max_range)
            return []

        ani = animation.FuncAnimation(fig, update, interval=2000, blit=False)
        plt.tight_layout()
        plt.show()
        self.client.terminate()


def main():
    parser = argparse.ArgumentParser(description="Live GeGI spherical heatmap plotter (via rosbridge)")
    parser.add_argument("--host", default="localhost", help="rosbridge host (default: localhost)")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge port (default: 9090)")
    parser.add_argument("--cloud-topic", default="/sphere_heatmap", help="PointCloud2 topic")
    parser.add_argument("--peak-topic", default="/source_directions", help="PoseArray peak topic")
    parser.add_argument("--isotope-topic", default="/source_isotopes", help="Isotope ID topic")
    args = parser.parse_args()

    plotter = LiveHeatmapPlotter(args.host, args.port, args.cloud_topic,
                                  args.peak_topic, args.isotope_topic)
    plotter.run()


if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()

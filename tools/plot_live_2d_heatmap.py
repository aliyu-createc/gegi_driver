#!/usr/bin/env python
"""
Per-Isotope 2D Heatmap Plotter - Projects spherical heatmap onto Y-Z plane.

Connects via rosbridge and subscribes to:
  /sphere_heatmap (PointCloud2) - spherical score distribution
  /source_directions (PoseArray) - peak directions
  /source_isotopes (String) - isotope identification labels

Produces a 2D heatmap view (Y vs Z) with isotope-labeled peak markers,
matching the legacy live_heatmap_node visualization style.

Usage:
  python plot_live_2d_heatmap.py [--host localhost] [--port 9090] [--radius 0.5]
"""
import argparse
import base64
import struct
import threading

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MultipleLocator
from matplotlib.colors import Normalize
from scipy.ndimage import gaussian_filter
import roslibpy


def parse_pointcloud2_xyz_score(msg):
    """Extract xyz positions and per-isotope scores from rosbridge PointCloud2 JSON."""
    fields = msg.get('fields', [])
    field_map = {f['name']: f for f in fields}
    x_off = field_map['x']['offset']
    y_off = field_map['y']['offset']
    z_off = field_map['z']['offset']
    has_cs137 = 'cs137' in field_map
    has_co60 = 'co60' in field_map
    cs137_off = field_map['cs137']['offset'] if has_cs137 else None
    co60_off = field_map['co60']['offset'] if has_co60 else None
    has_intensity = 'intensity' in field_map
    intensity_off = field_map['intensity']['offset'] if has_intensity else None

    step = msg['point_step']
    raw = base64.b64decode(msg.get('data', ''))

    points = []
    scores_cs137 = []
    scores_co60 = []
    scores_total = []
    for i in range(0, len(raw), step):
        if i + z_off + 4 > len(raw):
            break
        x = struct.unpack_from('<f', raw, i + x_off)[0]
        y = struct.unpack_from('<f', raw, i + y_off)[0]
        z = struct.unpack_from('<f', raw, i + z_off)[0]
        points.append([x, y, z])

        if has_cs137 and i + cs137_off + 4 <= len(raw):
            scores_cs137.append(struct.unpack_from('<f', raw, i + cs137_off)[0])
        else:
            scores_cs137.append(0.0)

        if has_co60 and i + co60_off + 4 <= len(raw):
            scores_co60.append(struct.unpack_from('<f', raw, i + co60_off)[0])
        else:
            scores_co60.append(0.0)

        if has_intensity and i + intensity_off + 4 <= len(raw):
            scores_total.append(struct.unpack_from('<f', raw, i + intensity_off)[0])
        else:
            scores_total.append(0.0)

    pts = np.array(points) if points else np.zeros((0, 3))
    return pts, {
        'cs137': np.array(scores_cs137),
        'co60': np.array(scores_co60),
        'total': np.array(scores_total),
    }


class PerIsotopeHeatmapPlotter(object):
    def __init__(self, host, port, cloud_topic, peak_topic, isotope_topic, radius):
        self.radius = radius
        self.points = None
        self.iso_scores = None
        self.peaks = []
        self.isotope_text = ""
        self.activity_data = {}  # isotope_name -> dose_rate_uSv_h
        # Specific gamma-ray constants (uSv*m^2 / MBq*h)
        self.gamma_constants = {'Cs-137': 0.0771, 'Co-60': 0.3059}
        self.lock = threading.Lock()
        self.new_data = False

        # Grid for 2D projection
        self.grid_res = 200
        self.grid_extent = radius * 1.1  # slightly larger than sphere radius

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

        self.activity_listener = roslibpy.Topic(
            self.client, '/activity/results', 'std_msgs/String')
        self.activity_listener.subscribe(self._activity_cb)

    def _cloud_cb(self, msg):
        pts, iso_scores = parse_pointcloud2_xyz_score(msg)
        with self.lock:
            self.points = pts
            self.iso_scores = iso_scores
            self.new_data = True

    def _peaks_cb(self, msg):
        peaks = []
        for pose in msg.get('poses', []):
            pos = pose.get('position', {})
            peaks.append([pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)])
        with self.lock:
            self.peaks = peaks

    def _isotope_cb(self, msg):
        with self.lock:
            self.isotope_text = msg.get('data', '')

    def _activity_cb(self, msg):
        import json
        try:
            data = json.loads(msg.get('data', '{}'))
            dose_rates = {}
            # Accumulate activity per display isotope (sum peaks for Co60)
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
                # For Co-60 both peaks give the same source activity; take max
                if display in activities:
                    activities[display] = max(activities[display], activity_mbq)
                else:
                    activities[display] = activity_mbq
            # Convert activity to dose rate: D_dot = Gamma * A / d^2
            d = self.radius  # source distance = sphere radius
            for display, a_mbq in activities.items():
                gamma = self.gamma_constants.get(display, 0.077)
                dose_rates[display] = gamma * a_mbq / (d * d)
            with self.lock:
                self.activity_data = dose_rates
        except (ValueError, TypeError):
            pass

    def _project_to_2d_grid(self, points, scores):
        """Project 3D sphere points onto Y-Z plane and create a 2D heatmap."""
        extent = self.grid_extent
        res = self.grid_res

        grid = np.zeros((res, res), dtype=np.float64)
        counts = np.zeros((res, res), dtype=np.float64)

        y_edges = np.linspace(-extent, extent, res + 1)
        z_edges = np.linspace(-extent, extent, res + 1)

        if len(points) == 0:
            return grid

        # Bin points by their Y-Z coordinates
        y_idx = np.digitize(points[:, 1], y_edges) - 1
        z_idx = np.digitize(points[:, 2], z_edges) - 1

        valid = (y_idx >= 0) & (y_idx < res) & (z_idx >= 0) & (z_idx < res)

        for i in range(len(points)):
            if valid[i]:
                grid[z_idx[i], y_idx[i]] += scores[i]
                counts[z_idx[i], y_idx[i]] += 1.0

        # Average where we have counts
        mask = counts > 0
        grid[mask] /= counts[mask]

        # Smooth for visual appearance
        grid = gaussian_filter(grid, sigma=3.0)

        # Correct for sphere-to-plane projection distortion.
        yc = np.linspace(-extent, extent, res)
        zc = np.linspace(-extent, extent, res)
        YY, ZZ = np.meshgrid(yc, zc)
        r2 = YY**2 + ZZ**2
        R2 = self.radius**2
        cos_factor = np.sqrt(np.clip(1.0 - r2 / R2, 0.0, 1.0))
        grid *= cos_factor
        # Zero outside sphere
        grid[r2 > R2] = 0.0

        return grid

    def _parse_isotope_labels(self, text):
        """Parse isotope text - node publishes pipe-separated 'Cs-137:150|Co-60:45'.
        Returns list of (name, count) tuples."""
        labels = []
        if not text or text == 'none':
            return labels
        parts = text.split('|')
        for part in parts:
            part = part.strip()
            # Format: "Isotope-Name:count"
            if ':' in part:
                name_part, count_part = part.rsplit(':', 1)
                try:
                    count = int(count_part)
                except ValueError:
                    count = 1
                name = name_part.strip()
            else:
                name = part
                count = 1
            # Validate isotope name
            matched = False
            for iso in ['Cs-137', 'Co-60', 'Am-241', 'Ba-133', 'Na-22']:
                if iso in name:
                    labels.append((iso, count))
                    matched = True
                    break
            if not matched and name:
                labels.append(('---', count))
        return labels

    def run(self):
        self.client.run()
        print("Connected to rosbridge. Waiting for heatmap data...")

        fig, ax = plt.subplots(figsize=(8, 8), facecolor='black')
        ax.set_facecolor('black')
        ax.set_xlabel("Y (left/right) [m]", color='white')
        ax.set_ylabel("Z (up/down) [m]", color='white')
        ax.set_title("Per-Isotope Heatmap", color='white', fontsize=14)
        ax.tick_params(colors='white')
        for spine in ax.spines.values():
            spine.set_color('white')

        extent = self.grid_extent
        im = [None]

        def update(frame):
            with self.lock:
                if not self.new_data or self.points is None:
                    return []
                pts = self.points.copy()
                iso_scores = {k: v.copy() for k, v in self.iso_scores.items()}
                peaks = list(self.peaks)
                iso_text = self.isotope_text
                self.new_data = False

            if len(pts) == 0:
                return []

            isotope_labels = self._parse_isotope_labels(iso_text)

            # Get dose-rate-based amplitudes for weighting
            with self.lock:
                dose_rates = dict(self.activity_data)

            # Collect per-peak amplitudes from dose rate (uSv/h)
            peak_amps = []
            for i in range(len(peaks)):
                if i < len(isotope_labels):
                    name, count = isotope_labels[i]
                else:
                    name, count = '---', 1
                # Use dose rate from activity node if available
                dr = dose_rates.get(name, 0.0)
                peak_amps.append((name, dr if dr > 0 else float(count)))

            max_amp = max(amp for _, amp in peak_amps) if peak_amps else 1.0
            if max_amp < 1e-6:
                max_amp = 1.0

            # Project per-isotope scores onto 2D grid
            grid_cs137 = self._project_to_2d_grid(pts, iso_scores.get('cs137', np.zeros(len(pts))))
            grid_co60 = self._project_to_2d_grid(pts, iso_scores.get('co60', np.zeros(len(pts))))

            # For each peak, extract a localized region from its isotope grid
            res = self.grid_res
            yc = np.linspace(-extent, extent, res)
            zc = np.linspace(-extent, extent, res)
            YY, ZZ = np.meshgrid(yc, zc)
            grid = np.zeros((res, res), dtype=np.float64)

            blob_radius = 0.10  # meters - tighter blob for better separation

            for i, peak in enumerate(peaks):
                py, pz = peak[1], peak[2]
                if i < len(peak_amps):
                    name, amp_val = peak_amps[i]
                    amp = amp_val / max_amp
                else:
                    name, amp = '---', 0.5

                # Select the right isotope grid
                if 'Cs-137' in name:
                    src_grid = grid_cs137
                elif 'Co-60' in name:
                    src_grid = grid_co60
                else:
                    src_grid = np.maximum(grid_cs137, grid_co60)

                # Circular mask around peak + Gaussian falloff (tight sigma for separation)
                dist = np.sqrt((YY - py)**2 + (ZZ - pz)**2)
                sigma = blob_radius * 0.4
                falloff = np.exp(-0.5 * (dist / sigma)**2)
                mask_region = falloff * src_grid * amp

                grid = np.maximum(grid, mask_region)

            # Normalize to [0, 1]
            gmax = grid.max()
            if gmax > 1e-9:
                grid /= gmax

            ax.cla()
            ax.set_facecolor('black')
            ax.set_xlabel("Y (left/right) [m]", color='white')
            ax.set_ylabel("Z (up/down) [m]", color='white')
            ax.set_title("Per-Isotope Heatmap", color='white', fontsize=14)
            ax.tick_params(colors='white')
            for spine in ax.spines.values():
                spine.set_color('white')

            # Display with 'jet' colormap (blue=low, red=high)
            im[0] = ax.imshow(grid, extent=[-extent, extent, -extent, extent],
                              origin='lower', cmap='jet', interpolation='bilinear',
                              aspect='equal', vmin=0, vmax=1)

            # Plot peak markers with isotope labels (real-world coordinates)
            for i, peak in enumerate(peaks):
                py, pz = peak[1], peak[2]
                if i < len(isotope_labels):
                    label, count = isotope_labels[i]
                else:
                    label, count = '---', 0

                ax.scatter(py, pz, marker='*', c='gold', s=250,
                           edgecolors='black', linewidths=0.8, zorder=10)

                # peak positions are now real-world (ray-plane) coordinates
                label_text = "{}\n({:.2f}cm, {:.2f}cm)".format(label, py*100, pz*100)
                offset_y = 0.05 if py >= 0 else -0.05
                offset_z = 0.06

                ax.annotate(label_text, xy=(py, pz),
                            xytext=(py + offset_y, pz + offset_z),
                            color='black', fontsize=10, fontweight='bold',
                            ha='center', va='bottom',
                            bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7, edgecolor='black'),
                            arrowprops=dict(arrowstyle='-', color='black', lw=1.5))

            ax.set_xlim(-extent, extent)
            ax.set_ylim(-extent, extent)
            ax.xaxis.set_major_locator(MultipleLocator(0.05))
            ax.yaxis.set_major_locator(MultipleLocator(0.05))

            return []

        ani = animation.FuncAnimation(fig, update, interval=2000, blit=False)
        plt.tight_layout()
        plt.show()
        self.client.terminate()


def main():
    parser = argparse.ArgumentParser(description="Per-isotope 2D heatmap plotter (via rosbridge)")
    parser.add_argument("--host", default="localhost", help="rosbridge host")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
    parser.add_argument("--cloud-topic", default="/sphere_heatmap", help="PointCloud2 topic")
    parser.add_argument("--peak-topic", default="/source_directions", help="PoseArray topic")
    parser.add_argument("--isotope-topic", default="/source_isotopes", help="Isotope ID topic")
    parser.add_argument("--radius", type=float, default=0.5, help="Sphere radius (m)")
    args = parser.parse_args()

    plotter = PerIsotopeHeatmapPlotter(args.host, args.port, args.cloud_topic,
                                        args.peak_topic, args.isotope_topic,
                                        args.radius)
    plotter.run()


if __name__ == "__main__":
    main()

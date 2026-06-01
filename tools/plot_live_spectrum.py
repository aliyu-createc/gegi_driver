#!/usr/bin/env python
"""
Live Spectrum Plotter - Runs natively on Windows via rosbridge websocket.

Connects to rosbridge_server in the Docker container and subscribes to
/spectrum topic. Displays a real-time matplotlib energy histogram.

Prerequisites:
  - rosbridge running in container (started automatically by full pipeline)
  - pip install roslibpy matplotlib numpy

Usage:
  python plot_live_spectrum.py [--host localhost] [--port 9090] [--cumulative]
"""
import argparse
import json
import os
import threading

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import roslibpy


class LiveSpectrumPlotter(object):
    def __init__(self, host, port, topic, cal_path, cumulative):
        self.cumulative = cumulative
        self.bin_edges = self._load_cal(cal_path)
        self.n_bins = len(self.bin_edges)
        self.counts_cumulative = np.zeros(self.n_bins, dtype=np.float64)
        self.counts_singles = np.zeros(self.n_bins, dtype=np.float64)
        self.lock = threading.Lock()
        self.singles_lock = threading.Lock()
        self.new_data = False
        self.new_singles_data = False

        # Activity results state
        self.activity_lock = threading.Lock()
        self.activity_results = None

        self.client = roslibpy.Ros(host=host, port=port)
        # All-events spectrum
        self.listener = roslibpy.Topic(self.client, topic,
                                       'radiation_detector_msgs/Spectrum')
        self.listener.subscribe(self._cb)

        # Singles-only spectrum
        self.singles_listener = roslibpy.Topic(
            self.client, '/spectrum_singles', 'radiation_detector_msgs/Spectrum')
        self.singles_listener.subscribe(self._singles_cb)

        # Subscribe to activity results
        self.activity_listener = roslibpy.Topic(
            self.client, '/activity/results', 'std_msgs/String')
        self.activity_listener.subscribe(self._activity_cb)

    def _load_cal(self, path):
        if path and os.path.exists(path):
            values = []
            with open(path, 'r') as f:
                for line in f:
                    t = line.strip()
                    if t:
                        values.append(float(t))
            return np.array(values)
        return np.linspace(0, 3000, 1024)

    def _cb(self, msg):
        arr = np.array(msg.get('spectrum', []), dtype=np.float64)
        with self.lock:
            n = min(len(arr), self.n_bins)
            self.counts_cumulative[:n] += arr[:n]
            self.new_data = True

    def _singles_cb(self, msg):
        arr = np.array(msg.get('spectrum', []), dtype=np.float64)
        with self.singles_lock:
            n = min(len(arr), self.n_bins)
            self.counts_singles[:n] += arr[:n]
            self.new_singles_data = True

    def _activity_cb(self, msg):
        try:
            data = json.loads(msg.get('data', '{}'))
            with self.activity_lock:
                self.activity_results = data
        except (ValueError, TypeError):
            pass

    def run(self):
        self.client.run()
        print("Connected to rosbridge. Waiting for spectrum data...")

        fig, (ax_cum, ax_snap) = plt.subplots(1, 2, figsize=(16, 5))
        xlim = self.bin_edges[-1] if len(self.bin_edges) > 0 else 3000

        # Left: cumulative
        ax_cum.set_xlabel("Energy (keV)")
        ax_cum.set_ylabel("Counts")
        ax_cum.set_title("Cumulative Spectrum")
        ax_cum.set_xlim(0, xlim)

        # Right: singles-only spectrum
        ax_snap.set_xlabel("Energy (keV)")
        ax_snap.set_ylabel("Counts")
        ax_snap.set_title("Singles-Only Spectrum (Photoelectric)")
        ax_snap.set_xlim(0, xlim)

        widths = np.diff(np.append(self.bin_edges, self.bin_edges[-1] + 2.0))
        bars_cum = ax_cum.bar(self.bin_edges[:self.n_bins], np.zeros(self.n_bins),
                              width=widths[:self.n_bins],
                              color='steelblue', edgecolor='none')
        bars_snap = ax_snap.bar(self.bin_edges[:self.n_bins], np.zeros(self.n_bins),
                                width=widths[:self.n_bins],
                                color='darkorange', edgecolor='none')

        # Isotope ROI colour shading
        isotope_roi_colors = {
            'Cs137': ('red', [655.0, 669.0]),
            'Co60_1173': ('green', [1165.0, 1181.0]),
            'Co60_1332': ('purple', [1325.0, 1340.0]),
        }
        for ax in (ax_cum, ax_snap):
            for iso_name, (color, roi) in isotope_roi_colors.items():
                ax.axvspan(roi[0], roi[1], alpha=0.15, color=color,
                           linewidth=2, edgecolor=color,
                           label=iso_name, zorder=0)
        # Add legend to right panel only (avoid clutter)
        ax_snap.legend(loc='upper right', fontsize=8)

        # Activity annotation text box (upper-right of cumulative plot)
        activity_text = ax_cum.text(
            0.98, 0.95, '', transform=ax_cum.transAxes,
            fontsize=9, verticalalignment='top', horizontalalignment='right',
            fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='lightyellow',
                      edgecolor='gray', alpha=0.9))

        def _format_activity(results):
            if not results:
                return "Activity: waiting..."
            lines = []
            for iso in results.get('isotopes', []):
                name = iso.get('isotope', '?')
                act = iso.get('activity_MBq', 0.0)
                sig = iso.get('sigma_activity_MBq', 0.0)
                valid = iso.get('valid', False)
                if valid:
                    lines.append("{}: {:.4f} \u00b1 {:.4f} MBq".format(
                        name, act, sig))
                else:
                    lines.append("{}: ---".format(name))
            total = results.get('total_activity_MBq', 0.0)
            lines.append("\u2500" * 24)
            lines.append("Total: {:.4f} MBq".format(total))
            window = results.get('live_time_s', 0.0)
            lines.append("Window: {:.1f}s".format(window))
            return "\n".join(lines)

        # Log scale toggle state
        self.log_scale = [False]

        def on_key(event):
            if event.key == 'l':
                self.log_scale[0] = not self.log_scale[0]
                scale = 'log' if self.log_scale[0] else 'linear'
                ax_cum.set_yscale(scale)
                ax_snap.set_yscale(scale)
                if self.log_scale[0]:
                    ax_cum.set_ylim(0.5, None)
                    ax_snap.set_ylim(0.5, None)
                fig.canvas.draw_idle()

        fig.canvas.mpl_connect('key_press_event', on_key)

        def update(frame):
            needs_redraw = False

            with self.lock:
                if self.new_data:
                    data_cum = self.counts_cumulative.copy()
                    self.new_data = False
                    needs_redraw = True
                else:
                    data_cum = None

            with self.singles_lock:
                if self.new_singles_data:
                    data_singles = self.counts_singles.copy()
                    self.new_singles_data = False
                    needs_redraw = True
                else:
                    data_singles = None

            if not needs_redraw:
                with self.activity_lock:
                    activity_text.set_text(_format_activity(self.activity_results))
                return list(bars_cum) + list(bars_snap)

            # Update cumulative bars
            if data_cum is not None:
                for bar, h in zip(bars_cum, data_cum):
                    bar.set_height(h)
                ymax_cum = data_cum.max() * 1.1 if data_cum.max() > 0 else 10
                if self.log_scale[0]:
                    ax_cum.set_ylim(0.5, ymax_cum * 2)
                else:
                    ax_cum.set_ylim(0, ymax_cum)
                ax_cum.set_title("All Events Spectrum | Total: {} counts".format(
                    int(data_cum.sum())))

            # Update singles bars
            if data_singles is not None:
                for bar, h in zip(bars_snap, data_singles):
                    bar.set_height(h)
                ymax_snap = data_singles.max() * 1.1 if data_singles.max() > 0 else 10
                if self.log_scale[0]:
                    ax_snap.set_ylim(0.5, ymax_snap * 2)
                else:
                    ax_snap.set_ylim(0, ymax_snap)
                ax_snap.set_title("Singles-Only Spectrum | Total: {} counts".format(
                    int(data_singles.sum())))

            with self.activity_lock:
                activity_text.set_text(_format_activity(self.activity_results))

            return list(bars_cum) + list(bars_snap)

        ani = animation.FuncAnimation(fig, update, interval=250, blit=False)
        plt.tight_layout()
        plt.show()
        self.client.terminate()


def main():
    parser = argparse.ArgumentParser(description="Live GeGI spectrum plotter (via rosbridge)")
    parser.add_argument("--host", default="localhost", help="rosbridge host (default: localhost)")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge port (default: 9090)")
    parser.add_argument("--topic", default="/spectrum", help="Spectrum topic")
    parser.add_argument("--cal", default="", help="Path to EnergyCal.csv")
    parser.add_argument("--cumulative", action="store_true",
                        help="Accumulate counts over time (default: show latest snapshot)")
    args = parser.parse_args()

    cal = args.cal
    if not cal:
        default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "config", "EnergyCal.csv")
        if os.path.exists(default):
            cal = default

    plotter = LiveSpectrumPlotter(args.host, args.port, args.topic, cal, args.cumulative)
    plotter.run()


if __name__ == "__main__":
    main()

    plotter = LiveSpectrumPlotter(args.topic, cal, args.cumulative)
    plotter.run()


if __name__ == "__main__":
    main()

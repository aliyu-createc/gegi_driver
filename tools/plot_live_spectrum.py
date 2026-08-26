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
import sys
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import roslibpy

# Isotope-ID screening engine (matched-filter peak search + library match):
# used to LABEL identified peaks on the live spectrum. Optional - the plotter
# runs without labels if the module/library is unavailable.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'phds_gegi_driver'))
try:
    import isotope_id
except ImportError:
    isotope_id = None


class LiveSpectrumPlotter(object):
    def __init__(self, host, port, topic, cal_path, cumulative,
                 library_path=""):
        self.cumulative = cumulative
        self.bin_edges = self._load_cal(cal_path)
        self.n_bins = len(self.bin_edges)

        # Nuclide library for live peak labels (optional).
        self.library = None
        if isotope_id is not None and library_path and \
                os.path.exists(library_path):
            try:
                self.library = isotope_id.load_nuclide_library(library_path)
                print("Peak labels: %d-nuclide ID library loaded"
                      % len(self.library['nuclides']))
            except Exception as e:
                print("Peak labels disabled (library load failed: %s)" % e)
        self.peak_artists = []
        self._last_id_time = 0.0
        self.counts_cumulative = np.zeros(self.n_bins, dtype=np.float64)
        self.lock = threading.Lock()
        self.new_data = False

        # Activity results state
        self.activity_lock = threading.Lock()
        self.activity_results = None

        self.client = roslibpy.Ros(host=host, port=port)
        # All-events spectrum
        self.listener = roslibpy.Topic(self.client, topic,
                                       'radiation_detector_msgs/Spectrum')
        self.listener.subscribe(self._cb)

        # Subscribe to activity results
        self.activity_listener = roslibpy.Topic(
            self.client, '/activity/results', 'std_msgs/String')
        self.activity_listener.subscribe(self._activity_cb)

        # Per-line labels from the PIPELINE's screening pass (single source of
        # truth - preferred over recomputing on the display buffer, which can
        # disagree). Falls back to local identification if the topic is quiet.
        self.lines_lock = threading.Lock()
        self.pipeline_lines = None       # (recv_time, payload dict)
        self.lines_listener = roslibpy.Topic(
            self.client, '/identified_lines', 'std_msgs/String')
        self.lines_listener.subscribe(self._lines_cb)
        self._drift_warned = 0.0

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

    def _draw_labels(self, ax, data, entries):
        """(Re)draw peak labels: entries = [(energy_keV, text)]."""
        for art in self.peak_artists:
            try:
                art.remove()
            except ValueError:
                pass
        self.peak_artists = []
        ymax = max(float(data.max()), 1.0)
        for energy, name in sorted(entries):
            idx = int(np.searchsorted(self.bin_edges, energy)) - 1
            lo, hi = max(0, idx - 5), min(len(data), idx + 6)
            height = float(data[lo:hi].max()) if hi > lo else 0.0
            art = ax.annotate(
                "{} ({:.0f})".format(name, energy),
                xy=(energy, height), xytext=(energy, height + 0.03 * ymax),
                rotation=90, fontsize=8, ha='center', va='bottom',
                color='crimson',
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                          edgecolor='crimson', alpha=0.75))
            self.peak_artists.append(art)

    @staticmethod
    def _format_label(line):
        """Display text for one labelled line. Decorations:
          'Co-57?'   contested - an unidentified nuclide also fits the peak
          '~Eu-152'  unknown peak, but this nuclide WOULD fit (candidate hint)
          '*'        backscatter-suspect region
          'annih.'   unmatched 511 keV
        """
        text = line.get('label', '?')
        tags = line.get('tags', [])
        if text == '?':
            cand = next((t.split(':', 1)[1] for t in tags
                         if t.startswith('candidates:')), None)
            if cand:
                text = '~' + cand
            elif 'annihilation' in tags:
                text = 'annih.'
        if 'backscatter-suspect' in tags:
            text += '*'
        return text

    def _entries_from_pipeline(self, payload):
        """Label entries from the recorder's /identified_lines payload.
        Only PERSISTENT lines are drawn (flicker suppression)."""
        return [(float(l['energy_keV']), self._format_label(l))
                for l in payload.get('lines', []) if l.get('persistent')]

    def _relabel_peaks(self, ax, data):
        """FALLBACK: identify the display buffer locally (used only when the
        pipeline's /identified_lines topic is quiet - e.g. bag replay)."""
        # Rebin toward ~0.8 keV so the matched filter is fast in the GUI loop.
        bin_w = float(np.median(np.diff(self.bin_edges))) or 1.0
        factor = max(1, int(round(0.8 / bin_w)))
        centers = self.bin_edges + bin_w / 2.0
        m = (len(data) // factor) * factor
        counts = data[:m].reshape(-1, factor).sum(axis=1).copy()
        cents = centers[:m].reshape(-1, factor).mean(axis=1)
        counts[-1] = 0.0     # overflow bin guard

        peaks, results, _ = isotope_id.identify(counts, cents, self.library)
        labeled = isotope_id.label_peaks(peaks, results)
        entries = [(l['energy_keV'], self._format_label(l)) for l in labeled]
        self._draw_labels(ax, data, entries)

    def _lines_cb(self, msg):
        try:
            payload = json.loads(msg.get('data', '{}'))
        except (ValueError, TypeError):
            return
        with self.lines_lock:
            self.pipeline_lines = (time.time(), payload)

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

        fig, ax_cum = plt.subplots(1, 1, figsize=(12, 6))
        xlim = self.bin_edges[-1] if len(self.bin_edges) > 0 else 3000

        # Left: cumulative
        ax_cum.set_xlabel("Energy (keV)")
        ax_cum.set_ylabel("Counts")
        ax_cum.set_title("Cumulative Spectrum")
        ax_cum.set_xlim(0, xlim)

        # STEP LINES, not bar charts: a bar chart makes one Rectangle patch per
        # channel - 15,000 bins x 2 panels = 30,000 patches updated in a Python
        # loop every frame, which crawls. A step line is ONE artist per panel,
        # updated with a single set_ydata call - the standard way to render a
        # finely binned spectrum.
        (line_cum,) = ax_cum.plot(self.bin_edges, np.zeros(self.n_bins),
                                  color='steelblue', linewidth=0.8,
                                  drawstyle='steps-mid')

        # Assay-ROI colour shading (no legend - the per-peak ID labels name
        # the isotopes; the shading just marks the assayed regions).
        isotope_roi_colors = {
            'Cs137': ('red', [655.0, 669.0]),
            'Co60_1173': ('green', [1165.0, 1181.0]),
            'Co60_1332': ('purple', [1325.0, 1340.0]),
        }
        for _, (color, roi) in isotope_roi_colors.items():
            ax_cum.axvspan(roi[0], roi[1], alpha=0.15, color=color,
                           linewidth=2, edgecolor=color, zorder=0)

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
                if self.log_scale[0]:
                    ax_cum.set_ylim(0.5, None)
                fig.canvas.draw_idle()
            elif event.key == 'c':
                # Clear the DISPLAY accumulation only (client-side). Does not touch
                # the detector or any recording - /spectrum is per-interval deltas.
                with self.lock:
                    self.counts_cumulative[:] = 0
                    self.new_data = True
                print("Display spectrum cleared (c).")

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

            if not needs_redraw:
                with self.activity_lock:
                    activity_text.set_text(_format_activity(self.activity_results))
                return [line_cum]

            # Update cumulative trace (single artist - fast at any bin count)
            if data_cum is not None:
                line_cum.set_ydata(data_cum[:self.n_bins])
                ymax_cum = data_cum.max() * 1.1 if data_cum.max() > 0 else 10
                if self.log_scale[0]:
                    ax_cum.set_ylim(0.5, ymax_cum * 2)
                else:
                    ax_cum.set_ylim(0, ymax_cum)
                ax_cum.set_title("All Events Spectrum | Total: {} counts".format(
                    int(data_cum.sum())))

            with self.activity_lock:
                activity_text.set_text(_format_activity(self.activity_results))

            # Live peak labels. Preferred source: the PIPELINE's own screening
            # result (/identified_lines) - identical labels to the heatmap and
            # the N42, with flicker suppression. Fallback: local identification
            # of the display buffer (e.g. bag replay with no recorder running).
            if data_cum is not None and time.time() - self._last_id_time > 10.0:
                self._last_id_time = time.time()
                with self.lines_lock:
                    fresh = (self.pipeline_lines
                             if self.pipeline_lines is not None and
                             time.time() - self.pipeline_lines[0] < 180.0
                             else None)
                try:
                    if fresh is not None:
                        _, payload = fresh
                        self._draw_labels(
                            ax_cum, data_cum,
                            self._entries_from_pipeline(payload))
                        drift = float(payload.get('drift_keV', 0.0) or 0.0)
                        if (abs(drift) > 1.0
                                and payload.get('n_drift_lines', 0) >= 2
                                and time.time() - self._drift_warned > 300.0):
                            self._drift_warned = time.time()
                            print("WARNING: energy-calibration drift %+.2f keV "
                                  "reported by the pipeline - check calibration."
                                  % drift)
                    elif self.library is not None and data_cum.sum() > 500:
                        self._relabel_peaks(ax_cum, data_cum)
                except Exception as e:
                    print("peak labelling failed: %s" % e)

            return [line_cum]

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
    parser.add_argument("--library", default="",
                        help="nuclide_library.yaml for live peak labels "
                             "(default: config/nuclide_library.yaml)")
    parser.add_argument("--cumulative", action="store_true",
                        help="Accumulate counts over time (default: show latest snapshot)")
    args = parser.parse_args()

    cal = args.cal
    if not cal:
        # Default to the SAME calibration the launch files give the spectrum
        # node (15,000 ch x 0.2 keV, matching the GeGI onboard N42 binning) -
        # a different edge set here would silently mis-scale the energy axis.
        cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "config")
        for name in ("EnergyCal_15000_0p2keV.csv", "EnergyCal.csv"):
            default = os.path.join(cfg, name)
            if os.path.exists(default):
                cal = default
                break

    library = args.library
    if not library:
        default = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "config", "nuclide_library.yaml")
        if os.path.exists(default):
            library = default

    plotter = LiveSpectrumPlotter(args.host, args.port, args.topic, cal,
                                  args.cumulative, library_path=library)
    plotter.run()


if __name__ == "__main__":
    main()

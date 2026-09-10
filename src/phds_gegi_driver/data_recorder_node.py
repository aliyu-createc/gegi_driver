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
import json
import math
import os
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import rospy
import rosbag
import yaml
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from radiation_detector_msgs.msg import ComptonEvent, Spectrum
from phds_gegi_driver.srv import StartTimedAcquisition, StartTimedAcquisitionRequest, GetRunInfo

# Isotope-ID screening layer (matched-filter peak search + nuclide library
# match). Optional: the recorder runs without it if the module or library is
# missing. NOTE: sync.sh/build.sh must copy isotope_id.py into devel/lib
# alongside the node files.
try:
    import isotope_id
except ImportError:
    isotope_id = None


# Specific gamma-ray dose-rate constants (uSv*m^2 / MBq*h). The authoritative
# values live in config/isotopes.yaml under `gamma_constants`; this dict is only
# the fallback used when that file is missing or lacks the section.
DEFAULT_GAMMA_CONSTANTS = {'Cs-137': 0.0771, 'Co-60': 0.3059}


def _norm_iso(name):
    """Normalise an isotope name for lookup: uppercase, drop punctuation.
    So 'Cs137', 'Cs-137', 'cs_137' and 'Co60_1173' all collapse sensibly."""
    return ''.join(ch for ch in str(name).upper() if ch.isalnum())


def load_gamma_constants(config_path):
    """Load specific gamma-ray dose-rate constants from isotopes.yaml.

    Returns a dict keyed by the normalised isotope name so lookups work whether
    the caller passes the yaml form ('Cs137') or the display form ('Cs-137').
    Falls back to DEFAULT_GAMMA_CONSTANTS if the file is missing/unreadable so
    dose weighting never hard-fails on a config problem.
    """
    table = {_norm_iso(k): float(v) for k, v in DEFAULT_GAMMA_CONSTANTS.items()}
    if config_path and os.path.isfile(config_path):
        try:
            with open(config_path) as f:
                cfg = yaml.safe_load(f) or {}
            for k, v in (cfg.get('gamma_constants', {}) or {}).items():
                table[_norm_iso(k)] = float(v)
        except Exception as exc:  # noqa: broad - never let dose weighting crash
            rospy.logwarn("Could not load gamma_constants from %s: %s",
                          config_path, exc)
    return table


def gamma_constant_for(table, iso_name, default=0.077):
    """Look up a gamma constant tolerant of naming ('Co-60' vs 'Co60_1173')."""
    key = _norm_iso(iso_name)
    if key in table:
        return table[key]
    # Fall back to a prefix match (e.g. 'CO601173' -> 'CO60').
    for k, v in table.items():
        if key.startswith(k) or k.startswith(key):
            return v
    return default


def _median(values):
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return 0.0
    mid = n // 2
    return ordered[mid] if n % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])


def dead_time_correction_factor(real_time_s, live_time_s, dead_time_percent=None):
    """Multiplier that converts a RAW net area/activity to a DEAD-TIME-CORRECTED one.

    The detector's live-time clock already excludes dead time, so real/live is
    the exact correction and is preferred. If the real/live counters are missing
    or nonsensical we fall back to the reported cumulative dead-time percentage,
    and finally to 1.0 (no correction) so a bad run-info read can never fabricate
    or destroy counts.

    IMPORTANT (calibration coupling): the calibration_factor values in
    isotopes.yaml were fitted against certificated sources using UNCORRECTED
    activities, so they already absorb the ~2% dead time of the commissioning
    runs. Turning this correction on therefore REQUIRES re-deriving the
    calibration factors from dead-time-corrected commissioning data, otherwise
    the ~2% is applied twice. See docs/PORTING.md and the recalibration step.
    """
    try:
        rt = float(real_time_s)
        lt = float(live_time_s)
        if rt > 0.0 and lt > 0.0 and lt <= rt:
            return rt / lt
    except (TypeError, ValueError):
        pass
    try:
        dt = float(dead_time_percent)
        if 0.0 <= dt < 100.0:
            return 1.0 / (1.0 - dt / 100.0)
    except (TypeError, ValueError):
        pass
    return 1.0


def currie_critical_level(background_counts):
    """Currie critical level L_C = 2.33*sqrt(B): the net-count DECISION threshold
    (~95% confidence) above which a line is DETECTED. Background-aware, unlike a
    fixed count gate."""
    return 2.33 * math.sqrt(max(0.0, float(background_counts or 0.0)))


def aggregate_run_activity(activity_reports, drop_threshold=0.75, dt_factor=1.0,
                           background_rates=None):
    """Collapse the per-window activity reports into ONE result per gamma line,
    deciding DETECTION on the RUN-TOTAL net.

    Detection is a run-level Currie decision: a line is detected when its total
    net over the whole run exceeds L_C = 2.33*sqrt(B_total), NOT when individual
    counting windows each clear a fixed count threshold. A weak line (e.g. Co-60's
    1332 keV) that never crosses the per-window bar but accumulates ample counts
    over the run is therefore still quantified. The activity node's per-window
    ``valid`` flag is deliberately IGNORED here - it remains only a live-display
    hint.

    Activity is TOTAL net / TOTAL live time (correctly window-weighted), scaled by
    the per-line efficiency product K reported by the activity node
    (A_Bq = total_net/(K*total_live)); older/synthetic reports without K fall back
    to the window activity/net scale.

    Partial start-up windows are still EXCLUDED, on COUNT RATE: a ramp window has
    partial counts against a full logged live time, so its rate is anomalously low
    (including it dragged a real run -6.8%). A genuinely short window has a NORMAL
    rate and is kept.

    background_rates (optional) = {label: {net_cps, net_cps_sigma}} from a no-source
    run: its net counts (rate x live) are subtracted per line, and its variance is
    folded into the detection threshold AND the counting sigma. This removes both
    the environmental peaks (e.g. ambient Cs-137) and the zero-background false
    positives (an empty ROI no longer has L_C = 0).

    Returns {line_label: {...}} for DETECTED lines only; undetected configured
    lines are reported as MDAs by non_detected_nuclide_mdas().
    """
    detection_floor_counts = 5.0
    per_line = {}
    for report in activity_reports:
        live = float(report.get('live_time_s', 0.0) or 0.0)
        for iso in report.get('isotopes', []):
            name = iso.get('isotope', '')
            net = float(iso.get('net_corrected', 0.0) or 0.0)
            if not name or net <= 0 or live <= 0:
                continue
            per_line.setdefault(name, []).append({
                'net': net, 'live': live,
                'gross': float(iso.get('gross_counts', 0.0) or 0.0),
                'bg': float(iso.get('background_counts', 0.0) or 0.0),
                'K': float(iso.get('efficiency_product', 0.0) or 0.0),
                'act': float(iso.get('activity_MBq', 0.0) or 0.0),
                'energy_keV': float(iso.get('energy_keV', 0.0) or 0.0),
                # Position-aware correction provenance (activity node folds the
                # factor into K, so the activity is already corrected; these
                # only document it in the N42).
                'pos_f': float(iso.get('position_factor', 1.0) or 1.0),
                'slant': float(iso.get('slant_distance_m', 0.0) or 0.0),
            })

    results = {}
    for name, windows in per_line.items():
        # Reference rate from SUBSTANTIVE windows only: a seconds-long flush
        # fragment carries huge rate variance (14 counts in 1.25 s reads ~2x
        # the true rate) and, with only two windows, can drag the median above
        # the genuine full window's rate - which then gets dropped as
        # "partial" while the fragment is kept (run 20260904_172346: activity
        # reported from 14 counts). Fragments are still rate-filtered and
        # summed normally; they just cannot steer the reference.
        max_live = max(w['live'] for w in windows)
        substantive = [w for w in windows if w['live'] >= 0.1 * max_live]
        median_rate = _median([w['net'] / w['live']
                               for w in (substantive or windows)])
        kept = [w for w in windows
                if median_rate <= 0
                or (w['net'] / w['live']) >= drop_threshold * median_rate]
        dropped = len(windows) - len(kept)
        if not kept:
            continue
        total_net = sum(w['net'] for w in kept)
        total_live = sum(w['live'] for w in kept)
        total_bg = sum(w['bg'] for w in kept)
        total_gross = sum(w['gross'] for w in kept)
        if total_net <= 0 or total_live <= 0:
            continue

        # Optional background subtraction (measured no-source rate).
        bg_counts = 0.0     # expected background counts in this run's ROI
        bg_var = 0.0        # variance of that from the background measurement
        if background_rates and name in background_rates:
            br = background_rates[name]
            bg_counts = float(br.get('net_cps', 0.0) or 0.0) * total_live
            bg_var = (float(br.get('net_cps_sigma', 0.0) or 0.0) * total_live) ** 2
        net_source = total_net - bg_counts

        # Run-level detection decision (Currie critical level). The subtracted
        # background and its measurement variance widen L_C, so a clean ROI (which
        # used to give L_C = 0) and ambient peaks no longer false-positive.
        # detection_floor_counts guards the residual B~0 pathology: with a truly
        # empty ROI L_C -> 0 and a couple of stray counts in one short partial
        # window "detect" (seen 2026-09-03: Co60_1173 0.097+-0.103 MBq from a
        # 1.5 s window in a Cs-only run). A handful-of-counts floor is orders
        # below any genuine assay signal.
        if net_source <= max(currie_critical_level(total_bg + bg_counts + bg_var),
                             detection_floor_counts):
            continue
        # Activity scale: prefer the reported efficiency product K (activity =
        # net_source/(K*total_live)); fall back to the window activity/net ratio for
        # reports predating it. dt_factor (real/live) scales the activity only -
        # the Poisson statistics below use RAW counts.
        ks = [w['K'] for w in kept if w['K'] > 0]
        if ks:
            activity = (net_source / (_median(ks) * total_live)) * dt_factor / 1.0e6
        else:
            ratios = [w['act'] / (w['net'] / w['live'])
                      for w in kept if w['act'] > 0 and w['net'] > 0]
            bq_per_cps = _median(ratios) if ratios else 0.0
            activity = bq_per_cps * (net_source / total_live) * dt_factor
        # Net-area sigma: Poisson on the raw counts (gross + sideband bg =
        # net + 2*bg) plus the background-subtraction variance.
        sigma_net = math.sqrt(total_net + 2.0 * total_bg + bg_var)
        # Position-corrected windows (factor meaningfully < 1) -> document the
        # median factor/slant distance in the N42. Uncorrected runs report 1.0.
        pos_windows = [w for w in kept if w.get('pos_f', 1.0) < 0.999]
        results[name] = {
            'isotope': name,
            'energy_keV': kept[0]['energy_keV'],
            'activity_MBq': activity,
            # Counting statistics ONLY - not the assay uncertainty.
            'counting_sigma_MBq': (activity * sigma_net / net_source
                                   if net_source > 0 else 0.0),
            'net_counts': net_source,
            'gross_counts': total_gross,
            'background_counts': total_bg + bg_counts,
            'live_time_s': total_live,
            'windows_used': len(kept),
            'windows_dropped': dropped,
            'position_corrected': bool(pos_windows),
            'position_factor': (_median([w['pos_f'] for w in pos_windows])
                                if pos_windows else 1.0),
            'slant_distance_m': (_median([w['slant'] for w in pos_windows])
                                 if pos_windows else 0.0),
        }
    return results


def group_by_radionuclide(line_results, radionuclide_of):
    """Combine per-line results into one result per radionuclide.

    Co-60 is measured on two photopeaks (1173 and 1332 keV) that quantify the
    SAME nuclide, so they are averaged into a single Co-60 activity. Their
    spread is reported: it is a direct internal-consistency check.
    """
    groups = {}
    for name, res in line_results.items():
        groups.setdefault(radionuclide_of(name), []).append(res)

    out = {}
    for nuclide, results in groups.items():
        activities = [r['activity_MBq'] for r in results]
        activity = sum(activities) / len(activities)
        sigma = math.sqrt(sum(r['counting_sigma_MBq'] ** 2 for r in results)) / len(results)
        spread = 0.0
        if len(activities) > 1 and activity > 0:
            spread = 100.0 * (max(activities) - min(activities)) / activity
        out[nuclide] = {
            'radionuclide': nuclide,
            'activity_MBq': activity,
            'counting_sigma_MBq': sigma,
            'line_spread_percent': spread,
            'lines': sorted(r['isotope'] for r in results),
            'net_counts': sum(r['net_counts'] for r in results),
        }
    return out


def cs137_co60_ratio(nuclides):
    """Cs-137 : Co-60 ratio for a mixed field, or None unless BOTH are detected.

    Reports two ratios (both as Cs-137 / Co-60):
      - activity_ratio: the physical isotopic ratio (calibration-corrected).
      - count_ratio: raw net counts, a calibration-INDEPENDENT fingerprint -
        useful while the calibration factors are provisional (pre-recalibration).

    Co-60 is the intended PRIMARY reference and Cs-137 the SECONDARY: Co-60's
    Compton continuum sits under the Cs-137 662 keV peak (raising its background
    and MDA) but Cs-137 does not reciprocally interfere with the Co-60 peaks.
    """
    cs = nuclides.get('Cs-137')
    co = nuclides.get('Co-60')
    if not cs or not co:
        return None
    a_cs = float(cs.get('activity_MBq', 0.0) or 0.0)
    a_co = float(co.get('activity_MBq', 0.0) or 0.0)
    net_cs = float(cs.get('net_counts', 0.0) or 0.0)
    net_co = float(co.get('net_counts', 0.0) or 0.0)
    return {
        'activity_ratio': (a_cs / a_co) if a_co > 0 else None,
        'count_ratio': (net_cs / net_co) if net_co > 0 else None,
        'cs137_MBq': a_cs,
        'co60_MBq': a_co,
    }


def run_line_totals(activity_reports):
    """Per-line RUN totals (all windows) for EVERY configured line, detected or
    not: {label: {net, gross, bg, live, energy}}. Used to build a background
    reference (which needs the undetected lines too)."""
    out = {}
    for report in activity_reports:
        live = float(report.get('live_time_s', 0.0) or 0.0)
        if live <= 0:
            continue
        for iso in report.get('isotopes', []):
            name = iso.get('isotope', '')
            if not name:
                continue
            t = out.setdefault(name, {'net': 0.0, 'gross': 0.0, 'bg': 0.0,
                                      'live': 0.0, 'energy': 0.0})
            t['net'] += float(iso.get('net_corrected', 0.0) or 0.0)
            t['gross'] += float(iso.get('gross_counts', 0.0) or 0.0)
            t['bg'] += float(iso.get('background_counts', 0.0) or 0.0)
            t['live'] += live
            t['energy'] = float(iso.get('energy_keV', 0.0) or 0.0)
    return out


def build_background_rates(activity_reports):
    """Per-line background NET RATE (cps) + 1-sigma, from a no-source run, for
    later subtraction. sigma is the Poisson net-area uncertainty per second:
    sqrt(gross + sideband_bg) / live. Returns {label: {net_cps, net_cps_sigma}}."""
    rates = {}
    for name, t in run_line_totals(activity_reports).items():
        live = t['live']
        if live <= 0:
            continue
        sigma_counts = math.sqrt(max(0.0, t['gross'] + t['bg']))
        rates[name] = {
            'net_cps': t['net'] / live,
            'net_cps_sigma': sigma_counts / live,
            'live_time_s': live,
        }
    return rates


def screening_xml_block(id_results, unknown_peaks, exclude_names=()):
    """N42 <Nuclide> entries for SCREENING identifications + an unknown-peaks
    remark. Pure function (unit-tested).

    Screening = the isotope-ID layer's spectral library match: it says a
    nuclide IS PRESENT but carries no activity (no calibration for it). Any
    nuclide already reported by the assay layer (quantified or MDA) is
    excluded so it is not listed twice. Returns (list_of_nuclide_xml, remark).
    """
    blocks = []
    for r in id_results or []:
        if not r.get('identified') or r.get('nuclide') in exclude_names:
            continue
        lines_txt = ", ".join(
            "{:.0f} keV (SNR {:.0f})".format(m['energy_keV'], m['snr'])
            for m in r.get('matched_lines', []))
        shared = "".join(
            " AMBIGUITY: {:.1f} keV peak also matches {}.".format(
                s['peak_keV'], "/".join(s['also']))
            for s in r.get('shared_peaks', []))
        blocks.append(
            '      <Nuclide>\n'
            '        <NuclideIdentifiedIndicator>true</NuclideIdentifiedIndicator>\n'
            '        <NuclideName>{name}</NuclideName>\n'
            '        <Remark>SCREENING identification (spectral library match; '
            'no calibration, so no activity is quoted): category {cat}, '
            'score {score:.2f}, lines {lines}.{shared}</Remark>\n'
            '      </Nuclide>'.format(
                name=r['nuclide'], cat=r.get('category', ''),
                score=r.get('score', 0.0), lines=lines_txt, shared=shared))
    remark = ''
    if unknown_peaks:
        remark = ('    <Remark>Screening: unidentified peaks at {} - matching '
                  'no library nuclide.</Remark>\n'.format(
                      ", ".join("{:.1f} keV (SNR {:.0f})".format(
                          p['energy_keV'], p['snr']) for p in unknown_peaks)))
    return blocks, remark


def load_background(path):
    """Load a background reference written by build_background_rates/save.
    Returns {label: {net_cps, net_cps_sigma}} or None if absent/unreadable."""
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except Exception:
        return None
    lines = data.get('lines') if isinstance(data, dict) else None
    return lines or None


def detected_line_labels(activity_reports):
    """Line labels DETECTED over the run (run-total net exceeds the Currie
    critical level). Delegates to aggregate_run_activity so the CSV filter, the
    N42 activities and the MDA decision all share ONE run-level detection rule
    rather than the activity node's per-window ``valid`` flag.
    """
    return set(aggregate_run_activity(activity_reports).keys())


def detected_nuclides(activity_reports, radionuclide_of):
    """Radionuclides detected in the run (>=1 valid window on ANY of their lines).

    Used to filter the CSV at the NUCLIDE level: if Co-60 is seen on 1173 keV,
    its 1332 keV line is kept too even when that line stayed below the per-window
    validity threshold - the two Co-60 lines are a consistency pair and dropping
    one hides that the weaker line is under threshold.
    """
    return set(radionuclide_of(l) for l in detected_line_labels(activity_reports))


def currie_mda_bq(total_background_counts, efficiency_product, total_live_time_s):
    """Currie detection limit L_D converted to activity (Bq).

    L_D = 2.71 + 4.65*sqrt(B) NET counts (95% confidence, Currie 1968), then
    A = L_D / (K * t_live) with K the efficiency product reported per line by the
    activity node (A_Bq = net / (K * t_live)). Returns None when it cannot be
    computed (no efficiency/geometry, or no live time).
    """
    try:
        K = float(efficiency_product)
        t = float(total_live_time_s)
    except (TypeError, ValueError):
        return None
    if K <= 0.0 or t <= 0.0:
        return None
    B = max(0.0, float(total_background_counts or 0.0))
    L_D = 2.71 + 4.65 * math.sqrt(B)
    return L_D / (K * t)


def non_detected_nuclide_mdas(activity_reports, detected, radionuclide_of):
    """MDA (Bq) for each configured nuclide that was NOT detected in the run.

    A nuclide is a non-detection only if NONE of its lines were detected. Its MDA
    is bounded by its MOST SENSITIVE line (lowest MDA). Background and live time
    are summed over the whole run: more counting time -> lower (better) MDA.
    """
    total_live = 0.0
    bg = {}       # label -> summed background counts
    K = {}        # label -> efficiency product (constant per run)
    nuclide_of = {}
    for report in activity_reports:
        total_live += float(report.get('live_time_s', 0.0) or 0.0)
        for iso in report.get('isotopes', []):
            label = iso.get('isotope') or ''
            if not label:
                continue
            bg[label] = bg.get(label, 0.0) + float(iso.get('background_counts', 0.0) or 0.0)
            kp = float(iso.get('efficiency_product', 0.0) or 0.0)
            if kp > 0.0:
                K[label] = kp
            nuclide_of[label] = radionuclide_of(label)

    lines_by_nuclide = {}
    for label, nuc in nuclide_of.items():
        lines_by_nuclide.setdefault(nuc, []).append(label)

    out = {}
    for nuc, labels in lines_by_nuclide.items():
        if any(l in detected for l in labels):
            continue  # nuclide was detected on at least one line
        candidates = []
        for l in labels:
            mda = currie_mda_bq(bg.get(l, 0.0), K.get(l, 0.0), total_live)
            if mda is not None:
                candidates.append((mda, l))
        if not candidates:
            continue
        best_mda, best_line = min(candidates)
        out[nuc] = {'mda_Bq': best_mda, 'line': best_line,
                    'lines': sorted(labels), 'live_time_s': total_live}
    return out


class DataRecorderNode(object):
    def __init__(self):
        self.output_dir = rospy.get_param("~output_dir", "/opt/phds_gegi_driver/data")
        cal_path = rospy.get_param("~calibration_file", "")
        self.calibration_file = cal_path
        self.isotopes_config = rospy.get_param("~isotopes_config", "")
        self.source_distance_m = rospy.get_param("~source_distance_m", 0.5)
        self.heatmap_grid_res = rospy.get_param("~heatmap_grid_res", 200)
        self.run_info_settle_s = rospy.get_param("~run_info_settle_s", 15.0)
        # How often to poll detector run-info WHILE recording, so the true
        # dead-time is captured during acquisition (a post-stop query fails once
        # the detector has stopped/reset). Polls are LOSSLESS since the inline
        # run-info fix (the 'i' prompt is one byte and the reply is decoded by
        # the driver's monitor thread), so poll densely: some runs lose every
        # reply to event-stream interleaving (dt saved as 0, ~1 run in 6
        # observed 2026-09-03/04) and more attempts are the only mitigation
        # short of a separate command socket in the C++ driver.
        self.run_info_poll_s = rospy.get_param("~run_info_poll_s", 15.0)
        # Prefix for the durable measurement identifier written into every asset
        # so the database can link the N42, activity peak rows and heatmaps of a
        # run back to one parent Measurement (spec DB-GEGI-002 / DB-GEGI-004).
        self.measurement_id_prefix = rospy.get_param("~measurement_id_prefix", "GEGI")

        # Systematic (non-counting) assay uncertainty, 1 sigma, in percent. This is
        # the position-dominated part of the uncertainty budget and is a property of
        # the METHOD, not of an individual run (see docs/DJR_assay_results_wording.md).
        #
        # The N42 activity uncertainty is built per run as
        #     u_combined = sqrt(u_counting^2 + u_systematic^2),  U(k=2) = 2*u_combined
        # so a well-counted run reports ~the budget value, while a marginal run near
        # the MDA correctly reports a much wider uncertainty. Writing the counting
        # sigma alone (~1-3%) into a durable record would badly understate the real
        # measurement uncertainty; a reader reasonably assumes the quoted figure IS
        # the measurement uncertainty.
        #
        # It is a parameter, not a constant, because the budget is rig-specific.
        #
        # Default 3.2% = RSS of: position 1.0 (position-aware correction ON -
        # validated 2026-09-03, 5-position Exp E repeat: centre + 4 corners all
        # within +-1% of cert, RSD 0.65%; 1.0 is the conservative allowance),
        # repeatability 2.1 (Exp D), certificate 1.5 (Co-60 BH-4103: 3%
        # expanded at k=2 -> 1.5% standard), efficiency transfer 1.1 (Exp B),
        # shielding 1.0 (Exp F), dead-time 0.6 (Exp C), Co-60 coincidence
        # summing 0.05. -> expanded U(k=2) ~= 6.4% (7.1% with 1.5% counting).
        # If the pipeline runs with position_correction DISABLED, set this back
        # to ~10.6 (position term 10.2, uncorrected Exp E) -> U(k=2) ~= 21%.
        self.assay_systematic_uncertainty_pct = rospy.get_param(
            "~assay_systematic_uncertainty_percent", 3.2)

        # Apply detector dead-time correction (real/live) to the saved net areas
        # and activities. Default ON: this is the physically correct behaviour.
        # WARNING: the calibration_factor values in isotopes.yaml MUST be derived
        # from dead-time-corrected data when this is on, or the correction is
        # double-counted (see dead_time_correction_factor()). Set false only to
        # reproduce legacy uncorrected numbers.
        self.apply_dead_time_correction = rospy.get_param(
            "~apply_dead_time_correction", True)

        # Report configured-but-not-detected nuclides in the N42 as a
        # non-detection with a Currie MDA ("Cs-137 not detected, < X MBq").
        # A non-detection is evidence only if it carries a limit; without this the
        # N42 simply omits absent nuclides. The CSV always omits absent lines
        # regardless (a per-window zero-activity row is pure noise).
        self.report_non_detected_mda = rospy.get_param(
            "~report_non_detected_mda", True)

        # Background subtraction. A no-source run is recorded once (set
        # ~record_background true, then take a run) into background_file; every
        # later run then subtracts those per-line net rates, removing ambient
        # peaks (environmental Cs-137) and zero-background false positives.
        self.background_file = rospy.get_param(
            "~background_file", os.path.join(self.output_dir, "background.yaml"))
        self.background_rates = load_background(self.background_file)
        if self.background_rates:
            rospy.loginfo("Data recorder: background subtraction ON (%d lines from %s)",
                          len(self.background_rates), self.background_file)

        # Ensure output dir exists
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # Load energy calibration for N42 export
        self.bin_edges = self._load_energy_cal(cal_path)

        # Dose-rate gamma constants, single-sourced from isotopes.yaml.
        self.gamma_constants = load_gamma_constants(self.isotopes_config)

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
        self.measurement_id = ""
        self.reference_datetime = ""
        # Latest valid detector run-info captured during the active recording.
        self._last_run_info = self._empty_run_info()
        self._run_info_lock = threading.Lock()
        self._run_info_timer = None   # periodic run-info poll during recording
        # Most recent VALID run-info of the SESSION (any run) - dead-time
        # fallback for runs that lose every reply (see the post-stop fetch).
        self._session_prior_run_info = None

        # Isotope-ID screening layer: identifies ANY library nuclide in the
        # accumulated spectrum (SNM/IND/NORM), beyond the assay-configured
        # Cs/Co. Periodic during recording (published on /identified_isotopes,
        # consumed live by the heatmap for imaging bands) + a final full-
        # resolution pass at stop that is embedded in the N42.
        self.nuclide_library = None
        lib_path = rospy.get_param("~nuclide_library", "")
        if lib_path and isotope_id is not None:
            try:
                self.nuclide_library = isotope_id.load_nuclide_library(lib_path)
                rospy.loginfo("Nuclide ID library: %d nuclides from %s",
                              len(self.nuclide_library['nuclides']), lib_path)
            except Exception as e:
                rospy.logwarn("Could not load nuclide library %s: %s",
                              lib_path, e)
        elif lib_path:
            rospy.logwarn("isotope_id module not importable - screening off")
        self.isotope_id_period_s = float(
            rospy.get_param("~isotope_id_period_s", 20.0))
        # Rolling live-spectrum window for CONTINUOUS screening: /spectrum
        # snapshots from the last ~isotope_id_window_s are summed and
        # identified whether or not a recording is active, so the heatmap's
        # imaging bands follow what is in front of the detector NOW (and a
        # removed source drains out of the window). The end-of-run N42 pass
        # still uses the run's own accumulated spectrum.
        self.isotope_id_window_s = float(
            rospy.get_param("~isotope_id_window_s", 180.0))
        self._live_spectra = deque()
        self._id_timer = None
        self._last_screening = None
        self.pub_identified = rospy.Publisher(
            "/identified_isotopes", String, queue_size=2, latch=True)
        # Per-LINE labels as data (JSON): every detected peak with its label
        # ("Cs-137", "U-235/Ra-226", "?"), physics tags and a persistence
        # flag, plus the energy-drift estimate. Single source of truth - the
        # spectrum plotter draws THESE instead of re-identifying its own
        # display buffer, so every view shows identical labels.
        self.pub_identified_lines = rospy.Publisher(
            "/identified_lines", String, queue_size=2, latch=True)
        self._prev_line_energies = []
        self.energy_drift_warn_kev = float(
            rospy.get_param("~energy_drift_warn_kev", 1.0))
        if self.nuclide_library is not None and self.isotope_id_period_s > 0:
            self._id_timer = rospy.Timer(
                rospy.Duration(max(10.0, self.isotope_id_period_s)),
                self._poll_isotope_id)

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
            "/compton_event", ComptonEvent, self._on_compton, queue_size=10000)
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
        # Track the effective source distance (base standoff + shielding plates)
        # published by the activity node, so saved dose/imaging use the same
        # geometry as the activity estimate.
        self.sub_distance = rospy.Subscriber(
            "/activity/effective_source_distance", Float64,
            self._on_source_distance, queue_size=2)

        # Latest isotope text for pairing with peaks
        self._latest_isotope_text = ""
        # Latest activity per isotope (name -> MBq)
        self._latest_activity_MBq = {}

        # NOTE: we deliberately do NOT poll run-info during recording. Every
        # get_run_info call reads and discards a slice of the shared event stream
        # (see _query_run_info_once), which would make the recorded spectrum/
        # heatmap unfaithful to the detector. Instead a single query is taken at
        # stop (_stop_recording), before the acquisition flag is cleared.

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

        # Reset before every run: FULL detector onboard clear ('x', data +
        # windows) + all node buffers. The data-only 'c' clear used here from
        # 2026-08-26 to 2026-09-03 was PROVEN INSUFFICIENT on 2026-09-03: the
        # GeGI re-streams retained spectral content that survives 'c' (~5% of
        # the previous run's rates appeared as e.g. a persistent 2.4 cps
        # Cs-137 line in Co-only runs; a GeGI-software full clear removed it,
        # and the next run captured run-info fine). The August 'x'-breaks-
        # run-info issue is covered by the 30 s run-info polling + retry in
        # _query_run_info_once; a short settle after 'x' gives the detector
        # time to finish the deep clear before acquisition starts. If a run
        # ever saves 0 dead-time again, check run-info first before blaming
        # this clear.
        self._clear_all_impl(full=True)
        rospy.sleep(2.0)

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
        from std_srvs.srv import TriggerResponse
        all_ok, message = self._clear_all_impl(full=True)
        return TriggerResponse(success=all_ok, message=message)

    def _clear_all_impl(self, full=False):
        """Pipeline reset: detector onboard data + every node buffer.

        full=True (run start AND manual /data_recorder/clear_all): the deep
        'x' (data+windows) clear, with 'c' as fallback on older driver
        builds. REQUIRED at run start since 2026-09-03: the GeGI re-streams
        retained spectral content that survives the data-only 'c' clear
        (~5% of the previous run's rates - the source of both the August
        phantom peaks, which the operator's manual full clears were silently
        suppressing, and the September "Cs shine" that wasn't shine; run
        20260903_214229 after a full clear was clean AND captured run-info).
        The 2026-08-26 observation that a pre-run 'x' broke get_run_info is
        covered by the run-info polling + retry; run start also settles 2 s
        after the clear.
        full=False: data-only 'c' clear (kept for callers that must not
        touch onboard windows state).
        Returns (all_ok, message).
        """
        from std_srvs.srv import Trigger
        from phds_gegi_driver.srv import ClearData

        results = []
        all_ok = True

        # 1) Detector onboard data.
        cleared = False
        hw_services = (("/detector/clear_data_and_windows",
                        "/detector/clear_data") if full
                       else ("/detector/clear_data",))
        for service in hw_services:
            try:
                rospy.wait_for_service(service, timeout=3.0)
                r = rospy.ServiceProxy(service, ClearData)()
                ok = bool(r.success)
                results.append("{}:{}".format(service, "ok" if ok else "fail"))
                cleared = ok
                if ok:
                    break
            except Exception as e:
                results.append("{}:err({})".format(service, e))
        all_ok = all_ok and cleared

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
        return all_ok, message

    def _start_recording(self, duration_minutes):
        start_dt = datetime.now()
        timestamp = start_dt.strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(self.output_dir, "{}_compton_events.bag".format(timestamp))

        # Align the activity node's counting window with this run BEFORE the
        # recording flag goes up, so a stale (pre-run, idle-diluted) window can
        # never be appended to this run's results.
        self._clear_activity_node()

        with self._run_info_lock:
            self._last_run_info = self._empty_run_info()

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
            # Durable measurement identity, stamped into every asset of this run.
            self.measurement_id = "{}-{}".format(self.measurement_id_prefix, timestamp)
            # ISO-8601 acquisition reference time (spec DB-ACT-003: activity
            # without a reference date/time is not durable).
            self.reference_datetime = start_dt.isoformat()
            self.bag = rosbag.Bag(bag_path, 'w')

        rospy.loginfo("Recording started: %d min, bag=%s", duration_minutes, bag_path)

        # Set a timer to stop recording after the duration
        duration_secs = duration_minutes * 60.0
        self.timer = threading.Timer(duration_secs, self._stop_recording)
        self.timer.daemon = True
        self.timer.start()

        # Poll detector run-info periodically DURING the run and keep the last
        # valid frame. A single pre-stop query alone misses it for a timed
        # acquisition: the detector auto-stops at the duration, so by the time we
        # ask it is idle and returns zeros. Polling is safe now that get_run_info
        # is served INLINE by the driver's monitor thread (it no longer consumes
        # any event-stream bytes), so it does not degrade the recorded data.
        self._run_info_timer = rospy.Timer(
            rospy.Duration(max(5.0, self.run_info_poll_s)), self._poll_run_info)
        # (Isotope-ID screening runs on its own permanent timer over the
        # rolling live window - see __init__ - so it needs no start here.)
        self._last_screening = None

    def _stop_recording(self):
        """Called when the timed acquisition ends. Save all files."""
        rospy.loginfo("Timed acquisition complete. Saving data files...")

        # Stop the periodic run-info poll; the last valid frame it captured
        # (mid-run, while the detector was definitely acquiring) is what we save.
        if getattr(self, '_run_info_timer', None) is not None:
            self._run_info_timer.shutdown()
            self._run_info_timer = None
        # (the isotope-ID timer is permanent - it keeps screening the rolling
        # live window between recordings, so it is NOT shut down here)

        # One more best-effort query (the detector is usually idle by now for a
        # timed run, returning zeros). Only overwrite the polled value if this one
        # is a genuine active frame (real_time > 0), so a post-stop idle reply
        # never clobbers a good mid-run capture.
        pre_stop_info = self._query_run_info_once()
        if pre_stop_info['valid'] and pre_stop_info['real_time_sec'] > 0.0:
            with self._run_info_lock:
                self._last_run_info = pre_stop_info

        # Flush the activity node's final (not-yet-full) counting window NOW, while
        # recording is still True so _on_activity appends it. Without this a long
        # counting_window_s (one window per run) would drop the run's only window.
        self._flush_activity_node()

        # Keep recording flag on briefly to capture any final heatmap/activity publishes
        # The heatmap node publishes every ~2s; wait one cycle to get final state.
        rospy.sleep(3.0)

        # Stop recording and snapshot totals NOW, BEFORE the run-info settle/fetch.
        # Otherwise the ~15-30 s spent settling and querying detector run-info would
        # keep accumulating into total_real_time_ms/total_live_time_ms and inflate
        # the reported run duration (e.g. 5 min -> ~338 s).
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
            measurement_id = self.measurement_id
            reference_datetime = self.reference_datetime
            duration_minutes = self.duration_minutes

        # Get detector-reported run info once the stream settles (acquisition is
        # already flagged stopped, so this no longer inflates the run totals).
        detector_run_info = self._fetch_detector_run_info_post_stop()

        # Dead-time correction factor for this run (one value, applied uniformly
        # to net areas and activities in the CSV and the N42). The per-window
        # activity node cannot supply live dead-time here because run-info polling
        # during acquisition is disabled (single-socket contention), so the
        # authoritative post-run figure is the one and only correction point.
        dt_factor = 1.0
        if self.apply_dead_time_correction:
            dt_factor = dead_time_correction_factor(
                detector_run_info.get('real_time_sec'),
                detector_run_info.get('live_time_sec'),
                detector_run_info.get('dead_time_percent'))
        rospy.loginfo("  Dead-time correction factor: %.5f (%s)", dt_factor,
                      "applied" if self.apply_dead_time_correction else "disabled")

        # Close bag file
        if bag:
            bag.close()
            rospy.loginfo("  Bag saved: %s_compton_events.bag", prefix)

        # If this run is a BACKGROUND measurement, write its per-line net rates as
        # the reference and do NOT subtract it from itself. Otherwise subtract the
        # loaded background from this sample.
        record_bg = bool(rospy.get_param("~record_background", False))
        if record_bg:
            self._write_background(activities, measurement_id)
        bg_rates = None if record_bg else self.background_rates

        # Final isotope-ID screening pass at FULL resolution over the RUN's
        # accumulated spectrum (the periodic passes use the rolling live
        # window, 4x rebinned). Result goes on /identified_isotopes and into
        # the N42 <AnalysisResults> as screening identifications.
        screening = self._run_isotope_id(rebin=1, source='run')

        # Save spectrum as N42, with the run's activity results embedded so the
        # N42 is a complete standards-compliant record (spectrum + activities).
        self._save_n42(prefix, spectrum, real_time_ms, live_time_ms,
                       measurement_id, reference_datetime, activities, dt_factor,
                       bg_rates, screening=screening)

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
                                    real_time_ms, live_time_ms,
                                    measurement_id, reference_datetime, dt_factor,
                                    bg_rates)
        except Exception as e:
            rospy.logerr("Failed to save activity CSV: %s", e)

        # Save the run manifest that links every asset to one parent Measurement
        # (spec DB-GEGI-002 / DB-GEGI-004).
        try:
            self._save_manifest(prefix, measurement_id, reference_datetime,
                                duration_minutes, real_time_ms, live_time_ms,
                                detector_run_info, activities)
        except Exception as e:
            rospy.logerr("Failed to save run manifest: %s", e)

        rospy.loginfo("All data files saved with prefix: %s (measurement_id=%s)",
                      prefix, measurement_id)

    def _flush_activity_node(self):
        """Ask the activity node to publish its final partial counting window.

        Best-effort: a failure just means the last window (usually a small tail)
        is missing, not a crash. Called at end-of-run so a long counting window
        (one per run) still records its data.
        """
        from std_srvs.srv import Trigger
        try:
            rospy.wait_for_service("/activity/flush", timeout=2.0)
            rospy.ServiceProxy("/activity/flush", Trigger)()
        except Exception as e:
            rospy.logwarn("Could not flush activity node (%s)", e)

    def _clear_activity_node(self):
        """Reset the activity node's counting window at recording START.

        The node's window free-runs, so without this the run's FIRST window
        can straddle the start and carry PRE-RECORDING (idle) live time:
        real bug 2026-08-26 - a 303 s run reported 378.5 s total live time,
        deflating every activity ~20% (and the rate-based partial-window
        filter sat exactly at its threshold and let the window through).
        Clearing here aligns window boundaries with the run: total window
        live time == recording live time. Symmetric partner of the flush at
        stop. Best-effort, like the flush.
        """
        from std_srvs.srv import Trigger
        try:
            rospy.wait_for_service("/activity/clear", timeout=2.0)
            rospy.ServiceProxy("/activity/clear", Trigger)()
        except Exception as e:
            rospy.logwarn("Could not clear activity node (%s)", e)

    @staticmethod
    def _empty_run_info():
        return {'valid': False, 'dead_time_percent': 0.0, 'real_time_sec': 0.0,
                'live_time_sec': 0.0, 'count_rate_hz': 0.0, 'message': 'not_queried'}

    def _run_isotope_id(self, rebin=1, source='live'):
        """Run the screening layer and publish/cache the result.

        source='live' (periodic): sum of the rolling ~isotope_id_window_s of
        /spectrum snapshots - reflects what is in front of the detector NOW,
        works with or without an active recording (a removed source drains
        out of the window). source='run' (end of run): the recording's own
        accumulated spectrum - faithful to the saved N42. rebin>1 sums
        adjacent channels for the cheap periodic pass (the matched filter is
        bin-width-aware). Returns (results, unknown) or None."""
        if self.nuclide_library is None:
            return None
        with self.lock:
            if source == 'run':
                counts = self.accumulated_spectrum.astype(np.float64).copy()
            else:
                if not self._live_spectra:
                    return None
                counts = np.zeros(len(self.accumulated_spectrum),
                                  dtype=np.float64)
                for _, arr in self._live_spectra:
                    counts += arr
        if counts.sum() < 100:
            return None
        # Overflow guard: out-of-range energies pile into the TOP bin (known
        # spectrum-node behaviour) and would fake a peak at the range end.
        counts[-1] = 0.0
        edges = np.asarray(self.bin_edges, dtype=np.float64)
        if len(edges) < 2:
            return None
        bin_w = float(np.median(np.diff(edges)))
        centers = edges + bin_w / 2.0
        n = min(len(centers), len(counts))
        counts, centers = counts[:n], centers[:n]
        if rebin > 1:
            m = (n // rebin) * rebin
            counts = counts[:m].reshape(-1, rebin).sum(axis=1)
            centers = centers[:m].reshape(-1, rebin).mean(axis=1)
        try:
            peaks, results, unknown = isotope_id.identify(
                counts, centers, self.nuclide_library)
        except Exception as e:
            rospy.logwarn("Isotope-ID pass failed: %s", e)
            return None
        self._last_screening = (results, unknown)
        identified = [r for r in results if r['identified']]
        text = "|".join("{}:{:.2f}".format(r['nuclide'], r['score'])
                        for r in identified) or "none"
        self.pub_identified.publish(String(data=text))
        if identified:
            rospy.loginfo("Screening ID: %s", text)

        # Per-line labels (single source of truth for every display) with
        # flicker suppression and physics tags.
        lines_out = isotope_id.label_peaks(peaks, results)
        self._prev_line_energies = isotope_id.mark_persistent(
            lines_out, self._prev_line_energies)
        drift, n_drift = isotope_id.energy_drift_kev(results)
        if n_drift >= 2 and abs(drift) > self.energy_drift_warn_kev:
            rospy.logwarn_throttle(
                300, "ENERGY-CALIBRATION DRIFT suspected: identified lines "
                "sit %+.2f keV from their library energies (%d lines). "
                "Labels and assay ROIs degrade with drift - check the "
                "energy calibration." % (drift, n_drift))
        self.pub_identified_lines.publish(String(data=json.dumps(
            {'lines': lines_out,
             'drift_keV': round(drift, 3),
             'n_drift_lines': n_drift})))
        return (results, unknown)

    def _poll_isotope_id(self, _event):
        """Periodic screening over the rolling live window (rospy.Timer
        thread). Runs whether or not a recording is active - live viewing
        gets identifications (and heatmap bands) too."""
        self._run_isotope_id(rebin=4, source='live')

    def _poll_run_info(self, _event):
        """Periodic run-info capture DURING recording; keeps the last valid frame.
        Runs in a rospy.Timer thread. Safe/lossless now that the driver serves
        get_run_info inline (no event-stream bytes consumed)."""
        if not self.recording:
            return
        info = self._query_run_info_once()
        if info['valid'] and info['real_time_sec'] > 0.0:
            with self._run_info_lock:
                self._last_run_info = info

    def _query_run_info_once(self, attempts=3, retry_gap_s=0.7):
        """Query /detector/get_run_info, retrying a couple of times.

        The driver serves run-info inline: it prompts the detector ('i') and
        waits up to ~1.5 s for the reply frame to be decoded by its monitor
        thread. Under event load the decode can land just AFTER that window -
        the service call fails but the frame IS cached driver-side, so an
        immediate retry returns it. Observed 2026-09-03 (run 153132): every
        30 s poll of a 5-min run failed ('not_queried' manifest, 0 dead-time
        CSV) while the driver console printed valid inline frames.
        """
        info = self._empty_run_info()
        for attempt in range(max(1, int(attempts))):
            if attempt:
                rospy.sleep(retry_gap_s)
            try:
                rospy.wait_for_service('/detector/get_run_info', timeout=2.0)
                proxy = rospy.ServiceProxy('/detector/get_run_info', GetRunInfo)
                resp = proxy()
                info['message'] = resp.message
                if resp.success:
                    info['valid'] = True
                    info['dead_time_percent'] = float(resp.dead_time_percent)
                    info['real_time_sec'] = float(resp.real_time_sec)
                    info['live_time_sec'] = float(resp.live_time_sec)
                    # count_rate_hz = detector's TRUE input singles rate (the
                    # rate axis for the Exp C rate/dead-time characterisation).
                    info['count_rate_hz'] = float(resp.count_rate_hz)
                    return info
            except Exception as e:
                info['message'] = str(e)
        return info

    def _fetch_detector_run_info_post_stop(self):
        """Return the run-info captured by the single pre-stop query.

        We do NOT query again here: the detector often resets to idle on stop
        (returning zeros), and every query discards event-stream data. The
        pre-stop query in _stop_recording is the one and only read for the run.
        """
        with self._run_info_lock:
            stored = dict(self._last_run_info)
        if stored.get('valid') and stored.get('real_time_sec', 0.0) > 0.0:
            rospy.loginfo("  Detector run info: real=%.3fs live=%.3fs dead=%.3f%% "
                          "rate=%.1f Hz",
                          stored['real_time_sec'], stored['live_time_sec'],
                          stored['dead_time_percent'],
                          stored.get('count_rate_hz', 0.0))
            # Remember across runs: the fallback below borrows this dead time
            # for a later run whose every run-info reply is lost.
            self._session_prior_run_info = dict(stored)
            return stored

        # FALLBACK (2026-09-04): some runs lose EVERY run-info reply to
        # event-stream interleaving (~1 in 6 observed; the complete fix needs
        # a dedicated command channel - PHDS question). Saving dt=0 silently
        # under-corrects the assay by the true dead time (0.6-2.6%), so borrow
        # the most recent VALID dead time from THIS session, clearly flagged.
        # Dead time is composition-stable session to session (Co ~2.5%,
        # Cs ~0.7%, mixed ~1.3% measured 2026-09-03/04), so the estimate is
        # good to ~+-0.5% - far better than no correction. count_rate/real/
        # live stay 0 (they are per-run quantities and unknown), which also
        # makes estimated runs recognisable in the CSV (dt > 0, rate = 0).
        prior = getattr(self, '_session_prior_run_info', None)
        if prior and prior.get('dead_time_percent', 0.0) > 0.0:
            est = self._empty_run_info()
            est['valid'] = True
            est['dead_time_percent'] = float(prior['dead_time_percent'])
            est['message'] = ('dead_time_ESTIMATED_from_prior_run (run-info '
                             'unavailable this run; count rate unknown)')
            rospy.logwarn("  Detector run info unavailable - using ESTIMATED "
                          "dead time %.3f%% from the previous valid run",
                          est['dead_time_percent'])
            return est

        rospy.logwarn("  Detector run info unavailable for this run")
        return self._empty_run_info()

    def _on_source_distance(self, msg):
        """Track the plate-derived source distance for dose/imaging outputs."""
        if msg.data > 0 and abs(msg.data - self.source_distance_m) > 1e-4:
            self.source_distance_m = float(msg.data)
            rospy.loginfo("Data recorder: source distance -> %.3fm", self.source_distance_m)

    def _on_compton(self, msg):
        with self.lock:
            if not self.recording or self.bag is None:
                return
            try:
                self.bag.write("/compton_event", msg, rospy.Time.now())
            except Exception:
                pass

    def _on_spectrum(self, msg):
        arr = np.array(msg.spectrum, dtype=np.uint32)
        # IDLE GUARD (same as the activity node): the spectrum node stamps
        # wall-clock real time on every snapshot even when the detector is not
        # acquiring. A zero-count snapshot = idle detector; counting its time
        # would inflate the run's real/live totals (e.g. the ~3 s stop grace).
        if arr.sum() <= 0:
            return
        with self.lock:
            # Rolling live window for continuous isotope-ID screening -
            # fed ALWAYS (recording or not). Same-length snapshots only.
            now = rospy.get_time()
            if len(arr) == len(self.accumulated_spectrum):
                self._live_spectra.append((now, arr))
            while (self._live_spectra and
                   now - self._live_spectra[0][0] > self.isotope_id_window_s):
                self._live_spectra.popleft()

            if not self.recording:
                return
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

    def _build_analysis_results_xml(self, activities, measurement_id, dt_factor=1.0,
                                    background_rates=None, screening=None):
        """Build the N42 <AnalysisResults> block: the run-level nuclide activities.

        N42.42 has native elements for nuclide activity results, so the spectrum
        and the activities derived from it travel together in one standards
        compliant record instead of being split across an N42 + a bespoke CSV.

        Activities are aggregated over the run (total net counts / total live
        time, partial start-up windows excluded) and reported per RADIONUCLIDE:
        Co-60's two photopeaks quantify the same nuclide and are averaged.

        screening (optional) = (id_results, unknown_peaks) from the isotope-ID
        layer's end-of-run pass: identified library nuclides NOT covered by the
        assay are appended as screening identifications (present, no activity),
        and peaks matching no library nuclide are recorded in a remark.
        """
        lines = aggregate_run_activity(activities, dt_factor=dt_factor,
                                       background_rates=background_rates)
        nuclides = (group_by_radionuclide(lines, self._controlled_radionuclide)
                    if lines else {})
        u_sys = self.assay_systematic_uncertainty_pct

        nuclide_xml = []
        for name in sorted(nuclides):
            res = nuclides[name]
            activity_bq = res['activity_MBq'] * 1.0e6
            u_count = (100.0 * res['counting_sigma_MBq'] / res['activity_MBq']
                       if res['activity_MBq'] > 0 else 0.0)
            u_expanded = 2.0 * math.sqrt(u_count ** 2 + u_sys ** 2)   # k=2
            nuclide_xml.append(
                '      <Nuclide>\n'
                '        <NuclideIdentifiedIndicator>true</NuclideIdentifiedIndicator>\n'
                '        <NuclideName>{name}</NuclideName>\n'
                '        <NuclideActivityValue units="Bq">{act:.6g}</NuclideActivityValue>\n'
                '        <NuclideActivityUncertaintyValue>{unc:.6g}</NuclideActivityUncertaintyValue>\n'
                '        <Remark>Lines: {lines}. Counting u={uc:.2f}% (1sigma); '
                'systematic u={us:.2f}% (1sigma); expanded U={ue:.1f}% (k=2). '
                'Line spread {spread:.2f}%.</Remark>\n'
                '      </Nuclide>'.format(
                    name=name, act=activity_bq,
                    unc=activity_bq * u_expanded / 100.0,
                    lines=" ".join(res['lines']), uc=u_count, us=u_sys,
                    ue=u_expanded, spread=res['line_spread_percent']))

        # Non-detections: a configured nuclide with no detected line is reported
        # ONCE as a bounded non-detection (Currie MDA), which is real evidence -
        # "screened down to < X and absent" - unlike a silent omission.
        if self.report_non_detected_mda:
            detected = set(lines.keys())   # run-level detected lines (from aggregate)
            mdas = non_detected_nuclide_mdas(
                activities, detected, self._controlled_radionuclide)
            for name in sorted(mdas):
                m = mdas[name]
                mda_bq = m['mda_Bq'] * dt_factor
                nuclide_xml.append(
                    '      <Nuclide>\n'
                    '        <NuclideIdentifiedIndicator>false</NuclideIdentifiedIndicator>\n'
                    '        <NuclideName>{name}</NuclideName>\n'
                    '        <NuclideActivityValue units="Bq">0</NuclideActivityValue>\n'
                    '        <Remark>Not detected. Currie MDA (L_D, 95% confidence) '
                    '= {mbq:.6g} MBq ({bq:.6g} Bq) over live time {live:.0f} s, '
                    'bounded by line {line}. The MDA reflects the background under '
                    'the ROI during this run.</Remark>\n'
                    '      </Nuclide>'.format(
                        name=name, mbq=mda_bq / 1.0e6, bq=mda_bq,
                        live=m['live_time_s'], line=m['line']))

        # Screening identifications from the isotope-ID layer: library nuclides
        # present in the spectrum but outside the calibrated assay set (e.g.
        # Eu-152, NORM lines). Assay-reported nuclides are excluded - the
        # quantified entry (or MDA) above is the authoritative one for those.
        scr_remark = ''
        if screening:
            id_results, unknown_pks = screening
            assay_names = set(nuclides.keys())
            if self.report_non_detected_mda:
                assay_names.update(
                    self._controlled_radionuclide(i.get('isotope', ''))
                    for rep in activities for i in rep.get('isotopes', []))
            scr_blocks, scr_remark = screening_xml_block(
                id_results, unknown_pks, exclude_names=assay_names)
            nuclide_xml.extend(scr_blocks)

        peak_xml = []
        for label in sorted(lines):
            res = lines[label]
            cps = (res['net_counts'] / res['live_time_s']
                   if res['live_time_s'] > 0 else 0.0)
            pos_note = ''
            if res.get('position_corrected'):
                pos_note = (' Position-corrected from imaged hotspot: slant '
                            'distance {slant:.3f} m, efficiency factor '
                            '{pf:.4f}.'.format(slant=res['slant_distance_m'],
                                               pf=res['position_factor']))
            peak_xml.append(
                '      <Peak>\n'
                '        <PeakEnergyValue units="keV">{e:.2f}</PeakEnergyValue>\n'
                '        <PeakNetAreaValue>{net:.1f}</PeakNetAreaValue>\n'
                '        <PeakNetCountRateValue units="cps">{cps:.4f}</PeakNetCountRateValue>\n'
                '        <Remark>{label}: {used} window(s) used, {dropped} partial '
                'window(s) excluded; live time {live:.1f} s.{pos}</Remark>\n'
                '      </Peak>'.format(
                    e=res['energy_keV'], net=res['net_counts'], cps=cps,
                    label=label, used=res['windows_used'],
                    dropped=res['windows_dropped'], live=res['live_time_s'],
                    pos=pos_note))

        if not nuclide_xml and not peak_xml:
            return ""

        ref = (' radMeasurementReferences="{}"'.format(measurement_id)
               if measurement_id else '')

        # Mixed-field Cs-137 : Co-60 ratio (only when BOTH are detected).
        ratio_remark = ''
        ratio = cs137_co60_ratio(nuclides)
        if ratio is not None:
            parts = []
            if ratio['activity_ratio'] is not None:
                parts.append('activity ratio {:.3g}'.format(ratio['activity_ratio']))
            if ratio['count_ratio'] is not None:
                parts.append('raw net-count ratio {:.3g} (calibration-independent)'
                             .format(ratio['count_ratio']))
            ratio_remark = (
                '    <Remark>Mixed field Cs-137 : Co-60 (as Cs-137 / Co-60): '
                '{parts}. Co-60 is the PRIMARY quantitative reference (two '
                'self-consistent lines, and unaffected by Cs-137); Cs-137 is '
                'SECONDARY - its 662 keV peak sits on the Co-60 Compton continuum, '
                'which raises its background and MDA.</Remark>\n'.format(
                    parts=', '.join(parts)))

        sections = []
        if nuclide_xml:
            sections.append('    <NuclideAnalysisResults>\n{}\n'
                            '    </NuclideAnalysisResults>\n'.format(
                                "\n".join(nuclide_xml)))
        if peak_xml:
            sections.append('    <PeakAnalysisResults>\n{}\n'
                            '    </PeakAnalysisResults>\n'.format(
                                "\n".join(peak_xml)))
        return (
            '  <AnalysisResults{ref}>\n'
            '    <Remark>Activities derived from the accumulated spectrum of this '
            'measurement. Detected nuclides quote the EXPANDED uncertainty (k=2, '
            '95%), combining per-run counting statistics with the systematic assay '
            'uncertainty ({us:.1f}% 1sigma, dominated by source position within the '
            'tray); it is NOT counting statistics alone. Non-detected configured '
            'nuclides carry a Currie MDA.</Remark>\n'
            '{ratio}'
            '{screening}'
            '{sections}'
            '  </AnalysisResults>\n'.format(
                ref=ref, us=u_sys, ratio=ratio_remark, screening=scr_remark,
                sections="".join(sections)))

    def _save_n42(self, prefix, spectrum, real_time_ms, live_time_ms,
                  measurement_id="", reference_datetime="", activities=None,
                  dt_factor=1.0, background_rates=None, screening=None):
        """Save spectrum in ANSI N42.42 XML format.

        The measurement_id is stamped onto <RadMeasurement id=...> so the raw
        N42 asset is linkable back to the parent Measurement (spec DB-GEGI-004).
        When activity results are supplied they are embedded as <AnalysisResults>,
        making the N42 a complete, self-describing record of the measurement.
        """
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
  <RadMeasurement id="{measurement_id}">
    <MeasurementClassCode>Foreground</MeasurementClassCode>
    <StartDateTime>{reference_datetime}</StartDateTime>
    <RealTimeDuration>PT{real_time:.3f}S</RealTimeDuration>
    <Spectrum>
      <LiveTimeDuration>PT{live_time:.3f}S</LiveTimeDuration>
      <ChannelData compressionCode="None">{spectrum_data}</ChannelData>
    </Spectrum>
  </RadMeasurement>
{analysis_results}</RadInstrumentData>
""".format(
            offset=offset,
            gain=gain,
            real_time=real_time_s,
            live_time=live_time_s,
            spectrum_data=spectrum_text,
            measurement_id=measurement_id,
            reference_datetime=reference_datetime,
            analysis_results=self._build_analysis_results_xml(
                activities or [], measurement_id, dt_factor, background_rates,
                screening=screening)
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
        d = self.source_distance_m
        dose_rates = {}
        for entry in peaks:
            iso_name = entry[3]
            activity_mbq = entry[4] if len(entry) > 4 else 0.0
            gamma = gamma_constant_for(self.gamma_constants, iso_name)
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

    # Map the driver's internal isotope/line labels to controlled radionuclide
    # names for the database ActivityResult (spec DB-ACT-001, sec 7.3). The internal
    # per-line label (e.g. "Co60_1332") is kept separately in the `isotope`
    # column; `radionuclide` carries the controlled source identity.
    _RADIONUCLIDE_MAP = {
        'Cs137': 'Cs-137',
        'Co60': 'Co-60',
        'Co60_1173': 'Co-60',
        'Co60_1332': 'Co-60',
    }

    @classmethod
    def _controlled_radionuclide(cls, name):
        if name in cls._RADIONUCLIDE_MAP:
            return cls._RADIONUCLIDE_MAP[name]
        if 'Cs137' in name or 'Cs-137' in name:
            return 'Cs-137'
        if 'Co60' in name or 'Co-60' in name:
            return 'Co-60'
        return name

    def _write_background(self, activities, measurement_id):
        """Write this (no-source) run's per-line net rates as the background
        reference used to subtract from later runs. Also updates the in-memory
        rates so the very next run subtracts without a relaunch.
        """
        rates = build_background_rates(activities)
        doc = {
            'source_measurement_id': measurement_id,
            'note': ('Per-line background NET rate (cps) for subtraction. '
                     'Re-record on a no-source run with '
                     'rosparam set /data_recorder/record_background true.'),
            'lines': rates,
        }
        try:
            with open(self.background_file, 'w') as f:
                yaml.safe_dump(doc, f, default_flow_style=False)
            self.background_rates = rates
            rospy.loginfo("  Background reference written: %s (%d lines)",
                          self.background_file, len(rates))
        except Exception as e:
            rospy.logerr("Failed to write background reference: %s", e)

    def _save_activity_csv(self, prefix, activities, detector_run_info,
                           run_real_time_ms=0, run_live_time_ms=0,
                           measurement_id="", reference_datetime="",
                           dt_factor=1.0, background_rates=None):
        """Save a RUN-LEVEL activity summary: ONE row per DETECTED line.

        Each row is the whole-run aggregate (total net / total live, ramp-up
        windows excluded) - the SAME numbers embedded in the N42 - so a single
        stable activity per line instead of a per-window series that varies window
        to window. Undetected lines are NOT written here; they appear once in the
        N42 as a Currie MDA. A blank run (nothing detected) therefore yields a
        header-only CSV - read the N42 for its MDAs.

        Columns are run TOTALS: net_peak_area (raw), net_corrected (x dt_factor),
        gross/background counts, live time, activity + counting sigma.
        measurement_id links a stray CSV to its Measurement (spec DB-GEGI-002/004).
        """
        filepath = os.path.join(self.output_dir, "{}_activity.csv".format(prefix))

        lines = aggregate_run_activity(activities, dt_factor=dt_factor,
                                       background_rates=background_rates)
        drdt = detector_run_info.get('dead_time_percent', 0.0)
        drcr = detector_run_info.get('count_rate_hz', 0.0)
        ts = activities[0].get('timestamp', 0) if activities else 0

        with open(filepath, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp_s", "measurement_id", "isotope", "valid",
                "live_time_s", "gross_counts", "background_counts",
                "net_peak_area", "net_corrected", "detector_run_dead_time_percent",
                "detector_run_count_rate_hz", "activity_MBq", "sigma_activity_MBq"
            ])

            for label in sorted(lines):
                r = lines[label]
                net_raw = r['net_counts']          # raw net (dt not yet applied)
                writer.writerow([
                    "{:.3f}".format(ts),
                    measurement_id,
                    label,
                    True,                          # only detected lines are written
                    "{:.3f}".format(r['live_time_s']),
                    "{:.0f}".format(r['gross_counts']),
                    "{:.1f}".format(r['background_counts']),
                    "{:.1f}".format(net_raw),
                    "{:.1f}".format(net_raw * dt_factor),
                    "{:.6f}".format(drdt),
                    "{:.1f}".format(drcr),
                    "{:.6f}".format(r['activity_MBq']),
                    "{:.6f}".format(r['counting_sigma_MBq']),
                ])

        rospy.loginfo("  Activity CSV saved: %s (%d detected line(s))",
                      filepath, len(lines))

    def _save_manifest(self, prefix, measurement_id, reference_datetime,
                       duration_minutes, run_real_time_ms, run_live_time_ms,
                       detector_run_info, activities):
        """Write a per-run manifest linking every asset to one Measurement.

        Anchors the database Measurement record (spec DB-GEGI-002/004): captures
        the acquisition/processing configuration and the list of raw and
        processed assets produced by this run.
        """
        import json
        filepath = os.path.join(self.output_dir, "{}_manifest.json".format(prefix))

        # Candidate assets with their database role; only list those written.
        candidates = [
            ("{}_spectrum.n42".format(prefix), "raw_spectrum_n42", "N42"),
            ("{}_compton_events.bag".format(prefix), "raw_compton_events", "rosbag"),
            ("{}_activity.csv".format(prefix), "processed_peak_results", "CSV"),
            ("{}_heatmap_raw.csv".format(prefix), "gamma_image_raw", "CSV"),
            ("{}_heatmap_raster.csv".format(prefix), "gamma_image_raster", "CSV"),
            ("{}_heatmap_3d.csv".format(prefix), "gamma_image_3d", "CSV"),
        ]
        assets = []
        for fname, role, kind in candidates:
            fpath = os.path.join(self.output_dir, fname)
            if os.path.exists(fpath):
                assets.append({
                    "file": fname,
                    "role": role,
                    "format": kind,
                    "size_bytes": os.path.getsize(fpath),
                })

        # Distinct radionuclides seen in the processed results.
        radionuclides = []
        for report in activities:
            for iso in report.get('isotopes', []):
                rn = self._controlled_radionuclide(iso.get('isotope', ''))
                if rn and rn not in radionuclides:
                    radionuclides.append(rn)

        manifest = {
            "measurement_id": measurement_id,
            "measurement_type": "gamma_spectroscopy+gamma_imaging",
            "reference_datetime": reference_datetime,
            "detector": {"manufacturer": "PHDS", "model": "GeGI", "kind": "HPGe"},
            "acquisition": {
                "requested_duration_minutes": duration_minutes,
                "run_real_time_s": run_real_time_ms / 1000.0,
                "run_live_time_s": run_live_time_ms / 1000.0,
                "source_distance_m": self.source_distance_m,
                "activity_basis": "direct_measured",
            },
            "processing_config": {
                "calibration_file": self.calibration_file,
                "isotopes_config": self.isotopes_config,
                "heatmap_grid_res": self.heatmap_grid_res,
            },
            "detector_run_info": detector_run_info,
            "radionuclides": radionuclides,
            "assets": assets,
        }

        with open(filepath, 'w') as f:
            json.dump(manifest, f, indent=2, sort_keys=True)

        rospy.loginfo("  Manifest saved: %s (%d assets, id=%s)",
                      filepath, len(assets), measurement_id)


def main():
    rospy.init_node("data_recorder_node")
    node = DataRecorderNode()
    rospy.spin()


if __name__ == "__main__":
    main()

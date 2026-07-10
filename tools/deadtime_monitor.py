#!/usr/bin/env python
"""Live dead-time / count-rate monitor for DJR Assay Experiment C.

The GeGI's get_run_info dead_time_percent is CUMULATIVE since acquisition start,
so it decays as a fixed start-up dead period is diluted over elapsed time (e.g.
~24% at 2 s -> ~1.7% at 28 s) and does NOT reflect the true loading. This tool
polls the service and computes the INSTANTANEOUS dead-time from the change in
real/live time between polls, which is the correct observable for the rate/
throughput characterisation:

    instantaneous DT = (1 - d_live/d_real) * 100

Run it (in the ROS env) while stepping the source closer; read the steady
instantaneous DT and count rate at each standoff. Optionally log to CSV.

  rosrun phds_gegi_driver deadtime_monitor.py            # or: python tools/deadtime_monitor.py
  python tools/deadtime_monitor.py --interval 3 --csv "data/Experiment C/deadtime_log.csv"
"""
from __future__ import print_function

import argparse
import time
from collections import deque

import rospy
from phds_gegi_driver.srv import GetRunInfo


def main():
    ap = argparse.ArgumentParser(description="GeGI instantaneous dead-time monitor")
    ap.add_argument("--service", default="/detector/get_run_info")
    ap.add_argument("--interval", type=float, default=3.0,
                    help="poll period seconds (>=2 recommended; realTime is 1 s "
                         "granular so short intervals are noisy)")
    ap.add_argument("--window", type=float, default=15.0,
                    help="seconds of real-time over which to average the "
                         "instantaneous dead-time (beats the 1 s realTime "
                         "granularity that makes single-step instDT noisy)")
    ap.add_argument("--csv", default=None, help="optional CSV log path")
    args = ap.parse_args()

    rospy.init_node("deadtime_monitor", anonymous=True)
    rospy.loginfo("Waiting for %s ...", args.service)
    rospy.wait_for_service(args.service)
    call = rospy.ServiceProxy(args.service, GetRunInfo)

    writer = None
    fh = None
    if args.csv:
        import csv as _csv
        fh = open(args.csv, "w")
        writer = _csv.writer(fh)
        writer.writerow(["wall_s", "real_s", "live_s", "cum_dt_pct",
                         "inst_dt_pct", "count_rate_hz"])

    print("%-9s %8s %8s %9s %10s %12s"
          % ("wall_s", "real_s", "live_s", "cumDT%", "instDT%", "rate_Hz"))
    hist = deque()   # (real, live) samples, for windowed instantaneous DT
    t0 = None
    while not rospy.is_shutdown():
        try:
            r = call()
        except rospy.ServiceException as exc:
            rospy.logwarn("service call failed: %s", exc)
            time.sleep(args.interval)
            continue
        if not getattr(r, "success", False):
            print("  (run-info invalid - is an acquisition running?)")
            time.sleep(args.interval)
            continue

        now = rospy.get_time()
        if t0 is None:
            t0 = now
        real, live = r.real_time_sec, r.live_time_sec
        cum_dt = r.dead_time_percent
        rate = r.count_rate_hz

        # Windowed instantaneous dead-time: compare against the oldest sample
        # that is >= --window seconds of real-time back, so d_real is large and
        # the 1 s realTime granularity contributes negligible error. Falls back
        # to the immediately previous sample until the window fills.
        inst_dt = float("nan")
        base = None
        for s in hist:
            if real - s[0] >= args.window:
                base = s
            else:
                break
        if base is None and hist:
            base = hist[-1]
        if base is not None:
            d_real = real - base[0]
            d_live = live - base[1]
            if d_real > 0:
                inst_dt = (1.0 - d_live / d_real) * 100.0
        hist.append((real, live))
        # keep a little more than the window so an old-enough baseline is available
        while len(hist) > 2 and (real - hist[0][0]) > args.window + 3 * args.interval:
            hist.popleft()

        wall = now - t0
        istr = "%10.2f" % inst_dt if inst_dt == inst_dt else "%10s" % "-"
        print("%-9.1f %8.1f %8.2f %9.2f %s %12.1f"
              % (wall, real, live, cum_dt, istr, rate))
        if writer:
            writer.writerow(["%.1f" % wall, "%.1f" % real, "%.3f" % live,
                             "%.3f" % cum_dt,
                             ("%.3f" % inst_dt) if inst_dt == inst_dt else "",
                             "%.1f" % rate])
            fh.flush()

        time.sleep(args.interval)

    if fh:
        fh.close()


if __name__ == "__main__":
    main()

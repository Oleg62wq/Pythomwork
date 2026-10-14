#!/usr/bin/env python3
"""Follow a person with the RHC stack: tracker + fusion -> UDP -> rhc_dwa (C) -> STM32.

    cd ~/Documents
    python3 -m scripts.follow_rhc                 # DRY RUN: rhc_dwa computes, STM gets only STOP
    python3 -m scripts.follow_rhc --drive         # motors on

This script does NOT open the STM UART and does NOT run the DWA any more.
  - it starts rhc_dwa (RHC_DWA_Encoder_2026-10-14/raspberry_pi/run.sh) as a child process;
    rhc_dwa owns the UART, reads the wheel velocities from the STM (O1 telemetry),
    runs the DWA every 0.09 s with u_prev = measured velocity and sends C1 commands;
  - for every camera frame it sends d, phi and the lidar points to rhc_dwa over UDP
    (127.0.0.1:5555) with the real acquisition times of the camera frame and of the scan;
  - it shows rhc_dwa's command in the video stream (:8080) and in the terminal.
If the person is lost nothing is sent: rhc_dwa stops the robot after 0.25 s.
Ctrl+C: rhc_dwa gets SIGINT and sends S1 (stop); the STM also stops by itself
after 0.35 s without commands.
"""
import argparse
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np

sys.path.insert(0, ".")

from tracker.config import TrackerConfig
from tracker.embedder import ReIDEmbedder
from tracker.track_manager import TargetTracker, State
from fusion.runtime import PersonLocator
from control.bridge import LidarToRobot, load_robot_cfg
from control.rhc_sensor_bridge import SensorBridge
from control import stream

if "--no-video" in sys.argv:          # no window, no :8080 stream, no JPEG encoding per frame
    stream.start = lambda *a, **k: None
    stream.push_frame = lambda *a, **k: None
    stream.set_ctrl = lambda *a, **k: None

RHC_DIR = os.path.expanduser("~/Documents/RHC_DWA_Encoder_2026-10-14")
MODES = {0: "SEARCH", 1: "TRACK", 2: "ALIGN", 3: "SPIN"}


class Daemon:
    """rhc_dwa as a child process; parses its CSV lines for the stream and the terminal."""

    def __init__(self, serial, port, drive, run_sh):
        argv = ["sh", run_sh, "--serial", serial, "--port", str(port)] + (["--drive"] if drive else [])
        print("[follow] start:", " ".join(argv))
        self.p = subprocess.Popen(argv, stdout=subprocess.PIPE, text=True, bufsize=1,
                                  start_new_session=True)   # our Ctrl+C does not hit it twice
        self.last = None        # (mode, cmd, actual, reason)
        self.target = (float("nan"), float("nan"), 0)
        self.prev_reason = None
        self.log = open(os.path.expanduser("~/Documents/rhc_log.csv"), "w")
        print("[follow] full rhc_dwa log: ~/Documents/rhc_log.csv")
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            line = line.strip()
            if not line:
                continue
            self.log.write("%.3f,%s\n" % (time.monotonic(), line))
            self.log.flush()
            if line.startswith("#"):
                print("\n[rhc] " + line)
                continue
            f = line.split(",")
            if len(f) < 16:
                print("\n[rhc] " + line)
                continue
            try:
                mode, hold, status = int(f[1]), int(f[2]), int(f[3])
                act = tuple(float(x) for x in f[4:7])
                cmd = tuple(float(x) for x in f[7:10])
                mask = int(f[10])
            except ValueError:
                print("\n[rhc] " + line)
                continue
            reason = f[15]
            name = "HOLD" if hold else MODES.get(mode, str(mode))
            if reason != "ok":
                name = "STOP"
            d, phi, n_obs = self.target
            stream.set_ctrl(d, phi, cmd, {"mode": name, "n_obs_used": n_obs})
            # short line (fits 80 columns); a new line every time the reason changes
            if reason != self.prev_reason:
                print()
                self.prev_reason = reason
            print(f"\r{reason[:18]:18s} {name:5s} d={d:4.2f} cmd={cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f} "
                  f"act={act[0]:+.2f},{act[1]:+.2f},{act[2]:+.2f} m={mask:2d} ",
                  end="", flush=True)
        print("\n[follow] rhc_dwa exited, code", self.p.wait())

    def alive(self):
        return self.p.poll() is None

    def stop(self):
        if self.alive():
            os.killpg(self.p.pid, signal.SIGINT)     # rhc_dwa sends S1 and closes the UART
            try:
                self.p.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGKILL)


def main():
    rc = load_robot_cfg()
    p = argparse.ArgumentParser()
    p.add_argument("--model", type=str, default=None, help="path to .hef")
    p.add_argument("--drive", action="store_true", help="motors on (default: dry run)")
    p.add_argument("--serial", default=rc.get("rhc_serial", "/dev/ttyUSB1"), help="UART of the STM")
    p.add_argument("--port", type=int, default=rc.get("rhc_udp_port", 5555))
    p.add_argument("--no-daemon", action="store_true", help="rhc_dwa is started by hand in another terminal")
    p.add_argument("--no-video", action="store_true", help="no video window and no :8080 stream (faster)")
    args = p.parse_args()

    stream.start()
    cfg = TrackerConfig()
    if args.model:
        cfg.model_path = args.model

    from tracker.detector import HailoYOLODetector
    from tracker.camera import PiCameraSource
    embedder = ReIDEmbedder(cfg)
    tracker = TargetTracker(cfg, embedder)
    detector = HailoYOLODetector(cfg)
    source = PiCameraSource(cfg)

    locator = PersonLocator(use_lidar=True).start()
    if locator.lidar is None:
        print("[follow] no lidar: no distance and no obstacles - exit")
        return
    conv = LidarToRobot(rc, locator.cfg.lidar_angle_sign)
    bridge = SensorBridge(port=args.port)
    daemon = None if args.no_daemon else Daemon(args.serial, args.port, args.drive,
                                                os.path.join(RHC_DIR, "raspberry_pi", "run.sh"))
    source.start()
    print("[follow] waiting for a person. Ctrl+C - stop.")

    last_range, last_range_t = None, -1e9
    n_sent = n_skip = 0
    ages, t_stat = [], time.monotonic()      # age of the camera frame when it is sent
    prof = {"cam": [], "det": [], "trk": [], "loc": []}   # ms per stage
    try:
        with detector:
            while daemon is None or daemon.alive():
                t0 = time.monotonic()
                frame, frame_ts = source.read_with_ts()
                if frame is None:
                    time.sleep(0.005)
                    continue
                t1 = time.monotonic()
                dets = detector.detect(frame)
                t2 = time.monotonic()
                info = tracker.step(frame, dets)
                t3 = time.monotonic()
                stream.push_frame(frame, info)
                tracking = info["state"] == State.TRACKING
                fix = locator.locate(info["bbox"] if tracking else None, frame_ts)
                t4 = time.monotonic()
                for k, v in (("cam", t1 - t0), ("det", t2 - t1), ("trk", t3 - t2), ("loc", t4 - t3)):
                    prof[k].append(v * 1000)
                if info["state"] == State.PERMANENTLY_LOST:
                    print("\n[follow] target lost for good")
                    break
                now = time.monotonic()

                # ---- person: d, phi in the robot frame (same logic as the old follow.py) ----
                if fix is None:
                    continue                                   # nothing sent -> rhc_dwa: target_stale -> STOP
                if fix.range_m is not None:                    # source == "lidar"
                    last_range, last_range_t = fix.range_m, now
                rng = last_range if now - last_range_t < rc["range_hold_s"] else None
                if rng is None:
                    continue
                d, phi, target_xy = conv.target(fix.bearing_deg, rng)

                # ---- obstacles: last scan, robot frame, person removed ----
                scan, scan_t = locator.lidar.buffer.latest()
                if scan is None or scan_t is None or now - scan_t > rc["scan_timeout_s"]:
                    n_skip += 1                                # never send "no obstacles" without a scan
                    continue
                pts = np.asarray(conv.scan(scan, target_xy), dtype=float).reshape(-1, 2)
                pts = pts[np.isfinite(pts).all(axis=1)]

                # ---- acquisition times (CLOCK_MONOTONIC of this Pi) ----
                now_ns = time.monotonic_ns()
                t_target = min(int((frame_ts if frame_ts is not None else now) * 1e9), now_ns)
                t_obst = min(int(scan_t * 1e9), now_ns)
                try:
                    bridge.send_frame(float(d), float(phi), 1.0, pts,
                                      target_time_ns=t_target, obstacle_time_ns=t_obst)
                    n_sent += 1
                    ages.append((now_ns - t_target) / 1e6)
                    if time.monotonic() - t_stat > 3.0:
                        ages.sort()
                        print(f"\n[follow] {len(ages) / (time.monotonic() - t_stat):4.1f} frames/s, "
                              f"frame age when sent: median {ages[len(ages) // 2]:.0f} ms, "
                              f"max {ages[-1]:.0f} ms (rhc_dwa limit = --max-age, stock 250)")
                        med = {k: sorted(v)[len(v) // 2] if v else 0 for k, v in prof.items()}
                        print(f"[follow] per frame, ms: camera {med['cam']:.0f}, hailo {med['det']:.0f}, "
                              f"tracker+ReID {med['trk']:.0f}, video+fusion {med['loc']:.0f}")
                        ages, t_stat = [], time.monotonic()
                        prof = {k: [] for k in prof}
                    if daemon is not None:
                        daemon.target = (d, phi, len(pts))
                except ValueError as e:                        # nan / bad time: skip this frame
                    n_skip += 1
                    if n_skip % 50 == 1:
                        print("\n[follow] frame not sent:", e)
    except KeyboardInterrupt:
        pass
    finally:
        if daemon is not None:
            daemon.stop()                                      # motors first
        try:
            locator.stop()
        except Exception:
            pass
        source.stop()
        bridge.close()
        print(f"\n[follow] stop. frames sent={n_sent} skipped={n_skip}")


if __name__ == "__main__":
    main()

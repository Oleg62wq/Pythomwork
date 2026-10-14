#!/bin/bash
# rhc_start.sh - one script for the RHC stack (rhc_dwa + STM firmware with encoders).
#
#   bash rhc_start.sh               install if needed + DRY RUN (STM gets only STOP)
#   bash rhc_start.sh --drive       install if needed + DRIVE (motors on)
#   bash rhc_start.sh --flash       also flash the STM through ST-LINK (openocd), then run
#   bash rhc_start.sh --serial /dev/ttyUSB1 --drive   force the STM port
#   bash rhc_start.sh --install     only install/check, do not run
#   bash rhc_start.sh --max-age 600 --drive   allow camera/lidar data up to 600 ms old
#                                   (stock rhc_dwa: 250 ms camera, 300 ms lidar; rebuilds rhc_dwa)
#
# Needs RHC_DWA_Encoder_2026-10-14.zip in ~/Downloads (or already unpacked in ~/Documents).
# Stop: Ctrl+C (rhc_dwa sends STOP, STM also stops itself after 0.35 s).
set -e
D=~/Documents
R=$D/RHC_DWA_Encoder_2026-10-14
DRIVE=""; FLASH=0; ONLY_INSTALL=0; SERIAL=""; MAXAGE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --drive) DRIVE="--drive" ;;
    --flash) FLASH=1 ;;
    --install) ONLY_INSTALL=1 ;;
    --serial) SERIAL="$2"; shift ;;
    --max-age) MAXAGE="$2"; shift ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done

# ---- 1. unpack the archive ----
if [ ! -f "$R/raspberry_pi/run.sh" ]; then
  ZIP=$(ls ~/Downloads/RHC_DWA_Encoder_2026-10-14*.zip 2>/dev/null | head -1)
  [ -n "$ZIP" ] || { echo "ERROR: put RHC_DWA_Encoder_2026-10-14.zip into ~/Downloads"; exit 1; }
  echo "[rhc] unpack $ZIP"
  (cd $D && unzip -q -o "$ZIP")
fi
chmod +x $R/raspberry_pi/bin/* $R/raspberry_pi/run.sh

# ---- 2. rhc_dwa binary works on this Pi? otherwise build it from source ----
case "$(uname -m)" in
  aarch64|arm64) BIN=$R/raspberry_pi/bin/rhc_dwa_aarch64 ;;
  armv7l|armv8l) BIN=$R/raspberry_pi/bin/rhc_dwa_armhf ;;
  *) BIN="" ;;
esac
if [ -z "$BIN" ] || ! "$BIN" 2>&1 | grep -q Usage; then
  echo "[rhc] ready binary does not run here -> building from source"
  command -v make >/dev/null && command -v cc >/dev/null || sudo apt-get install -y build-essential
  make -C $R/raspberry_pi build/rhc_dwa
  [ -n "$BIN" ] && cp $R/raspberry_pi/build/rhc_dwa "$BIN"
fi
echo "[rhc] rhc_dwa OK"

# ---- 2b. data age limits of rhc_dwa (stock 250 ms camera / 300 ms lidar) ----
SRC=$R/raspberry_pi
for f in src/main.c src/io.c; do [ -f $SRC/$f.orig ] || cp $SRC/$f $SRC/$f.orig; done
[ -z "$BIN" ] || [ -f "$BIN.orig" ] || cp "$BIN" "$BIN.orig"
# every run sets the limit again (no saved state: an interrupted run cannot leave a mix)
for f in src/main.c src/io.c; do cp $SRC/$f.orig $SRC/$f; done
if [ -z "$MAXAGE" ]; then
  [ -z "$BIN" ] || cp "$BIN.orig" "$BIN"
  rm -f $SRC/build/rhc_dwa
  echo "[rhc] rhc_dwa: stock age limits (250 ms camera, 300 ms lidar)"
else
  [ "$MAXAGE" -ge 300 ] 2>/dev/null || { echo "ERROR: --max-age must be a number >= 300 (ms)"; exit 1; }
  NS=$((MAXAGE * 1000000))
  sed -i "s/250000000ull/${NS}ull/g; s/300000000ull/${NS}ull/g" $SRC/src/main.c $SRC/src/io.c
  command -v make >/dev/null && command -v cc >/dev/null || sudo apt-get install -y build-essential
  rm -f $SRC/build/rhc_dwa
  make -s -C $SRC build/rhc_dwa
  [ -z "$BIN" ] || cp $SRC/build/rhc_dwa "$BIN"
  echo "[rhc] rhc_dwa rebuilt: camera and lidar data accepted up to $MAXAGE ms"
fi

# ---- 3. bridge + follow_rhc.py into the tracker project ----
mkdir -p $D/control $D/scripts
cp $R/raspberry_pi/tools/rhc_sensor_bridge.py $D/control/rhc_sensor_bridge.py
cat > $D/scripts/follow_rhc.py <<'XEOF'
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
XEOF
python3 -m py_compile $D/scripts/follow_rhc.py $D/control/rhc_sensor_bridge.py
echo "[rhc] follow_rhc.py OK"

# ---- 4. flash the STM (optional) ----
if [ $FLASH = 1 ]; then
  command -v openocd >/dev/null || sudo apt-get install -y openocd
  echo "[rhc] flashing STM through ST-LINK (USB of the Nucleo must be plugged into the Pi)"
  sh $R/firmware/flash_openocd.sh
  sleep 1
fi
[ $ONLY_INSTALL = 1 ] && { echo "[rhc] INSTALL_OK"; exit 0; }

# ---- 5. find the STM port (USB-UART 1a86, not the lidar CP2102) ----
if [ -z "$SERIAL" ]; then
  SERIAL=$(ls /dev/serial/by-id/*1a86* 2>/dev/null | head -1)
  [ -n "$SERIAL" ] || SERIAL=/dev/ttyUSB1
fi
[ -e "$SERIAL" ] || { echo "ERROR: STM port $SERIAL not found (ls -l /dev/serial/by-id/)"; exit 1; }
echo "[rhc] STM port: $SERIAL"

# ---- 6. free the port and the lidar ----
pkill -f "bin/rhc_dwa_|build/rhc_dwa" 2>/dev/null || true
pkill -f "python3 -m scripts[.]follow" 2>/dev/null || true
pkill -f miniterm 2>/dev/null || true
pkill -f serial_monitor.py 2>/dev/null || true
sleep 0.3
# ---- 6b. what answers on the STM port? ----
python3 - "$SERIAL" <<'XEOF'
import sys, time, serial
s = serial.Serial(sys.argv[1], 115200, timeout=0.2)
time.sleep(0.2); s.reset_input_buffer()
data = b""; t0 = time.time()
while time.time() - t0 < 1.5:
    data += s.read(512)
if b"O1|" in data:
    print("[rhc] STM: NEW firmware, telemetry O1 OK (%d lines in 1.5 s)" % data.count(b"O1|"))
else:
    s.write(b"PING\n"); time.sleep(0.3); data += s.read(512)
    if b"Rot/min" in data or b"PONG" in data:
        print("[rhc] STM: OLD firmware on this port -> flash: bash rhc_start.sh --flash")
    elif not data:
        print("[rhc] STM: port is SILENT -> wrong port, STM not powered, or press RESET on the Nucleo")
    else:
        print("[rhc] STM: unknown answer:", data[:120])
s.close()
XEOF
cd $D
[ -f lidar_reset.py ] && python3 lidar_reset.py || true

# ---- 7. run ----
if [ -n "$DRIVE" ]; then echo "[rhc] DRIVE: motors ON. Ctrl+C = stop"; else echo "[rhc] DRY RUN: motors stay stopped. Add --drive to move"; fi
exec python3 -m scripts.follow_rhc --serial "$SERIAL" $DRIVE

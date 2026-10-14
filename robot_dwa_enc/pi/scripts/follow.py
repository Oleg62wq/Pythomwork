#!/usr/bin/env python3
"""Follow a person: tracker + calibration (fusion) + DWA (static target / HOLD).

    cd ~/Documents
    python3 -m scripts.follow            # dry run: prints angle, distance, vx vy w
    python3 -m scripts.follow --drive    # + sends M3 to the STM

Two threads:
  main     - camera -> detector -> tracker -> fusion -> last PersonFix
  control  - exactly every 0.09 s: last fix + last scan + MEASURED velocity -> DWA -> [vx, vy, w]

u_prev of the DWA = body velocity measured by the wheel encoders (STM line E|...).
Without fresh encoder data (old firmware, dry run) the previous command is used.
robot.json: "u_prev_from_encoders": false switches back to the old behaviour.
"""
import argparse
import sys
import threading
import time

sys.path.insert(0, ".")

from tracker.config import TrackerConfig
from tracker.embedder import ReIDEmbedder
from tracker.track_manager import TargetTracker, State
from fusion.runtime import PersonLocator
from control.bridge import LidarToRobot, load_robot_cfg
from control.dwa_py import Dwa

TS = 0.09  # = Ts in control/dwa_v4_static_stop.c


class Shared:
    """Last fix from the tracker, shared by the two threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.fix = None          # PersonFix | None
        self.fix_wall = 0.0      # time.monotonic() when received
        self.running = True


def control_loop(sh: Shared, locator: PersonLocator, rc: dict):
    dwa = Dwa()
    conv = LidarToRobot(rc, locator.cfg.lidar_angle_sign)
    use_enc = rc.get("u_prev_from_encoders", True)
    enc_age = rc.get("enc_max_age_s", 0.20)
    last_range, last_range_t = None, -1e9
    t_next = time.monotonic()
    n = 0
    while sh.running:
        now = time.monotonic()
        with sh.lock:
            fix, fix_t = sh.fix, sh.fix_wall

        # ---- angle and distance to the person -> d, phi in the robot frame ----
        d = phi = float("nan")
        conf = 0.0
        target_xy = None
        if fix is not None and now - fix_t < rc["fix_timeout_s"]:
            if fix.range_m is not None:                      # source == "lidar"
                last_range, last_range_t = fix.range_m, now
            rng = last_range if now - last_range_t < rc["range_hold_s"] else None
            if rng is not None:                              # DWA needs a distance
                d, phi, target_xy = conv.target(fix.bearing_deg, rng)
                conf = 1.0

        # ---- obstacles: last scan ----
        scan, scan_t = (locator.lidar.buffer.latest()
                        if locator.lidar is not None else (None, None))
        if scan_t is None or now - scan_t > rc["scan_timeout_s"]:
            scan = None
        pts = conv.scan(scan, target_xy)

        # ---- u_prev: real velocity from the encoders (None -> previous command) ----
        u_meas = NUCLEO.velocity(enc_age) if use_enc else None
        u, info = dwa.step(d, phi, conf, pts, u_prev=u_meas)
        NUCLEO.send(*u)
        stream.set_ctrl(d, phi, u, info)

        n += 1
        if n % 5 == 0:
            src = fix.source if fix is not None else "-"
            up = info["u_prev"]
            print(f"\r[CTRL] {info['mode']:6s} d={d:5.2f} phi={phi:+5.2f} ({src:11s}) "
                  f"-> vx={u[0]:+.2f} vy={u[1]:+.2f} w={u[2]:+.2f}  "
                  f"u0[{info['u_prev_src']}]={up[0]:+.2f},{up[1]:+.2f},{up[2]:+.2f}  "
                  f"obs={info['n_obs_used']:3d}   ",
                  end="", flush=True)

        t_next += TS
        sl = t_next - time.monotonic()
        if sl > 0:
            time.sleep(sl)
        else:
            print(f"\n[CTRL] loop late by {-sl*1000:.0f} ms")
            t_next = time.monotonic()


from control.nucleo import Nucleo
from control import stream
NUCLEO = None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", type=str, default=None, help="path to .hef")
    p.add_argument("--drive", action="store_true")
    args = p.parse_args()
    global NUCLEO
    NUCLEO = Nucleo(args.drive)
    stream.start()

    cfg = TrackerConfig()
    if args.model:
        cfg.model_path = args.model
    rc = load_robot_cfg()

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
    source.start()
    print("[follow] waiting for a person. Ctrl+C or q - stop.")

    sh = Shared()
    ctrl = threading.Thread(target=control_loop, args=(sh, locator, rc), daemon=True)
    ctrl.start()
    try:
        with detector:
            while True:
                frame, frame_ts = source.read_with_ts()
                if frame is None:
                    time.sleep(0.005); continue
                info = tracker.step(frame, detector.detect(frame))
                stream.push_frame(frame, info)
                tracking = info["state"] == State.TRACKING
                fix = locator.locate(info["bbox"] if tracking else None, frame_ts)
                with sh.lock:
                    sh.fix, sh.fix_wall = fix, time.monotonic()
                if info["state"] == State.PERMANENTLY_LOST:
                    print("\n[follow] target lost for good"); break
    except KeyboardInterrupt:
        pass
    finally:
        sh.running = False
        ctrl.join(timeout=1.0)
        NUCLEO.stop()            # motors first
        try:
            locator.stop()
        except Exception:
            pass
        source.stop()
        print("\n[follow] stop.")


if __name__ == "__main__":
    main()

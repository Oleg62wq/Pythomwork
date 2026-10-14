#!/usr/bin/env python3
"""Check the encoder block BEFORE driving with the DWA.

ROBOT ON THE STAND, wheels in the air (the Hall sensors see the wheels the same way).
    cd ~/Documents && python3 -m scripts.enc_check
Press q to stop at any moment.

For every step it sends a constant M3 command for 2 s and prints the mean of
the measured velocity (E|... lines) over the last second. Expected:
  - same sign as the command on every axis;
  - measured / command close to 1 when mask = 15 (all 4 wheels seen by Hall);
  - mask < 15: some wheel is too slow for the Hall sensor, its setpoint is used.
"""
import sys
import time

sys.path.insert(0, ".")
from control.nucleo import Nucleo

STEPS = [("forward 0.2", (0.20, 0.0, 0.0)),
         ("stop", (0.0, 0.0, 0.0)),
         ("back 0.2", (-0.20, 0.0, 0.0)),
         ("stop", (0.0, 0.0, 0.0)),
         ("left 0.15", (0.0, 0.15, 0.0)),
         ("stop", (0.0, 0.0, 0.0)),
         ("turn left 0.8", (0.0, 0.0, 0.8)),
         ("stop", (0.0, 0.0, 0.0)),
         ("turn right 0.8", (0.0, 0.0, -0.8)),
         ("stop", (0.0, 0.0, 0.0)),
         ("forward 0.4", (0.40, 0.0, 0.0)),
         ("stop", (0.0, 0.0, 0.0))]
TS = 0.09


def main():
    nuc = Nucleo(True)
    if nuc.n_enc == 0:
        print("\n[enc_check] STM sends no E|... lines -> flash the firmware with WheelOdometry first")
    print("\n%-16s %-22s %-22s %-5s %s" % ("step", "command vx vy w", "measured vx vy w", "mask", "age"))
    try:
        for name, u in STEPS:
            acc, n_mask = [], {}
            t0 = time.monotonic()
            while time.monotonic() - t0 < 2.0:
                nuc.send(*u)
                time.sleep(TS)
                v = nuc.velocity()
                if v is not None and time.monotonic() - t0 > 1.0:
                    acc.append(v)
                    age, mask = nuc.velocity_info()
                    n_mask[mask] = n_mask.get(mask, 0) + 1
            if acc:
                m = [sum(a[i] for a in acc) / len(acc) for i in range(3)]
                mask = max(n_mask, key=n_mask.get)
                age, _ = nuc.velocity_info()
                print("%-16s %+6.2f %+6.2f %+6.2f   %+6.2f %+6.2f %+6.2f   %-5d %.0f ms"
                      % (name, *u, *m, mask, age * 1000))
            else:
                print("%-16s %+6.2f %+6.2f %+6.2f   no encoder data" % (name, *u))
    except KeyboardInterrupt:
        pass
    finally:
        nuc.stop()


if __name__ == "__main__":
    main()

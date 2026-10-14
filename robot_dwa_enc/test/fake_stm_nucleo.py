#!/usr/bin/env python3
"""nucleo.py against a fake STM on a pseudo-terminal: PING/ENC/M3/M4/STOP + E|... stream."""
import os, pty, sys, threading, time, tty
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))
import control.nucleo as N

N.LOG_FILE = ""
master, slave = pty.openpty()
tty.setraw(slave)
rx = []
state = {"enc": 0, "v": (0, 0, 0)}

def fake_stm():
    buf = b""; last = time.monotonic()
    os.set_blocking(master, False)
    while True:
        try:
            buf += os.read(master, 256)
        except BlockingIOError:
            pass
        except OSError:
            return
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1); line = line.strip()
            if not line: continue
            rx.append(line.decode())
            if line == b"PING": os.write(master, b"PONG\r\n")
            elif line.startswith(b"ENC|"):
                state["enc"] = int(line[4:]); os.write(master, b"ENC: %d\r\n" % state["enc"])
            elif line.startswith(b"M3|"):
                vx, vy, w = map(int, line[3:].split(b"|")); state["v"] = (vx, vy, w)
                os.write(master, b"M3: %d|%d|%d\r\n" % (vx, vy, w))
            elif line.startswith(b"M4|"): state["v"] = (0, 0, 0)
        if state["enc"] and time.monotonic() - last > state["enc"] / 1000:
            last = time.monotonic(); vx, vy, w = state["v"]
            os.write(master, b"E|%d|%d|%d|15|123\r\n" % (int(vx * 0.9), int(vy * 0.9), int(w * 0.9)))
        time.sleep(0.002)

threading.Thread(target=fake_stm, daemon=True).start()
N.SIGN_VX = -1   # check the frame conversion too
nuc = N.Nucleo(True, port=os.ttyname(slave))
print("E lines after start:", nuc.n_enc > 0)
nuc.send(0.30, 0.10, -0.50)
time.sleep(0.15)
v = nuc.velocity(); age, mask = nuc.velocity_info()
print("sent (0.30, 0.10, -0.50) -> measured", [round(x, 3) for x in v], "mask", mask, "age %.0f ms" % (age * 1000))
ok = abs(v[0] - 0.27) < 1e-9 and abs(v[1] - 0.09) < 1e-9 and abs(v[2] + 0.45) < 1e-9
nuc.stop()
time.sleep(0.1)
print("commands seen by STM:", rx[:3], "...", rx[-3:])
print("velocity() after stop (stale):", nuc.velocity(0.05))
ok = ok and rx[-1] == "ENC|0" and "M3|-300|100|-500" in rx
print("ALL OK" if ok else "FAIL"); sys.exit(0 if ok else 1)

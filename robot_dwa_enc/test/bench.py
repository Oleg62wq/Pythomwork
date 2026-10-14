#!/usr/bin/env python3
"""Time of one DWA step: serial vs OpenMP (and with the lidar prefilter)."""
import math, os, subprocess, sys, tempfile, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); CTRL = os.path.join(HERE, "..", "pi", "control")
sys.path.insert(0, CTRL)
from dwa_py import Dwa

tmp = tempfile.mkdtemp()
libs = {"serial": [], "omp": ["-fopenmp"], "omp+prefilter": ["-fopenmp", "-DDWA4_PREFILTER=1"]}
ang = np.linspace(-math.pi, math.pi, 360, endpoint=False)
scan = np.c_[1.25 / np.maximum(np.abs(np.cos(ang)), np.abs(np.sin(ang))) * np.cos(ang),
             1.25 / np.maximum(np.abs(np.cos(ang)), np.abs(np.sin(ang))) * np.sin(ang)]   # 2.5 x 2.5 room
for name, fl in libs.items():
    so = os.path.join(tmp, name + ".so")
    subprocess.run(["gcc", "-O3", "-march=native", "-std=c11", "-fPIC", "-shared", *fl, "-o", so,
                    os.path.join(CTRL, "dwa_v4_static_stop.c"), "-lm"], check=True, stderr=subprocess.DEVNULL)
    d = Dwa(so)
    u0 = [0.3, 0.0, 0.2]
    d.step(1.2, 0.15, 1.0, scan, u_prev=u0)
    ts = []
    for _ in range(30):
        t = time.perf_counter(); u, info = d.step(1.2, 0.15, 1.0, scan, u_prev=u0); ts.append(time.perf_counter() - t)
    print("%-14s %6.2f ms/step  (mode %s, obs used %d, u=%s)" % (name, 1000 * np.median(ts), info["mode"], info["n_obs_used"], np.round(u, 3)))
print("threads:", os.environ.get("OMP_NUM_THREADS", "all %d cores" % os.cpu_count()))

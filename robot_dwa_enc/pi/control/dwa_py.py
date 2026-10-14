"""
dwa_py.py - Python wrapper (ctypes) of libdwa4.so
(C port of DWA_HumanFollower_v3_StaticTarget_Stop.m).

Build the library on the Pi (in ~/Documents/control):
    gcc -O3 -mcpu=native -std=c11 -fopenmp -fPIC -shared -o libdwa4.so dwa_v4_static_stop.c -lm

Usage:
    from control.dwa_py import Dwa
    dwa = Dwa()                                            # once
    u, info = dwa.step(d, phi, conf, scan_xy, u_prev=v)    # every Ts = 0.09 s

u_prev = CURRENT body velocity [vx, vy, w] from the wheel encoders
(Nucleo.velocity()). If it is None the previous command is used instead
(old behaviour, for runs without the STM).

scan_xy - numpy array (N, 2) of lidar points in the ROBOT frame
(x forward, y left, m), NaN = no return. d, phi - person in the robot frame,
phi > 0 = left.
"""
import ctypes as C
import os
import numpy as np


class _State(C.Structure):
    _fields_ = [("initialized", C.c_int),
                ("x_est", C.c_double), ("y_est", C.c_double),
                ("lost_t", C.c_double),
                ("scan_sign", C.c_double),
                ("stop_active", C.c_int)]


class _Out(C.Structure):
    _fields_ = [("u_cmd", C.c_double * 3),
                ("mode_id", C.c_int),
                ("d_est", C.c_double), ("phi_est", C.c_double),
                ("best_idx", C.c_int),
                ("n_candidates", C.c_int),
                ("n_obs_used", C.c_int),
                ("stop_active", C.c_int)]


MODES = {0: "SEARCH", 1: "TRACK", 2: "ALIGN", 3: "SPIN"}


class Dwa:
    def __init__(self, lib_path=None):
        lib_path = lib_path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "libdwa4.so")
        self._lib = C.CDLL(lib_path)
        f = self._lib.dwa4_step
        f.restype = None
        f.argtypes = [C.POINTER(_State), C.POINTER(C.c_double),
                      C.c_double, C.c_double, C.c_double,
                      C.c_void_p, C.c_int,
                      C.POINTER(C.c_double), C.POINTER(_Out)]
        self._f = f
        self._lib.dwa4_init.restype = None
        self._lib.dwa4_init.argtypes = [C.POINTER(_State)]
        self.reset()

    def reset(self):
        """Clear the planner memory (MATLAB: clear the persistent variables)."""
        self._s = _State()
        self._lib.dwa4_init(C.byref(self._s))
        self._out = _Out()
        self._pose = (C.c_double * 3)()
        self.u_prev = np.zeros(3)     # last command

    def step(self, d, phi, confidence, scan_xy=None, u_prev=None):
        """One step. u_prev: measured body velocity (encoders); None = last command."""
        src = "enc"
        if u_prev is None:
            u_prev, src = self.u_prev, "cmd"
        up = (C.c_double * 3)(*map(float, u_prev))

        if scan_xy is None or len(scan_xy) == 0:
            ptr, n = None, 0
        else:
            pts = np.ascontiguousarray(scan_xy, dtype=np.float64)
            assert pts.ndim == 2 and pts.shape[1] == 2, "scan_xy must be (N, 2)"
            ptr, n = pts.ctypes.data, pts.shape[0]   # dwa4_point_t = {double x, y}

        d = float("nan") if d is None else float(d)
        phi = float("nan") if phi is None else float(phi)
        self._f(C.byref(self._s), self._pose, d, phi, float(confidence),
                ptr, n, up, C.byref(self._out))

        o = self._out
        u = np.array(o.u_cmd[:])
        self.u_prev = u
        mode = "HOLD" if o.stop_active else MODES.get(o.mode_id, o.mode_id)
        info = {"mode": mode, "mode_id": o.mode_id, "hold": bool(o.stop_active),
                "d_est": o.d_est, "phi_est": o.phi_est, "best_idx": o.best_idx,
                "n_obs_used": o.n_obs_used,
                "u_prev": np.array(up[:]), "u_prev_src": src}
        return u, info

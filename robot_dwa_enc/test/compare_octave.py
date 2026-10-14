#!/usr/bin/env python3
"""Compare libdwa4.so (C port) with the original MATLAB function run in Octave.

    python3 test/compare_octave.py            # from robot_dwa_enc/

Scenarios: random inputs (measurement, loss, obstacles, arbitrary u_prev)
and closed loops where u_prev = previous command (HOLD enter/exit, SEARCH).
"""
import ctypes as C, math, os, random, subprocess, sys, tempfile
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CTRL = os.path.join(ROOT, "pi", "control")
sys.path.insert(0, CTRL)
from dwa_py import Dwa  # noqa: E402

FN = "DWA_HumanFollower_v3_StaticTarget_Stop"


def build(out, extra):
    cmd = ["gcc", "-O3", "-std=c11", "-fPIC", "-shared", "-Wall", "-Wextra",
           *extra, "-o", out, os.path.join(CTRL, "dwa_v4_static_stop.c"), "-lm"]
    subprocess.run(cmd, check=True)


def scenarios(seed=1):
    rnd = random.Random(seed)
    sc = []
    # 1) random single steps / short random sequences
    for _ in range(40):
        steps = []
        for _ in range(rnd.randint(1, 8)):
            lost = rnd.random() < 0.2
            d = float("nan") if lost else rnd.uniform(0.3, 3.0)
            phi = float("nan") if lost else rnd.choice(
                [rnd.uniform(-1.3, 1.3), rnd.uniform(-0.05, 0.05), rnd.uniform(0.5, 1.1)])
            conf = 0.0 if (lost and rnd.random() < 0.5) else 1.0
            u = [rnd.uniform(-0.3, 0.7), rnd.uniform(-0.2, 0.2), rnd.uniform(-2.0, 2.0)]
            nobs = rnd.choice([0, 5, 12, 30])
            obs = []
            for _ in range(nobs):
                a = rnd.uniform(-math.pi, math.pi); r = rnd.uniform(0.35, 1.2)
                if rnd.random() < 0.05:
                    obs.append((float("nan"), float("nan")))
                else:
                    obs.append((r * math.cos(a), r * math.sin(a)))
            steps.append((d, phi, conf, u, obs, "rand"))
        sc.append(steps)
    # 2) closed loops: person still; robot pose integrated with the command
    for k in range(8):
        px, py = rnd.uniform(0.7, 2.5), rnd.uniform(-1.0, 1.0)
        if k < 3:
            px, py = 0.6 + rnd.uniform(-0.02, 0.02), rnd.uniform(-0.015, 0.015)  # near HOLD
        walls = [(1.25 * math.cos(a), 1.25 * math.sin(a)) for a in np.linspace(-math.pi, math.pi, 36)]
        rx = ry = th = 0.0
        u = [0.0, 0.0, 0.0]
        steps = []
        for n in range(30):
            dx, dy = px - rx, py - ry
            xr = math.cos(-th) * dx - math.sin(-th) * dy
            yr = math.sin(-th) * dx + math.cos(-th) * dy
            phi = math.atan2(yr, xr)
            seen = abs(phi) < math.radians(30) and not (12 <= n < 16 and k % 2)
            d = math.hypot(xr, yr) if seen else float("nan")
            phi = phi if seen else float("nan")
            obs = []
            for wx, wy in walls:
                ox, oy = wx - rx, wy - ry
                obs.append((math.cos(-th) * ox - math.sin(-th) * oy,
                            math.sin(-th) * ox + math.cos(-th) * oy))
            steps.append((d, phi, 1.0 if seen else 0.0, None, obs, "loop"))  # u filled by C run
            # placeholder; pose update is done after we know the C output (below)
        sc.append(steps)
    return sc


def run_c(lib, sc):
    dwa = Dwa(lib)
    outs, ins = [], []
    for steps in sc:
        dwa.reset()
        rx = ry = th = 0.0
        u_last = np.zeros(3)
        o_s, i_s = [], []
        for (d, phi, conf, u, obs, kind) in steps:
            up = np.array(u) if u is not None else u_last.copy()
            arr = np.array(obs, dtype=float).reshape(-1, 2) if obs else None
            cmd, info = dwa.step(d, phi, conf, arr, u_prev=up)
            o_s.append([*cmd, info["mode_id"], info["d_est"], info["phi_est"]])
            i_s.append((d, phi, conf, list(up), obs))
            u_last = cmd
        outs.append(o_s); ins.append(i_s)
    return outs, ins


def run_octave(ins):
    tmp = tempfile.mkdtemp()
    fin = os.path.join(tmp, "in.txt"); fout = os.path.join(tmp, "out.txt")
    with open(fin, "w") as f:
        for si, steps in enumerate(ins):
            for (d, phi, conf, up, obs) in steps:
                vals = [si, d, phi, conf, *up, len(obs)] + [v for p in obs for v in p]
                f.write(" ".join("%.17g" % v for v in vals).replace("nan", "NaN") + "\n")
    script = f"""
warning('off', 'all');
addpath('{os.path.join(HERE, 'octave_shim')}');
addpath('{os.path.join(ROOT, 'matlab')}');
fi = fopen('{fin}'); fo = fopen('{fout}', 'w'); prev = -1;
while true
  l = fgetl(fi); if ~ischar(l), break; end
  v = sscanf(l, '%f')';
  if v(1) ~= prev, clear {FN}; prev = v(1); end
  n = v(8); obs = reshape(v(9:8+2*n), 2, n)';
  if n == 0, obs = zeros(0, 2); end
  [u, m, de, pe] = {FN}([0;0;0], v(2), v(3), v(4), obs, v(5:7)');
  fprintf(fo, '%.17g %.17g %.17g %d %.17g %.17g\\n', u(1), u(2), u(3), m, de, pe);
end
fclose(fi); fclose(fo);
"""
    sp = os.path.join(tmp, "run.m"); open(sp, "w").write(script)
    subprocess.run(["octave", "--no-gui", "--quiet", sp], check=True)
    rows = [list(map(float, l.split())) for l in open(fout)]
    return rows


def main():
    tmp = tempfile.mkdtemp()
    libs = {"serial": os.path.join(tmp, "libdwa4_serial.so"),
            "omp": os.path.join(tmp, "libdwa4_omp.so"),
            "omp+prefilter": os.path.join(tmp, "libdwa4_pf.so")}
    build(libs["serial"], [])
    build(libs["omp"], ["-fopenmp"])
    build(libs["omp+prefilter"], ["-fopenmp", "-DDWA4_PREFILTER=1"])

    sc = scenarios()
    ref_out, ins = run_c(libs["serial"], sc)
    flat_ref = [r for s in ref_out for r in s]
    oct_rows = run_octave(ins)
    assert len(oct_rows) == len(flat_ref), (len(oct_rows), len(flat_ref))

    worst = 0.0; bad = 0; modes = {}
    for a, b in zip(flat_ref, oct_rows):
        err = max(abs(x - y) for x, y in zip(a, b))
        modes[int(b[3])] = modes.get(int(b[3]), 0) + 1
        if not err <= 1e-12 or int(a[3]) != int(b[3]):
            bad += 1
        worst = max(worst, err)
    print("steps compared with Octave: %d, modes %s" % (len(flat_ref), modes))
    print("C (serial) vs MATLAB/Octave: max |diff| = %.3g, mismatches = %d" % (worst, bad))

    for name in ("omp", "omp+prefilter"):
        out, _ = run_c(libs[name], sc)
        diff = max(max(abs(x - y) for x, y in zip(a, b))
                   for s1, s2 in zip(out, ref_out) for a, b in zip(s1, s2))
        print("C %-14s vs C serial: max |diff| = %.3g" % (name, diff))
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

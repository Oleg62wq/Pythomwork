#!/usr/bin/env python3
"""Build pi/install_dwa4_enc.txt (one bash file, ASCII only) from the files in pi/."""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI = os.path.join(ROOT, "pi")

FILES = [("control/dwa4.h", "$C/dwa4.h"),
         ("control/dwa_v4_static_stop.c", "$C/dwa_v4_static_stop.c"),
         ("control/dwa_py.py", "$C/dwa_py.py"),
         ("control/nucleo.py", "$C/nucleo_enc_new.py"),
         ("scripts/enc_check.py", "$S/enc_check.py")]

PATCH = r'''import os, re, sys
C = os.path.expanduser("~/Documents/control")
S = os.path.expanduser("~/Documents/scripts")

# 1) nucleo.py: new file, but keep YOUR values of these constants from the old one
new = open(C + "/nucleo_enc_new.py").read()
old_path = C + "/nucleo.py.bak_v3"
old = open(old_path).read() if os.path.exists(old_path) else ""
for name in ("PORT", "V_MAX", "W_MAX", "LOG_TX", "LOG_RX", "LOG_FILE", "SIGN_VX", "SIGN_VY", "SIGN_W"):
    m = re.search(r"^%s = (.*)$" % name, old, re.M)
    if m:
        new = re.sub(r"^%s = .*$" % name, lambda _: "%s = %s" % (name, m.group(1)), new, count=1, flags=re.M)
        print("nucleo.py: kept  %s = %s" % (name, m.group(1)))
open(C + "/nucleo.py", "w").write(new)
os.remove(C + "/nucleo_enc_new.py")

# 2) follow.py: u_prev of the DWA = velocity from the encoders
f = S + "/follow.py"
s = open(f).read()
if "u_prev=u_meas" in s:
    print("follow.py: already patched")
else:
    m = re.search(r"^([ \t]*)u, info = dwa\.step\(d, phi, conf, pts\)[ \t]*$", s, re.M)
    if not m:
        sys.exit("follow.py: line 'u, info = dwa.step(d, phi, conf, pts)' NOT FOUND - nothing changed")
    ind = m.group(1)
    s = (s[:m.start()]
         + ind + 'u_meas = NUCLEO.velocity(rc.get("enc_max_age_s", 0.20)) if rc.get("u_prev_from_encoders", True) else None\n'
         + ind + "u, info = dwa.step(d, phi, conf, pts, u_prev=u_meas)"
         + s[m.end():])
    s = s.replace("obs={info['n_obs_used']:3d}", "u0={info['u_prev_src']} obs={info['n_obs_used']:3d}", 1)
    open(f, "w").write(s)
    print("follow.py: patched (u_prev from encoders)")
'''

TEST = r'''import sys
sys.path.insert(0, ".")
from control.dwa_py import Dwa
d = Dwa()
u, i = d.step(1.5, 0.07, 1.0, None, u_prev=[0.0, 0.0, 0.0])
print("DWA4 test: mode=%s vx=%+.3f vy=%+.3f w=%+.3f u0=%s" % (i["mode"], u[0], u[1], u[2], i["u_prev_src"]))
'''


def main():
    out = ["# install_dwa4_enc.txt - new DWA (static target + HOLD) + u_prev from encoders",
           "# On the Pi:   bash ~/Downloads/install_dwa4_enc.txt",
           "# Backups (*.bak_v3) are made once; rollback: see the end of this file.",
           "set -e",
           "C=~/Documents/control",
           "S=~/Documents/scripts",
           "for f in $C/dwa_py.py $C/nucleo.py $S/follow.py; do",
           "  if [ -f \"$f\" ] && [ ! -f \"$f.bak_v3\" ]; then cp \"$f\" \"$f.bak_v3\"; echo \"backup: $f.bak_v3\"; fi",
           "done",
           ""]
    for src, dst in FILES:
        body = open(os.path.join(PI, src)).read()
        assert "XEOF" not in body
        out += ["cat > %s <<'XEOF'" % dst, body.rstrip("\n"), "XEOF", ""]
    out += ["cat > /tmp/dwa4_patch.py <<'XEOF'", PATCH.rstrip("\n"), "XEOF",
            "python3 /tmp/dwa4_patch.py",
            "",
            "cd $C",
            "gcc -O3 -mcpu=native -std=c11 -fopenmp -fPIC -shared -o libdwa4.so dwa_v4_static_stop.c -lm",
            "echo BUILD_OK",
            "python3 -m py_compile $C/dwa_py.py $C/nucleo.py $S/follow.py $S/enc_check.py",
            "cd ~/Documents",
            "cat > /tmp/dwa4_test.py <<'XEOF'", TEST.rstrip("\n"), "XEOF",
            "OMP_NUM_THREADS=2 python3 /tmp/dwa4_test.py",
            "echo INSTALL_OK",
            "",
            "# Rollback to the old DWA (v2/v3) and old nucleo/follow:",
            "#   cd ~/Documents/control && cp dwa_py.py.bak_v3 dwa_py.py && cp nucleo.py.bak_v3 nucleo.py \\",
            "#     && cp ../scripts/follow.py.bak_v3 ../scripts/follow.py",
            ""]
    text = "\n".join(out)
    bad = [(n, l) for n, l in enumerate(text.splitlines(), 1) if any(ord(ch) > 127 for ch in l)]
    assert not bad, bad[:3]
    open(os.path.join(PI, "install_dwa4_enc.txt"), "w").write(text)
    print("written pi/install_dwa4_enc.txt, %d lines" % text.count("\n"))


if __name__ == "__main__":
    main()

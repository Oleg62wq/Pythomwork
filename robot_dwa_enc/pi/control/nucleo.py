# Send (vx, vy, w) to STM: M3|vx mm/s|vy mm/s|w mrad/s
# Read the measured body velocity back from STM: E|vx mm/s|vy mm/s|w mrad/s|mask|t
# Exit: press q (or Ctrl+C) -> motors get zero speed, then program stops.
# Full log of PI->STM and STM->PI: terminal + ~/Documents/stm_log.txt
import math, os, sys, time, signal, threading

V_MAX = 0.6   # m/s
W_MAX = 1.5   # rad/s
PORT = "/dev/ttyUSB1"
LOG_TX = True     # print what goes to STM
LOG_RX = True     # print what comes from STM
LOG_ENC = False   # print every E|... line (50 per second!)
LOG_FILE = os.path.expanduser("~/Documents/stm_log.txt")   # "" = no file
SIGN_VX = 1       # -1 if the STM "forward" is opposite to the camera
SIGN_VY = 1
SIGN_W = 1
ENC_PERIOD_MS = 20      # STM sends E|... every 20 ms (0 = off)
ENC_MAX_AGE_S = 0.20    # older encoder data -> velocity() returns None


def _clip(x, lim):
    return max(-lim, min(lim, x))


class Nucleo:
    def __init__(self, enabled, port=PORT, baud=115200):
        self.ser = None
        self._estop = False
        self._interrupted = False
        self._tty = None
        self._lock = threading.RLock()
        self._vel = None          # (t_monotonic, vx, vy, w, mask) in the DWA frame
        self.n_enc = 0            # E|... lines received
        self._logf = open(LOG_FILE, "a") if (enabled and LOG_FILE) else None
        if enabled:
            import serial
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self._log("INFO", "port %s opened" % port)
            threading.Thread(target=self._reader, daemon=True).start()
            self._write("\nPING\n")
            time.sleep(0.3)
            self._write("ENC|%d\n" % ENC_PERIOD_MS)
            time.sleep(0.15)
            if ENC_PERIOD_MS and self.n_enc == 0:
                self._log("INFO", "no E|... from STM: firmware without encoder block? "
                                  "DWA will use the previous command as u_prev")
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, self._on_signal)
        self._start_keys()

    # ---- log ----
    def _log(self, tag, text):
        t = time.time()
        line = "%s.%03d %-8s %s" % (time.strftime("%H:%M:%S", time.localtime(t)),
                                    int(t * 1000) % 1000, tag, text)
        sys.stdout.write("\r\033[K" + line + "\n")
        sys.stdout.flush()
        if self._logf:
            try:
                self._logf.write(line + "\n")
                self._logf.flush()
            except Exception:
                pass

    def _write(self, cmd):
        self.ser.write(cmd.encode())
        if LOG_TX:
            self._log("PI->STM", cmd.strip().replace("\n", " | "))

    def _on_enc(self, line):
        # E|vx|vy|w|mask|t  (mm/s, mm/s, mrad/s) in the STM frame
        try:
            p = line.split(b"|")
            vx, vy, w = int(p[1]) / 1000.0, int(p[2]) / 1000.0, int(p[3]) / 1000.0
            mask = int(p[4]) if len(p) > 4 else 15
        except (ValueError, IndexError):
            return False
        # STM frame -> DWA frame (inverse of what send() does)
        self._vel = (time.monotonic(), vx / SIGN_VX, vy / SIGN_VY, w / SIGN_W, mask)
        self.n_enc += 1
        return True

    def _reader(self):
        buf = b""
        while True:
            ser = self.ser
            if ser is None:
                return
            try:
                d = ser.read(256)
            except Exception:
                return
            if not d:
                continue
            buf += d
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip(b"\r ")
                if line.startswith(b"E|") and self._on_enc(line) and not LOG_ENC:
                    continue
                if line and LOG_RX:
                    self._log("STM->PI", line.decode(errors="replace"))

    # ---- measured velocity for the DWA ----
    def velocity(self, max_age=ENC_MAX_AGE_S):
        """[vx, vy, w] measured by the wheel encoders (m/s, m/s, rad/s, DWA frame),
        or None if there is no fresh E|... line (old firmware, dry run, link lost)."""
        v = self._vel
        if v is None or time.monotonic() - v[0] > max_age:
            return None
        return [v[1], v[2], v[3]]

    def velocity_info(self):
        """(age_s, mask) of the last E|... line; mask bit i = wheel i seen by its Hall sensor."""
        v = self._vel
        if v is None:
            return float("nan"), 0
        return time.monotonic() - v[0], v[4]

    # ---- keyboard: q = stop ----
    def _start_keys(self):
        if not sys.stdin.isatty():
            return
        import termios, tty
        fd = sys.stdin.fileno()
        self._tty = (fd, termios.tcgetattr(fd))
        tty.setcbreak(fd)
        print("[nucleo] press q to stop motors and exit")

        def loop():
            while not self._estop:
                ch = os.read(fd, 1)
                if ch in (b"q", b"Q"):
                    print("\n[nucleo] q pressed -> STOP")
                    self.stop()
                    os.kill(os.getpid(), signal.SIGINT)
                    return
        threading.Thread(target=loop, daemon=True).start()

    def _restore_tty(self):
        if self._tty is not None:
            import termios
            fd, old = self._tty
            self._tty = None
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _on_signal(self, *a):
        self.stop()
        if not self._interrupted:
            self._interrupted = True
            raise KeyboardInterrupt

    # ---- motors ----
    def send(self, vx, vy, w):
        if self.ser is None or self._estop:
            return
        if not all(math.isfinite(v) for v in (vx, vy, w)):
            vx = vy = w = 0.0
        cmd = "M3|%d|%d|%d\n" % (round(_clip(vx * SIGN_VX, V_MAX) * 1000),
                                 round(_clip(vy * SIGN_VY, V_MAX) * 1000),
                                 round(_clip(w * SIGN_W, W_MAX) * 1000))
        if cmd == "M3|0|0|0\n":
            cmd = "M4|0|0|0|0\n"   # firmware bug: M3 zero can hang STM
        with self._lock:
            if self.ser is None or self._estop:
                return
            self._write(cmd)

    def stop(self):
        self._estop = True
        self._restore_tty()
        with self._lock:
            if self.ser is None:
                return
            try:
                for _ in range(10):
                    self._write("M4|0|0|0|0\n")
                    self.ser.flush()
                    time.sleep(0.03)
                self._write("STOP\n")
                self._write("ENC|0\n")
                self.ser.flush()
                time.sleep(0.3)          # let the reader print STM answers
                self._log("INFO", "motors stopped")
            except Exception as e:
                self._log("INFO", "stop error: %s" % e)
            ser, self.ser = self.ser, None
            try:
                ser.close()
            except Exception:
                pass

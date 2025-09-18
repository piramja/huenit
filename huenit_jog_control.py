#!/usr/bin/env python3
"""
huenit_jog_control.py
--------------------------------
Continuous, smooth keyboard jog control for the HUENIT arm, with a **single-line HUD**
that updates in place (no console spam).

Controls
  - Arrow Left/Right  → X− / X+
  - Arrow Up/Down     → Y+ / Y−
  - W / S             → Z+ / Z−
  - A                 → Vacuum ON  (M1400 A1023)
  - D                 → Vacuum OFF (M1400 A0)
  - +/-               → Decrease/Increase jog speed
  - Q / ESC           → Quit

Notes
  - High-rate control loop + easing + residuals for smooth motion.
  - A background thread polls encoders (M1008 A3) at low rate for HUD; the control loop never blocks on it.
  - The HUD writes one line with \\r (carriage return) so each refresh replaces the previous line.
  - Requires: pip install pyserial
"""

import os, sys, time, re, math, threading


# ------------------------- Configuration --------------------------------------

BAUD = 115200

# Jog parameters
CONTROL_HZ               = 80          # high control rate for tiny steps
JOG_TARGET_MM_S_DEFAULT  = 120.0        # target jog speed (per axis) while held
FEED_MM_MIN              = 1200        # feedrate for G1 steps (mm/min)
RAMP_ALPHA               = 0.16        # easing factor (0..1), smaller = softer
MIN_STEP_MM              = 0.002       # send only if per-axis exceeds this
IDLE_SYNC_AFTER_S        = 0.6         # issue M400 if idle this long (prevents backlog)

# Encoder poller (HUD only; control never blocks)
HUD_POSE_HZ              = 2.0         # 0..5 Hz recommended
ENC_TIMEOUT_MS           = 120         # short timeout for HUD polling

# Residual accumulators to avoid quantization loss
_residual = {'X':0.0, 'Y':0.0, 'Z':0.0}

# Vacuum
VAC_ON_CMD  = "M1400 A1023"
VAC_OFF_CMD = "M1400 A0"
VAC_SETTLE_MS = 0

# Axes
AXES = ("X","Y","Z")
VERBOSE = False  # keep console quiet; HUD handles status


# --- Console helpers -----------------------------------------------------------
ANSI_ESC_RE = re.compile(r'\x1B\[[0-9;?]*[ -/]*[@-~]')
def _normalize_console_text(text: str) -> str:
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    text = ANSI_ESC_RE.sub("", text)
    text = text.replace("\t", " ")
    lines = [ln.lstrip() for ln in text.split("\n")]
    return "\n".join(lines)

def hud_write(line: str, width: int = 120):
    """Overwrite a single console line in-place using CR. No trailing LF."""
    s = _normalize_console_text(line)
    if len(s) < width:
        s = s + " " * (width - len(s))  # clear remnants from previous longer line
    sys.stdout.write("\r" + s[:width])
    sys.stdout.flush()

def println_cr(msg: str):
    """Occasional full line with \\n then \\r to avoid indent drift (for errors, exit, etc.)."""
    s = _normalize_console_text(str(msg))
    if not s.endswith("\n"):
        s += "\n"
    s += "\r"
    sys.stdout.write(s)
    sys.stdout.flush()

# --- Auto-detect HUENIT port (Windows & Linux) --------------------------------
def auto_detect_huenit_port():
    from serial.tools import list_ports
    VID, PID = 0x0403, 0x6015  # FTDI FT-X

    env_port = os.environ.get("HUENIT_PORT")
    if env_port:
        return env_port

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found.")

    candidates = []
    for p in ports:
        vid = getattr(p, "vid", None)
        pid = getattr(p, "pid", None)
        prod = (getattr(p, "product", "") or "").lower()
        manu = (getattr(p, "manufacturer", "") or "").lower()
        sn   = (getattr(p, "serial_number", "") or "")

        score = 0
        if vid == VID and pid == PID:
            score += 5
        if "huenit" in prod or "huearm" in prod:
            score += 3
        if sn and "HUEARM" in sn.upper():
            score += 2
        if "ftdi" in manu:
            score += 1
        if score > 0:
            candidates.append((score, p.device))

    if not candidates:
        for p in ports:
            prod = (getattr(p, "product", "") or "").lower()
            if "huenit" in prod or "huearm" in prod:
                candidates.append((1, p.device))

    if not candidates:
        overview = ", ".join(f"{p.device}({getattr(p,'product',None)})" for p in ports)
        raise RuntimeError(f"HUENIT controller not found. Available ports: {overview}")

    candidates.sort(key=lambda x: x[0], reverse=True)
    return candidates[0][1]

# ------------------------- Configuration --------------------------------------
try:
    PORT = auto_detect_huenit_port()
except Exception as e:
    println_cr(f"Error detecting HUENIT port: {e}")
    sys.exit(1)


# ------------------------- Keyboard (non-blocking) ----------------------------
class Keyboard:
    def __init__(self):
        self.win = os.name == 'nt'
        if self.win:
            import msvcrt
            self.msvcrt = msvcrt
        else:
            import termios, tty, fcntl
            self.termios = termios; self.tty = tty; self.fcntl = fcntl
            self.fd = sys.stdin.fileno()
            self.old_term = termios.tcgetattr(self.fd)
            self.old_flags = fcntl.fcntl(self.fd, self.fcntl.F_GETFL)
    def __enter__(self):
        if self.win: return self
        self.tty.setraw(self.fd)
        self.fcntl.fcntl(self.fd, self.fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)
        return self
    def __exit__(self, *exc):
        if not self.win:
            self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.old_term)
            self.fcntl.fcntl(self.fd, self.fcntl.F_SETFL, self.old_flags)
    def get_keys(self):
        keys = []
        if self.win:
            while self.msvcrt.kbhit():
                ch = self.msvcrt.getch()
                if ch in (b'\x00', b'\xe0'):
                    ch2 = self.msvcrt.getch()
                    table = {b'H':'UP', b'P':'DOWN', b'K':'LEFT', b'M':'RIGHT'}
                    if ch2 in table: keys.append(table[ch2])
                else:
                    try: keys.append(ch.decode('utf-8','ignore'))
                    except: pass
        else:
            try:
                data = os.read(self.fd, 64)
            except BlockingIOError:
                data = b''
            if data:
                s = data.decode('utf-8','ignore')
                i = 0
                while i < len(s):
                    c = s[i]
                    if c == '\x1b' and i+2 < len(s) and s[i+1]=='[':
                        code = s[i+2]
                        table = {'A':'UP','B':'DOWN','D':'LEFT','C':'RIGHT'}
                        if code in table:
                            keys.append(table[code]); i += 3; continue
                    keys.append(c); i += 1
        return keys

# ------------------------- Serial / G-code I/O --------------------------------
import serial
OK_PAT  = re.compile(rb"\bok\b", re.I)
ENC_PAT = re.compile(
    r'\bX\s*[:=]\s*(-?\d+(?:\.\d+)?)\s*'
    r'Y\s*[:=]\s*(-?\d+(?:\.\d+)?)\s*'
    r'Z\s*[:=]\s*(-?\d+(?:\.\d+)?)',
    re.I
)

class GCodeIO:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.rx = threading.Thread(target=self._rx_loop, daemon=True); self.rx.start()
    def close(self):
        try: self.ser.close()
        except: pass
    def _rx_loop(self):
        while self.ser.is_open:
            try:
                n=self.ser.in_waiting
                if n:
                    d=self.ser.read(n)
                    if d:
                        with self.lock: self.buf.extend(d)
                else:
                    time.sleep(0.003)
            except: break
    def read_available_text(self, clear=True):
        with self.lock:
            if not self.buf: return ""
            data = bytes(self.buf)
            if clear: self.buf.clear()
        try: return data.decode(errors="ignore")
        except: return ""
    def send(self, line, wait_ok=False, timeout=1.0):
        # keep console quiet unless VERBOSE; HUD handles status
        self.ser.write((line.strip()+"\n").encode("ascii","ignore")); self.ser.flush()
        if not wait_ok:
            return b""
        t0=time.time(); acc=b""
        while time.time()-t0<timeout:
            time.sleep(0.003)
            with self.lock:
                if self.buf:
                    acc+=bytes(self.buf); self.buf.clear()
            if OK_PAT.search(acc):
                if VERBOSE:
                    txt = _normalize_console_text(acc.decode(errors="ignore"))
                    if not txt.endswith("\n"): txt += "\n"
                    txt += "\r"
                    sys.stdout.write(txt); sys.stdout.flush()
                return acc
        if acc and VERBOSE:
            txt = _normalize_console_text(acc.decode(errors="ignore"))
            if not txt.endswith("\n"): txt += "\n"
            txt += "\r"
            sys.stdout.write(txt); sys.stdout.flush()
        return acc
    def query_encoders(self, timeout_ms=ENC_TIMEOUT_MS):
        _ = self.read_available_text(clear=True)
        self.send("M1008 A3", wait_ok=False)
        t0 = time.time()*1000.0
        acc = ""
        while (time.time()*1000.0 - t0) < timeout_ms:
            txt = self.read_available_text(clear=True)
            if txt:
                acc += txt
                m = ENC_PAT.search(acc)
                if m:
                    x,y,z = map(float, m.groups())
                    return {"X":x, "Y":y, "Z":z}
            time.sleep(0.003)
        return None

# ------------------------- Background encoder poller (HUD) --------------------
class PosePoller(threading.Thread):
    def __init__(self, g: GCodeIO, hz: float):
        super().__init__(daemon=True)
        self.g = g
        self.period = 1.0 / max(0.1, hz) if hz > 0 else None
        self.running = True
        self.lock = threading.Lock()
        self.pose = None
    def run(self):
        if self.period is None:
            return
        while self.running:
            t0 = time.time()
            pos = self.g.query_encoders(timeout_ms=ENC_TIMEOUT_MS)
            with self.lock:
                if pos: self.pose = pos
            dt = time.time() - t0
            time.sleep(max(0.0, self.period - dt))
    def get_pose(self):
        with self.lock:
            return dict(self.pose) if self.pose else None
    def stop(self):
        self.running = False

# ------------------------- Jog Controller -------------------------------------
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def main():
    println_cr(f"HUENIT Jog Control (HUD) — Port: {PORT} @ {BAUD}")
    println_cr("Keys: ←/→=X  ↑/↓=Y  W/S=Z+/-  A=VacON  D=VacOFF  +/- speed  Q/ESC quit")

    try:
        g = GCodeIO(PORT, BAUD)
    except Exception as e:
        println_cr(f"Serial error: {e}")
        sys.exit(1)

    with Keyboard() as kb:
        poller = PosePoller(g, HUD_POSE_HZ)
        poller.start()
        try:
            # Absolute then relative
            g.send("G90", wait_ok=True)
            g.send("G21", wait_ok=True)
            g.send("G91", wait_ok=True)

            vac_on = False
            jog_target = JOG_TARGET_MM_S_DEFAULT
            vx = vy = vz = 0.0
            t_last = time.time()
            t_last_idle = t_last

            running = True
            while running:
                # Keys
                held = {"x":0, "y":0, "z":0}
                for k in kb.get_keys():
                    kl = k.lower()
                    if k in ("LEFT","RIGHT","UP","DOWN"):
                        if k == "LEFT":  held["x"] = -1
                        if k == "RIGHT": held["x"] = +1
                        if k == "UP":    held["y"] = +1
                        if k == "DOWN":  held["y"] = -1
                    elif kl == 'w':
                        held["z"] = +1
                    elif kl == 's':
                        held["z"] = -1
                    elif kl == 'a':
                        if not vac_on:
                            g.send(VAC_ON_CMD, wait_ok=True)
                            if VAC_SETTLE_MS>0: g.send(f"G4 P{VAC_SETTLE_MS}", wait_ok=True)
                            vac_on = True
                    elif kl == 'd':
                        if vac_on:
                            g.send(VAC_OFF_CMD, wait_ok=True)
                            if VAC_SETTLE_MS>0: g.send(f"G4 P{VAC_SETTLE_MS}", wait_ok=True)
                            vac_on = False
                    elif kl in ('+', '='):
                        jog_target = clamp(jog_target * 1.15, 1.0, 200.0)
                    elif kl == '-':
                        jog_target = clamp(jog_target / 1.15, 1.0, 200.0)
                    elif kl in ('q', '\x1b'):
                        running = False

                # Timing
                now = time.time()
                dt = now - t_last
                period = 1.0/CONTROL_HZ
                if dt < period:
                    time.sleep(period - dt)
                    now = time.time()
                    dt = now - t_last
                t_last = now

                # Smooth velocities
                tx = held["x"] * jog_target
                ty = held["y"] * jog_target
                tz = held["z"] * jog_target
                vx += (tx - vx) * RAMP_ALPHA
                vy += (ty - vy) * RAMP_ALPHA
                vz += (tz - vz) * RAMP_ALPHA

                # Displacements + residuals
                dx = vx * dt + _residual['X']
                dy = vy * dt + _residual['Y']
                dz = vz * dt + _residual['Z']

                cmd_parts = []
                if abs(dx) >= MIN_STEP_MM:
                    cmd_parts.append(f"X{dx:.4f}"); _residual['X'] = 0.0
                else:
                    _residual['X'] = dx
                if abs(dy) >= MIN_STEP_MM:
                    cmd_parts.append(f"Y{dy:.4f}"); _residual['Y'] = 0.0
                else:
                    _residual['Y'] = dy
                if abs(dz) >= MIN_STEP_MM:
                    cmd_parts.append(f"Z{dz:.4f}"); _residual['Z'] = 0.0
                else:
                    _residual['Z'] = dz

                if cmd_parts:
                    g.send("G1 " + " ".join(cmd_parts) + f" F{FEED_MM_MIN:.1f}", wait_ok=False)
                    t_last_idle = now
                else:
                    # idle; flush planner after idle timeout
                    if (now - t_last_idle) >= IDLE_SYNC_AFTER_S:
                        g.send("M400", wait_ok=True, timeout=2.0)
                        t_last_idle = now

                # HUD line
                pose = poller.get_pose()
                if pose:
                    pos_txt = f"X:{pose['X']:+07.2f} Y:{pose['Y']:+07.2f} Z:{pose['Z']:+07.2f}"
                else:
                    pos_txt = "X:  ----  Y:  ----  Z:  ----"
                v_txt = f"v=({vx:+05.1f},{vy:+05.1f},{vz:+05.1f}) mm/s"
                keys_txt = f"held: x={held['x']} y={held['y']} z={held['z']}"
                hud_write(f"[HUENIT JOG] {pos_txt}  {v_txt}  speed:{jog_target:05.1f}  vac:{'ON' if vac_on else 'OFF'}  {keys_txt}")

        finally:
            try:
                g.send("G90", wait_ok=True)  # back to absolute
            except Exception:
                pass
            poller.stop()
            g.close()
    println_cr("\nBye.")

if __name__ == "__main__":
    main()

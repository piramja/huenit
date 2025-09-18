#!/usr/bin/env python3
"""
huenit_teach_replay.py
------------------------------------------
Manual teach & replay for the HUENIT arm using encoder queries.

Controls:
  - c       → snapshot current encoder pose (via "M1008 A3")
  - v       → toggle vacuum (ON = "M1400 A1023", OFF = "M1400 A0") and record as event
  - r       → reset: clear current sequence (poses + events)
  - s       → CSV save/load: after 's' press 'w' (write) or 'l' (load)
  - ENTER   → replay current sequence (can be repeated any time)
  - q / ESC → quit

Replay:
  - Constant feedrate motion between poses
  - Vacuum events executed inline in sequence order
  - Optional dwell after vacuum toggles and between consecutive poses

Notes:
  - Please run in a *real* terminal (IDE consoles often buffer input).
  - Dependency: pip install pyserial
"""

import os, sys, time, re, math, threading, csv
from datetime import datetime

# ------------------------- Configuration --------------------------------------

BAUD = 115200
FEED_REPLAY_MM_MIN = 2000        # constant feedrate during replay (mm/min)
MOVE_TIMEOUT_S     = 8.0
MOTOR_ON_BEFORE_REPLAY = True
VAC_SETTLE_MS      = 1000        # dwell after vacuum on/off (ms) to build/release pressure
DWELL_BETWEEN_POSES_MS = 0       # small dwell between consecutive poses (ms)

AXES = ("X","Y","Z")
VERBOSE = True

# Encoder / Vacuum
ENC_TIMEOUT_MS = 300
VAC_ON_CMD  = "M1400 A1023"
VAC_OFF_CMD = "M1400 A0"

# CSV
DEFAULT_CSV = "teach_track.csv"


def print_r(msg):
    print(msg + "\r")


# --- Automatic HUENIT port detection (Windows & Linux) -------------------------
# Prerequisite: pip install pyserial
# Usage:
#   PORT = auto_detect_huenit_port()   # returns e.g. "COM20" or "/dev/ttyUSB0"
def auto_detect_huenit_port():
    from serial.tools import list_ports

    # HUENIT dmesg signature: FTDI FT-X, VID/PID 0403:6015, Product "HUENIT_HUEARM", Serial "*_HUEARM"
    VID, PID = 0x0403, 0x6015

    # 1) explicit override via environment variable
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
        # strongest signal: exact VID/PID
        if vid == VID and pid == PID:
            score += 5
        # product/serial markers
        if "huenit" in prod or "huearm" in prod:
            score += 3
        if sn and "HUEARM" in sn.upper():
            score += 2
        # manufacturer hint (FTDI)
        if "ftdi" in manu:
            score += 1

        if score > 0:
            candidates.append((score, p.device, p))

    # Fallback: if VID/PID isn't provided, match on product name only
    if not candidates:
        for p in ports:
            prod = (getattr(p, "product", "") or "").lower()
            if "huenit" in prod or "huearm" in prod:
                candidates.append((1, p.device, p))

    if not candidates:
        overview = ", ".join(f"{p.device}({getattr(p,'product',None)})" for p in ports)
        raise RuntimeError(f"HUENIT controller not found. Available ports: {overview}")

    candidates.sort(key=lambda x: x[0], reverse=True)
    port = candidates[0][1]

    return port

# ------------------------- Port Detection ------------------------------------

try:
    PORT = auto_detect_huenit_port()
except Exception as e:
    print_r("Error detecting HUENIT port:", e)
    sys.exit(1)

# ------------------------- Keyboard (raw, non-blocking) -----------------------
class Keyboard:
    def __init__(self):
        self.win = os.name == 'nt'
        if not self.win:
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
    def getch(self):
        if self.win:
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                try: return ch.decode('utf-8','ignore')
                except: return ''
            return None
        try: return sys.stdin.read(1)
        except (IOError, OSError): return None
    def waitch(self, prompt=None):
        if prompt: print_r(prompt, flush=True)
        while True:
            ch = self.getch()
            if ch: return ch
            time.sleep(0.01)

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
    def send(self, line, wait_ok=True, timeout=1.2):
        if VERBOSE: print_r(f">> {line}")
        self.ser.write((line.strip()+"\n").encode("ascii","ignore")); self.ser.flush()
        if not wait_ok: return b""
        t0=time.time(); acc=b""
        while time.time()-t0<timeout:
            time.sleep(0.003)
            with self.lock:
                if self.buf:
                    acc+=bytes(self.buf); self.buf.clear()
            if OK_PAT.search(acc):
                if VERBOSE and acc: sys.stdout.write(acc.decode(errors="ignore") + "\r")
                return acc
        if VERBOSE and acc: sys.stdout.write(acc.decode(errors="ignore") + "\r")
        return acc
    def query_encoders(self, timeout_ms=ENC_TIMEOUT_MS):
        """Send 'M1008 A3' and parse one line 'X:.. Y:.. Z:..'. Return dict or None."""
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
            time.sleep(0.005)
        return None

# ------------------------- Sequence container ---------------------------------
# Sequence is a list of events in order:
#   {"type": "pose", "X":..., "Y":..., "Z":...}
#   {"type": "vac",  "on": True/False}
def capture_pose(g: GCodeIO):
    pos = g.query_encoders()
    if pos is None:
        print_r("[POSE] no encoder response."); return None
    ev = {"type":"pose", "X":pos.get("X",0.0), "Y":pos.get("Y",0.0), "Z":pos.get("Z",0.0)}
    print_r(f"[POSE] {{'X':{ev['X']:.3f}, 'Y':{ev['Y']:.3f}, 'Z':{ev['Z']:.3f}}}")
    return ev

def toggle_vac(g: GCodeIO, current_on: bool):
    new_on = not current_on
    g.send(VAC_ON_CMD if new_on else VAC_OFF_CMD, wait_ok=True)
    print_r(f"[VAC] {'ON' if new_on else 'OFF'}")
    return {"type":"vac", "on": new_on}, new_on

# ------------------------- CSV I/O --------------------------------------------
def save_csv(seq, filename=DEFAULT_CSV):
    try:
        with open(filename, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["# teach_replay sequence"])
            w.writerow(["# generated:", datetime.now().isoformat()])
            w.writerow(["type","X","Y","Z","on"])
            for ev in seq:
                if ev["type"]=="pose":
                    w.writerow(["P", f"{ev['X']:.6f}", f"{ev['Y']:.6f}", f"{ev['Z']:.6f}", ""])
                elif ev["type"]=="vac":
                    w.writerow(["V", "", "", "", "1" if ev["on"] else "0"])
        print_r(f"[CSV] saved: {filename}")
    except Exception as e:
        print_r("[CSV] save error:", e)

def load_csv(filename=DEFAULT_CSV):
    seq = []
    try:
        with open(filename, "r", newline="") as f:
            r = csv.reader(f)
            for row in r:
                if not row or row[0].startswith("#"): continue
                if row[0].lower()=="type": continue
                t = row[0].strip().upper()
                if t=="P":
                    x = float(row[1]) if row[1] else 0.0
                    y = float(row[2]) if row[2] else 0.0
                    z = float(row[3]) if row[3] else 0.0
                    seq.append({"type":"pose", "X":x, "Y":y, "Z":z})
                elif t=="V":
                    on = row[4].strip() in ("1","true","True","ON","on")
                    seq.append({"type":"vac", "on":on})
        print_r(f"[CSV] loaded: {filename}  (events: {len(seq)})")
    except Exception as e:
        print_r("[CSV] load error:", e)
    return seq

# ------------------------- Replay ---------------------------------------------
def replay_sequence(g: GCodeIO, seq):
    if not seq:
        print_r("No sequence — nothing to replay."); return
    print_r(f"Replay … (events: {len(seq)})")
    if MOTOR_ON_BEFORE_REPLAY:
        g.send("M17", wait_ok=True)
    g.send("G90"); g.send("G21")

    vac_on = False
    first_pose_done = False

    # Execute events strictly in order
    for i, ev in enumerate(seq):
        nxt = seq[i+1] if i+1 < len(seq) else None

        if ev["type"] == "vac":
            if ev["on"] != vac_on:
                g.send(VAC_ON_CMD if ev["on"] else VAC_OFF_CMD, wait_ok=True)
                vac_on = ev["on"]
                g.send(f"G4 P{VAC_SETTLE_MS}", wait_ok=True)  # pressure build-up/bleed
        elif ev["type"] == "pose":
            if not first_pose_done:
                g.send(f"G0 X{ev['X']:.3f} Y{ev['Y']:.3f} Z{ev['Z']:.3f}", wait_ok=True, timeout=MOVE_TIMEOUT_S)
                first_pose_done = True
            else:
                g.send(f"G1 X{ev['X']:.3f} Y{ev['Y']:.3f} Z{ev['Z']:.3f} F{FEED_REPLAY_MM_MIN:.1f}", wait_ok=True, timeout=MOVE_TIMEOUT_S)
            # small dwell ONLY between two consecutive poses
            if nxt is not None and nxt.get("type") == "pose" and DWELL_BETWEEN_POSES_MS > 0:
                g.send(f"G4 P{DWELL_BETWEEN_POSES_MS}", wait_ok=True)

    g.send("M400", wait_ok=True)
    print_r("Replay finished.")

# ------------------------- Main Loop ------------------------------------------
def main():
    print_r(f"Teach-&-Replay (encoder, manual, VAC/CSV) — Port: {PORT} @ {BAUD}")
    print_r("Keys: c=snapshot  v=vacuum toggle  r=reset  s=save/load  ENTER=replay  q/ESC=quit\n")

    try:
        g = GCodeIO(PORT, BAUD)
    except Exception as e:
        print_r("Serial error:", e); sys.exit(1)

    seq = []       # mixed event list
    vac_on = False # current vacuum state (for toggle)

    with Keyboard() as kb:
        try:
            g.send("G90"); g.send("G21")

            while True:
                ch = kb.waitch()
                if ch in ['\r','\n']:  # ENTER -> replay
                    replay_sequence(g, seq)
                elif ch.lower() in ['q','\x1b']:
                    print_r("Bye!"); break
                elif ch.lower()=='c':
                    ev = capture_pose(g)
                    if ev: seq.append(ev)
                elif ch.lower()=='v':
                    ev, vac_on = toggle_vac(g, vac_on)
                    seq.append(ev)
                elif ch.lower()=='r':
                    seq.clear(); vac_on = False
                    print_r("[RESET] sequence cleared.")
                elif ch.lower()=='s':
                    print_r("[S] (w)rite or (l)oad?")
                    ch2 = kb.waitch()
                    if ch2 and ch2.lower()=='w':
                        fname = DEFAULT_CSV  # or use timestamped name
                        save_csv(seq, fname)
                    elif ch2 and ch2.lower()=='l':
                        fname = DEFAULT_CSV
                        seq = load_csv(fname)
                        # after load start with vacuum OFF (explicitly toggle if needed)
                        vac_on = False
                    else:
                        print_r("[S] cancelled.")
                else:
                    # ignore other keys
                    pass

        finally:
            g.close()

if __name__ == "__main__":
    main()

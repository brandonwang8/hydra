#!/usr/bin/env python3
"""
teleop_joystick_verbose.py
Single-key joystick-style teleop for Motor2040 with optional verbose logging.

Run:
  python teleop_joystick_verbose.py           → clean, filtered output
  python teleop_joystick_verbose.py --verbose → full raw board output

Keys:
  Motor 1: 'a' = -STEP, 'd' = +STEP  (uppercase = BIG_STEP)
  Motor 2: 's' = -STEP, 'w' = +STEP  (uppercase = BIG_STEP)
  z -> STATUS
  r -> RESET
  x -> EXIT (drive to 0°, then quits)
  q -> Quit (no EXIT)
"""

import sys, time, glob, threading, queue, select, re, serial, termios, tty
from textwrap import dedent

# ---------- CONFIG ----------
BAUD = 115200
SERIAL_TIMEOUT = 0.05
POLL_DELAY = 0.02
STEP = 30.0
BIG_STEP = 90.0
SEND_RATE_HZ = 10.0
MIN_SEND_INTERVAL = 1.0 / SEND_RATE_HZ
SEND_DELTA_THRESHOLD = 0.02
PORTS = ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0")
IN_Q_MAXSIZE = 500
# ----------------------------

VERBOSE = "--verbose" in sys.argv

def find_port():
    for c in PORTS:
        if glob.glob(c):
            return c
    return None


class SerialReader(threading.Thread):
    """Reader thread that buffers partial lines and sanitizes output"""
    def __init__(self, ser, out_q):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_q = out_q
        self.running = True
        self.buffer = ""

    def run(self):
        while self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    time.sleep(0.01)
                    continue
                try:
                    text = data.decode("utf-8", errors="replace")
                except Exception:
                    text = str(data)
                self.buffer += text
                # Split only on newline
                while "\n" in self.buffer:
                    line, self.buffer = self.buffer.split("\n", 1)
                    line = line.strip()
                    if not VERBOSE:
                        # sanitize nonprintable control chars but preserve text
                        line = re.sub(r"[^ -~]", "", line)
                    if line:
                        try:
                            self.out_q.put_nowait(line)
                        except queue.Full:
                            pass
            except Exception:
                time.sleep(0.05)

    def stop(self):
        self.running = False


class KeyReader:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
    def get_key(self, timeout=0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None


def send(ser, msg):
    try:
        ser.write((msg.strip() + "\n").encode())
    except Exception as e:
        print("Serial write error:", e)


def parse_status(line):
    parts = line.split()
    out = {"sp1": None, "sp2": None}
    try:
        for i, tok in enumerate(parts):
            if tok.upper() == "SP1" and i + 1 < len(parts):
                out["sp1"] = float(parts[i + 1])
            elif tok.upper() == "SP2" and i + 1 < len(parts):
                out["sp2"] = float(parts[i + 1])
    except Exception:
        pass
    return out


def print_local(sp1, sp2):
    print(f"[local] SP1={sp1:.2f}°, SP2={sp2:.2f}°")


def main():
    port = find_port()
    if not port:
        print("No serial port found.")
        return

    print(f"Connecting to {port} @ {BAUD}...")
    try:
        ser = serial.Serial(port, BAUD, timeout=SERIAL_TIMEOUT)
        time.sleep(0.4)
    except serial.SerialException as e:
        print("Serial open failed:", e)
        return

    qin = queue.Queue(maxsize=IN_Q_MAXSIZE)
    reader = SerialReader(ser, qin)
    reader.start()

    sp1, sp2 = 0.0, 0.0
    send(ser, "STATUS")
    time.sleep(0.5)
    while ser.in_waiting:
        raw = ser.readline().decode(errors="ignore").strip()
        print("<<", raw)

    print(dedent(f"""
    Controls:
      Motor1: 'a' (-{STEP}), 'd' (+{STEP}) | 'A'/'D' (±{BIG_STEP})
      Motor2: 's' (-{STEP}), 'w' (+{STEP}) | 'S'/'W' (±{BIG_STEP})
      z: STATUS, r: RESET, x: EXIT, q: quit
      Verbose = {VERBOSE}
    """))
    print_local(sp1, sp2)

    last_send = 0.0
    last_sp1, last_sp2 = sp1, sp2
    try:
        with KeyReader() as kr:
            while True:
                # print queued serial lines
                try:
                    line = qin.get_nowait()
                    print("<<", line)
                    if "STATUS" in line.upper():
                        parsed = parse_status(line)
                        if parsed["sp1"] is not None:
                            sp1 = parsed["sp1"]
                        if parsed["sp2"] is not None:
                            sp2 = parsed["sp2"]
                            print_local(sp1, sp2)
                except queue.Empty:
                    pass

                k = kr.get_key(timeout=POLL_DELAY)
                if not k:
                    continue

                # Motor control
                if k in ("a","A","d","D","s","S","w","W"):
                    step = BIG_STEP if k.isupper() else STEP
                    if k.lower() == "a": sp1 -= step
                    elif k.lower() == "d": sp1 += step
                    elif k.lower() == "s": sp2 -= step
                    elif k.lower() == "w": sp2 += step

                    now = time.monotonic()
                    if (now - last_send) >= MIN_SEND_INTERVAL:
                        if abs(sp1 - last_sp1) > SEND_DELTA_THRESHOLD:
                            send(ser, f"SET_POS 1 {sp1:.2f}")
                            last_sp1 = sp1
                        if abs(sp2 - last_sp2) > SEND_DELTA_THRESHOLD:
                            send(ser, f"SET_POS 2 {sp2:.2f}")
                            last_sp2 = sp2
                        last_send = now
                    print_local(sp1, sp2)

                elif k == "z" or k == "Z":
                    send(ser, "STATUS")

                elif k == "r":
                    print("RESET...")
                    send(ser, "RESET")

                elif k == "x" or k == "X":
                    print("EXIT (waiting for EXIT COMPLETE)...")
                    send(ser, "EXIT")
                    t0 = time.time()
                    while time.time() - t0 < 10:
                        while ser.in_waiting:
                            l = ser.readline().decode(errors="ignore").strip()
                            print("<<", l)
                            if "EXIT COMPLETE" in l.upper():
                                reader.stop(); ser.close(); print("Done."); return
                        time.sleep(0.05)
                    print("Timeout waiting for EXIT COMPLETE.")
                    reader.stop(); ser.close(); return

                elif k == "q" | k == "Q":
                    print("Quitting (no EXIT).")
                    reader.stop()
                    ser.close()
                    return
    except KeyboardInterrupt:
        print("\nUser interrupted.")  
    finally:
        reader.stop()
        ser.close()


if __name__ == "__main__":
    main()

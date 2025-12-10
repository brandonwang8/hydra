#!/usr/bin/env python3
"""
teleop_arrow_fixed.py

Arrow-key teleop with hold-to-ramp and robust serial handling.

- Left/Right -> Motor1 (- / +)
- Up/Down    -> Motor2 (+ / -)
- single tap = single small step
- hold arrow = continuous velocity (ramp up to MAX_VEL)
- s -> single STATUS request
- r -> single RESET request
- x -> EXIT (board drives to 0, script waits for EXIT COMPLETE then quits)
- q -> quit teleop (no EXIT)
"""

import sys, time, glob, threading, queue, select
import serial
import termios, tty
from textwrap import dedent

# ---------- CONFIG ----------
BAUD = 115200
SERIAL_TIMEOUT = 0.05
POLL_DELAY = 0.02

SMALL_STEP = 5.0        # single-tap degrees
ACCEL = 180.0           # deg/s^2
MAX_VEL = 120.0         # deg/s
HOLD_RELEASE_TIMEOUT = 0.18

SEND_RATE_HZ = 10.0           # max SET_POS sends per second
MIN_SEND_INTERVAL = 1.0 / SEND_RATE_HZ
SEND_DELTA_THRESHOLD = 0.02   # degrees - only send if setpoint changed more than this

PORT_CANDIDATES = ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0")
STATUS_PARSE_TIMEOUT = 0.4
DEDUPE_WINDOW = 0.12
# ----------------------------

def find_port():
    for c in PORT_CANDIDATES:
        if glob.glob(c):
            return c
    return None

# ---------- Serial reader thread ----------
class SerialReader(threading.Thread):
    def __init__(self, ser, out_q):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_q = out_q
        self.running = True
        self._last_line = None
        self._last_time = 0.0

    def run(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if not line:
                        continue
                    now = time.monotonic()
                    # Basic dedupe: avoid huge bursts of the exact same line
                    if line == self._last_line and (now - self._last_time) < DEDUPE_WINDOW:
                        continue
                    self._last_line = line
                    self._last_time = now
                    # Put into queue for main thread to handle (non-blocking)
                    try:
                        self.out_q.put_nowait(line)
                    except queue.Full:
                        # if queue is full, drop lines to avoid blocking
                        pass
                else:
                    time.sleep(0.005)
            except Exception:
                time.sleep(0.05)

    def stop(self):
        self.running = False

# ---------- Terminal key reader ----------
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
        if not r:
            return None
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':
            # read remaining bytes of escape sequence if available quickly
            seq = ch1
            # try to read up to 2 more bytes (common for arrow keys)
            for _ in range(2):
                r2, _, _ = select.select([sys.stdin], [], [], 0.01)
                if r2:
                    seq += sys.stdin.read(1)
                else:
                    break
            return seq
        return ch1

# ---------- Helpers ----------
def send(ser, msg):
    try:
        ser.write((msg.strip() + "\n").encode())
    except Exception as e:
        print("Serial write error:", e)

def parse_status_line(line):
    parts = line.split()
    out = {'p1': None, 'sp1': None, 'p2': None, 'sp2': None}
    try:
        for i, tok in enumerate(parts):
            if tok.upper() == "POS1" and i+1 < len(parts):
                out['p1'] = float(parts[i+1])
            elif tok.upper() == "SP1" and i+1 < len(parts):
                out['sp1'] = float(parts[i+1])
            elif tok.upper() == "POS2" and i+1 < len(parts):
                out['p2'] = float(parts[i+1])
            elif tok.upper() == "SP2" and i+1 < len(parts):
                out['sp2'] = float(parts[i+1])
    except Exception:
        pass
    return out

def print_local_sp(sp1, sp2):
    print(f"[local] SP1={sp1:.2f}°, SP2={sp2:.2f}°")

# ---------- Main ----------
def main():
    port = find_port()
    if not port:
        print("No serial port found. Make sure /dev/serial0 or equivalent is present.")
        return

    print(f"Connecting to {port} @ {BAUD}...")
    try:
        ser = serial.Serial(port, BAUD, timeout=SERIAL_TIMEOUT)
        time.sleep(0.4)
    except serial.SerialException as e:
        print("Failed to open serial port:", e)
        return

    # queue for incoming lines (moderate size to avoid drops)
    in_q = queue.Queue(maxsize=200)
    reader = SerialReader(ser, in_q)
    reader.start()

    # local setpoints
    sp1, sp2 = 0.0, 0.0

    # sync: wait briefly for READY then probe STATUS to initialize local SPs
    print("Waiting for READY (0.8s)...")
    ready = False
    t0 = time.time()
    while time.time() - t0 < 0.8:
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print("<<", line)
                if "READY" in line.upper():
                    ready = True
                    break
        else:
            time.sleep(0.01)

    # request status (attempt to populate local setpoints)
    send(ser, "STATUS")
    t0 = time.time()
    while time.time() - t0 < STATUS_PARSE_TIMEOUT:
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print("<<", line)
                if line.upper().startswith("STATUS"):
                    parsed = parse_status_line(line)
                    if parsed['sp1'] is not None:
                        sp1 = parsed['sp1']
                    if parsed['sp2'] is not None:
                        sp2 = parsed['sp2']
                    break
        else:
            time.sleep(0.01)

    print(dedent(f"""
    Arrow teleop:
      Left/Right -> Motor1 (- / +)
      Up/Down    -> Motor2 (+ / -)

      single tap = single small step ({SMALL_STEP}°)
      hold arrow = continuous velocity (ACCEL={ACCEL} deg/s², MAX_VEL={MAX_VEL} deg/s)

      s -> single STATUS
      r -> single RESET
      x -> EXIT (board will drive both motors to 0°; teleop waits for EXIT COMPLETE then quits)
      q -> Quit teleop (no EXIT)
    """))
    print_local_sp(sp1, sp2)

    # continuous mode state
    active_dir = None        # 'left','right','up','down' or None
    last_event_time = 0.0
    vel = 0.0
    last_update = time.monotonic()
    last_send_time = 0.0
    last_sent_sp1, last_sent_sp2 = sp1, sp2

    try:
        with KeyReader() as kr:
            while True:
                now = time.monotonic()
                key = kr.get_key(timeout=POLL_DELAY)

                # handle immediate keyboard single-key commands
                if key and not key.startswith('\x1b'):
                    if key == 's':
                        send(ser, "STATUS")
                    elif key == 'r':
                        print("Sending RESET...")
                        send(ser, "RESET")
                        # quick read of immediate responses (non-blocking)
                        ttmp = time.time()
                        while time.time() - ttmp < 0.25:
                            if ser.in_waiting:
                                line = ser.readline().decode(errors="ignore").strip()
                                if line:
                                    print("<<", line)
                            else:
                                time.sleep(0.01)
                        # resync status
                        send(ser, "STATUS")
                    elif key == 'x':
                        print("Sending EXIT (board will drive to 0). Waiting for EXIT COMPLETE...")
                        send(ser, "EXIT")
                        tstart = time.time()
                        got_exit_complete = False
                        while time.time() - tstart < 15.0:
                            # prefer consuming from reader queue to capture everything
                            try:
                                line = in_q.get(timeout=0.05)
                                print("<<", line)
                                if "EXIT COMPLETE" in line.upper():
                                    got_exit_complete = True
                                    break
                            except queue.Empty:
                                pass
                        if not got_exit_complete:
                            print("No EXIT COMPLETE received (timeout).")
                        reader.stop()
                        ser.close()
                        print("Exiting teleop after EXIT.")
                        return
                    elif key == 'q':
                        print("Quitting teleop (no EXIT).")
                        reader.stop()
                        ser.close()
                        return
                    # else ignore other ordinary keys

                # handle arrow sequences (escape sequences)
                if key and key.startswith('\x1b'):
                    seq = key
                    # map common cursor sequences:
                    if seq in ('\x1b[A', '\x1bOA'):   # Up
                        dir_seen = 'up'
                    elif seq in ('\x1b[B', '\x1bOB'): # Down
                        dir_seen = 'down'
                    elif seq in ('\x1b[C', '\x1bOC'): # Right
                        dir_seen = 'right'
                    elif seq in ('\x1b[D', '\x1bOD'): # Left
                        dir_seen = 'left'
                    else:
                        dir_seen = None

                    if dir_seen:
                        last_event_time = now
                        # on initial press, apply single small step immediately
                        if active_dir is None:
                            if dir_seen == 'left':
                                sp1 -= SMALL_STEP
                                send(ser, f"SET_POS 1 {sp1:.2f}")
                                last_sent_sp1 = sp1
                                last_send_time = now
                            elif dir_seen == 'right':
                                sp1 += SMALL_STEP
                                send(ser, f"SET_POS 1 {sp1:.2f}")
                                last_sent_sp1 = sp1
                                last_send_time = now
                            elif dir_seen == 'up':
                                sp2 += SMALL_STEP
                                send(ser, f"SET_POS 2 {sp2:.2f}")
                                last_sent_sp2 = sp2
                                last_send_time = now
                            elif dir_seen == 'down':
                                sp2 -= SMALL_STEP
                                send(ser, f"SET_POS 2 {sp2:.2f}")
                                last_sent_sp2 = sp2
                                last_send_time = now
                            active_dir = dir_seen
                            vel = 0.0
                            print_local_sp(sp1, sp2)
                        else:
                            # if same direction, just refresh last_event_time; if different, switch
                            if dir_seen != active_dir:
                                active_dir = dir_seen
                                vel = 0.0
                                last_event_time = now

                # continuous update (acceleration) if a direction is active and recent
                dt = now - last_update if last_update else 0.0
                last_update = now
                if active_dir and (now - last_event_time) <= HOLD_RELEASE_TIMEOUT:
                    vel += ACCEL * dt
                    if vel > MAX_VEL:
                        vel = MAX_VEL
                    move = vel * dt
                    if active_dir == 'left':
                        sp1 -= move
                    elif active_dir == 'right':
                        sp1 += move
                    elif active_dir == 'up':
                        sp2 += move
                    elif active_dir == 'down':
                        sp2 -= move

                    # send rate-limited and only if changed enough
                    if (now - last_send_time) >= MIN_SEND_INTERVAL:
                        send1 = abs(sp1 - last_sent_sp1) > SEND_DELTA_THRESHOLD
                        send2 = abs(sp2 - last_sent_sp2) > SEND_DELTA_THRESHOLD
                        if send1:
                            send(ser, f"SET_POS 1 {sp1:.2f}")
                            last_sent_sp1 = sp1
                            last_send_time = now
                        if send2:
                            send(ser, f"SET_POS 2 {sp2:.2f}")
                            last_sent_sp2 = sp2
                            last_send_time = now
                    # print local setpoints occasionally (not every loop)
                    # print every ~0.12s
                    if int(now * 10) % 2 == 0:
                        print_local_sp(sp1, sp2)
                else:
                    # consider released
                    if active_dir and (now - last_event_time) > HOLD_RELEASE_TIMEOUT:
                        active_dir = None
                        vel = 0.0

                # consume & print one incoming serial line per loop (keeps UI responsive, prevents flood)
                try:
                    line = in_q.get_nowait()
                    # parse STATUS lines to keep local SP in sync
                    if line.upper().startswith("STATUS"):
                        parsed = parse_status_line(line)
                        updated = False
                        if parsed['sp1'] is not None:
                            sp1 = parsed['sp1']; updated = True
                        if parsed['sp2'] is not None:
                            sp2 = parsed['sp2']; updated = True
                        print("<<", line)
                        if updated:
                            print_local_sp(sp1, sp2)
                    else:
                        # print other lines but avoid flooding for repeated OKs
                        print("<<", line)
                except queue.Empty:
                    pass

    except KeyboardInterrupt:
        print("\nUser interrupted. Closing.")
    finally:
        try:
            reader.stop()
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()

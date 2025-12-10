# SPDX-License-Identifier: MIT
import time
import board
import pwmio
import digitalio
import rotaryio
import busio
from adafruit_motor import motor

# === Motor + Encoder configuration ===
MOTOR_PINS = {
    'A': (board.MOTOR_A_P, board.MOTOR_A_N, board.ENCODER_A_A, board.ENCODER_A_B),
    'B': (board.MOTOR_B_P, board.MOTOR_B_N, board.ENCODER_B_A, board.ENCODER_B_B),
    'C': (board.MOTOR_C_P, board.MOTOR_C_N, board.ENCODER_C_A, board.ENCODER_C_B),
    'D': (board.MOTOR_D_P, board.MOTOR_D_N, board.ENCODER_D_A, board.ENCODER_D_B),
}

FREQUENCY = 25000
GEAR_RATIO = 50
COUNTS_PER_REV = 12 * GEAR_RATIO

# PID tuning
KP = 0.14
KI = 0.0
KD = 0.0022

UPDATES = 100
UPDATE_RATE = 1 / UPDATES
SPEED_SCALE = 5.0

# DONE detection / exit behavior
TOLERANCE = 5.0        # degrees
STABLE_TIME = 0.25     # seconds of stability to consider done
EXIT_TIMEOUT = 10.0    # seconds max to drive to zero on EXIT

# user stop
user_sw = digitalio.DigitalInOut(board.USER_SW)
user_sw.direction = digitalio.Direction.INPUT
user_sw.pull = digitalio.Pull.UP

# motor/encoder factory
def make_motor(pin_p, pin_n, enc_a, enc_b):
    pwm_p = pwmio.PWMOut(pin_p, frequency=FREQUENCY)
    pwm_n = pwmio.PWMOut(pin_n, frequency=FREQUENCY)
    enc = rotaryio.IncrementalEncoder(enc_b, enc_a, divisor=1)
    m = motor.DCMotor(pwm_p, pwm_n)
    m.decay_mode = motor.SLOW_DECAY
    return m, enc

mot1, enc1 = make_motor(*MOTOR_PINS['A'])
mot2, enc2 = make_motor(*MOTOR_PINS['D'])

uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0.01)

class PID:
    def __init__(self, kp, ki, kd, sample_rate):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0.0
        self._last_value = 0.0
        self._sum = 0.0
        self.sample_rate = sample_rate
    def calculate(self, value):
        error = self.setpoint - value
        self._sum += error * self.sample_rate
        d = (value - self._last_value) / self.sample_rate
        self._last_value = value
        return (error * self.kp) + (self._sum * self.ki) - (d * self.kd)

pid1 = PID(KP, KI, KD, UPDATE_RATE)
pid2 = PID(KP, KI, KD, UPDATE_RATE)

def to_deg(pos):
    return (pos * 360.0) / COUNTS_PER_REV

def safe_uart_write(msg):
    try:
        uart.write((msg + "\n").encode())
    except Exception:
        pass

def stop_motors():
    mot1.throttle = 0
    mot2.throttle = 0

def reinit_pid_state():
    # clear integrators & last values, keep encoder raw counts unchanged
    pid1._sum = 0.0
    pid1._last_value = 0.0
    pid2._sum = 0.0
    pid2._last_value = 0.0
    pid1.setpoint = 0.0
    pid2.setpoint = 0.0

def drive_to_zero(timeout=EXIT_TIMEOUT, tolerance=TOLERANCE, stable_time=STABLE_TIME):
    """Drive both motors to setpoint 0 using PID loop and wait until stable or timeout.
       Returns True if reached stable zero, False if timed out."""
    start = time.monotonic()
    stable_since1 = None
    stable_since2 = None
    while True:
        now = time.monotonic()
        if now - start > timeout:
            return False
        # read positions
        p1 = to_deg(enc1.position)
        p2 = to_deg(enc2.position)
        # compute control outputs
        v1 = pid1.calculate(p1)
        v2 = pid2.calculate(p2)
        thr1 = max(min(v1 / SPEED_SCALE, 1.0), -1.0)
        thr2 = max(min(v2 / SPEED_SCALE, 1.0), -1.0)
        mot1.throttle = thr1
        mot2.throttle = thr2
        # stability checks
        if abs(p1 - pid1.setpoint) < tolerance:
            if stable_since1 is None:
                stable_since1 = now
        else:
            stable_since1 = None
        if abs(p2 - pid2.setpoint) < tolerance:
            if stable_since2 is None:
                stable_since2 = now
        else:
            stable_since2 = None
        if (stable_since1 is not None and now - stable_since1 >= stable_time) and \
           (stable_since2 is not None and now - stable_since2 >= stable_time):
            # Both motors stable within tolerance -> stop and return success
            mot1.throttle = 0
            mot2.throttle = 0
            return True
        time.sleep(UPDATE_RATE)

# announce ready on startup
safe_uart_write("READY")
print("READY")
time.sleep(0.1)
serial_buffer = ""

while True:
    # emergency stop
    if not user_sw.value:
        stop_motors()
        safe_uart_write("STOP")
        print("STOP")
        break

    data = uart.read(128)
    if data:
        try:
            serial_buffer += data.decode()
        except UnicodeError:
            serial_buffer = ""
        while "\n" in serial_buffer:
            line, serial_buffer = serial_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].upper()

            if cmd == "SET_POS" and len(parts) == 3:
                _, motor_id, val = parts
                try:
                    valf = float(val)
                except Exception:
                    safe_uart_write("ERR BAD_VALUE")
                    continue
                if motor_id == "1":
                    pid1.setpoint = valf
                    safe_uart_write(f"OK 1 SET_POS {valf}")
                elif motor_id == "2":
                    pid2.setpoint = valf
                    safe_uart_write(f"OK 2 SET_POS {valf}")
                else:
                    safe_uart_write("ERR UNKNOWN_MOTOR")

            elif cmd == "STATUS":
                p1 = to_deg(enc1.position)
                p2 = to_deg(enc2.position)
                thr1 = mot1.throttle if mot1.throttle else 0.0
                thr2 = mot2.throttle if mot2.throttle else 0.0
                s1 = "DONE" if abs(p1 - pid1.setpoint) < TOLERANCE else "MOVING"
                s2 = "DONE" if abs(p2 - pid2.setpoint) < TOLERANCE else "MOVING"
                safe_uart_write(
                    f"STATUS POS1 {p1:.2f} SP1 {pid1.setpoint:.2f} THR1 {thr1:.2f} {s1} "
                    f"POS2 {p2:.2f} SP2 {pid2.setpoint:.2f} THR2 {thr2:.2f} {s2}"
                )

            elif cmd == "RESET":
                safe_uart_write("RESET START")
                stop_motors()
                # IMPORTANT: do not change encoder positions here (user wanted to keep measured positions)
                reinit_pid_state()
                time.sleep(0.1)
                safe_uart_write("RESET COMPLETE")
                safe_uart_write("READY")
                print("RESET COMPLETE")

            elif cmd == "EXIT":
                safe_uart_write("EXIT START")
                # setpoints -> 0 (command motors to go home)
                pid1.setpoint = 0.0
                pid2.setpoint = 0.0
                # actively drive to zero using the PID loop and waiting until stability
                ok = drive_to_zero(timeout=EXIT_TIMEOUT, tolerance=TOLERANCE, stable_time=STABLE_TIME)
                if not ok:
                    # timed out: stop motors and report timeout but still send EXIT COMPLETE
                    mot1.throttle = 0
                    mot2.throttle = 0
                    safe_uart_write("EXIT TIMEOUT")
                safe_uart_write("EXIT COMPLETE")
                time.sleep(0.1)
                safe_uart_write("READY")
                print("READY")
                # do not break; remain running and accept new connections/commands

            else:
                safe_uart_write("ERR UNKNOWN_CMD")

    # PID control update (normal operation)
    pos1 = to_deg(enc1.position)
    pos2 = to_deg(enc2.position)
    vel1 = pid1.calculate(pos1)
    vel2 = pid2.calculate(pos2)
    mot1.throttle = max(min(vel1 / SPEED_SCALE, 1.0), -1.0)
    mot2.throttle = max(min(vel2 / SPEED_SCALE, 1.0), -1.0)
    time.sleep(UPDATE_RATE)

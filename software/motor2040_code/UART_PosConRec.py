# SPDX-License-Identifier: MIT
import time
import board
import pwmio
import digitalio
import rotaryio
import busio
from adafruit_motor import motor

# === Motor + Encoder configuration ===
MOTOR_PORT = 'B'  # <-- change this letter to A, B, C, or D

MOTOR_PINS = {
    'A': (board.MOTOR_A_P, board.MOTOR_A_N, board.ENCODER_A_A, board.ENCODER_A_B),
    'B': (board.MOTOR_B_P, board.MOTOR_B_N, board.ENCODER_B_A, board.ENCODER_B_B),
    'C': (board.MOTOR_C_P, board.MOTOR_C_N, board.ENCODER_C_A, board.ENCODER_C_B),
    'D': (board.MOTOR_D_P, board.MOTOR_D_N, board.ENCODER_D_A, board.ENCODER_D_B),
}

try:
    MOTOR_P, MOTOR_N, ENC_A, ENC_B = MOTOR_PINS[MOTOR_PORT.upper()]
except KeyError:
    raise ValueError("MOTOR_PORT must be one of 'A','B','C','D'")

FREQUENCY = 25000
GEAR_RATIO = 50
COUNTS_PER_REV = 12 * GEAR_RATIO

# PID Gains
KP = 0.14
KI = 0.0
KD = 0.0022

UPDATES = 100
UPDATE_RATE = 1 / UPDATES
SPEED_SCALE = 5.4

# User stop switch
user_sw = digitalio.DigitalInOut(board.USER_SW)
user_sw.direction = digitalio.Direction.INPUT
user_sw.pull = digitalio.Pull.UP

# Motor setup
pwm_p = pwmio.PWMOut(MOTOR_P, frequency=FREQUENCY)
pwm_n = pwmio.PWMOut(MOTOR_N, frequency=FREQUENCY)
mot = motor.DCMotor(pwm_p, pwm_n)
mot.decay_mode = motor.SLOW_DECAY

# Encoder (note: working pin order)
encoder = rotaryio.IncrementalEncoder(ENC_B, ENC_A)

# UART on GP0 (TX) / GP1 (RX)
uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0.01)

# PID Controller
class PID:
    def __init__(self, kp, ki, kd, sample_rate):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = 0
        self._last_value = 0
        self._sum = 0
        self.sample_rate = sample_rate

    def calculate(self, value):
        error = self.setpoint - value
        self._sum += error * self.sample_rate
        d = (value - self._last_value) / self.sample_rate
        self._last_value = value
        return (error * self.kp) + (self._sum * self.ki) - (d * self.kd)

pos_pid = PID(KP, KI, KD, UPDATE_RATE)

def to_deg(pos):
    return (pos * 360.0) / COUNTS_PER_REV

serial_buffer = ""
last_pos = last_sp = last_thr = None

print("READY (UART ACTIVE) - Send commands like: SET_POS 90")

while True:
    # Emergency stop button
    if not user_sw.value:
        mot.throttle = 0
        uart.write(b"STOP\n")
        print("STOP")
        break

    # UART command processing
    data = uart.read(1)
    if data:
        c = data.decode()
        serial_buffer += c
        if "\n" in serial_buffer:
            lines = serial_buffer.split("\n")
            for line in lines[:-1]:
                line = line.strip()
                if line.startswith("SET_POS"):
                    try:
                        _, val = line.split()
                        pos_pid.setpoint = float(val)
                        uart.write(f"OK SET_POS {val}\n".encode())
                        print(f"[USB DBG] SET_POS {val}")
                    except:
                        uart.write(b"ERR\n")
            serial_buffer = lines[-1]

    # PID control update
    pos = to_deg(encoder.position)
    vel = pos_pid.calculate(pos)
    thr = max(min(vel / SPEED_SCALE, 1.0), -1.0)
    mot.throttle = thr

    # Output only if changed
    rp = round(pos, 2)
    rsp = round(pos_pid.setpoint, 2)
    rthr = round(thr, 2)

    if (rp != last_pos) or (rsp != last_sp) or (rthr != last_thr):
        msg = f"POS {rp:.2f} SP {rsp:.2f} THR {rthr:.2f}\n"
        uart.write(msg.encode())
        print(msg, end="")  # USB debug (optional)
        last_pos, last_sp, last_thr = rp, rsp, rthr

    time.sleep(UPDATE_RATE)

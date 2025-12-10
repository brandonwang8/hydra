# SPDX-License-Identifier: MIT
import time
import sys
import board
import pwmio
import digitalio
import rotaryio
import supervisor
from adafruit_motor import motor

# --- Motor + Encoder configuration ---
MOTOR_P = board.MOTOR_A_P
MOTOR_N = board.MOTOR_A_N
ENC_A = board.ENCODER_A_A
ENC_B = board.ENCODER_A_B
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

# Encoder
encoder = rotaryio.IncrementalEncoder(ENC_B, ENC_A)

# PID controller
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

print("READY - Send commands like: SET_POS 90")

serial_buffer = ""
last_pos = None
last_sp = None
last_thr = None

while True:
    if not user_sw.value:
        mot.throttle = 0
        print("STOP")
        break

    # Read serial characters
    while supervisor.runtime.serial_bytes_available:
        c = sys.stdin.read(1)
        if c:
            serial_buffer += c
            if "\n" in serial_buffer:
                lines = serial_buffer.split("\n")
                for line in lines[:-1]:
                    line = line.strip()
                    if line.startswith("SET_POS"):
                        try:
                            _, val = line.split()
                            pos_pid.setpoint = float(val)
                            print(f"OK SET_POS {val}")
                        except:
                            print("ERR")
                serial_buffer = lines[-1]

    # PID control
    pos = to_deg(encoder.position)
    vel = pos_pid.calculate(pos)
    thr = max(min(vel / SPEED_SCALE, 1.0), -1.0)
    mot.throttle = thr

    # Only print if something changed
    if (last_pos != round(pos,2)) or (last_sp != round(pos_pid.setpoint,2)) or (last_thr != round(thr,2)):
        print(f"POS {pos:.2f} SP {pos_pid.setpoint:.2f} THR {thr:.2f}")
        last_pos = round(pos,2)
        last_sp = round(pos_pid.setpoint,2)
        last_thr = round(thr,2)

    time.sleep(UPDATE_RATE)

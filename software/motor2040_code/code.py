# SPDX-License-Identifier: MIT
import time
import board
import pwmio
import digitalio
import rotaryio
import busio
from adafruit_motor import motor

# === Motor + Encoder configuration ===
MOTOR_B_P = board.MOTOR_B_P
MOTOR_B_N = board.MOTOR_B_N
MOTOR_C_P = board.MOTOR_C_P
MOTOR_C_N = board.MOTOR_C_N

ENC_B_A = board.ENCODER_B_A
ENC_B_B = board.ENCODER_B_B
ENC_C_A = board.ENCODER_C_A
ENC_C_B = board.ENCODER_C_B

FREQUENCY = 25000
GEAR_RATIO = 50
COUNTS_PER_REV = 12 * GEAR_RATIO

# PID gains
KP = 0.14
KI = 0.0
KD = 0.0022

UPDATES = 100
UPDATE_RATE = 1 / UPDATES
SPEED_SCALE = 5.4

# --- User stop switch ---
user_sw = digitalio.DigitalInOut(board.USER_SW)
user_sw.direction = digitalio.Direction.INPUT
user_sw.pull = digitalio.Pull.UP

# --- Motor setup helper ---
def make_motor(pin_p, pin_n):
    pwm_p = pwmio.PWMOut(pin_p, frequency=FREQUENCY)
    pwm_n = pwmio.PWMOut(pin_n, frequency=FREQUENCY)
    m = motor.DCMotor(pwm_p, pwm_n)
    m.decay_mode = motor.SLOW_DECAY
    return m

motB = make_motor(MOTOR_B_P, MOTOR_B_N)
motC = make_motor(MOTOR_C_P, MOTOR_C_N)

# --- Encoders ---
encB = rotaryio.IncrementalEncoder(ENC_B_A, ENC_B_B)
encC = rotaryio.IncrementalEncoder(ENC_C_B, ENC_C_A)

# --- UART on GP0/GP1 ---
uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0.01)

# --- PID controller ---
class PID:
    def __init__(self, kp, ki, kd, sample_rate):
        self.kp = kp
        self.ki = ki
        self.kd = kd
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

pidB = PID(KP, KI, KD, UPDATE_RATE)
pidC = PID(KP, KI, KD, UPDATE_RATE)

# --- Helper functions ---
def to_deg(pos):
    return (pos * 360.0) / COUNTS_PER_REV

def safe_uart_write(msg):
    try:
        uart.write((msg + "\n").encode())
    except Exception:
        pass

# --- State tracking ---
last_pos_B = None
last_sp_B = None
last_thr_B = None
last_pos_C = None
last_sp_C = None
last_thr_C = None

print("READY (UART ACTIVE) - Send: SET_POS B 90 or SET_POS C -45")

serial_buffer = ""

# === MAIN LOOP ===
while True:
    # --- Emergency stop ---
    if not user_sw.value:
        motB.throttle = 0
        motC.throttle = 0
        safe_uart_write("STOP")
        print("STOP")
        break

    # --- UART command handling ---
    data = uart.read(1)
    if data:
        try:
            c = data.decode()
        except Exception:
            c = ""
        serial_buffer += c
        if "\n" in serial_buffer:
            lines = serial_buffer.split("\n")
            for line in lines[:-1]:
                line = line.strip()
                if line.startswith("SET_POS"):
                    parts = line.split()
                    if len(parts) == 3:
                        _, motor_id, val = parts
                        try:
                            valf = float(val)
                            if motor_id.upper() == "B":
                                pidB.setpoint = valf
                                safe_uart_write("OK B SET_POS {}".format(val))
                                print("SET_POS B", val)
                            elif motor_id.upper() == "C":
                                pidC.setpoint = valf
                                safe_uart_write("OK C SET_POS {}".format(val))
                                print("SET_POS C", val)
                            else:
                                safe_uart_write("ERR UNKNOWN_MOTOR")
                        except Exception as e:
                            safe_uart_write("ERR {}".format(e))
            serial_buffer = lines[-1]

    # --- PID + motor updates ---
    posB = to_deg(encB.position)
    velB = pidB.calculate(posB)
    thrB = max(min(velB / SPEED_SCALE, 1.0), -1.0)
    motB.throttle = thrB

    posC = to_deg(encC.position)
    velC = pidC.calculate(posC)
    thrC = max(min(velC / SPEED_SCALE, 1.0), -1.0)
    motC.throttle = thrC

    # --- Only report when values change ---
    rpB = round(posB, 2)
    rspB = round(pidB.setpoint, 2)
    rthrB = round(thrB, 2)
    rpC = round(posC, 2)
    rspC = round(pidC.setpoint, 2)
    rthrC = round(thrC, 2)

    if (rpB != last_pos_B) or (rspB != last_sp_B) or (rthrB != last_thr_B):
        msg = "POS_B {:.2f} SP_B {:.2f} THR_B {:.2f}".format(rpB, rspB, rthrB)
        safe_uart_write(msg)
        print(msg)
        last_pos_B, last_sp_B, last_thr_B = rpB, rspB, rthrB

    if (rpC != last_pos_C) or (rspC != last_sp_C) or (rthrC != last_thr_C):
        msg = "POS_C {:.2f} SP_C {:.2f} THR_C {:.2f}".format(rpC, rspC, rthrC)
        safe_uart_write(msg)
        print(msg)
        last_pos_C, last_sp_C, last_thr_C = rpC, rspC, rthrC

    time.sleep(UPDATE_RATE)

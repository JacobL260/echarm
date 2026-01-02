# control.py
import time
import threading
import math

from config import *
from hardware import Stepper, adc_volt, adc_lock, stop_event

# ============================================================
# SHARED STATE
# ============================================================
robot_state = {
    "joint_angles": [0.0]*NUM_AXES,  # radians
    "target_q": None
}
state_lock = threading.Lock()

# ============================================================
# UTILS
# ============================================================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def volt_to_joint_angle(v, idx):
    """
    Convert ADC voltage to joint angle (degrees) for actuator idx.
    Uses the simple linear scaling:
        joint_angle_deg = v / VREF * POT_MAX_DEG * ACT_TO_POT_RATIO[idx]
    """
    return (v / VREF) * POT_MAX_DEG * ACT_TO_POT_RATIO[idx]

def volts_to_joint_angles(volts):
    return [volt_to_joint_angle(v, i) for i, v in enumerate(volts)]

# ============================================================
# PID CONTROLLER
# ============================================================

class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0
        self.last_e = 0
        self.last_t = None
        self.i_limit = 100

    def compute(self, cmd, fb):
        now = time.time()
        dt = 1e-3 if self.last_t is None else max(now - self.last_t, 1e-6)
        e = cmd - fb
        self.i = max(-self.i_limit, min(self.i_limit, self.i + e * dt))
        d = (e - self.last_e) / dt
        self.last_e, self.last_t = e, now
        return self.kp*e + self.ki*self.i + self.kd*d

# ============================================================
# ACTUATOR THREAD
# ============================================================

class Actuator(threading.Thread):
    def __init__(self, idx):
        super().__init__(daemon=True)
        self.idx = idx
        self.pid = PID(**PID_PARAMS[idx])
        self.stepper = Stepper(idx)

        self.cmd_deg = 0.0
        self.dt = 1.0 / CTRL_HZ
        self.stepper.start()

    def set_command(self, deg):
        lim = ACT_SOFT_LIMITS[self.idx]
        self.cmd_deg = clamp(deg, lim["min"], lim["max"])

        if deg != self.cmd_deg:
            print(f"Actuator {self.idx} command {deg:.1f}° out of limits, clamped to {self.cmd_deg:.1f}°")

    def run(self):
        while not stop_event.is_set():
            with adc_lock:
                v = adc_volt[self.idx]

            # Convert voltage → joint angle (degrees)
            fb_deg = volt_to_joint_angle(v, self.idx)

            # PID computes velocity command in deg/s
            vel_deg = self.pid.compute(self.cmd_deg, fb_deg)

            # Convert joint velocity → motor steps/s
            vel_steps = vel_deg * ACT_TO_MOTOR_RATIO[self.idx]

            self.stepper.set_velocity(vel_steps)
            time.sleep(self.dt)

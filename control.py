# control.py
import time
import threading

from config import *
from hardware import Stepper, stop_event

class PID:
    def __init__(self, kp, ki, kd):
        """
        Docstring for __init__
        
        :param self: Description
        :param kp: Description
        :param ki: Description
        :param kd: Description
        """
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


class Actuator(threading.Thread):
    def __init__(self, idx, adc_reader):
        super().__init__(daemon=True)
        self.idx = idx
        self.pid = PID(**PID_PARAMS[idx])
        self.stepper = Stepper(idx)

        self.pos_cmd = 0
        self.vel_cmd_input = None

        self.dt = 1.0 / CTRL_HZ
        self.fb = 0
        self.buffer = {"pos_cmd": 0.0, "vel_cmd": 0.0, "fb": 0.0, "vel": 0.0}
        self.lock = threading.Lock()
        self.adc_reader = adc_reader
        self.stepper.start()

    # --------------------------
    # Thread loop
    # --------------------------
    def run(self):
        while not stop_event.is_set():
            # Read feedback from ADC
            with self.adc_reader.lock:
                v = self.adc_reader.volt[self.idx]

            # Feedback in degrees
            self.fb = (v / VREF) * POT_MAX_DEG * ACT_TO_POT_RATIO[self.idx]

            # Compute motor velocity
            if self.vel_cmd_input is not None:
                # External velocity command
                motor_vel = self.vel_cmd_input * ACT_TO_MOTOR_RATIO[self.idx]
                vel_cmd_to_buffer = self.vel_cmd_input
            else:
                # PID computes velocity for position control
                motor_vel = self.pid.compute(self.pos_cmd, self.fb) * ACT_TO_MOTOR_RATIO[self.idx]
                vel_cmd_to_buffer = motor_vel / ACT_TO_MOTOR_RATIO[self.idx]

            # Apply velocity to stepper
            self.stepper.set_velocity(motor_vel)

            # Update buffer
            with self.lock:
                self.buffer["pos_cmd"] = self.pos_cmd
                self.buffer["vel_cmd"] = vel_cmd_to_buffer
                self.buffer["fb"] = self.fb
                self.buffer["vel"] = self.stepper.velocity

            time.sleep(self.dt)

    # --------------------------
    # Set position command
    # --------------------------
    def set_position(self, deg):
        lim = ACT_SOFT_LIMITS[self.idx]
        self.pos_cmd = max(lim["min"], min(lim["max"], deg))
        if deg > lim["max"] or deg < lim["min"]:
            print(f"Actuator {self.idx} position command {deg}° out of limits, modified to {self.pos_cmd}°")
        self.vel_cmd_input = None  # PID will generate vel_cmd

    # --------------------------
    # Set velocity command directly
    # --------------------------
    def set_velocity(self, vel):
        self.vel_cmd_input = vel  # bypass PID
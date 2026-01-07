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
        """
        Docstring for __init__
        
        :param self: Description
        :param idx: Description
        :param adc_reader: Description
        """
        super().__init__(daemon=True)
        self.idx = idx
        self.pid = PID(**PID_PARAMS[idx])
        self.stepper = Stepper(idx)
        self.cmd_deg = 0
        self.dt = 1.0 / CTRL_HZ
        self.fb = 0
        self.buffer = {"cmd": 0.0, "fb": 0.0, "vel": 0.0}
        self.lock = threading.Lock()
        self.adc_reader = adc_reader
        self.stepper.start()

    def set_command(self, deg):
        lim = ACT_SOFT_LIMITS[self.idx]
        self.cmd_deg = max(lim["min"], min(lim["max"], deg))
        if deg > lim["max"] or deg < lim["min"]:
            print(f"Actuator {self.idx} command {deg}° out of limits, modified to {self.cmd_deg}°")

    def run(self):
        while not stop_event.is_set():
            with self.adc_reader.lock:
                v = self.adc_reader.volt[self.idx]

            self.fb = (v / VREF) * POT_MAX_DEG * ACT_TO_POT_RATIO[self.idx]

            vel = self.pid.compute(self.cmd_deg, self.fb)
            self.stepper.set_velocity(vel * ACT_TO_MOTOR_RATIO[self.idx])

            with self.lock:
                self.buffer["fb"] = self.fb
                self.buffer["cmd"] = self.cmd_deg
                self.buffer["vel"] = self.stepper.velocity

            time.sleep(self.dt)
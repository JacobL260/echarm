import time
import threading
import math
import random

from config import *
from utils import is_raspberry_pi

ON_PI = is_raspberry_pi()

if ON_PI:
    from gpiozero import Button as GPIOButton
    print("Running on a RASP PI")
else:
    print("Running in LAPTOP MODE")

# SHARED STATE
stop_event = threading.Event()

class ADCReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.dt = 1.0 / ADC_HZ
        self.volt = [0.0] * NUM_AXES
        self.lock = threading.Lock()
        self._simulate = False

        if ON_PI:
            try:
                import board
                import busio
                from adafruit_ads1x15.ads1115 import ADS1115
                from adafruit_ads1x15.analog_in import AnalogIn

                # Initialize I2C and ADS1115 boards
                i2c = busio.I2C(board.SCL, board.SDA)
                self.ads_list = [ADS1115(i2c, address=addr) for addr in ADS_ADDRESSES]
                for ads in self.ads_list:
                    ads.gain = 1  # ±4.096V

                # Prepare AnalogIn channels based on mapping
                self.channels = [
                    AnalogIn(self.ads_list[ads_idx], channel)
                    for ads_idx, channel in ADC_CHANNEL_MAP
                ]

            except Exception as e:
                print("ADC init failed, using simulation:", e)
                self._simulate = True
                self.t0 = time.time()
        else:
            # Simulate if not on Raspberry Pi
            self._simulate = True
            self.t0 = time.time()

    def run(self):
        while not stop_event.is_set():
            with self.lock:
                if ON_PI and not self._simulate:
                    for i, ch in enumerate(self.channels):
                        self.volt[i] = ch.voltage
                else:
                    t = time.time() - self.t0
                    for i in range(NUM_AXES):
                        self.volt[i] = (math.sin(t * 0.5 + i) * 0.5 + 0.5) * VREF
            time.sleep(self.dt)

    def is_simulating(self):
        return self._simulate


class Stepper:
    """Stepper motor controlled via TB6600 using STEP/DIR with velocity control"""

    MAX_SPEED = 1000.0  # steps per second

    def __init__(self, idx, pins=None):
        self.idx = idx
        self.pos_steps = 0.0
        self.velocity = 0.0
        self.running = False

        if pins is None:
            pins = STEPPER_PINS[idx]

        self.step_pin, self.dir_pin = pins

        if ON_PI:
            from gpiozero import OutputDevice
            self.step = OutputDevice(self.step_pin)
            self.dir = OutputDevice(self.dir_pin)
        else:
            self.step = None
            self.dir = None

    def set_velocity(self, steps_per_sec):
        """Set velocity in steps/sec, clamped to ±MAX_SPEED."""
        v = max(-self.MAX_SPEED, min(self.MAX_SPEED, float(steps_per_sec)))
        self.velocity = v

        if ON_PI:
            if self.dir:
                self.dir.value = 1 if v > 0 else 0

    def start(self):
        """Start the velocity control thread."""
        if not self.running and not stop_event.is_set():
            self.running = True
            self._thread = threading.Thread(target=self._run_velocity, daemon=True)
            self._thread.start()

    def stop_velocity(self):
        self.velocity = 0.0

    def _run_velocity(self):
        while self.running and not stop_event.is_set():
            v = self.velocity
            if v == 0:
                time.sleep(0.01)
                continue

            delay = 1.0 / abs(v)
            direction = 1 if v > 0 else -1

            if ON_PI and self.step:
                self.step.on()
                time.sleep(delay / 2)
                self.step.off()
                time.sleep(delay / 2)
            else:
                time.sleep(delay)

            self.pos_steps += direction

class Button(threading.Thread):
    """Button that updates its state directly in robot_state."""

    def __init__(self, pin=None, idx=0):
        super().__init__(daemon=True)
        self.pin = pin
        self.dt = 1.0 / BUTTON_HZ
        self._simulate = False
        self._level = 0
        self._prev_level = 0
        self.idx = idx
        self.buffer = {"pressed": False, "was_pressed": False, "was_released": False}
        self.lock = threading.Lock()

        if ON_PI:
            try:
                self.hw_button = GPIOButton(self.pin, pull_up=True)
            except Exception as e:
                print(f"Button {self.idx} init failed, using simulation:", e)
                self._simulate = True
                self.t0 = time.time()
        else:
            self._simulate = True
            self.t0 = time.time()

        # Simulation state
        self._sim_next_change = time.time() + random.uniform(1.0, 3.0)
        self._sim_pressed_duration = 1  # seconds

    def run(self):
        while not stop_event.is_set():
            self._prev_level = self._level

            if ON_PI and not self._simulate:
                self._level = int(self.hw_button.is_pressed)
            else:
                now = time.time()
                if self._level == 0 and now >= self._sim_next_change:
                    self._level = 1
                    self._sim_next_change = now + self._sim_pressed_duration
                elif self._level == 1 and now >= self._sim_next_change:
                    self._level = 0
                    self._sim_next_change = now + random.uniform(1.0, 3.0)

            with self.lock:
                self.buffer["pressed"] = self._level == 1
                self.buffer["was_pressed"] = self._level == 1 and self._prev_level == 0
                self.buffer["was_released"] = self._level == 0 and self._prev_level == 1

            time.sleep(self.dt)

    def is_pressed(self):
        return self._level == 1

    def is_released(self):
        return self._level == 0

    def is_simulating(self):
        return self._simulate
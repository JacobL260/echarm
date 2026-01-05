import time
import threading
import math
import random

from config import *
from utils import is_raspberry_pi

ON_PI = is_raspberry_pi()

if ON_PI:
    import board, busio, pigpio
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_ads1x15.analog_in import AnalogIn
    print("Running on a RASP PI")
else:
    print("Running in LAPTOP MODE")

# SHARED STATE
stop_event = threading.Event()

class ADCReader(threading.Thread):
    """Thread that reads ADC voltages at regular intervals into shared adc_volt array."""
    def __init__(self, ads=None, src=None):
        super().__init__(daemon=True)
        self.dt = 1.0 / ADC_HZ
        self.ads = ads
        self.src = src
        self._simulate = False
        self.volt = [0.0] * NUM_AXES
        self.lock = threading.Lock()

        if ON_PI and self.ads is None:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)

                # Create ADS instances from config
                self.ads = [ADS1115(i2c, addr) for addr in ADS_ADDRESSES]

                # Create AnalogIn channels from config map
                self.src = [
                    AnalogIn(self.ads[a], getattr(ADS1115, ch))
                    for a, ch in ADC_CHANNEL_MAP
                ]

            except Exception as e:
                print("ADC init failed, using simulation:", e)
                self._simulate = True
                self.t0 = time.time()

        elif not ON_PI:
            self._simulate = True
            self.t0 = time.time()

    def run(self):
        """Read ADC voltages into shared adc_volt array at regular intervals."""
        while not stop_event.is_set():
            with self.lock:
                if ON_PI and self.src:
                    for i, s in enumerate(self.src):
                        self.volt[i] = s.voltage
                else:
                    t = time.time() - self.t0
                    for i in range(NUM_AXES):
                        self.volt[i] = (math.sin(t * 0.5 + i) * 0.5 + 0.5) * VREF  # simulate voltages
            time.sleep(self.dt)

    def is_simulating(self):
        """True if ADCReader is using the simulator instead of real ADC hardware."""
        return getattr(self, "_simulate", True)

class Stepper:
    """Stepper motor controlled via TB6600 using STEP/DIR with velocity control"""
    MAX_SPEED = 1000.0  # steps per second

    def __init__(self, idx, pins=None):
        self.idx = idx
        self.pos_steps = 0.0 # current position in steps
        self.velocity = 0.0 # steps per second
        self.running = False

        if pins is None:
            pins = STEPPER_PINS[idx]

        self.step_pin, self.dir_pin = pins

        if ON_PI:
            self.pi = pigpio.pi()
            self.pi.set_mode(self.step_pin, pigpio.OUTPUT)
            self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)
        else:
            self.pi = None

    def set_velocity(self, steps_per_sec):
        """
        Set velocity in steps/second:
        "+" = forward, "-" = reverse, 0 = stop Velocity is clamped to ±MAX_SPEED.
        """
        # clamp velocity
        v = max(-self.MAX_SPEED, min(self.MAX_SPEED, float(steps_per_sec)))
        self.velocity = v

        if ON_PI and self.pi:
            self.pi.write(self.dir, 1 if v > 0 else 0)

    def start(self):
        """Start the velocity control thread."""
        if not self.running:
            self.running = True
            self._thread = threading.Thread(target=self._run_velocity, daemon=True)
            self._thread.start()

    def stop_velocity(self):
        self.velocity = 0.0

    def _run_velocity(self):
        while self.running:
            v = self.velocity
            if v == 0:
                time.sleep(0.01)
                continue

            delay = 1.0 / abs(v)
            direction = 1 if v > 0 else -1

            if ON_PI and self.pi:
                self.pi.write(self.step, 1)
                time.sleep(delay / 2)
                self.pi.write(self.step, 0)
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
        self.buffer = {"pressed": False, "was_pressed": False, "was_release": False}
        self.lock = threading.Lock()

        # Hardware setup
        if ON_PI:
            try:
                self.pi = pigpio.pi()
                self.pi.set_mode(self.pin, pigpio.INPUT)
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

            # Read level
            if ON_PI and not self._simulate:
                self._level = self.pi.read(self.pin)
            else:
                # Simulate press/release
                now = time.time()
                if self._level == 0 and now >= self._sim_next_change:
                    self._level = 1
                    self._sim_next_change = now + self._sim_pressed_duration
                elif self._level == 1 and now >= self._sim_next_change:
                    self._level = 0
                    self._sim_next_change = now + random.uniform(1.0, 3.0)

            # Edge detection and update robot_state
            with self.lock:
                self.buffer["pressed"] = self._level == 1
                self.buffer["was_pressed"] = self._level == 1 and self._prev_level == 0
                self.buffer["was_released"] = self._level == 0 and self._prev_level == 1

            time.sleep(self.dt)

    def is_pressed(self):
        """Return current logical state of the button."""
        return self._level == 1

    def is_released(self):
        """Return current logical state of the button."""
        return self._level == 0

    def is_simulating(self):
        return self._simulate
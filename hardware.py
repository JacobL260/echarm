import time
import threading
import math
import random

from config import *
from utils import is_raspberry_pi

ON_PI = is_raspberry_pi()

if ON_PI:
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
        self.t0 = None
        self.adc_available = False

        # Decide mode strictly
        if ADC_MODE == "simulation":
            print("ADC MODE: SIMULATION")
            self._simulate = True
            self.t0 = time.time()

        elif ADC_MODE == "hardware":
            print("ADC MODE: HARDWARE")
            self._simulate = False
            self.adc_available = True

            try:
                self._init_hardware()
            except Exception as e:
                print("⚠ ADC hardware init failed.")
                print("⚠ Positional control NOT available.")
                print(f"⚠ Error: {e}")

                self.adc_available = False
                self._simulate = False  # no sim fallback

        else:
            raise ValueError(f"Invalid ADC_MODE: {ADC_MODE}")

    # --------------------------
    # Hardware Init
    # --------------------------
    def _init_hardware(self):
        try:
            import board
            import busio
            from adafruit_ads1x15.ads1115 import ADS1115
            from adafruit_ads1x15.analog_in import AnalogIn

            i2c = busio.I2C(board.SCL, board.SDA)

            self.ads_list = [ADS1115(i2c, address=addr) for addr in ADS_ADDRESSES]
            for ads in self.ads_list:
                ads.gain = 1  # ±4.096V

            self.channels = [
                AnalogIn(self.ads_list[ads_idx], channel)
                for ads_idx, channel in ADC_CHANNEL_MAP
            ]
        except Exception as e:
            raise RuntimeError("ADC hardware initialization failed") from e

    # --------------------------
    # Thread Loop
    # --------------------------
    def run(self):
        while not stop_event.is_set():
            if self._simulate:
                values = self._read_sim()
            else:
                values = self._read_hardware()

            with self.lock:
                self.volt = values

            time.sleep(self.dt)

    # --------------------------
    # Read Methods
    # --------------------------
    def _read_sim(self):
        t = time.time() - self.t0
        return [(math.sin(t * 0.5 + i) * 0.5 + 0.5) * VREF for i in range(NUM_AXES)]

    def _read_hardware(self):
        if not self.adc_available:
            return self.volt  # return last known values (zeros)

        return [ch.voltage for ch in self.channels]

class Stepper:
    """Stepper motor controlled via TB6600 using STEP/DIR with velocity control"""

    MAX_SPEED = 1000.0  # steps per second

    def __init__(self, idx, pins=None):
        self.idx = idx
        self.pos_steps = 0.0
        self.velocity = 0.0
        self.running = False
        self._thread = None

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

        if ON_PI and self.dir:
            self.dir.value = 1 if v > 0 else 0

    def start(self):
        """Start the velocity control thread."""
        if not self.running and not stop_event.is_set():
            self.running = True
            self._thread = threading.Thread(target=self._run_velocity, daemon=True)
            self._thread.start()

    def stop(self):
        """Stop the motor completely."""
        self.running = False
        self.velocity = 0.0
        if self._thread:
            self._thread.join(timeout=0.5)
            self._thread = None

    def stop_velocity(self):
        """Stop motor movement but keep thread alive."""
        self.velocity = 0.0

    def _run_velocity(self):
        """Thread loop to generate step pulses according to velocity."""
        last_time = time.perf_counter()
        while self.running and not stop_event.is_set():
            v = abs(self.velocity)
            if v > 0:
                step_interval = 1.0 / v  # seconds per step
                now = time.perf_counter()
                if now - last_time >= step_interval:
                    last_time = now
                    if ON_PI and self.step:
                        # generate a short STEP pulse
                        self.step.on()
                        self.step.off()
                    # update logical position
                    self.pos_steps += 1 if self.velocity > 0 else -1
            else:
                # Sleep a tiny bit to avoid busy waiting when velocity=0
                time.sleep(0.001)

class Button(threading.Thread):
    """Button that updates its state, with optional simulation."""

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

        if BUTTON_MODE == "simulation":
            print(f"Button {self.idx} MODE: SIMULATION")
            self._simulate = True
            self.t0 = time.time()

        elif BUTTON_MODE == "hardware":
            print(f"Button {self.idx} MODE: HARDWARE")
            self._simulate = False
            self._init_hardware()
        else:
            raise ValueError(f"Invalid BUTTON_MODE: {BUTTON_MODE}")

        # Simulation state
        self._sim_next_change = time.time() + random.uniform(1.0, 3.0)
        self._sim_pressed_duration = 1  # seconds

    # --------------------------
    # Hardware init
    # --------------------------
    def _init_hardware(self):
        try:
            from gpiozero import Button as GPIOButton
            self.hw_button = GPIOButton(self.pin, pull_up=True)
        except Exception as e:
            raise RuntimeError(f"Button {self.idx} hardware initialization failed") from e

    # --------------------------
    # Thread loop
    # --------------------------
    def run(self):
        while not stop_event.is_set():
            self._prev_level = self._level

            if self._simulate:
                now = time.time()
                if self._level == 0 and now >= self._sim_next_change:
                    self._level = 1
                    self._sim_next_change = now + self._sim_pressed_duration
                elif self._level == 1 and now >= self._sim_next_change:
                    self._level = 0
                    self._sim_next_change = now + random.uniform(1.0, 3.0)
            else:
                self._level = int(self.hw_button.is_pressed)

            # Update buffer safely
            with self.lock:
                self.buffer["pressed"] = self._level == 1
                self.buffer["was_pressed"] = self._level == 1 and self._prev_level == 0
                self.buffer["was_released"] = self._level == 0 and self._prev_level == 1

            time.sleep(self.dt)

    # --------------------------
    # State accessors
    # --------------------------
    def is_pressed(self):
        return self._level == 1

    def is_released(self):
        return self._level == 0

    def is_simulating(self):
        return self._simulate
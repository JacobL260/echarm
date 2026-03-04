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
    """TB6600 stepper driver using pigpio waveforms (any GPIO pin)"""

    MAX_SPEED = 1000  # steps per second

    def __init__(self, idx, pins=None):
        self.idx = idx
        self.velocity = 0
        self.wave_id = None

        if pins is None:
            pins = STEPPER_PINS[idx]

        self.step_pin, self.dir_pin = pins

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running")

        self.pi.set_mode(self.step_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)

    def set_velocity(self, steps_per_sec):
        """Set constant velocity in steps/sec"""

        v = max(-self.MAX_SPEED, min(self.MAX_SPEED, float(steps_per_sec)))
        self.velocity = v

        # Stop existing waveform
        self.stop()

        if v == 0:
            return

        # Set direction
        self.pi.write(self.dir_pin, 1 if v > 0 else 0)

        freq = abs(v)
        period_us = int(1_000_000 / freq)
        half_period = period_us // 2

        pulses = [
            pigpio.pulse(1 << self.step_pin, 0, half_period),
            pigpio.pulse(0, 1 << self.step_pin, half_period)
        ]

        self.pi.wave_add_generic(pulses)
        self.wave_id = self.pi.wave_create()

        self.pi.wave_send_repeat(self.wave_id)

    def stop(self):
        """Stop motor"""
        self.pi.wave_tx_stop()

        if self.wave_id is not None:
            self.pi.wave_delete(self.wave_id)
            self.wave_id = None

    def cleanup(self):
        self.stop()
        self.pi.stop()

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
import time
import threading
import math
import sys

# Check is running on pi vs laptop
def is_raspberry_pi():
    try:
        with open('/proc/device-tree/model') as f:
            return 'Raspberry Pi' in f.read()
    except:
        return False

ON_PI = is_raspberry_pi()

# If on pi, load the corresponding packages
if ON_PI:
    import board, busio, pigpio
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_ads1x15.analog_in import AnalogIn
else:
    print("Running in LAPTOP MODE")

# BIG IDEA NUMBERS
NUM_AXES = 6
VREF = 3.3
POT_MAX_DEG = 270
ADC_HZ = 200
CTRL_HZ = 50

# INDIVIDUAL AXES PARAMTERS
POT_ZERO_OFFSETS = [0.0] * NUM_AXES
ACT_SOFT_LIMITS = [
    {"min": -60, "max": 60},
    {"min": -60, "max": 60},
    {"min": -60, "max": 60},
    {"min": -60, "max": 60},
    {"min": -60, "max": 60},
    {"min": -60, "max": 60}
] # based off mechanical design, needs validated

# Different potiometer setups require different direction signs, geared makes negative and belts makes positive
ACT_TO_POT_RATIO = [-13, -13, -13, -13, -13, -13] # based off mechanical design, needs validated

# Some actuators need to have their stepper direction inverted based off mechanical gearings
ACT_TO_MOTOR_RATIO = [180, -180, -180, -180, -180, -180]  # based off mechanical design, needs validated

PID_PARAMS = [
    {"kp": 0.6, "ki": 0.05, "kd": 0.0},
    {"kp": 0.6, "ki": 0.05, "kd": 0.0},
    {"kp": 0.6, "ki": 0.05, "kd": 0.0},
    {"kp": 0.6, "ki": 0.05, "kd": 0.0},
    {"kp": 0.6, "ki": 0.05, "kd": 0.0},
    {"kp": 0.6, "ki": 0.05, "kd": 0.0}
]

# SHARED STATE
adc_volt = [0.0] * NUM_AXES
adc_lock = threading.Lock()
stop_event = threading.Event()


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0.0
        self.last_e = 0.0
        self.last_t = None
        # anti-windup limit (degrees * seconds)
        self.i_limit = 100.0

    def compute(self, cmd, fb):
        now = time.time()
        dt = 1e-3 if self.last_t is None else max(now - self.last_t, 1e-6)
        e = cmd - fb
        self.i += e * dt
        # clamp integral (anti-windup)
        if self.i > self.i_limit:
            self.i = self.i_limit
        elif self.i < -self.i_limit:
            self.i = -self.i_limit
        d = (e - self.last_e) / dt
        self.last_e = e
        self.last_t = now
        out = self.kp * e + self.ki * self.i + self.kd * d
        return out


class ADCReader(threading.Thread):
    """Thread that reads ADC voltages at regular intervals into shared adc_volt array."""
    def __init__(self,):
        super().__init__(daemon=True)
        self.dt = 1.0 / ADC_HZ
        if ON_PI:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.ads = [
                    ADS1115(i2c, address=0x48),
                    ADS1115(i2c, address=0x49)
                ]
                # create AnalogIn channels using ADS1115 channel enums
                self.src = [
                    AnalogIn(self.ads[0], ADS1115.P0),
                    AnalogIn(self.ads[0], ADS1115.P1),
                    AnalogIn(self.ads[0], ADS1115.P2),
                    AnalogIn(self.ads[1], ADS1115.P0),
                    AnalogIn(self.ads[1], ADS1115.P1),
                    AnalogIn(self.ads[1], ADS1115.P2),
                ]
            except Exception as e:
                print("ADC init failed, falling back to simulation:", e)
                self.ads = None
                self.src = None
                self.t0 = time.time()
                # treat as not on pi for ADC reads
                self._simulate = True
        else:
            self.t0 = time.time()
            self._simulate = True

    def run(self):
        """Read ADC voltages into shared adc_volt array at regular intervals."""
        while not stop_event.is_set():
            with adc_lock:
                if ON_PI and self.src:
                    for i, s in enumerate(self.src):
                        # write into shared adc_volt array
                        adc_volt[i] = s.voltage
                else:
                    t = time.time() - self.t0
                    for i in range(NUM_AXES):
                        adc_volt[i] = (math.sin(t * 0.5 + i) * 0.5 + 0.5) * VREF  # simulate voltages
            time.sleep(self.dt)

    def is_simulating(self):
        """True if ADCReader is using the simulator instead of real ADC hardware."""
        return getattr(self, "_simulate", True)

class Stepper:
    """Stepper motor controlled via TB6600 using STEP/DIR with velocity control"""
    MAX_SPEED = 1000.0  # steps per second

    def __init__(self, idx):
        self.idx = idx
        self.pos_steps = 0.0 # current position in steps

        self.velocity = 0.0 # steps per second
        self.running = False
        self._thread = None

        if ON_PI:
            self.pi = pigpio.pi()
            pins = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10), (11, 12)][idx]
            self.step, self.dir = pins
            self.pi.set_mode(self.step, pigpio.OUTPUT)
            self.pi.set_mode(self.dir, pigpio.OUTPUT)
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


class Actuator(threading.Thread):
    def __init__(self, idx):
        super().__init__(daemon=True)
        self.idx = idx
        self.stepper = Stepper(idx)
        self.pid = PID(**PID_PARAMS[idx])
        self.cmd_deg = 0.0
        self.target_steps = 0.0
        self.dt = 1.0 / CTRL_HZ
        # limit how many steps actuator will hold as target (prevent runaway) or so say CHATGPT
        self.max_target_steps = 1000000

    def set_command(self, deg):
        """Set desired position command of the actuator in degrees, enforcing soft limits."""
        lim = ACT_SOFT_LIMITS[self.idx]
        if deg > lim["max"]:
            # Nested IF loops checks whether the actuator is already commanded to the limit for debugging print
            if self.cmd_deg != lim["max"]:
                print(f"A{self.idx}: Command {deg:.1f}° exceeds limit, moving to +{lim['max']:.1f}°")
            self.cmd_deg = lim["max"]
        elif deg < lim["min"]:
            if self.cmd_deg != lim["min"]:
                print(f"A{self.idx}: Command {deg:.1f}° exceeds limit, moving to {lim['min']:.1f}°")
            self.cmd_deg = lim["min"]
        else:
            self.cmd_deg = deg

    def run(self):
        """Control loop thread for the actuator."""
        while not stop_event.is_set():
            with adc_lock:
                v = adc_volt[self.idx]
            fb_deg = (v / VREF) * POT_MAX_DEG * ACT_TO_POT_RATIO[self.idx] # Position of actuator in degrees NEEDS TO INCORPORATE OFFSETS
            pid_velocity = self.pid.compute(self.cmd_deg, fb_deg) # PID output in actuator degrees per second
            self.stepper.set_velocity(pid_velocity  * ACT_TO_MOTOR_RATIO[self.idx] * ACT_TO_MOTOR_RATIO[self.idx]) # Stepper velocity in steps per second
            time.sleep(self.dt)

    def stop(self):
        self.stepper.stop()


def main():
    reader = ADCReader()
    reader.start()
    acts = [Actuator(i) for i in range(NUM_AXES)]
    for a in acts:
        a.start()

    # set commands once and hold them constant
    INIT_COMMANDS = [10.0, 0.0, -10.0, 20.0, -20.0, 5.0]  # adjust as you like
    for i, a in enumerate(acts):
        a.set_command(INIT_COMMANDS[i] if i < len(INIT_COMMANDS) else 0.0)

    last_print = 0.0
    try:
        print("Press CTRL+C to exit")
        while True:
            # print simulated volts and current commands every 0.5s
            if (time.time() - last_print) >= 0.5:
                with adc_lock:
                    volts = list(adc_volt)
                cmds = [a.cmd_deg for a in acts]
                print("Simulation:     ","SIMULATED" if reader.is_simulating else "HARDWARE")
                print("volts:          ", " ".join(f"{v:.3f}V" for v in volts))
                print("commands:       ", " ".join(f"{c:.2f}°" for c in cmds))
                last_print = time.time()
            time.sleep(0.05)

    except KeyboardInterrupt:
        stop_event.set()
        time.sleep(0.1) # give threads a moment to finish
        for a in acts:
            a.stop()


if __name__ == "__main__":
    main()
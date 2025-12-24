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
STEPS_PER_DEG = [180, 180, 200, 160, 160, 160] #needs validateed
POT_ZERO_OFFSETS = [150, 147.5, 152, 149, 149, 149] #needs validated
ACT_SOFT_LIMITS = [
    {"min": -60, "max": 60},
    {"min": -60, "max": 60},
    {"min": -60, "max": 60},
    {"min": -60, "max": 60},
    {"min": -60, "max": 60},
    {"min": -60, "max": 60}
]
DIRECTION_SIGN = [1, 1, 1, 1, 1, 1] #needs validated

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
    def compute(self, cmd, fb):
        now = time.time()
        dt = 1e-3 if self.last_t is None else max(now - self.last_t, 1e-6)
        e = cmd -fb
        self.i += e *dt
        d = (e-self.last_e) / dt
        self.last_e = e
        self.last_t = now
        return self.kp*e + self.ki*self.i + self.kd*d

class ADCReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.dt = 1.0 / ADC_HZ
        if ON_PI:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.ads = [
                ADS1115(i2c, address=0x48),
                ADS1115(i2c, address=0x49)
            ]
            self.src = [
                AnalogIn(self.ads[0], ADS11115.P0),
                AnalogIn(self.ads[0], ADS11115.P1),
                AnalogIn(self.ads[0], ADS11115.P2),
                AnalogIn(self.ads[1], ADS11115.P0),
                AnalogIn(self.ads[1], ADS11115.P1),
                AnalogIn(self.ads[1], ADS11115.P2),
            ]
        else:
            self.t0 = time.time()

        def run(self):
            while not stop_event.is_set():
                with adc_lock:
                    if ON_PI:
                        for i, s in enumerate(self.src):
                            adc_volts[i] = s.voltage
                    else:
                        t = time.time() - self.t0
                        for i in range(NUM_AXES):
                            adc_volts[i] = (math.sin(t*0.5+i)*0.5+0.5)*VREF # Generates voltages to simulate
            time.sleep(self.dt)

class Stepper:
    def __init__(self, idx):
        self.idx = idx
        self.pos_steps = 0.0
        if ON_PI:
            self.pi = pigpio.pi()
            pins = [(1,2), (3,4), (5,6), (7,8), (9,10), (11,12)][idx]
            self.step, self.dir = pins
            self.pi.set_mode(self.step, pigpio.OUTPUT)
            self.pi.set_mode(self.dir, pigpio.OUTPUT)
    
    def step_toward(self, target_steps):
        delta = target_steps - self.pos_steps
    # NEED TO RETHINK IF THIS IS HOW I WANT TO DRIVE THE STEPPER MOTOR


    def stop(self):
        pass

class Actuator(threading.Thread):
    def __init__(self, idx):
        super().__init__(daemon=True)
        self.idx = idx
        self.stepper = Stepper(idx)
        self.pid = PID(**PID_PARAMS[idx])
        self.cmd_deg = 0.0
        self.target_steps = 0.0
        self.dt = 1.0 / CTRL_HZ

    def set_command(self, deg):
        lim = ACT_SOFT_LIMITS[self.idx]
        if deg > lim["max"]:
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
            while not stop_event.is_set():
                with adc_lock:
                    v = adc_volts[self.idx]
                fb_deg = (v / VREF) * POT_MAX_DEG - POT_ZERO_OFFSETS[self.idx]
                delta_deg = self.pid.compute(self.cmd_deg, fb_deg)
                self.target_steps += delta_deg * STEPS_PER_DEG[self.idx]
                self.stepper.step_toward(self.target_steps)
                time.sleep(self.dt)

def main():
    ADCReader().start()
    acts = [Actuator(i) for i in range(NUM_AXES)]
    for a in acts:
        a.start()

    t0 = time.time()
    try:
        print("Press CTRL+C to exit")
        while True:
            t = time.time()-t0
            for i, a in enumerate(acts):
                a.set_command(30*math.sin(0.3*t+i))
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop_event.set()

if __name__ == "__main__":
    main()
import time
from gpiozero import OutputDevice

# ======================
# CONFIG — CHANGE THESE
# ======================
STEP_PIN = 23   # GPIO pin connected to STEP+
DIR_PIN  = 24   # GPIO pin connected to DIR+
ENABLE_PIN = None  # Optional: set GPIO number or leave as None

STEPS_PER_SECOND = 500   # Speed (adjust slower/faster)
RUN_TIME = 3.0           # Seconds per direction
# ======================


# GPIO setup
step = OutputDevice(STEP_PIN)
dir = OutputDevice(DIR_PIN)

if ENABLE_PIN is not None:
    enable = OutputDevice(ENABLE_PIN)
    enable.off()  # TB6600 usually enables when LOW
else:
    enable = None


def step_motor(steps, delay):
    for _ in range(steps):
        step.on()
        time.sleep(delay / 2)
        step.off()
        time.sleep(delay / 2)


try:
    print("Stepper test running. Ctrl+C to stop.")

    delay = 1.0 / STEPS_PER_SECOND
    steps = int(STEPS_PER_SECOND * RUN_TIME)

    while True:
        print("Direction: FORWARD")
        dir.on()   # HIGH = forward (depends on wiring)
        step_motor(steps, delay)

        time.sleep(1)

        print("Direction: REVERSE")
        dir.off()  # LOW = reverse
        step_motor(steps, delay)

        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping stepper test.")

finally:
    step.off()
    dir.off()
    if enable:
        enable.on()

import RPi.GPIO as GPIO
import time

# Pin setup
STEP_PIN = 5
DIR_PIN = 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

GPIO.output(DIR_PIN, GPIO.HIGH)  # Set direction

def step_motor(delay, steps):
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)

try:
    while True:
        # Accelerate
        for delay in [0.005, 0.003, 0.002, 0.0015, 0.001]:
            step_motor(delay, 200)

        # Decelerate
        for delay in [0.0015, 0.002, 0.003, 0.005]:
            step_motor(delay, 200)

except KeyboardInterrupt:
    print("Stopping motor")

finally:
    GPIO.cleanup()
import lgpio
import time

# GPIO pin definitions
STEP = 18  # Step pin
DIR = 23   # Direction pin

# Open GPIO chip
h = lgpio.gpiochip_open(0)

# Claim pins as outputs
lgpio.gpio_claim_output(h, STEP)
lgpio.gpio_claim_output(h, DIR)

def stepper_step(delay, steps, direction):
    lgpio.gpio_write(h, DIR, direction)
    
    for _ in range(steps):
        lgpio.gpio_write(h, STEP, 1)
        time.sleep(delay)
        lgpio.gpio_write(h, STEP, 0)
        time.sleep(delay)

try:
    while True:
        stepper_step(0.001, 200000, 1)  # Forward
        time.sleep(1)

        stepper_step(0.001, 200000, 0)  # Reverse
        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    lgpio.gpiochip_close(h)
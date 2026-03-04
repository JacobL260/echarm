import lgpio
import time

STEP_PIN = 5
DIR_PIN = 6

# open gpio chip
h = lgpio.gpiochip_open(0)

# claim pins as outputs
lgpio.gpio_claim_output(h, STEP_PIN)
lgpio.gpio_claim_output(h, DIR_PIN)

# set direction
lgpio.gpio_write(h, DIR_PIN, 1)   # 1 = clockwise, 0 = counterclockwise

steps = 20000000000
delay = 0.01   # controls speed

print("Running stepper...")

for i in range(steps):
    lgpio.gpio_write(h, STEP_PIN, 1)
    time.sleep(delay)
    lgpio.gpio_write(h, STEP_PIN, 0)
    time.sleep(delay)

print("Done")

# close gpio chip
lgpio.gpiochip_close(h)
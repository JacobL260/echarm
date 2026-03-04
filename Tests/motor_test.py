import lgpio
import time
import math

STEP_PIN = 5
DIR_PIN = 6

h = lgpio.gpiochip_open(0)

try:
    lgpio.gpio_claim_output(h, STEP_PIN, 0)
    lgpio.gpio_claim_output(h, DIR_PIN, 0)

    lgpio.gpio_write(h, DIR_PIN, 1)

    while True:
        t = time.time()
        speed = 1000
        delay = 1.0 / abs(speed)

        lgpio.gpio_write(h, STEP_PIN, 1)
        time.sleep(0.001)   # 10µs pulse width
        lgpio.gpio_write(h, STEP_PIN, 0)

        time.sleep(max(delay - 0.001, 0))

except KeyboardInterrupt:
    print("Stopping motor...")

finally:
    lgpio.gpio_write(h, STEP_PIN, 0)
    lgpio.gpiochip_close(h)
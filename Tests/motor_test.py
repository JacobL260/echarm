import lgpio
import time
import math

STEP_PIN = 5
DIR_PIN = 6

# Open GPIO chip (Pi 5 usually uses gpiochip4)
h = lgpio.gpiochip_open(4)

lgpio.gpio_claim_output(h, STEP_PIN)
lgpio.gpio_claim_output(h, DIR_PIN)

try:
    lgpio.gpio_write(h, DIR_PIN, 1)  # Set direction

    while True:
        # Smooth varying speed using sine wave
        t = time.time()
        speed = 200 + 180 * math.sin(t)   # steps/sec (20–380 range approx)
        delay = 1.0 / abs(speed)

        # Step pulse
        lgpio.gpio_write(h, STEP_PIN, 1)
        time.sleep(delay / 2)
        lgpio.gpio_write(h, STEP_PIN, 0)
        time.sleep(delay / 2)

except KeyboardInterrupt:
    print("Stopping motor...")

finally:
    lgpio.gpio_write(h, STEP_PIN, 0)
    lgpio.gpiochip_close(h)
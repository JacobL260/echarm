import lgpio
import time
import atexit

PIN1 = 5
PIN2 = 6

# Open gpiochip (Pi 5 default is 0)
h = lgpio.gpiochip_open(0)

# Ensure clean shutdown
def cleanup():
    try:
        lgpio.gpio_write(h, PIN1, 0)
        lgpio.gpio_write(h, PIN2, 0)
        lgpio.gpiochip_close(h)
    except:
        pass

atexit.register(cleanup)

# Claim pins as outputs (start LOW)
lgpio.gpio_claim_output(h, PIN1, 0)
lgpio.gpio_claim_output(h, PIN2, 0)

try:
    while True:
        print("ON")
        lgpio.gpio_write(h, PIN1, 1)
        lgpio.gpio_write(h, PIN2, 1)
        time.sleep(0.5)

        print("OFF")
        lgpio.gpio_write(h, PIN1, 0)
        lgpio.gpio_write(h, PIN2, 0)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopping test...")
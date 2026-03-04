import lgpio
from time import sleep

# Direction pin from controller
DIR = 6

# Step pin from controller
STEP = 5

# 0/1 used to signify clockwise or counterclockwise
CW = 1
CCW = 0

steps = 200        # 200 steps = 360 degrees (depends on driver)
speed = 0.0005     # delay between steps

# Open GPIO chip
h = lgpio.gpiochip_open(0)

# Claim pins as outputs
lgpio.gpio_claim_output(h, DIR)
lgpio.gpio_claim_output(h, STEP)

# Set initial direction
lgpio.gpio_write(h, DIR, CW)

try:
    # Run forever
    while True:

        """
        Change Direction: Changing direction requires time to switch.
        The time is dictated by the stepper motor and controller.
        """

        sleep(1.0)

        # Set clockwise direction
        lgpio.gpio_write(h, DIR, CW)

        # Run for 200 steps
        for x in range(steps):

            # Set step high
            lgpio.gpio_write(h, STEP, 1)

            # Motor speed control
            sleep(speed)

            # Set step low
            lgpio.gpio_write(h, STEP, 0)

            sleep(speed)

        sleep(1.0)

        # Change direction
        lgpio.gpio_write(h, DIR, CCW)

        for x in range(steps):

            lgpio.gpio_write(h, STEP, 1)
            sleep(speed)

            lgpio.gpio_write(h, STEP, 0)
            sleep(speed)

# Cleanup on CTRL+C
except KeyboardInterrupt:
    print("cleanup")
    lgpio.gpiochip_close(h)
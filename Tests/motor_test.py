import lgpio
import time

# --- Configuration ---
PUL_PIN = 5  # Step pin
DIR_PIN = 6  # Direction pin

# Speed settings (smaller delay = faster speed)
STEP_DELAY = 0.0005 

# Open the gpiochip (Pi 5 usually uses chip 0)
h = lgpio.gpiochip_open(0)

# Setup pins as outputs
lgpio.gpio_claim_output(h, PUL_PIN)
lgpio.gpio_claim_output(h, DIR_PIN)

def move_stepper(steps, direction, delay):
    # Set Direction: 1 for CW, 0 for CCW
    lgpio.gpio_write(h, DIR_PIN, direction)
    
    
    for _ in range(steps):
        lgpio.gpio_write(h, PUL_PIN, 1)
        time.sleep(delay)
        lgpio.gpio_write(h, PUL_PIN, 0)
        time.sleep(delay)
    
    # Disable driver to save power/heat (optional)

try:
    print("Moving Forward...")
    move_stepper(800, 1, STEP_DELAY) # 800 steps (assuming 1/4 microstepping)
    
    time.sleep(1)
    
    print("Moving Backward...")
    move_stepper(800, 0, STEP_DELAY)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    # Cleanup
    lgpio.gpiochip_close(h)
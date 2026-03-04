import lgpio
import time

# --- Configuration ---
STEP_PIN = 5  
DIR_PIN  = 6  

# Open the GPIO chip
h = lgpio.gpiochip_open(0)

# Setup pins
lgpio.gpio_claim_output(h, STEP_PIN)
lgpio.gpio_claim_output(h, DIR_PIN)

def pulse(delay):
    """Sends a single pulse to the TB6600"""
    lgpio.gpio_write(h, STEP_PIN, 1)
    time.sleep(0.00002) # 10 microsecond pulse
    lgpio.gpio_write(h, STEP_PIN, 0)
    time.sleep(delay)

try:
    # Set direction (1 = CW, 0 = CCW)
    lgpio.gpio_write(h, DIR_PIN, 1)
    
    print("Starting motor... Press Ctrl+C to stop.")
    
    # Initial slow delay (seconds)
    current_delay = 0.002 
    
    while True:
        # --- Speed Up Phase ---
        print("Accelerating...")
        for _ in range(2000):
            pulse(current_delay)
            if current_delay > 0.0003:  # Don't go faster than this limit
                current_delay -= 0.000001 # Slightly decrease delay (increase speed)

        # --- Slow Down Phase ---
        print("Decelerating...")
        for _ in range(2000):
            pulse(current_delay)
            if current_delay < 0.002:   # Don't go slower than this limit
                current_delay += 0.000001 # Slightly increase delay (decrease speed)

except KeyboardInterrupt:
    print("\nStopping motor...")

finally:
    lgpio.gpiochip_close(h)
    print("GPIO Released.")
import lgpio
import time

# --- Configuration ---
STEP_PIN = 18
DIR_PIN  = 23 

# Open the GPIO chip (Pi 5 = chip 0)
h = lgpio.gpiochip_open(0)

# Setup pins
lgpio.gpio_claim_output(h, STEP_PIN)
lgpio.gpio_claim_output(h, DIR_PIN)

def pulse(delay):
    """Sends a single pulse to the TB6600"""
    lgpio.gpio_write(h, STEP_PIN, 1)
    time.sleep(0.00002) # 20 microsecond pulse width for stability
    lgpio.gpio_write(h, STEP_PIN, 0)
    time.sleep(delay)

try:
    # Set direction (1 = Clockwise)
    lgpio.gpio_write(h, DIR_PIN, 1)
    
    # STARTING POINT: Very slow (0.01s = 100 steps per second)
    current_delay = 0.01 
    # ENDING POINT: Very fast (0.0002s = 5000 steps per second)
    min_delay = 0.0002   
    # STEP SIZE: How much to shave off the delay each time
    step_increment = 0.00005 

    print("--- Starting Acceleration Loop ---")

    while current_delay > min_delay:
        # Perform 100 steps at the CURRENT speed to keep it stable
        for _ in range(100):
            pulse(current_delay)
        
        # Print the current delay (rounded for readability)
        print(f"Current Delay: {current_delay:.6f}s")
        
        # Incrementally speed up (by decreasing the delay)
        current_delay -= step_increment
        
        # Safety: don't let the delay become zero or negative
        if current_delay <= 0:
            break

    print("--- Top Speed Reached! ---")
    # Maintain top speed indefinitely
    while True:
        pulse(min_delay)

except KeyboardInterrupt:
    print("\nMotor Stopped by User.")

finally:
    lgpio.gpiochip_close(h)
    print("GPIO Released.")
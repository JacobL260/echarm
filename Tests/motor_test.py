import lgpio
import time

# --- Configuration ---
# Pi 5 GPIO Numbers (BCM)
STEP_PIN = 5  
DIR_PIN  = 6  

# Speed Control
# Lower value = Faster rotation
# 0.0005 is a safe starting speed (1ms per full step cycle)
STEP_DELAY = 0.0005 

# Open the GPIO chip (Pi 5 uses chip 0)
h = lgpio.gpiochip_open(0)

# Initialize pins as outputs
lgpio.gpio_claim_output(h, STEP_PIN)
lgpio.gpio_claim_output(h, DIR_PIN)

def move_motor(steps, direction, delay):
    """
    Rotates the motor using the TB6600.
    :param steps: Number of pulses to send
    :param direction: 1 for CW, 0 for CCW
    :param delay: Time between pulses (controls speed)
    """
    # Set the direction
    lgpio.gpio_write(h, DIR_PIN, direction)
    
    print(f"Moving {'Clockwise' if direction else 'Counter-Clockwise'}...")
    
    for _ in range(steps):
        # Trigger the pulse
        lgpio.gpio_write(h, STEP_PIN, 1)
        time.sleep(0.00001) # 10 microsecond pulse width for stability
        lgpio.gpio_write(h, STEP_PIN, 0)
        
        # Wait before the next step
        time.sleep(delay)

try:
    # Example: 1600 steps (usually 1 full rotation at 1/8 microstepping)
    move_motor(1600, 1, STEP_DELAY)
    time.sleep(0.5)
    move_motor(1600, 0, STEP_DELAY)

except KeyboardInterrupt:
    print("\nStopping motor...")

finally:
    # Release GPIO resources
    lgpio.gpiochip_close(h)
    print("GPIO Closed.")
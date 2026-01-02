import time
import threading
import numpy as np

from control import volts_to_joint_angles, robot_state, state_lock, Actuator
from hardware import ADCReader, stop_event
from planner import planner_loop
from config import NUM_AXES

def get_camera_point():
    # Replace with RealSense / camera API
    return np.array([0.1, 0.0, 0.2])  # meters

def main():
    # Start ADC reader
    reader = ADCReader()
    reader.start()
    print("STARTED ADC READER")

    # Start actuators
    acts = [Actuator(i) for i in range(NUM_AXES)]
    for a in acts:
        a.start()
    print("STARTED ACTUATORS")
    
    # Start planner thread
    planner = threading.Thread(target=planner_loop, args=(get_camera_point,), daemon=True)
    planner.start()
    print("STARTED PLANNER")

    try:
        print("Simulation running. Press Ctrl+C to exit.")
        while True:
            # Read simulated ADC voltages
            with adc_reader.ads_lock if hasattr(adc_reader, "ads_lock") else threading.Lock():
                volts = adc_reader.src_voltages if hasattr(adc_reader, "src_voltages") else adc_reader._simulate_voltages()

            # Convert voltages → joint angles (degrees)
            joint_angles = volts_to_joint_angles(volts)

            # Save joint angles thread-safely
            with state_lock:
                robot_state["joint_angles"] = joint_angles
                target_q = robot_state.get("target_q", None)

            # Print for debugging
            print("Simulated joint angles (deg):", np.round(joint_angles, 2))
            if target_q is not None:
                print("Planner target q (rad):", np.round(target_q, 3))

            # Return the simulated joint angles for use elsewhere
            # (e.g., testing, plotting, or logging)
            # Here we just keep printing them; you could also `yield joint_angles` if using as generator

            time.sleep(0.05)

    except KeyboardInterrupt:
        stop_event.set()

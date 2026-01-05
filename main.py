import time

from config import NUM_AXES, MAIN_STATUS_HZ, BUTTON_PINS
from hardware import stop_event, ADCReader, Button
from control import Actuator
from kinematics import FKThread
from hardware import ADCReader
from hardware import Button

def main():
    # Initialize Robot State
    robot_state = {
        "timestamp": 0.0,
        "adc_voltages": [0.0] * NUM_AXES,
        "actuators": {i: {"cmd": 0.0, "fb": 0.0, "vel": 0.0} for i in range(NUM_AXES)},
        "buttons": {i: {"pressed": False, "was_pressed": False, "was_released": False} for i in range(len(BUTTON_PINS))},
        "kinematics": {"T_ee": None, "position": None, "rotation": None}
    }

    # Create and start ADC Reader
    adc_reader = ADCReader()
    adc_reader.start()

    # Create and start Actuators
    actuators = [Actuator(i, adc_reader) for i in range(NUM_AXES)]
    for act in actuators:
        act.start()

    # Start FK Thread
    fk_thread = FKThread(robot_state, actuators)
    fk_thread.start()

    # Create and start Buttons
    buttons = [Button(pin=pin, idx=i) for i, pin in enumerate(BUTTON_PINS)]
    for btn in buttons:
        btn.start()

    # Main Loop: print actuator and button states
    try:
        print("Robot running — Ctrl+C to exit...")
        while not stop_event.is_set():
            # Timestamp
            robot_state["timestamp"] = time.time()

            # Snapshot ADC Voltages
            with adc_reader.lock:
                robot_state["adc_voltages"] = adc_reader.volt.copy()

            # Snapshot Actuators
            for act in actuators:
                with act.lock:
                    robot_state["actuators"][act.idx] = act.buffer.copy()

            # Snapshot Buttons
            for btn in buttons:
                with btn.lock:
                    robot_state["buttons"][btn.idx] = btn.buffer.copy()

            # Snapshot Forward Kinematics
            if hasattr(fk_thread, "fk_buffer"):
                robot_state["kinematics"] = fk_thread.fk_buffer.copy()
            
            print(robot_state["kinematics"])

            time.sleep(1.0 / MAIN_STATUS_HZ)

    except KeyboardInterrupt:
        print("\nStopping all threads...")
        stop_event.set()

    # Join threads cleanly
    adc_reader.join()
    for act in actuators:
        act.join()
    fk_thread.join()
    for btn in buttons:
        btn.join()
    print("All threads stopped.")

if __name__ == "__main__":
    main()
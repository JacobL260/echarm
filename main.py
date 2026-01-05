import time

from config import NUM_AXES, MAIN_STATUS_HZ, BUTTON_PINS
from hardware import stop_event, adc_volt, adc_lock
from control import Actuator
from kinematics import FKThread
from hardware import ADCReader
from hardware import Button

def main():
    # Initialize Robot State
    robot_state = {
        "timestamp": time.time(),
        "adc_voltages": [0.0] * NUM_AXES,
        "actuators": {},
        "buttons": {},
        "kinematics": {
            "fk": {
                "T_ee": None,
                "position": None,
                "rotation": None
            },
            "actuator_cmd_mode": "joint"
        }
    }

    # Create and start ADC Reader
    adc_reader = ADCReader()
    adc_reader.start()

    # Create and start Actuators (prepopulate robot_state and start thread immediately)
    actuators = []
    for i in range(NUM_AXES):
        # Prepopulate robot_state
        robot_state["actuators"][i] = {
            "command_deg": 0.0,
            "feedback_deg": 0.0,
            "velocity": 0.0
        }

        # Create actuator object and start its thread
        act = Actuator(i, robot_state)
        act.start()
        actuators.append(act)

    # Start FK Thread
    fk_thread = FKThread(robot_state)
    fk_thread.start()

    # Create and start Buttons (they update robot_state themselves)
    buttons = []
    for i, pin in enumerate(BUTTON_PINS):
        btn = Button(pin=pin, robot_state=robot_state, idx=i)
        btn.start()
        buttons.append(btn)

    # Main Loop: print actuator and button states
    try:
        print("Press Ctrl+C to exit. Printing actuator and button states:")
        while True:
            robot_state["timestamp"] = time.time()

            # Safely copy ADC voltages
            with adc_lock:
                robot_state["adc_voltages"] = adc_volt.copy()

            # Prepare simplified status dicts for printing (rounded)
            actuator_status = {
                i: {
                    "cmd": round(a["command_deg"], 2),
                    "fb": round(a["feedback_deg"], 2),
                    "vel": round(a["velocity"], 2)
                } for i, a in robot_state["actuators"].items()
            }
            button_status = {
                i: {
                    "pressed": b["pressed"],
                } for i, b in robot_state["buttons"].items()
            }

            # Print status
            print(f"Actuators: {actuator_status} | Buttons: {button_status}", end="\r")

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
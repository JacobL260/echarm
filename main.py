import time

from config import NUM_AXES, FK_HZ, CTRL_HZ
from hardware import stop_event, adc_volt, adc_lock
from control import Actuator
from kinematics import FKThread, forward_kinematics
from hardware import ADCReader



def main():
    # 1. Initialize Robot State
    robot_state = {
        "timestamp": time.time(),
        "adc_voltages": [0.0] * NUM_AXES,
        "actuators": {},  # dict keyed by actuator index
        "kinematics": {
            "fk": {
                "T_ee": None,
                "position": None,
                "rotation": None
            }
        }
    }

    # 2. Create Actuators
    acts = []
    for i in range(NUM_AXES):
        robot_state["actuators"][i] = {"command_deg":0, "feedback_deg":0, "velocity":0.0}
        acts.append(Actuator(i,robot_state))

    # Prepopulate robot_state["actuators"]
    for a in acts:
        robot_state["actuators"][a.idx] = {
            "command_deg": a.cmd_deg,
            "feedback_deg": a.fb,
            "velocity": 0.0
        }

    # 3. Start ADC Reader
    reader = ADCReader()
    reader.start()

    # 4. Start Actuator Threads
    for a in acts:
        a.start()

    # 5. Start FK Thread
    fk_thread = FKThread(robot_state, rate_hz=FK_HZ)
    fk_thread.start()

    # 6. Main Loop
    try:
        print("Starting main loop. Press Ctrl+C to exit.")
        while True:
            robot_state["timestamp"] = time.time()

            with adc_lock:
                robot_state["adc_voltages"] = adc_volt.copy()

            # Example: display robot state in one line
            actuators_info = {
                i: {"cmd": a["command_deg"], "fb": a["feedback_deg"]}
                for i, a in robot_state["actuators"].items()
            }

            ee_pos = robot_state["kinematics"]["fk"]["position"]
            print(f"EE Pos: {ee_pos} | Actuators: {actuators_info}", end="\r")

            # Loop at roughly 20 Hz
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping all threads...")
        stop_event.set()

    # 7. Join Threads Cleanly
    reader.join()
    for a in acts:
        a.join()
    fk_thread.join()
    print("All threads stopped.")

if __name__ == "__main__":
    main()
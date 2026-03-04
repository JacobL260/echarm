import time
import sys
import select
import threading

from config import NUM_AXES, MAIN_STATUS_HZ, BUTTON_PINS
from hardware import stop_event, ADCReader, Button
from control import Actuator
from kinematics import FKThread

def main():
    # Initialize Robot State
    robot_state = {
        "timestamp": 0.0,
        "adc_voltages": [0.0] * NUM_AXES,
        "actuators": {i: {"pos_cmd": 0.0, "vel_cmd": 0.0, "fb": 0.0, "vel": 0.0} for i in range(NUM_AXES)},
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

    # Start command interface thread
    cmd_thread = threading.Thread(
        target=command_interface,
        args=(actuators,),
        daemon=True
    )
    cmd_thread.start()

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
            
            # print(robot_state["adc_voltages"])

            time.sleep(1.0 / MAIN_STATUS_HZ)

    except KeyboardInterrupt:
        print("\nStopping all threads...")
        stop_event.set()

    # Join threads cleanly
    adc_reader.join(timeout=1.0)
    for act in actuators:
        act.join(timeout=1.0)
    fk_thread.join(timeout=1.0)
    for btn in buttons:
        btn.join(timeout=1.0)
    print("All threads stopped.")

def command_interface(actuators):
    print("\nCommand interface ready.")
    print("Commands:")
    print("  v <idx> <vel>   → set velocity")
    print("  p <idx> <deg>   → set position")
    print("  stop            → zero all velocities")
    print("  exit            → stop program\n")

    while not stop_event.is_set():

        # Non-blocking stdin check (works over SSH)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            line = sys.stdin.readline().strip()
            parts = line.split()

            if not parts:
                continue

            cmd = parts[0].lower()

            try:
                # --------------------------
                # Velocity command
                # --------------------------
                if cmd == "v" and len(parts) == 3:
                    idx = int(parts[1])
                    vel = float(parts[2])

                    if 0 <= idx < len(actuators):
                        actuators[idx].set_velocity(vel)
                        print(f"[CMD] Actuator {idx} velocity → {vel}")
                    else:
                        print("Invalid actuator index")

                # --------------------------
                # Position command
                # --------------------------
                elif cmd == "p" and len(parts) == 3:
                    idx = int(parts[1])
                    pos = float(parts[2])

                    if 0 <= idx < len(actuators):
                        actuators[idx].set_position(pos)
                        print(f"[CMD] Actuator {idx} position → {pos}")
                    else:
                        print("Invalid actuator index")

                # --------------------------
                # Stop all
                # --------------------------
                elif cmd == "stop":
                    for act in actuators:
                        act.set_velocity(0)
                    print("[CMD] All actuators stopped")

                # --------------------------
                # Exit program
                # --------------------------
                elif cmd == "exit":
                    print("Stopping program...")
                    stop_event.set()

                else:
                    print("Unknown command")

            except ValueError:
                print("Invalid command format")

if __name__ == "__main__":
    main()
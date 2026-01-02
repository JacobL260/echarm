import time

from hardware import ADCReader, stop_event, adc_volt, adc_lock
from control import Actuator
from config import NUM_AXES


def main():
    reader = ADCReader()
    reader.start()

    acts = [Actuator(i) for i in range(NUM_AXES)]
    for a in acts:
        a.start()

    try:
        print("Starting main loop. Press Ctrl+C to exit.")
        while True:
            with adc_lock:
                volts = adc_volt.copy()

            actautors_state = []
            for a in acts:
                actautors_state.append({
                    "index":a.idx,
                    "command_deg": a.cmd_deg,
                    "feedback_deg":a.fb,
                    "velocity":a.stepper.velocity if hasattr(a.stepper, "velocity") else 0.0
                })

            robot_state = {
                "timestamp": time.time(),
                "adc_voltages": volts,
                "actuators": actautors_state
            }

            line = (
                " | ".join(f"A{i}: cmd={s['command_deg']:.1f}°, fb={s['feedback_deg']:.1f}°"
                           for i, s in enumerate(robot_state["actuators"]))
            )
            print(line.ljust(120), end='\r', flush=True)
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop_event.set()  # signal all threads to stop
        reader.join()     # wait for ADCReader to finish
        for a in acts:
            a.join()      # wait for all actuators to finish
        print("\nStopped all threads.")



if __name__ == "__main__":
    main()
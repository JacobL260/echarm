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

            positions = [a.fb for a in acts]
                

            print(
                "Voltages: " + " | ".join(f"A{i}: {v:.3f} V" for i, v in enumerate(volts))
            )
            print(
                "Positions: " + " | ".join(f"A{i}: {p:.2f}°" for i, p in enumerate(positions))
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop_event.set()  # signal all threads to stop
        reader.join()     # wait for ADCReader to finish
        for a in acts:
            a.join()      # wait for all actuators to finish
        print("\nStopped all threads.")



if __name__ == "__main__":
    main()

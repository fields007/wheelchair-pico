"""
Wheelchair Hall sensor diagnostic.

GPIO
----
GP10  Front-right Hall sensor
GP11  Front-left Hall sensor
GP12  Back-right Hall sensor
GP13  Back-left Hall sensor

Functions
---------
run(stop_requested=..., log=...)
stop_outputs()

Compatible with existing main.py.

All motor and servo outputs remain disabled.
"""

from machine import Pin
from time import ticks_ms, ticks_diff, sleep_ms


# ============================================================
# SETTINGS
# ============================================================

LOOP_PERIOD_MS = 20

LOG_INTERVAL_MS = 1000

# Change to False if your Hall sensors have
# push-pull outputs rather than open-collector outputs.
USE_PULL_UP = True


# ============================================================
# HALL SENSOR INITIALIZATION
# ============================================================

if USE_PULL_UP:
    pin_mode = Pin.PULL_UP
else:
    pin_mode = None

hall_FR = Pin(10, Pin.IN, pin_mode)
hall_FL = Pin(11, Pin.IN, pin_mode)
hall_BR = Pin(12, Pin.IN, pin_mode)
hall_BL = Pin(13, Pin.IN, pin_mode)

HALL_SENSORS = {
    "FR": hall_FR,
    "FL": hall_FL,
    "BR": hall_BR,
    "BL": hall_BL
}


# ============================================================
# SAFETY: DISABLE OUTPUTS
# ============================================================

def stop_outputs():

    # Ensure motor PWM pins are LOW.
    for pin_number in (14, 15, 17, 18):
        Pin(pin_number, Pin.OUT, value=0)

    # Disable steering and handbrake servo signals.
    Pin(9, Pin.OUT, value=0)
    Pin(22, Pin.OUT, value=0)

    # Release reverse signal.
    Pin(8, Pin.OPEN_DRAIN, value=1)


# ============================================================
# SENSOR READING
# ============================================================

def read_hall_sensors():

    return {
        name: pin.value()
        for name, pin in HALL_SENSORS.items()
    }


# ============================================================
# MAIN LOOP
# ============================================================

def run(stop_requested=None, log=print):

    stop_outputs()

    log("Hall sensor diagnostic started.")

    log("GP10: Front right")
    log("GP11: Front left")
    log("GP12: Back right")
    log("GP13: Back left")

    previous_values = read_hall_sensors()

    pulse_counts = {
        name: 0
        for name in HALL_SENSORS
    }

    last_log_ms = ticks_ms()

    try:

        while True:

            if (
                stop_requested is not None
                and stop_requested()
            ):
                log("Hall sensor diagnostic stopped.")
                return

            current_values = read_hall_sensors()

            # Count falling edges (1 -> 0).
            for name in HALL_SENSORS:

                if (
                    previous_values[name] == 1
                    and current_values[name] == 0
                ):
                    pulse_counts[name] += 1

            previous_values = current_values

            now = ticks_ms()

            elapsed = ticks_diff(
                now,
                last_log_ms
            )

            if elapsed >= LOG_INTERVAL_MS:

                dt = elapsed / 1000.0

                frequencies = {
                    name: pulse_counts[name] / dt
                    for name in HALL_SENSORS
                }

                message = (
                    "HALL | "
                    "FR:{} ({:.1f}Hz) | "
                    "FL:{} ({:.1f}Hz) | "
                    "BR:{} ({:.1f}Hz) | "
                    "BL:{} ({:.1f}Hz)"
                ).format(
                    current_values["FR"],
                    frequencies["FR"],

                    current_values["FL"],
                    frequencies["FL"],

                    current_values["BR"],
                    frequencies["BR"],

                    current_values["BL"],
                    frequencies["BL"]
                )

                # Print locally.
                print(message)

                # Send through main.py logging system.
                if log is not print:
                    log(message)

                # Reset counters.
                for name in pulse_counts:
                    pulse_counts[name] = 0

                last_log_ms = now

            sleep_ms(LOOP_PERIOD_MS)

    finally:

        stop_outputs()


# ============================================================
# DIRECT EXECUTION
# ============================================================

if __name__ == "__main__":

    try:
        run()

    except KeyboardInterrupt:
        print("Interrupted.")

    finally:
        stop_outputs()
        print("Diagnostic stopped.")
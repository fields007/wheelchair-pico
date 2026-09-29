"""
Wheelchair supervisor.

NORMAL OPERATION
----------------
The wheelchair operates normally without requiring Wi-Fi.

NETWORK CONNECTION
------------------
Press GP9 to request an Internet connection.

When GP9 is pressed:
    1. Wheelchair outputs are stopped.
    2. GP2 LED turns on.
    3. Wi-Fi connection is attempted.
    4. MQTT connection is attempted.
    5. Five quick flashes indicate success.
    6. Three slow flashes indicate failure.
    7. Wheelchair control resumes.

When connected, log messages are also sent to MQTT.

If Wi-Fi or MQTT is later lost, wheelchair operation continues.
GP9 can be pressed again to reconnect.

LOGGING
-------
Messages are:
- printed over USB/REPL
- written to maintenance.log
- sent to MQTT when available
"""

from machine import Pin
from time import sleep_ms

import chair_logic
import system.wifi_manager as wifi_manager


# ============================================================
# SETTINGS
# ============================================================

LOG_FILE = "maintenance.log"


# ============================================================
# HARDWARE
# ============================================================

# GP9 ---- button ---- GND
#
# released = 1
# pressed  = 0

connection_button = Pin(
    9,
    Pin.IN,
    Pin.PULL_UP
)


# Status LED on GP2.

led = Pin(
    2,
    Pin.OUT
)

led.value(0)


# ============================================================
# STATE
# ============================================================

connection_requested = False


# ============================================================
# LOCAL LOG
# ============================================================

def start_log():

    try:

        with open(LOG_FILE, "w") as file:
            file.write("BOOT\n")

    except Exception as error:

        print(
            "Could not create log:",
            error
        )


# ============================================================
# LOGGING
# ============================================================

def log(message=""):

    message = str(message)

    # USB / REPL
    print(message)

    # Local file
    try:

        with open(LOG_FILE, "a") as file:
            file.write(message)
            file.write("\n")

    except Exception:
        pass

    # MQTT
    #
    # send_log() simply returns False if MQTT is unavailable.
    # Network failure must never stop chair operation.

    try:

        wifi_manager.send_log(
            message
        )

    except Exception:
        pass


# ============================================================
# CONNECTION BUTTON
# ============================================================

def connection_button_pressed():
    """
    Called from chair_logic every control-loop iteration.

    Returns True once when GP9 is pressed.
    """

    global connection_requested

    if connection_button.value() != 0:
        return False

    sleep_ms(50)

    if connection_button.value() != 0:
        return False

    # Prevent repeatedly detecting the same held button.
    if connection_requested:
        return False

    connection_requested = True

    log("Network connection requested.")

    return True


def wait_for_button_release():
    """
    Wait until GP9 has been released before allowing another
    connection request.
    """

    global connection_requested

    while connection_button.value() == 0:
        sleep_ms(20)

    connection_requested = False


# ============================================================
# LED
# ============================================================

def flash_led(count, on_ms=150, off_ms=150):

    for _ in range(count):

        led.value(1)
        sleep_ms(on_ms)

        led.value(0)
        sleep_ms(off_ms)


def connection_success_pattern():

    led.value(0)

    sleep_ms(200)

    flash_led(
        count=5,
        on_ms=100,
        off_ms=100
    )

    led.value(0)


def connection_failed_pattern():

    led.value(0)

    flash_led(
        count=3,
        on_ms=500,
        off_ms=500
    )

    led.value(0)


# ============================================================
# NETWORK CONNECTION
# ============================================================

def connect_network():
    """
    Chair outputs are already stopped when this function is
    called.

    Try Wi-Fi + MQTT and then return so wheelchair control can
    resume.
    """

    log("")
    log("==============================")
    log("NETWORK CONNECTION")
    log("==============================")

    log("Wheelchair outputs temporarily disabled.")

    # Solid LED while connecting.
    led.value(1)

    success = False

    try:

        success = wifi_manager.connect(
            log=log
        )

    except Exception as error:

        log(
            "Network connection error: {}".format(
                error
            )
        )

        success = False

    if success:

        log("Remote logging connected.")

        connection_success_pattern()

    else:

        log("Remote logging unavailable.")

        connection_failed_pattern()

    wait_for_button_release()

    log("Resuming wheelchair control.")

    led.value(0)


# ============================================================
# BOOT
# ============================================================

start_log()


try:

    log("Starting wheelchair control.")

    led.value(0)

    while True:

        # ----------------------------------------------------
        # RUN CHAIR
        # ----------------------------------------------------

        chair_logic.run(
            stop_requested=connection_button_pressed,
            log=log
        )

        # chair_logic.run() only returns when GP9 requests
        # networking.

        chair_logic.stop_outputs()

        log("Chair control loop stopped.")

        # ----------------------------------------------------
        # NETWORK
        # ----------------------------------------------------

        connect_network()

        # Loop around and start chair_logic.run() again.
        #
        # Its throttle arming logic therefore starts fresh,
        # requiring neutral before throttle becomes active again.


# ============================================================
# CTRL-C
# ============================================================

except KeyboardInterrupt:

    chair_logic.stop_outputs()

    led.value(0)

    log("")
    log("KeyboardInterrupt.")
    log("Wheelchair control stopped.")


# ============================================================
# UNEXPECTED ERROR
# ============================================================

except Exception as error:

    chair_logic.stop_outputs()

    led.value(0)

    log("")
    log(
        "FATAL ERROR: {}".format(
            error
        )
    )

    raise
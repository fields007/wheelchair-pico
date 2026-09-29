"""
Wheelchair supervisor.

NORMAL OPERATION
----------------
The wheelchair starts immediately and operates without requiring
Wi-Fi or MQTT.

NETWORK CONNECTION
------------------
Press GP9 to request an Internet connection.

When GP9 is pressed:
    1. Wheelchair outputs are stopped.
    2. GP2 LED turns on continuously.
    3. Wi-Fi connection is attempted.
    4. MQTT connection is attempted.
    5. Five quick flashes indicate success.
    6. Three slow flashes indicate failure.
    7. Wheelchair control resumes.

When MQTT is connected:
- live chair diagnostics are sent remotely
- remote commands are received

If networking is later lost, wheelchair operation continues.
Press GP9 again to reconnect.

REMOTE COMMANDS
---------------
Supported commands:

    status
    check-update
    update
    rollback
    reboot
    add-wifi|SSID|PASSWORD

Update, rollback and reboot always stop chair control first.

LOGGING
-------
Normal chair diagnostics:
- printed over USB / REPL
- latest message queued for MQTT
- NOT continuously written to flash

Important supervisor events:
- printed over USB / REPL
- written to maintenance.log
- queued for MQTT

This avoids writing joystick telemetry to flash five times per
second and avoids publishing MQTT synchronously from the
real-time chair control loop.

RECOVERY
--------
If chair_logic.py cannot be imported or crashes, outputs are
stopped and the supervisor enters recovery mode.

GP9 can then be used to establish networking, allowing remote
rollback, update, status and reboot commands.
"""

from machine import Pin
from time import (
    sleep_ms,
    ticks_ms,
    ticks_diff
)

import machine
import sys

import system.wifi_manager as wifi_manager
import system.mqtt_manager as mqtt_manager
import system.command_manager as command_manager
import system.updater as updater


# ============================================================
# SETTINGS
# ============================================================

LOG_FILE = "maintenance.log"

# How often MQTT is checked while the chair is running.
#
# chair_logic calls supervisor_service() every 20 ms, but
# network servicing is deliberately less frequent.
MQTT_SERVICE_INTERVAL_MS = 200

# Only the latest unsent telemetry message is retained.
#
# If networking is temporarily slow, old telemetry is discarded
# rather than delaying chair control.
pending_telemetry = None

# Supervisor event messages are small and infrequent.
#
# Keep a short RAM queue so important messages can be sent when
# MQTT is serviced.
EVENT_QUEUE_LIMIT = 20
pending_events = []


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


# GP2 status LED.

led = Pin(
    2,
    Pin.OUT
)

led.value(0)


# ============================================================
# STATE
# ============================================================

connection_requested = False

last_mqtt_service_ms = ticks_ms()

chair_logic = None

recovery_mode = False

recovery_error = None


# ============================================================
# LOCAL LOG FILE
# ============================================================

def start_log():

    try:

        with open(
            LOG_FILE,
            "w"
        ) as file:

            file.write(
                "BOOT\n"
            )

    except Exception as error:

        print(
            "Could not create log:",
            error
        )


def write_event_to_file(message):

    try:

        with open(
            LOG_FILE,
            "a"
        ) as file:

            file.write(
                str(message)
            )

            file.write(
                "\n"
            )

    except Exception:
        pass


# ============================================================
# MQTT QUEUES
# ============================================================

def queue_event(message):

    global pending_events

    message = str(message)

    if len(pending_events) >= EVENT_QUEUE_LIMIT:

        # Remove oldest event.
        pending_events.pop(0)

    pending_events.append(
        message
    )


def queue_telemetry(message):

    global pending_telemetry

    # Deliberately overwrite any older unsent telemetry.
    pending_telemetry = str(
        message
    )


# ============================================================
# LOGGING
# ============================================================

def event_log(message=""):
    """
    Important supervisor event.

    Printed immediately, stored in maintenance.log and queued
    for MQTT.
    """

    message = str(message)

    print(
        message
    )

    write_event_to_file(
        message
    )

    queue_event(
        message
    )


def chair_log(message=""):
    """
    Logging callback supplied to chair_logic.

    Chair diagnostics are printed immediately and the newest
    message is retained for remote MQTT telemetry.

    They are NOT continuously written to flash.
    """

    message = str(message)

    print(
        message
    )

    queue_telemetry(
        message
    )


# ============================================================
# LED
# ============================================================

def flash_led(
    count,
    on_ms=150,
    off_ms=150
):

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
# CONNECTION BUTTON
# ============================================================

def check_connection_button():
    """
    Check GP9 without blocking for 50 ms.

    Returns True once per physical button press.
    """

    global connection_requested

    if connection_button.value() != 0:

        if connection_requested:
            connection_requested = False

        return False

    if connection_requested:
        return False

    # Short debounce.
    sleep_ms(20)

    if connection_button.value() != 0:
        return False

    connection_requested = True

    event_log(
        "Network connection requested."
    )

    return True


def wait_for_button_release():

    global connection_requested

    while connection_button.value() == 0:

        sleep_ms(20)

    connection_requested = False


# ============================================================
# MQTT COMMAND CALLBACK
# ============================================================

def mqtt_command_received(command_text):
    """
    Called by mqtt_manager when a command arrives.

    The command manager only validates and queues it.
    No update, reboot or other substantial work happens here.
    """

    command_manager.receive(
        command_text,
        log=event_log
    )


mqtt_manager.set_command_callback(
    mqtt_command_received
)


# ============================================================
# MQTT OUTPUT
# ============================================================

def flush_one_mqtt_message():
    """
    Send at most one queued MQTT message.

    Important events have priority over telemetry.

    Sending only one message per service cycle prevents a
    backlog from causing a long burst of blocking network work.
    """

    global pending_telemetry

    if not mqtt_manager.is_connected():
        return False

    # Important event first.
    if pending_events:

        message = pending_events[0]

        try:

            if mqtt_manager.send_log(
                message
            ):

                pending_events.pop(0)

                return True

        except Exception:
            pass

        return False

    # Then latest telemetry.
    if pending_telemetry is not None:

        message = pending_telemetry

        try:

            if mqtt_manager.send_log(
                message
            ):

                # Only clear it if no newer telemetry replaced it
                # during the operation.
                if pending_telemetry == message:
                    pending_telemetry = None

                return True

        except Exception:
            pass

    return False


# ============================================================
# NETWORK SERVICE WHILE DRIVING
# ============================================================

def service_mqtt_if_due():
    """
    Periodically poll MQTT and send one queued log message.

    Returns quickly when MQTT is not connected.
    """

    global last_mqtt_service_ms

    now = ticks_ms()

    if ticks_diff(
        now,
        last_mqtt_service_ms
    ) < MQTT_SERVICE_INTERVAL_MS:

        return

    last_mqtt_service_ms = now

    if not mqtt_manager.is_connected():
        return

    # Check for incoming command.
    try:

        mqtt_manager.check_messages()

    except Exception:
        return

    # Send at most one outgoing message.
    flush_one_mqtt_message()


# ============================================================
# CHAIR STOP CALLBACK
# ============================================================

def supervisor_service():
    """
    Called by chair_logic once per 20 ms control-loop iteration.

    Returns True when chair_logic should stop and return to
    main.py.

    This callback:
    - checks GP9
    - periodically services MQTT
    - checks whether a stop-required remote command is pending
    """

    # GP9 always requests a stop so networking can be attempted.
    if check_connection_button():

        return True

    # Periodic MQTT servicing.
    service_mqtt_if_due()

    # See whether a remote command is waiting.
    command = command_manager.pending_command()

    if command is None:
        return False

    if command_manager.requires_chair_stop(
        command
    ):

        return True

    # Non-stop commands are handled by the supervisor below.
    #
    # We still return True temporarily because status,
    # check-update and add-wifi can involve networking or flash
    # I/O. Keeping that work outside the 20 ms chair loop is
    # preferable to doing it during active control.
    return True


# ============================================================
# NETWORK CONNECTION
# ============================================================

def connect_network():
    """
    Attempt Wi-Fi and then MQTT.

    Chair outputs must already be stopped before this function
    is called.

    GP2:
        solid while connecting
        5 quick flashes = Wi-Fi + MQTT success
        3 slow flashes = failure
    """

    event_log("")
    event_log(
        "=============================="
    )
    event_log(
        "NETWORK CONNECTION"
    )
    event_log(
        "=============================="
    )

    event_log(
        "Wheelchair outputs temporarily disabled."
    )

    # Solid LED during the whole connection attempt.
    led.value(1)

    wifi_ok = False
    mqtt_ok = False

    # --------------------------------------------------------
    # WIFI
    # --------------------------------------------------------

    try:

        wifi_ok = wifi_manager.connect(
            log=event_log
        )

    except Exception as error:

        event_log(
            "Wi-Fi connection error: {}".format(
                error
            )
        )

        wifi_ok = False

    # --------------------------------------------------------
    # MQTT
    # --------------------------------------------------------

    if wifi_ok:

        try:

            mqtt_ok = mqtt_manager.connect(
                log=event_log
            )

        except Exception as error:

            event_log(
                "MQTT connection error: {}".format(
                    error
                )
            )

            mqtt_ok = False

    # --------------------------------------------------------
    # RESULT
    # --------------------------------------------------------

    if wifi_ok and mqtt_ok:

        event_log(
            "Remote logging and commands connected."
        )

        connection_success_pattern()

    else:

        event_log(
            "Remote connection unavailable."
        )

        connection_failed_pattern()

    wait_for_button_release()

    led.value(0)

    return (
        wifi_ok
        and mqtt_ok
    )


# ============================================================
# STATUS
# ============================================================

def report_status():

    event_log("")
    event_log(
        "=============================="
    )
    event_log(
        "STATUS"
    )
    event_log(
        "=============================="
    )

    # Software version
    try:

        version = updater.current_version()

    except Exception:

        version = "unknown"

    event_log(
        "Version: {}".format(
            version
        )
    )

    # Wi-Fi
    try:

        wifi_connected = (
            wifi_manager.is_connected()
        )

    except Exception:

        wifi_connected = False

    event_log(
        "Wi-Fi: {}".format(
            "connected"
            if wifi_connected
            else "disconnected"
        )
    )

    if wifi_connected:

        try:

            event_log(
                "SSID: {}".format(
                    wifi_manager.current_ssid()
                )
            )

        except Exception:
            pass

        try:

            event_log(
                "IP: {}".format(
                    wifi_manager.ip_address()
                )
            )

        except Exception:
            pass

    # MQTT
    try:

        mqtt_connected = (
            mqtt_manager.is_connected()
        )

    except Exception:

        mqtt_connected = False

    event_log(
        "MQTT: {}".format(
            "connected"
            if mqtt_connected
            else "disconnected"
        )
    )

    event_log(
        "Mode: {}".format(
            "RECOVERY"
            if recovery_mode
            else "NORMAL"
        )
    )

    if recovery_error is not None:

        event_log(
            "Recovery reason: {}".format(
                recovery_error
            )
        )


# ============================================================
# CHECK UPDATE
# ============================================================

def handle_check_update():

    event_log("")
    event_log(
        "Checking for software update."
    )

    try:

        result = updater.check_update()

        local_version = result[
            "local"
        ]

        remote_version = result[
            "remote"
        ]

        event_log(
            "Installed version: {}".format(
                local_version
            )
        )

        event_log(
            "GitHub version: {}".format(
                remote_version
            )
        )

        if result[
            "update_available"
        ]:

            event_log(
                "Update available."
            )

        else:

            event_log(
                "No newer update available."
            )

    except Exception as error:

        event_log(
            "Update check failed: {}".format(
                error
            )
        )


# ============================================================
# ADD WIFI
# ============================================================

def handle_add_wifi(command):

    ssid = command.get(
        "ssid"
    )

    password = command.get(
        "password"
    )

    if ssid is None or password is None:

        event_log(
            "Invalid add-wifi command."
        )

        return

    try:

        wifi_manager.add_network(
            ssid,
            password
        )

        # Never log the password.
        event_log(
            "Saved Wi-Fi network: {}".format(
                ssid
            )
        )

    except Exception as error:

        event_log(
            "Could not save Wi-Fi network '{}': {}".format(
                ssid,
                error
            )
        )


# ============================================================
# UPDATE
# ============================================================

def handle_update():

    event_log("")
    event_log(
        "Software update requested."
    )

    event_log(
        "Wheelchair outputs are disabled."
    )

    try:

        installed = updater.update(
            log=event_log
        )

    except Exception as error:

        event_log(
            "Update failed: {}".format(
                error
            )
        )

        return

    if not installed:

        event_log(
            "No update installed."
        )

        return

    event_log(
        "Update installed successfully."
    )

    event_log(
        "Rebooting."
    )

    # Give USB and MQTT a brief opportunity to transmit output.
    flush_one_mqtt_message()

    sleep_ms(500)

    machine.reset()


# ============================================================
# ROLLBACK
# ============================================================

def handle_rollback():

    event_log("")
    event_log(
        "Rollback requested."
    )

    event_log(
        "Wheelchair outputs are disabled."
    )

    try:

        restored = updater.rollback(
            log=event_log
        )

    except Exception as error:

        event_log(
            "Rollback failed: {}".format(
                error
            )
        )

        return

    if not restored:

        event_log(
            "Rollback was not performed."
        )

        return

    event_log(
        "Rollback completed successfully."
    )

    event_log(
        "Rebooting."
    )

    flush_one_mqtt_message()

    sleep_ms(500)

    machine.reset()


# ============================================================
# REBOOT
# ============================================================

def handle_reboot():

    event_log("")
    event_log(
        "Reboot requested."
    )

    event_log(
        "Wheelchair outputs are disabled."
    )

    flush_one_mqtt_message()

    sleep_ms(500)

    machine.reset()


# ============================================================
# COMMAND PROCESSING
# ============================================================

def process_pending_command():
    """
    Remove and process one queued command.

    Called only after chair control has returned to main.py or
    from recovery mode.
    """

    command = command_manager.take_command()

    if command is None:
        return

    command_name = command.get(
        "command"
    )

    event_log(
        "Processing command: {}".format(
            command_manager.describe(
                command
            )
        )
    )

    if command_name == command_manager.COMMAND_STATUS:

        report_status()

    elif command_name == command_manager.COMMAND_CHECK_UPDATE:

        handle_check_update()

    elif command_name == command_manager.COMMAND_ADD_WIFI:

        handle_add_wifi(
            command
        )

    elif command_name == command_manager.COMMAND_UPDATE:

        handle_update()

    elif command_name == command_manager.COMMAND_ROLLBACK:

        handle_rollback()

    elif command_name == command_manager.COMMAND_REBOOT:

        handle_reboot()

    else:

        event_log(
            "Unsupported command."
        )


# ============================================================
# CHAIR LOGIC LOADING
# ============================================================

def load_chair_logic():
    """
    Import chair_logic.

    Import is deliberately performed here rather than at the
    top of main.py.

    This allows main.py to remain alive if a remotely installed
    chair_logic.py is broken.
    """

    global chair_logic
    global recovery_mode
    global recovery_error

    try:

        # Remove an old cached module if necessary.
        if "chair_logic" in sys.modules:

            del sys.modules[
                "chair_logic"
            ]

        import chair_logic as loaded_chair_logic

        chair_logic = loaded_chair_logic

        recovery_mode = False
        recovery_error = None

        event_log(
            "Chair logic loaded."
        )

        return True

    except Exception as error:

        chair_logic = None

        recovery_mode = True

        recovery_error = (
            "chair_logic import failed: {}".format(
                error
            )
        )

        event_log(
            recovery_error
        )

        return False


# ============================================================
# STOP CHAIR
# ============================================================

def stop_chair_outputs():
    """
    Best-effort output stop.

    If chair_logic imported correctly, use its normal
    stop_outputs() function.
    """

    if chair_logic is None:
        return

    try:

        chair_logic.stop_outputs()

    except Exception as error:

        event_log(
            "Could not call chair stop_outputs(): {}".format(
                error
            )
        )


# ============================================================
# RECOVERY MODE
# ============================================================

def recovery_loop():
    """
    Recovery supervisor.

    No chair control is run here.

    GP9 can establish Wi-Fi/MQTT.

    Once MQTT is connected, commands can be received and
    processed, particularly:

        rollback
        update
        status
        reboot
    """

    global last_mqtt_service_ms

    led.value(0)

    event_log("")
    event_log(
        "=============================="
    )
    event_log(
        "RECOVERY MODE"
    )
    event_log(
        "=============================="
    )

    event_log(
        "Wheelchair control is disabled."
    )

    event_log(
        "Press GP9 to connect to the network."
    )

    while True:

        # ----------------------------------------------------
        # GP9 NETWORK CONNECTION
        # ----------------------------------------------------

        if check_connection_button():

            connect_network()

            event_log(
                "Recovery mode remains active."
            )

        # ----------------------------------------------------
        # MQTT
        # ----------------------------------------------------

        now = ticks_ms()

        if ticks_diff(
            now,
            last_mqtt_service_ms
        ) >= MQTT_SERVICE_INTERVAL_MS:

            last_mqtt_service_ms = now

            if mqtt_manager.is_connected():

                try:

                    mqtt_manager.check_messages()

                except Exception:
                    pass

                flush_one_mqtt_message()

        # ----------------------------------------------------
        # COMMAND
        # ----------------------------------------------------

        if command_manager.has_pending_command():

            process_pending_command()

        sleep_ms(20)


# ============================================================
# BOOT
# ============================================================

start_log()

event_log(
    "Starting wheelchair supervisor."
)

led.value(0)


# ============================================================
# MAIN SUPERVISOR
# ============================================================

try:

    # --------------------------------------------------------
    # LOAD CHAIR LOGIC
    # --------------------------------------------------------

    if not load_chair_logic():

        recovery_loop()

    # --------------------------------------------------------
    # NORMAL OPERATION
    # --------------------------------------------------------

    while True:

        event_log(
            "Starting wheelchair control."
        )

        try:

            chair_logic.run(
                stop_requested=supervisor_service,
                log=chair_log
            )

        except KeyboardInterrupt:

            raise

        except Exception as error:

            # Chair logic itself failed at runtime.
            stop_chair_outputs()

            recovery_mode = True

            recovery_error = (
                "chair_logic runtime error: {}".format(
                    error
                )
            )

            event_log("")
            event_log(
                recovery_error
            )

            event_log(
                "Entering recovery mode."
            )

            recovery_loop()

        # ----------------------------------------------------
        # CHAIR HAS RETURNED
        # ----------------------------------------------------

        stop_chair_outputs()

        event_log(
            "Chair control loop stopped."
        )

        # ----------------------------------------------------
        # GP9 NETWORK REQUEST
        # ----------------------------------------------------

        if connection_requested:

            event_log(
                "Entering network connection mode..."
            )

            connect_network()

            event_log(
                "Resuming wheelchair control."
            )

            continue

        # ----------------------------------------------------
        # REMOTE COMMAND
        # ----------------------------------------------------

        if command_manager.has_pending_command():

            process_pending_command()

            # update / rollback / reboot normally reset the Pico.
            #
            # status / check-update / add-wifi return here and
            # normal chair control resumes.

            event_log(
                "Resuming wheelchair control."
            )

            continue

        # This normally should not happen, but if chair_logic
        # returned without a known reason simply restart it.

        event_log(
            "Chair control returned without a pending request."
        )


# ============================================================
# CTRL-C
# ============================================================

except KeyboardInterrupt:

    stop_chair_outputs()

    led.value(0)

    event_log("")
    event_log(
        "KeyboardInterrupt."
    )

    event_log(
        "Wheelchair control stopped."
    )


# ============================================================
# SUPERVISOR FAILURE
# ============================================================

except Exception as error:

    stop_chair_outputs()

    led.value(0)

    event_log("")
    event_log(
        "SUPERVISOR ERROR: {}".format(
            error
        )
    )

    raise
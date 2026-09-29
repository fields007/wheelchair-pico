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

After a remotely requested reboot, successful update or
successful rollback, the supervisor automatically attempts to
reconnect to Wi-Fi and MQTT once.

A normal power-on or unrelated reboot does NOT automatically
connect.

When MQTT is connected:
- live chair diagnostics can be sent remotely
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
    logs-on
    logs-off
    add-wifi|SSID|PASSWORD

Update, rollback and reboot always stop chair control first.

REMOTE TELEMETRY
----------------
The commands:

    logs-off

and:

    logs-on

disable and enable only the continuous chair telemetry sent
through MQTT.

Local USB / REPL chair diagnostics continue regardless.

Important supervisor event messages are still sent remotely
during normal connected operation.

Remote telemetry defaults to ON after every reboot.

LOGGING
-------
Normal chair diagnostics:
- printed over USB / REPL
- latest message queued for MQTT when remote telemetry is ON
- NOT continuously written to flash

Normal important supervisor events:
- printed over USB / REPL
- written to maintenance.log
- queued for MQTT

Update / rollback progress:
- printed over USB / REPL
- written to maintenance.log
- NOT queued for MQTT

This is deliberate because update/rollback is followed by a
reboot and queued MQTT messages cannot be guaranteed to be sent.

After the reboot and successful automatic reconnection, a new
confirmation message is sent remotely.

RECOVERY
--------
If chair_logic.py cannot be imported or crashes, outputs are
stopped and the supervisor enters recovery mode.

GP9 can then be used to establish networking, allowing remote
rollback, update, status and reboot commands.

If recovery mode is entered immediately after a remotely
requested reboot/update/rollback, the one-shot automatic
network connection is still attempted.
"""

from machine import Pin
from time import (
    sleep_ms,
    ticks_ms,
    ticks_diff
)

import machine
import sys
import os

import system.wifi_manager as wifi_manager
import system.mqtt_manager as mqtt_manager
import system.command_manager as command_manager
import system.updater as updater


# ============================================================
# SETTINGS
# ============================================================

LOG_FILE = "maintenance.log"

# One-shot file used to request automatic network reconnection
# after a remotely requested reset.
#
# Its contents describe why the reset occurred:
#
#     reboot
#     update
#     rollback
#
RECONNECT_FILE = "reconnect_after_reboot.flag"

MQTT_SERVICE_INTERVAL_MS = 200

pending_telemetry = None

remote_telemetry_enabled = True

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
# RECONNECT-AFTER-REBOOT FLAG
# ============================================================

def request_reconnect_after_reboot(reason):
    """
    Ask the next boot to reconnect automatically.

    reason should normally be:
        reboot
        update
        rollback
    """

    try:

        with open(
            RECONNECT_FILE,
            "w"
        ) as file:

            file.write(
                str(reason)
            )

        return True

    except Exception as error:

        local_event_log(
            "Could not create reconnect flag: {}".format(
                error
            )
        )

        return False


def take_reconnect_after_reboot():
    """
    Read and consume the one-shot reconnect flag.

    Returns:
        None
            if there is no automatic reconnect request

        string
            reboot / update / rollback

    The file is removed before networking is attempted so the
    request cannot accidentally repeat on later boots.
    """

    try:

        with open(
            RECONNECT_FILE,
            "r"
        ) as file:

            reason = file.read().strip()

    except OSError:

        return None

    try:

        os.remove(
            RECONNECT_FILE
        )

    except Exception as error:

        local_event_log(
            "Could not remove reconnect flag: {}".format(
                error
            )
        )

        # Do not automatically reconnect if the one-shot flag
        # could not be consumed.
        return None

    if reason not in (
        "reboot",
        "update",
        "rollback"
    ):

        # Compatibility with the previous version, which wrote
        # "1" into the flag.
        if reason == "1":

            return "reboot"

        return "reboot"

    return reason


# ============================================================
# MQTT QUEUES
# ============================================================

def queue_event(message):

    global pending_events

    message = str(message)

    if len(pending_events) >= EVENT_QUEUE_LIMIT:

        pending_events.pop(0)

    pending_events.append(
        message
    )


def queue_telemetry(message):

    global pending_telemetry

    pending_telemetry = str(
        message
    )


def clear_mqtt_queues():
    """
    Discard messages that were queued while MQTT was offline.

    This is particularly useful after an automatic reboot:
    boot and connection messages printed before MQTT existed
    should not later appear remotely as though they were live.
    """

    global pending_events
    global pending_telemetry

    pending_events = []
    pending_telemetry = None


# ============================================================
# LOGGING
# ============================================================

def local_event_log(message=""):
    """
    Important local event.

    Printed over USB / REPL and stored in maintenance.log.

    It is deliberately NOT queued for MQTT.

    Used for operations such as update/rollback where a reboot
    is imminent and MQTT delivery cannot be guaranteed.
    """

    message = str(message)

    print(
        message
    )

    write_event_to_file(
        message
    )


def event_log(message=""):
    """
    Important supervisor event.

    Printed immediately, stored in maintenance.log and queued
    for MQTT.

    These messages are NOT affected by logs-on / logs-off.
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

    Chair diagnostics are always printed locally.

    When remote telemetry is enabled, the newest diagnostic
    message is also retained for MQTT.

    Chair diagnostics are NOT continuously written to flash.
    """

    message = str(message)

    print(
        message
    )

    if remote_telemetry_enabled:

        queue_telemetry(
            message
        )


# ============================================================
# REMOTE TELEMETRY CONTROL
# ============================================================

def handle_logs_off():

    global remote_telemetry_enabled
    global pending_telemetry

    remote_telemetry_enabled = False

    pending_telemetry = None

    event_log(
        "Remote real-time logs disabled."
    )


def handle_logs_on():

    global remote_telemetry_enabled

    remote_telemetry_enabled = True

    event_log(
        "Remote real-time logs enabled."
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

    global connection_requested

    if connection_button.value() != 0:

        if connection_requested:
            connection_requested = False

        return False

    if connection_requested:
        return False

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
    if (
        remote_telemetry_enabled
        and
        pending_telemetry is not None
    ):

        message = pending_telemetry

        try:

            if mqtt_manager.send_log(
                message
            ):

                if pending_telemetry == message:

                    pending_telemetry = None

                return True

        except Exception:
            pass

    return False


def send_remote_now(message):
    """
    Attempt to send one message immediately over MQTT.

    This is used only when MQTT is already known to be
    connected, for example after a successful automatic
    post-reboot reconnect.

    The message is still printed and stored locally.
    """

    message = str(message)

    print(
        message
    )

    write_event_to_file(
        message
    )

    if not mqtt_manager.is_connected():
        return False

    try:

        return mqtt_manager.send_log(
            message
        )

    except Exception:

        return False


# ============================================================
# NETWORK SERVICE WHILE DRIVING
# ============================================================

def service_mqtt_if_due():

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

    try:

        mqtt_manager.check_messages()

    except Exception:
        return

    flush_one_mqtt_message()


# ============================================================
# CHAIR STOP CALLBACK
# ============================================================

def supervisor_service():

    if check_connection_button():

        return True

    service_mqtt_if_due()

    command = command_manager.pending_command()

    if command is None:
        return False

    if command_manager.requires_chair_stop(
        command
    ):

        return True

    # Non-stop commands are still processed outside the
    # real-time chair loop.
    return True


# ============================================================
# NETWORK CONNECTION
# ============================================================

def connect_network(
    wait_for_release=True
):

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

    # Solid GP2 while connecting.
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
    # RESULT / LED
    # --------------------------------------------------------

    if wifi_ok and mqtt_ok:

        event_log(
            "Remote logging and commands connected."
        )

        # Five quick flashes.
        connection_success_pattern()

    else:

        event_log(
            "Remote connection unavailable."
        )

        # Three slow flashes.
        connection_failed_pattern()

    if wait_for_release:

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

    try:

        version = updater.current_version()

    except Exception:

        version = "unknown"

    event_log(
        "Version: {}".format(
            version
        )
    )

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
        "Remote telemetry: {}".format(
            "ON"
            if remote_telemetry_enabled
            else "OFF"
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

    # From this point onward, update progress is local only.
    #
    # We do not queue a series of MQTT messages immediately
    # before deliberately rebooting the Pico.

    local_event_log("")
    local_event_log(
        "Software update requested."
    )

    local_event_log(
        "Wheelchair outputs are disabled."
    )

    try:

        installed = updater.update(
            log=local_event_log
        )

    except Exception as error:

        # The update failed, so the Pico is NOT rebooting.
        # Report the failure remotely as a real normal event.
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

    local_event_log(
        "Update installed successfully."
    )

    # --------------------------------------------------------
    # DIAGNOSTIC: CREATE RECONNECT FLAG
    # --------------------------------------------------------

    flag_ok = request_reconnect_after_reboot(
        "update"
    )

    local_event_log(
        "Reconnect flag created: {}".format(
            flag_ok
        )
    )

    # --------------------------------------------------------
    # DIAGNOSTIC: READ FLAG BACK BEFORE REBOOT
    # --------------------------------------------------------

    try:

        with open(
            RECONNECT_FILE,
            "r"
        ) as file:

            flag_contents = (
                file.read().strip()
            )

        local_event_log(
            "Reconnect flag contains: {}".format(
                flag_contents
            )
        )

    except Exception as error:

        local_event_log(
            "Could not verify reconnect flag: {}".format(
                error
            )
        )

    local_event_log(
        "Rebooting after update."
    )

    sleep_ms(500)

    machine.reset()


# ============================================================
# ROLLBACK
# ============================================================

def handle_rollback():

    local_event_log("")
    local_event_log(
        "Rollback requested."
    )

    local_event_log(
        "Wheelchair outputs are disabled."
    )

    try:

        restored = updater.rollback(
            log=local_event_log
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

    local_event_log(
        "Rollback completed successfully."
    )

    request_reconnect_after_reboot(
        "rollback"
    )

    local_event_log(
        "Rebooting after rollback."
    )

    sleep_ms(500)

    machine.reset()


# ============================================================
# REBOOT
# ============================================================

def handle_reboot():

    local_event_log("")
    local_event_log(
        "Reboot requested."
    )

    local_event_log(
        "Wheelchair outputs are disabled."
    )

    request_reconnect_after_reboot(
        "reboot"
    )

    local_event_log(
        "Rebooting."
    )

    sleep_ms(500)

    machine.reset()


# ============================================================
# COMMAND PROCESSING
# ============================================================

def process_pending_command():

    command = command_manager.take_command()

    if command is None:
        return

    command_name = command.get(
        "command"
    )

    # For commands that immediately lead to a reboot, don't
    # create another MQTT event that may never be delivered.
    if command_name in (
        command_manager.COMMAND_UPDATE,
        command_manager.COMMAND_ROLLBACK,
        command_manager.COMMAND_REBOOT
    ):

        local_event_log(
            "Processing command: {}".format(
                command_manager.describe(
                    command
                )
            )
        )

    else:

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

    elif command_name == command_manager.COMMAND_LOGS_OFF:

        handle_logs_off()

    elif command_name == command_manager.COMMAND_LOGS_ON:

        handle_logs_on()

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

    global chair_logic
    global recovery_mode
    global recovery_error

    try:

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
# AUTOMATIC POST-REBOOT CONNECTION
# ============================================================

def report_post_reboot_success(reason):
    """
    Send a truthful confirmation after MQTT has actually
    reconnected.

    These messages are transmitted immediately rather than
    merely placed in the normal queue.
    """

    if reason == "update":

        send_remote_now(
            "Reconnected after software update."
        )

        try:

            version = updater.current_version()

        except Exception:

            version = "unknown"

        send_remote_now(
            "Installed version: {}".format(
                version
            )
        )

    elif reason == "rollback":

        send_remote_now(
            "Reconnected after rollback."
        )

        try:

            version = updater.current_version()

        except Exception:

            version = "unknown"

        send_remote_now(
            "Installed version: {}".format(
                version
            )
        )

    elif reason == "reboot":

        send_remote_now(
            "Reconnected after remote reboot."
        )


def automatic_reconnect(reason):
    """
    Perform the one-shot automatic network connection requested
    by a remote reboot/update/rollback.
    """

    local_event_log("")
    local_event_log(
        "Automatic network reconnect requested after {}.".format(
            reason
        )
    )

    connected = connect_network(
        wait_for_release=False
    )

    if connected:

        # Discard messages that were generated while MQTT was
        # unavailable. They were local boot/connection events,
        # not messages that were actually delivered remotely.
        clear_mqtt_queues()

        # Now send a fresh message over the connection that
        # actually exists.
        report_post_reboot_success(
            reason
        )

    return connected


# ============================================================
# RECOVERY MODE
# ============================================================

def recovery_loop(
    auto_reconnect_reason=None
):

    global last_mqtt_service_ms

    led.value(0)

    local_event_log("")
    local_event_log(
        "=============================="
    )
    local_event_log(
        "RECOVERY MODE"
    )
    local_event_log(
        "=============================="
    )

    local_event_log(
        "Wheelchair control is disabled."
    )

    if auto_reconnect_reason is not None:

        automatic_reconnect(
            auto_reconnect_reason
        )

    event_log(
        "Press GP9 to connect to the network."
    )

    while True:

        # ----------------------------------------------------
        # GP9 NETWORK CONNECTION
        # ----------------------------------------------------

        if check_connection_button():

            connect_network(
                wait_for_release=True
            )

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

local_event_log(
    "Starting wheelchair supervisor."
)

led.value(0)

# Consume the one-shot reconnect reason immediately.
auto_reconnect_reason = (
    take_reconnect_after_reboot()
)

# Diagnostic output. This tells us whether a reconnect request
# survived the previous reset.
local_event_log(
    "Post-reboot reconnect reason: {}".format(
        auto_reconnect_reason
    )
)


# ============================================================
# MAIN SUPERVISOR
# ============================================================

try:

    # --------------------------------------------------------
    # LOAD CHAIR LOGIC
    # --------------------------------------------------------

    if not load_chair_logic():

        recovery_loop(
            auto_reconnect_reason=auto_reconnect_reason
        )

    # --------------------------------------------------------
    # AUTOMATIC POST-REBOOT NETWORK CONNECTION
    # --------------------------------------------------------

    if auto_reconnect_reason is not None:

        # chair_logic has loaded, but chair control has not yet
        # started. Outputs therefore remain stopped while
        # networking is established.
        automatic_reconnect(
            auto_reconnect_reason
        )

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

            connect_network(
                wait_for_release=True
            )

            event_log(
                "Resuming wheelchair control."
            )

            continue

        # ----------------------------------------------------
        # REMOTE COMMAND
        # ----------------------------------------------------

        if command_manager.has_pending_command():

            process_pending_command()

            # update / rollback / reboot normally reset the
            # Pico. Other commands return here.

            event_log(
                "Resuming wheelchair control."
            )

            continue

        event_log(
            "Chair control returned without a pending request."
        )


# ============================================================
# CTRL-C
# ============================================================

except KeyboardInterrupt:

    stop_chair_outputs()

    led.value(0)

    local_event_log("")
    local_event_log(
        "KeyboardInterrupt."
    )

    local_event_log(
        "Wheelchair control stopped."
    )


# ============================================================
# SUPERVISOR FAILURE
# ============================================================

except Exception as error:

    stop_chair_outputs()

    led.value(0)

    local_event_log("")
    local_event_log(
        "SUPERVISOR ERROR: {}".format(
            error
        )
    )

    raise
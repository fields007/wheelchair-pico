"""
Wheelchair supervisor.

NORMAL OPERATION
----------------
Wheelchair control starts without requiring Wi-Fi or MQTT.

NETWORK CONNECTION
------------------
Press GP26 to request a network connection.

The supervisor:
    1. Stops wheelchair control.
    2. Displays CONNECTING on the OLED.
    3. Attempts Wi-Fi connection once.
    4. Attempts MQTT connection once.
    5. Displays the result for five seconds.
    6. Resumes wheelchair control.

NETWORK STATUS
--------------
OFFLINE
CONNECTING
WIFI OK
MQTT CONNECT
CONNECTED
WIFI FAILED
MQTT FAILED
RECOVERY
STOPPED
ERROR

OLED
----
Main messages use the standard 8x8 font.

Secondary messages use a smaller 3x5 bitmap font.

Text is centered horizontally and vertically.

Long messages wrap automatically.

Successful connection display:

    Connected to
    [Wi-Fi SSID]

    WiFi + MQTT OK

REMOTE COMMANDS
---------------
status
check-update
update
rollback
reboot
logs-on
logs-off
add-wifi|SSID|PASSWORD

GPIO
----
GP26: Wi-Fi connection button, active LOW
GP20: OLED SDA
GP21: OLED SCL

No status LED is used.

IMPORTANT
---------
The supervisor assumes chair_logic.stop_outputs() safely
disables wheelchair movement before network operations.

Network connection operations are blocking and must only
run while chair outputs are stopped.
"""

from machine import Pin, I2C
from time import sleep_ms, ticks_ms, ticks_diff

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

RECONNECT_FILE = "reconnect_after_reboot.flag"

MQTT_SERVICE_INTERVAL_MS = 200

CONNECTION_RESULT_DISPLAY_MS = 5000

EVENT_QUEUE_LIMIT = 20

pending_events = []
pending_telemetry = None

remote_telemetry_enabled = True


# ============================================================
# HARDWARE
# ============================================================

# GP26 ---- button ---- GND

connection_button = Pin(
    26,
    Pin.IN,
    Pin.PULL_UP
)


# ============================================================
# OLED SETTINGS
# ============================================================

OLED_WIDTH = 128
OLED_HEIGHT = 64

OLED_ADDRESS = 0x3C

OLED_SDA = 20
OLED_SCL = 21

OLED_I2C_FREQUENCY = 100000

# Standard font

OLED_CHAR_WIDTH = 8
OLED_CHAR_HEIGHT = 8

# Smaller font

SMALL_CHAR_WIDTH = 3
SMALL_CHAR_HEIGHT = 5
SMALL_CHAR_SPACING = 1

# Keep text away from screen edges

OLED_MARGIN_X = 20
OLED_MARGIN_Y = 5

# Main text maximum width

OLED_MAX_CHARS = 10

# Spacing between main lines

OLED_LINE_SPACING = 4

# Spacing before secondary information

OLED_SECTION_SPACING = 7

oled = None

oled_status = "OFFLINE"
oled_detail = ""


# ============================================================
# SMALL BITMAP FONT
# ============================================================

# Each character consists of three columns and five rows.
#
# The characters are stored as five strings.
# Each string represents one horizontal row.
#
# 1 = illuminated pixel
# 0 = unilluminated pixel
#
# This font is independent of the SSD1306 driver.

SMALL_FONT = {

    "A": ("010", "101", "111", "101", "101"),
    "B": ("110", "101", "110", "101", "110"),
    "C": ("011", "100", "100", "100", "011"),
    "D": ("110", "101", "101", "101", "110"),
    "E": ("111", "100", "110", "100", "111"),
    "F": ("111", "100", "110", "100", "100"),
    "G": ("011", "100", "101", "101", "011"),
    "H": ("101", "101", "111", "101", "101"),
    "I": ("111", "010", "010", "010", "111"),
    "J": ("001", "001", "001", "101", "010"),
    "K": ("101", "101", "110", "101", "101"),
    "L": ("100", "100", "100", "100", "111"),
    "M": ("101", "111", "111", "101", "101"),
    "N": ("101", "111", "111", "111", "101"),
    "O": ("010", "101", "101", "101", "010"),
    "P": ("110", "101", "110", "100", "100"),
    "Q": ("010", "101", "101", "111", "011"),
    "R": ("110", "101", "110", "101", "101"),
    "S": ("011", "100", "010", "001", "110"),
    "T": ("111", "010", "010", "010", "010"),
    "U": ("101", "101", "101", "101", "111"),
    "V": ("101", "101", "101", "101", "010"),
    "W": ("101", "101", "111", "111", "101"),
    "X": ("101", "101", "010", "101", "101"),
    "Y": ("101", "101", "010", "010", "010"),
    "Z": ("111", "001", "010", "100", "111"),

    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("110", "001", "010", "100", "111"),
    "3": ("110", "001", "010", "001", "110"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "110", "001", "110"),
    "6": ("011", "100", "110", "101", "010"),
    "7": ("111", "001", "010", "010", "010"),
    "8": ("010", "101", "010", "101", "010"),
    "9": ("010", "101", "011", "001", "110"),

    " ": ("000", "000", "000", "000", "000"),
    ".": ("000", "000", "000", "000", "010"),
    ":": ("000", "010", "000", "010", "000"),
    "-": ("000", "000", "111", "000", "000"),
    "+": ("000", "010", "111", "010", "000"),
    "/": ("001", "001", "010", "100", "100"),
    "_": ("000", "000", "000", "000", "111"),
    "!": ("010", "010", "010", "000", "010"),
    "?": ("110", "001", "010", "000", "010"),
    "(": ("001", "010", "010", "010", "001"),
    ")": ("100", "010", "010", "010", "100"),
    "=": ("000", "111", "000", "111", "000"),
    "&": ("010", "101", "010", "101", "011"),
    "'": ("010", "010", "000", "000", "000"),
}


# ============================================================
# OLED INITIALIZATION
# ============================================================

def initialize_oled():

    global oled

    try:
        import ssd1306

        i2c = I2C(
            0,
            sda=Pin(OLED_SDA),
            scl=Pin(OLED_SCL),
            freq=OLED_I2C_FREQUENCY
        )

        devices = i2c.scan()

        if OLED_ADDRESS not in devices:
            print("OLED not detected.")
            oled = None
            return False

        oled = ssd1306.SSD1306_I2C(
            OLED_WIDTH,
            OLED_HEIGHT,
            i2c,
            addr=OLED_ADDRESS,
            chunk_size=16
        )

        print("OLED initialized.")

        return True

    except Exception as error:

        print(
            "OLED initialization failed:",
            error
        )

        oled = None
        return False


# ============================================================
# OLED TEXT WRAPPING
# ============================================================

def wrap_oled_text(text):

    """
    Wrap standard-size text into lines.

    Prefer splitting at spaces.

    Long words are split into segments.
    """

    text = str(text).strip()

    if not text:
        return []

    lines = []

    words = text.split()

    current_line = ""

    for word in words:

        while len(word) > OLED_MAX_CHARS:

            if current_line:

                lines.append(current_line)
                current_line = ""

            lines.append(
                word[:OLED_MAX_CHARS]
            )

            word = word[OLED_MAX_CHARS:]

        if not word:
            continue

        if not current_line:

            current_line = word

        elif (
            len(current_line) + 1 + len(word)
            <= OLED_MAX_CHARS
        ):

            current_line += " " + word

        else:

            lines.append(current_line)
            current_line = word

    if current_line:
        lines.append(current_line)

    return lines


# ============================================================
# OLED STANDARD TEXT
# ============================================================

def oled_centered_text(text, y):

    """
    Draw standard 8x8 text centered horizontally.
    """

    if oled is None:
        return

    text = str(text)[:OLED_MAX_CHARS]

    text_width = (
        len(text) * OLED_CHAR_WIDTH
    )

    x = (
        OLED_WIDTH - text_width
    ) // 2

    oled.text(
        text,
        x,
        y
    )


# ============================================================
# OLED SMALL TEXT
# ============================================================

def oled_small_character(character, x, y):

    """
    Draw one 3x5 bitmap character.
    """

    if oled is None:
        return

    character = character.upper()

    bitmap = SMALL_FONT.get(
        character,
        SMALL_FONT["?"]
    )

    for row in range(SMALL_CHAR_HEIGHT):

        for column in range(SMALL_CHAR_WIDTH):

            if bitmap[row][column] == "1":

                oled.pixel(
                    x + column,
                    y + row,
                    1
                )


def oled_small_text(text, x, y):

    """
    Draw text using the smaller 3x5 font.
    """

    text = str(text)

    step = (
        SMALL_CHAR_WIDTH
        + SMALL_CHAR_SPACING
    )

    for index, character in enumerate(text):

        oled_small_character(
            character,
            x + index * step,
            y
        )


def oled_centered_small_text(text, y):

    """
    Draw small text centered horizontally.
    """

    if oled is None:
        return

    text = str(text)

    step = (
        SMALL_CHAR_WIDTH
        + SMALL_CHAR_SPACING
    )

    text_width = (
        len(text) * step
        - SMALL_CHAR_SPACING
    )

    x = (
        OLED_WIDTH - text_width
    ) // 2

    oled_small_text(
        text,
        x,
        y
    )


def wrap_small_text(text):

    """
    Wrap small text while preserving screen margins.
    """

    text = str(text).strip()

    if not text:
        return []

    step = (
        SMALL_CHAR_WIDTH
        + SMALL_CHAR_SPACING
    )

    usable_width = (
        OLED_WIDTH - 2 * OLED_MARGIN_X
    )

    max_chars = max(
        1,
        (
            usable_width + SMALL_CHAR_SPACING
        ) // step
    )

    lines = []
    current_line = ""

    for word in text.split():

        while len(word) > max_chars:

            if current_line:
                lines.append(current_line)
                current_line = ""

            lines.append(
                word[:max_chars]
            )

            word = word[max_chars:]

        if not word:
            continue

        if not current_line:

            current_line = word

        elif (
            len(current_line) + 1 + len(word)
            <= max_chars
        ):

            current_line += " " + word

        else:

            lines.append(current_line)
            current_line = word

    if current_line:
        lines.append(current_line)

    return lines


# ============================================================
# OLED STATUS DISPLAY
# ============================================================

def display_status(status, detail="", extra=""):

    """
    Display centered status information.

    Layout:

        MAIN STATUS
        optional main detail

        smaller secondary information

    When extra is supplied:

        status = main heading
        detail = main detail
        extra  = small secondary information

    Otherwise:

        status = main heading
        detail = small secondary information
    """

    global oled_status
    global oled_detail

    oled_status = str(status)
    oled_detail = str(detail)

    if oled is None:
        return

    try:

        oled.fill(0)

        # --------------------------------------------
        # Determine main and secondary text
        # --------------------------------------------

        main_lines = wrap_oled_text(
            status
        )

        if extra:

            main_lines.extend(
                wrap_oled_text(detail)
            )

            secondary_lines = wrap_small_text(
                extra
            )

        else:

            secondary_lines = wrap_small_text(
                detail
            )

        # --------------------------------------------
        # Calculate heights
        # --------------------------------------------

        main_count = len(main_lines)
        secondary_count = len(secondary_lines)

        main_height = 0

        if main_count:

            main_height = (
                main_count * OLED_CHAR_HEIGHT
                + (main_count - 1) * OLED_LINE_SPACING
            )

        secondary_height = 0

        if secondary_count:

            secondary_height = (
                secondary_count * SMALL_CHAR_HEIGHT
                + (secondary_count - 1) * 3
            )

        section_spacing = 0

        if main_count and secondary_count:

            section_spacing = OLED_SECTION_SPACING

        total_height = (
            main_height
            + section_spacing
            + secondary_height
        )

        # --------------------------------------------
        # Ensure content fits vertically
        # --------------------------------------------

        available_height = (
            OLED_HEIGHT - 2 * OLED_MARGIN_Y
        )

        while (
            total_height > available_height
            and len(main_lines) > 1
        ):

            main_lines.pop()

            main_count = len(main_lines)

            main_height = (
                main_count * OLED_CHAR_HEIGHT
                + (main_count - 1) * OLED_LINE_SPACING
            )

            total_height = (
                main_height
                + section_spacing
                + secondary_height
            )

        while (
            total_height > available_height
            and secondary_lines
        ):

            secondary_lines.pop()

            secondary_count = len(secondary_lines)

            if secondary_count:

                secondary_height = (
                    secondary_count * SMALL_CHAR_HEIGHT
                    + (secondary_count - 1) * 3
                )

            else:

                secondary_height = 0
                section_spacing = 0

            total_height = (
                main_height
                + section_spacing
                + secondary_height
            )

        # --------------------------------------------
        # Vertical centering
        # --------------------------------------------

        y = (
            OLED_HEIGHT - total_height
        ) // 2

        # --------------------------------------------
        # Draw main text
        # --------------------------------------------

        for line in main_lines:

            oled_centered_text(
                line,
                y
            )

            y += (
                OLED_CHAR_HEIGHT
                + OLED_LINE_SPACING
            )

        # Remove the final line spacing

        if main_lines:

            y -= OLED_LINE_SPACING

        # --------------------------------------------
        # Draw secondary text
        # --------------------------------------------

        if secondary_lines:

            y += section_spacing

            for line in secondary_lines:

                oled_centered_small_text(
                    line,
                    y
                )

                y += (
                    SMALL_CHAR_HEIGHT + 3
                )

        oled.show()

    except Exception as error:

        print(
            "OLED update failed:",
            error
        )


def hold_connection_result():

    if CONNECTION_RESULT_DISPLAY_MS > 0:

        sleep_ms(
            CONNECTION_RESULT_DISPLAY_MS
        )


# ============================================================
# STATE
# ============================================================

connection_requested = False

last_mqtt_service_ms = ticks_ms()

chair_logic = None

recovery_mode = False
recovery_error = None


# ============================================================
# LOCAL LOGGING
# ============================================================

def start_log():

    try:

        with open(LOG_FILE, "a") as file:

            file.write(
                "\n==============================\n"
            )

            file.write("BOOT\n")

    except Exception as error:

        print(
            "Could not open log:",
            error
        )


def write_event_to_file(message):

    try:

        with open(LOG_FILE, "a") as file:

            file.write(str(message))
            file.write("\n")

    except Exception:
        pass


def local_event_log(message=""):

    message = str(message)

    print(message)

    write_event_to_file(message)


def event_log(message=""):

    message = str(message)

    print(message)

    write_event_to_file(message)

    queue_event(message)


def chair_log(message=""):

    message = str(message)

    print(message)

    if remote_telemetry_enabled:
        queue_telemetry(message)


# ============================================================
# MQTT QUEUES
# ============================================================

def queue_event(message):

    message = str(message)

    if len(pending_events) >= EVENT_QUEUE_LIMIT:
        pending_events.pop(0)

    pending_events.append(message)


def queue_telemetry(message):

    global pending_telemetry

    pending_telemetry = str(message)


def clear_mqtt_queues():

    global pending_events
    global pending_telemetry

    pending_events = []
    pending_telemetry = None


# ============================================================
# REMOTE TELEMETRY
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
# RECONNECT FLAG
# ============================================================

def request_reconnect_after_reboot(reason):

    try:

        with open(RECONNECT_FILE, "w") as file:
            file.write(str(reason))

        return True

    except Exception as error:

        local_event_log(
            "Could not create reconnect flag: {}".format(
                error
            )
        )

        return False


def take_reconnect_after_reboot():

    try:

        with open(RECONNECT_FILE, "r") as file:
            reason = file.read().strip()

    except OSError:
        return None

    try:

        os.remove(RECONNECT_FILE)

    except Exception as error:

        local_event_log(
            "Could not remove reconnect flag: {}".format(
                error
            )
        )

        return None

    if reason in ("reboot", "update", "rollback"):
        return reason

    return "reboot"


# ============================================================
# CONNECTION BUTTON
# ============================================================

def check_connection_button():

    global connection_requested

    if connection_button.value() != 0:

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
# MQTT CALLBACK
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

    if pending_events:

        message = pending_events[0]

        try:

            if mqtt_manager.send_log(message):

                pending_events.pop(0)
                return True

        except Exception:
            pass

        return False

    if (
        remote_telemetry_enabled
        and pending_telemetry is not None
    ):

        message = pending_telemetry

        try:

            if mqtt_manager.send_log(message):

                if pending_telemetry == message:
                    pending_telemetry = None

                return True

        except Exception:
            pass

    return False


def send_remote_now(message):

    message = str(message)

    print(message)
    write_event_to_file(message)

    if not mqtt_manager.is_connected():
        return False

    try:
        return mqtt_manager.send_log(message)

    except Exception:
        return False


# ============================================================
# MQTT SERVICE
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

    except Exception as error:

        # Avoid repeatedly writing failures to flash.
        print(
            "MQTT service error:",
            error
        )

        return

    flush_one_mqtt_message()


# ============================================================
# SUPERVISOR CALLBACK
# ============================================================

def supervisor_service():

    if check_connection_button():
        return True

    service_mqtt_if_due()

    command = command_manager.pending_command()

    if command is None:
        return False

    return True


# ============================================================
# NETWORK HELPERS
# ============================================================

def wifi_is_connected():

    try:
        return bool(wifi_manager.is_connected())

    except Exception:
        return False


def mqtt_is_connected():

    try:
        return bool(mqtt_manager.is_connected())

    except Exception:
        return False


def get_connected_ssid():

    """
    Return the currently connected Wi-Fi SSID.

    Returns an empty string if unavailable.
    """

    if not wifi_is_connected():
        return ""

    try:

        ssid = wifi_manager.current_ssid()

        if ssid is None:
            return ""

        if isinstance(ssid, bytes):
            ssid = ssid.decode("utf-8")

        return str(ssid)

    except Exception:
        return ""


def connect_wifi_with_retries():

    """
    Attempt Wi-Fi connection once.

    Uses the existing wifi_manager implementation.
    """

    if wifi_is_connected():

        event_log(
            "Wi-Fi already connected."
        )

        display_status(
            "WIFI OK",
            "Already connected",
            get_connected_ssid()
        )

        return True

    display_status(
        "CONNECTING",
        "WiFi"
    )

    event_log(
        "Attempting Wi-Fi connection."
    )

    try:

        result = wifi_manager.connect(
            log=event_log
        )

        if result or wifi_is_connected():

            event_log(
                "Wi-Fi connection established."
            )

            display_status(
                "WIFI OK",
                "Connecting MQTT",
                get_connected_ssid()
            )

            return True

    except Exception as error:

        event_log(
            "Wi-Fi connection error: {}".format(
                error
            )
        )

    event_log(
        "Wi-Fi connection attempt failed."
    )

    return False


def connect_mqtt_with_retries():

    """
    Attempt MQTT connection once.
    """

    if mqtt_is_connected():

        event_log(
            "MQTT already connected."
        )

        return True

    display_status(
        "MQTT CONNECT",
        "Connecting..."
    )

    event_log(
        "Attempting MQTT connection."
    )

    try:

        result = mqtt_manager.connect(
            log=event_log
        )

        if result or mqtt_is_connected():

            event_log(
                "MQTT connection established."
            )

            return True

    except Exception as error:

        event_log(
            "MQTT connection error: {}".format(
                error
            )
        )

    event_log(
        "MQTT connection attempt failed."
    )

    return False


# ============================================================
# NETWORK CONNECTION
# ============================================================

def connect_network(wait_for_release=True):

    event_log("")
    event_log("==============================")
    event_log("NETWORK CONNECTION")
    event_log("==============================")

    event_log(
        "Wheelchair outputs temporarily disabled."
    )

    display_status(
        "CONNECTING",
        "Starting..."
    )

    wifi_ok = connect_wifi_with_retries()

    mqtt_ok = False

    if wifi_ok:

        mqtt_ok = connect_mqtt_with_retries()

    # --------------------------------------------------------
    # RESULT
    # --------------------------------------------------------

    if wifi_ok and mqtt_ok:

        event_log(
            "Remote logging and commands connected."
        )

        ssid = get_connected_ssid()

        if ssid:

            display_status(
                "Connected to",
                ssid,
                "WiFi + MQTT OK"
            )

        else:

            display_status(
                "CONNECTED",
                "WiFi + MQTT OK"
            )

    elif not wifi_ok:

        event_log(
            "Wi-Fi connection failed."
        )

        display_status(
            "WIFI FAILED",
            "Check WiFi"
        )

    else:

        event_log(
            "MQTT connection failed."
        )

        display_status(
            "MQTT FAILED",
            "WiFi connected",
            get_connected_ssid()
        )

    if wait_for_release:
        wait_for_button_release()

    hold_connection_result()

    return wifi_ok and mqtt_ok


# ============================================================
# STATUS
# ============================================================

def report_status():

    event_log("")
    event_log("==============================")
    event_log("STATUS")
    event_log("==============================")

    try:
        version = updater.current_version()

    except Exception:
        version = "unknown"

    event_log(
        "Version: {}".format(version)
    )

    wifi_connected = wifi_is_connected()

    event_log(
        "Wi-Fi: {}".format(
            "connected" if wifi_connected else "disconnected"
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

    event_log(
        "MQTT: {}".format(
            "connected" if mqtt_is_connected() else "disconnected"
        )
    )

    event_log(
        "Remote telemetry: {}".format(
            "ON" if remote_telemetry_enabled else "OFF"
        )
    )

    event_log(
        "Mode: {}".format(
            "RECOVERY" if recovery_mode else "NORMAL"
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
    event_log("Checking for software update.")

    try:

        result = updater.check_update()

        local_version = result["local"]
        remote_version = result["remote"]

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

        if result["update_available"]:

            event_log("Update available.")

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

    ssid = command.get("ssid")
    password = command.get("password")

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

    flag_ok = request_reconnect_after_reboot(
        "update"
    )

    local_event_log(
        "Reconnect flag created: {}".format(
            flag_ok
        )
    )

    try:

        with open(RECONNECT_FILE, "r") as file:
            flag_contents = file.read().strip()

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

    display_status(
        "REBOOTING",
        "Update installed"
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

    display_status(
        "REBOOTING",
        "Rollback complete"
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

    display_status("REBOOTING")

    local_event_log("Rebooting.")

    sleep_ms(500)

    machine.reset()


# ============================================================
# COMMAND PROCESSING
# ============================================================

def process_pending_command():

    command = command_manager.take_command()

    if command is None:
        return

    command_name = command.get("command")

    if command_name in (
        command_manager.COMMAND_UPDATE,
        command_manager.COMMAND_ROLLBACK,
        command_manager.COMMAND_REBOOT
    ):

        local_event_log(
            "Processing command: {}".format(
                command_manager.describe(command)
            )
        )

    else:

        event_log(
            "Processing command: {}".format(
                command_manager.describe(command)
            )
        )

    if command_name == command_manager.COMMAND_STATUS:

        report_status()

    elif command_name == command_manager.COMMAND_CHECK_UPDATE:

        handle_check_update()

    elif command_name == command_manager.COMMAND_ADD_WIFI:

        handle_add_wifi(command)

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

        event_log("Unsupported command.")


# ============================================================
# CHAIR LOGIC LOADING
# ============================================================

def load_chair_logic():

    global chair_logic
    global recovery_mode
    global recovery_error

    try:

        if "chair_logic" in sys.modules:
            del sys.modules["chair_logic"]

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

        event_log(recovery_error)

        display_status(
            "RECOVERY",
            "Logic load failed"
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

        clear_mqtt_queues()

        report_post_reboot_success(reason)

    return connected


# ============================================================
# RECOVERY MODE
# ============================================================

def recovery_loop(auto_reconnect_reason=None):

    global last_mqtt_service_ms

    display_status(
        "RECOVERY",
        "Control disabled"
    )

    local_event_log("")
    local_event_log("==============================")
    local_event_log("RECOVERY MODE")
    local_event_log("==============================")

    local_event_log(
        "Wheelchair control is disabled."
    )

    if auto_reconnect_reason is not None:

        automatic_reconnect(
            auto_reconnect_reason
        )

    event_log(
        "Press GP26 to connect to the network."
    )

    while True:

        if check_connection_button():

            connect_network(
                wait_for_release=True
            )

            event_log(
                "Recovery mode remains active."
            )

        now = ticks_ms()

        if ticks_diff(
            now,
            last_mqtt_service_ms
        ) >= MQTT_SERVICE_INTERVAL_MS:

            last_mqtt_service_ms = now

            if mqtt_is_connected():

                try:
                    mqtt_manager.check_messages()

                except Exception:
                    pass

                flush_one_mqtt_message()

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

initialize_oled()

display_status(
    "OFFLINE",
    "Starting..."
)

auto_reconnect_reason = (
    take_reconnect_after_reboot()
)

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
    # AUTOMATIC POST-REBOOT CONNECTION
    # --------------------------------------------------------

    if auto_reconnect_reason is not None:

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
            event_log(recovery_error)

            event_log(
                "Entering recovery mode."
            )

            display_status(
                "RECOVERY",
                "Runtime error"
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
        # GP26 NETWORK REQUEST
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

    display_status(
        "STOPPED",
        "KeyboardInterrupt"
    )

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

    display_status(
        "ERROR",
        "Supervisor failed"
    )

    local_event_log("")
    local_event_log(
        "SUPERVISOR ERROR: {}".format(
            error
        )
    )

    raise
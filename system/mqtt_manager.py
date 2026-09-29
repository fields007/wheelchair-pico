"""
MQTT communication manager.

Responsibilities:
- connect to the configured MQTT broker
- subscribe to the wheelchair command topic
- receive command messages
- publish log messages
- report MQTT connection state

This module does NOT interpret commands.
It simply passes received command strings to a callback.

This module assumes Wi-Fi / Internet connectivity already exists.

When imported:
    Nothing happens automatically.

When run directly:
    An MQTT self-test is performed.
"""

import ssl
import machine
import binascii

from time import sleep_ms
from umqtt.simple import MQTTClient


# ============================================================
# SETTINGS
# ============================================================

MQTT_HOST = "db20b5685311422ebaec44a84fcd7036.s1.eu.hivemq.cloud"
MQTT_PORT = 8883

MQTT_USER = "wheelchair-pico"

# Put your current HiveMQ password here.
MQTT_PASSWORD = "jufxi8-tatqec-baThox"

MQTT_LOG_TOPIC = b"wheelchair/logs"
MQTT_COMMAND_TOPIC = b"wheelchair/command"

MQTT_KEEPALIVE = 60


# ============================================================
# STATE
# ============================================================

mqtt_client = None

command_callback = None

mqtt_connected = False


# ============================================================
# INTERNAL MESSAGE CALLBACK
# ============================================================

def _message_received(topic, message):
    """
    Internal callback used by umqtt.simple.

    Command messages are decoded and passed to the callback
    registered by set_command_callback().
    """

    if topic != MQTT_COMMAND_TOPIC:
        return

    try:

        command = message.decode()

    except Exception:

        return

    if command_callback is None:
        return

    try:

        command_callback(command)

    except Exception as error:

        # A bad command handler must not crash MQTT processing.
        print(
            "Command callback error:",
            error
        )


# ============================================================
# COMMAND CALLBACK
# ============================================================

def set_command_callback(callback):
    """
    Set the function that receives command strings.

    Example:

        def command_received(command):
            print(command)

        set_command_callback(command_received)

    The callback receives one string argument.
    """

    global command_callback

    command_callback = callback


# ============================================================
# CONNECTION
# ============================================================

def connect(log=print):
    """
    Connect to the MQTT broker and subscribe to commands.

    Wi-Fi must already be connected.

    Returns True on success.
    Returns False on failure.
    """

    global mqtt_client
    global mqtt_connected

    mqtt_connected = False
    mqtt_client = None

    try:

        client_id = binascii.hexlify(
            machine.unique_id()
        )

        context = ssl.SSLContext(
            ssl.PROTOCOL_TLS_CLIENT
        )

        # This is the TLS configuration already tested
        # successfully with the Pico and HiveMQ.
        context.verify_mode = ssl.CERT_NONE

        client = MQTTClient(
            client_id=client_id,
            server=MQTT_HOST,
            port=MQTT_PORT,
            user=MQTT_USER,
            password=MQTT_PASSWORD,
            keepalive=MQTT_KEEPALIVE,
            ssl=context
        )

        client.set_callback(
            _message_received
        )

        log("Connecting to MQTT.")

        client.connect()

        log("MQTT connected.")

        client.subscribe(
            MQTT_COMMAND_TOPIC
        )

        log(
            "Subscribed to {}".format(
                MQTT_COMMAND_TOPIC.decode()
            )
        )

        mqtt_client = client
        mqtt_connected = True

        return True

    except Exception as error:

        mqtt_client = None
        mqtt_connected = False

        log(
            "MQTT connection failed: {}".format(
                error
            )
        )

        return False


# ============================================================
# DISCONNECT
# ============================================================

def disconnect():
    """
    Disconnect MQTT if possible.

    Safe to call even when MQTT is already disconnected.
    """

    global mqtt_client
    global mqtt_connected

    if mqtt_client is not None:

        try:
            mqtt_client.disconnect()

        except Exception:
            pass

    mqtt_client = None
    mqtt_connected = False


# ============================================================
# STATUS
# ============================================================

def is_connected():
    """
    Return whether this module currently considers MQTT
    connected.

    A later network error will cause this state to become False.
    """

    return (
        mqtt_connected
        and mqtt_client is not None
    )


# ============================================================
# PUBLISH LOG
# ============================================================

def send_log(message):
    """
    Publish one message to wheelchair/logs.

    Returns True if publishing succeeded.
    Returns False if MQTT is unavailable or publishing fails.

    A network failure is deliberately swallowed.
    """

    global mqtt_client
    global mqtt_connected

    if not is_connected():
        return False

    try:

        mqtt_client.publish(
            MQTT_LOG_TOPIC,
            str(message).encode()
        )

        return True

    except Exception:

        mqtt_client = None
        mqtt_connected = False

        return False


# ============================================================
# RECEIVE / SERVICE MQTT
# ============================================================

def check_messages():
    """
    Check once for incoming MQTT messages.

    This function does NOT wait for a message.

    If a command is waiting, _message_received() will be called.

    Returns True if MQTT remains usable.
    Returns False if MQTT is unavailable or an error occurs.
    """

    global mqtt_client
    global mqtt_connected

    if not is_connected():
        return False

    try:

        mqtt_client.check_msg()

        return True

    except Exception:

        mqtt_client = None
        mqtt_connected = False

        return False


# ============================================================
# SELF-TEST
# ============================================================

def self_test():
    """
    Test MQTT communication.

    This test assumes Wi-Fi is already available or can be
    established using system.wifi_manager.

    The test:
    - connects to Wi-Fi
    - connects to MQTT
    - subscribes to wheelchair/command
    - sends a test log message
    - waits for incoming commands

    No commands are executed.
    Received messages are only printed.

    Stop the test with Ctrl-C.
    """

    print()
    print("==============================")
    print("MQTT MANAGER SELF-TEST")
    print("==============================")
    print()

    # --------------------------------------------------------
    # WIFI
    # --------------------------------------------------------

    print("1. Connecting to Wi-Fi...")

    try:

        from system import wifi_manager

    except ImportError:

        # This makes direct execution a little more convenient
        # on MicroPython installations where the import context
        # differs.
        import wifi_manager

    if not wifi_manager.is_connected():

        if not wifi_manager.connect():

            print()
            print("FAILED: Wi-Fi connection failed.")
            return

    print()
    print("OK: Wi-Fi connected.")
    print(
        "IP: {}".format(
            wifi_manager.ip_address()
        )
    )

    # --------------------------------------------------------
    # RECEIVE CALLBACK
    # --------------------------------------------------------

    print()
    print("2. Installing test command callback.")

    def test_command_received(command):

        print()
        print("------------------------------")
        print("COMMAND RECEIVED")
        print("------------------------------")
        print(command)
        print("------------------------------")
        print()

    set_command_callback(
        test_command_received
    )

    # --------------------------------------------------------
    # MQTT CONNECTION
    # --------------------------------------------------------

    print()
    print("3. Connecting to MQTT...")

    if not connect():

        print()
        print("==============================")
        print("MQTT SELF-TEST FAILED")
        print("==============================")
        return

    # --------------------------------------------------------
    # PUBLISH TEST
    # --------------------------------------------------------

    print()
    print("4. Testing MQTT publishing...")

    if send_log("MQTT manager self-test connected."):

        print("OK: Test log published.")

    else:

        print("FAILED: Could not publish test log.")

        disconnect()

        return

    # --------------------------------------------------------
    # RECEIVE TEST
    # --------------------------------------------------------

    print()
    print("5. Testing MQTT reception.")
    print()
    print(
        "Send a command from the Mac, for example:"
    )
    print()
    print("    wheelchair-send hello")
    print()
    print("Waiting for commands.")
    print("Press Ctrl-C to finish the test.")
    print()

    try:

        while True:

            if not check_messages():

                print()
                print("MQTT connection was lost.")
                break

            sleep_ms(50)

    except KeyboardInterrupt:

        print()
        print("Self-test stopped.")

    # --------------------------------------------------------
    # CLEANUP
    # --------------------------------------------------------

    disconnect()

    print("MQTT disconnected.")

    print()
    print("==============================")
    print("MQTT SELF-TEST FINISHED")
    print("==============================")
    print()


# ============================================================
# RUN DIRECTLY
# ============================================================

if __name__ == "__main__":

    self_test()
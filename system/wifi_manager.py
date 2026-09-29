"""
Wi-Fi manager.

Responsibilities:
- load known Wi-Fi networks from wifi.json
- save Wi-Fi networks to wifi.json
- add or update a saved Wi-Fi network
- connect to a known Wi-Fi network
- report current Wi-Fi status

This module knows nothing about MQTT, commands, LEDs,
wheelchair control, or software updates.

When imported:
    Nothing happens automatically.

When run directly:
    A non-destructive Wi-Fi self-test is performed.
"""

import network
import json

from time import sleep_ms, ticks_ms, ticks_diff


# ============================================================
# SETTINGS
# ============================================================

WIFI_FILE = "wifi.json"

# Maximum time spent trying each configured network.
WIFI_TIMEOUT_MS = 10000


# ============================================================
# STATE
# ============================================================

wlan = None


# ============================================================
# CONFIGURATION
# ============================================================

def load_networks():
    """
    Load the list of configured Wi-Fi networks.

    Returns an empty list if the file cannot be read or its
    contents are invalid.
    """

    try:

        with open(WIFI_FILE, "r") as file:
            config = json.load(file)

    except Exception:
        return []

    networks = config.get(
        "networks",
        []
    )

    if not isinstance(networks, list):
        return []

    return networks


def save_networks(networks):
    """
    Save the supplied network list to wifi.json.

    Returns True on success.
    Returns False on failure.
    """

    if not isinstance(networks, list):
        return False

    try:

        config = {
            "networks": networks
        }

        with open(WIFI_FILE, "w") as file:

            json.dump(
                config,
                file
            )

        return True

    except Exception:
        return False


def add_network(ssid, password):
    """
    Add a Wi-Fi network to wifi.json.

    If the SSID already exists, its password is replaced.

    Returns True if successfully saved.
    Returns False otherwise.

    This function only stores the credentials.
    It does NOT change the current Wi-Fi connection.
    """

    if not ssid:
        return False

    networks = load_networks()

    found = False

    for entry in networks:

        if not isinstance(entry, dict):
            continue

        if entry.get("ssid") == ssid:

            entry["password"] = password

            found = True

            break

    if not found:

        networks.append(
            {
                "ssid": ssid,
                "password": password
            }
        )

    return save_networks(
        networks
    )


# ============================================================
# CONNECTION
# ============================================================

def connect(log=print):
    """
    Try each configured Wi-Fi network.

    Returns True if connected.
    Returns False if no configured network can be reached.
    """

    global wlan

    log("Loading Wi-Fi configuration.")

    networks = load_networks()

    if not networks:

        log(
            "No usable Wi-Fi networks configured."
        )

        return False

    log(
        "Found {} configured Wi-Fi network(s).".format(
            len(networks)
        )
    )

    # Use the current MicroPython station-interface API.
    wlan = network.WLAN(
        network.WLAN.IF_STA
    )

    wlan.active(True)

    # Give the CYW43 Wi-Fi subsystem time to initialise.
    sleep_ms(500)

    # Something may already have connected Wi-Fi.
    if wlan.isconnected():

        log("Wi-Fi already connected.")

        log(
            "IP address: {}".format(
                wlan.ifconfig()[0]
            )
        )

        return True

    # Perform a scan before connecting.
    #
    # On the Pico 2 W this also gives the CYW43 Wi-Fi
    # subsystem an opportunity to finish initialising before
    # wlan.connect() is called.
    try:

        log("Scanning for Wi-Fi networks.")

        wlan.scan()

    except Exception as error:

        log(
            "Initial Wi-Fi scan failed: {}".format(
                error
            )
        )

    for entry in networks:

        if not isinstance(entry, dict):
            continue

        ssid = entry.get("ssid")

        password = entry.get(
            "password",
            ""
        )

        if not ssid:
            continue

        log(
            "Trying Wi-Fi: {}".format(
                ssid
            )
        )

        # Disconnect from any previous failed attempt.
        try:

            wlan.disconnect()

        except Exception:

            pass

        sleep_ms(200)

        try:

            wlan.connect(
                ssid,
                password
            )

        except Exception as error:

            log(
                "Could not start Wi-Fi connection to {}: {}".format(
                    ssid,
                    error
                )
            )

            continue

        start = ticks_ms()

        while not wlan.isconnected():

            if ticks_diff(
                ticks_ms(),
                start
            ) >= WIFI_TIMEOUT_MS:

                log(
                    "Wi-Fi timed out: {}".format(
                        ssid
                    )
                )

                break

            sleep_ms(100)

        if wlan.isconnected():

            log(
                "Connected to Wi-Fi: {}".format(
                    ssid
                )
            )

            log(
                "IP address: {}".format(
                    wlan.ifconfig()[0]
                )
            )

            return True

    log(
        "Could not connect to any configured Wi-Fi network."
    )

    return False


# ============================================================
# STATUS
# ============================================================

def is_connected():
    """
    Return True if Wi-Fi is currently connected.
    """

    if wlan is None:
        return False

    try:

        return wlan.isconnected()

    except Exception:

        return False


def ip_address():
    """
    Return the current IP address.

    Returns None when Wi-Fi is not connected.
    """

    if not is_connected():
        return None

    try:

        return wlan.ifconfig()[0]

    except Exception:

        return None


def current_ssid():
    """
    Return the SSID of the currently connected network.

    Returns None if unavailable.
    """

    if not is_connected():
        return None

    try:

        return wlan.config("ssid")

    except Exception:

        return None


# ============================================================
# SELF-TEST
# ============================================================

def self_test():
    """
    Perform a non-destructive test of this module.

    The test:
    - reads wifi.json
    - displays configured SSIDs
    - does NOT display passwords
    - attempts to connect
    - displays the resulting Wi-Fi status

    No Wi-Fi credentials are modified.
    """

    print()
    print("==============================")
    print("WIFI MANAGER SELF-TEST")
    print("==============================")
    print()

    # --------------------------------------------------------
    # CONFIGURATION TEST
    # --------------------------------------------------------

    print("1. Reading wifi.json...")

    networks = load_networks()

    if not networks:

        print("FAILED: No usable networks found.")
        print()
        return

    print(
        "OK: Found {} configured network(s).".format(
            len(networks)
        )
    )

    for index, entry in enumerate(networks):

        if not isinstance(entry, dict):
            continue

        ssid = entry.get(
            "ssid",
            "<missing SSID>"
        )

        print(
            "   {}: {}".format(
                index + 1,
                ssid
            )
        )

    print()

    # --------------------------------------------------------
    # CONNECTION TEST
    # --------------------------------------------------------

    print("2. Testing Wi-Fi connection...")
    print()

    success = connect(
        log=print
    )

    print()

    if not success:

        print("FAILED: Could not connect to Wi-Fi.")
        print()
        return

    print("OK: Wi-Fi connected.")

    # --------------------------------------------------------
    # STATUS FUNCTIONS
    # --------------------------------------------------------

    print()
    print("3. Testing status functions...")

    print(
        "   is_connected(): {}".format(
            is_connected()
        )
    )

    print(
        "   current_ssid(): {}".format(
            current_ssid()
        )
    )

    print(
        "   ip_address(): {}".format(
            ip_address()
        )
    )

    print()

    # --------------------------------------------------------
    # RESULT
    # --------------------------------------------------------

    if (
        is_connected()
        and ip_address() is not None
    ):

        print("==============================")
        print("WIFI SELF-TEST PASSED")
        print("==============================")

    else:

        print("==============================")
        print("WIFI SELF-TEST FAILED")
        print("==============================")

    print()


# ============================================================
# RUN DIRECTLY
# ============================================================

if __name__ == "__main__":

    self_test()
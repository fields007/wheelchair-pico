"""Wi-Fi management for the wheelchair Pico 2 W.

Loads remembered networks from wifi.json, scans for available
networks, and connects to the first remembered network it finds.
"""

import json
import network
import time


WIFI_FILE = "wifi.json"
CONNECTION_TIMEOUT_MS = 10000


def load_networks():
    """Return the list of remembered Wi-Fi networks."""

    with open(WIFI_FILE, "r") as f:
        config = json.load(f)

    return config.get("networks", [])


def connect():
    """Connect to an available remembered Wi-Fi network.

    Returns the active WLAN object if successful.
    Returns None if no remembered network can be connected.
    """

    networks = load_networks()

    wlan = network.WLAN(network.WLAN.IF_STA)
    wlan.active(True)

    print("Scanning for WiFi...")

    available = set()

    for result in wlan.scan():
        try:
            ssid = result[0].decode("utf-8")
            if ssid:
                available.add(ssid)
        except Exception:
            pass

    for saved in networks:

        ssid = saved.get("ssid")
        password = saved.get("password")

        if not ssid or ssid not in available:
            continue

        print("Connecting to:", ssid)

        wlan.connect(ssid, password)

        start = time.ticks_ms()

        while time.ticks_diff(
            time.ticks_ms(),
            start
        ) < CONNECTION_TIMEOUT_MS:

            if wlan.isconnected():

                print("WiFi connected:", ssid)
                print("IP:", wlan.ipconfig("addr4"))

                return wlan

            time.sleep_ms(100)

        print("Connection failed:", ssid)

        wlan.disconnect()

    print("No remembered WiFi available.")

    return None


def disconnect(wlan):
    """Disconnect and disable the Wi-Fi radio."""

    if wlan is None:
        return

    try:
        wlan.disconnect()
    except Exception:
        pass

    wlan.active(False)

    print("WiFi disabled.")
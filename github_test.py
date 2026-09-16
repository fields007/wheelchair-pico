import network
import time
import json
import requests


# ============================================================
# CONNECT TO REMEMBERED WIFI
# ============================================================

print("Loading wifi.json...")

with open("wifi.json", "r") as f:
    config = json.load(f)

saved_networks = config["networks"]


wlan = network.WLAN(network.WLAN.IF_STA)
wlan.active(True)


print("Scanning for WiFi...")

available = wlan.scan()

available_ssids = []

for result in available:
    try:
        ssid = result[0].decode("utf-8")
        available_ssids.append(ssid)
    except:
        pass


connected = False


for saved in saved_networks:

    ssid = saved["ssid"]
    password = saved["password"]

    if ssid not in available_ssids:
        continue

    print("Connecting to:", ssid)

    wlan.connect(ssid, password)

    for _ in range(100):

        if wlan.isconnected():
            connected = True
            break

        time.sleep_ms(100)

    if connected:
        break

    wlan.disconnect()


if not connected:

    print("WiFi connection FAILED.")

else:

    print("WiFi connected!")
    print("IP:", wlan.ipconfig("addr4"))


    # ========================================================
    # FETCH VERSION FROM GITHUB
    # ========================================================

    VERSION_URL = (
        "https://" +
        "raw.githubusercontent.com/" +
        "YOUR_GITHUB_USERNAME/" +
        "YOUR_REPOSITORY_NAME/" +
        "main/version.txt"
    )


    print()
    print("Fetching version from GitHub...")


    try:

        response = requests.get(VERSION_URL)

        print("HTTP status:", response.status_code)

        if response.status_code == 200:

            github_version = response.text.strip()

            print("GitHub version:", github_version)

        else:

            print("Could not fetch version.txt")

        response.close()


    except Exception as e:

        print("GitHub request FAILED:")
        print(e)
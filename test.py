import network
import time

SSID = "Wifi"
PASSWORD = "YOUR_PASSWORD"

wlan = network.WLAN(network.WLAN.IF_STA)

print("Activating Wi-Fi...")
wlan.active(True)

print("Scanning...")
networks = wlan.scan()
print("Found", len(networks), "network(s)")

print("Connecting...")
wlan.connect(SSID, PASSWORD)

for i in range(30):
    status = wlan.status()
    print("status:", status)

    if wlan.isconnected():
        print("CONNECTED")
        print("IP:", wlan.ipconfig("addr4"))
        break

    time.sleep(1)
else:
    print("FAILED")
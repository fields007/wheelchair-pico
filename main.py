"""Wheelchair startup supervisor.

Eventually the startup sequence will be:

1. Connect to remembered Wi-Fi.
2. Check GitHub for an update.
3. Install update if necessary.
4. Disable Wi-Fi.
5. Start chair_logic.py.

For now this only tests the Wi-Fi manager.
"""

import wifi_manager


print()
print("=== WHEELCHAIR STARTUP ===")
print()

wlan = wifi_manager.connect()

if wlan is not None:
    print()
    print("Startup WiFi test successful.")

else:
    print()
    print("Starting without WiFi.")


wifi_manager.disconnect(wlan)

print()
print("Startup finished.")
"""Rift CV1 Control Center — manage an Oculus Rift CV1 on Linux/SteamVR.

Wraps the OpenHMD (rift-kalman-filter) + SteamVR-OpenHMD stack:
  * status of headset USB/HID, tracking sensors, HDMI link, driver, runtime
  * wake test (verifies the display link end-to-end)
  * USB reset for the "HID not bound after headset reboot" wedge
  * Touch controller pairing via ouvrt
  * OpenVR runtime switching (SteamVR <-> WiVRn) and SteamVR launch
  * CLI mode for scripting (status / fix-usb / test / export / …)
"""

#!/usr/bin/env python3
"""Fall back to the AP hotspot if wlan0 hasn't connected within TIMEOUT seconds of boot."""

import subprocess
import syslog
import time

AP_CONN = "Hotspot"
TIMEOUT = 30
INTERVAL = 2


def wlan0_connected():
    out = subprocess.run(
        ["nmcli", "-t", "-f", "DEVICE,STATE", "dev", "status"],
        capture_output=True, text=True
    ).stdout
    return "wlan0:connected" in out


def main():
    elapsed = 0
    while elapsed < TIMEOUT:
        if wlan0_connected():
            syslog.syslog("wifi_fallback: wlan0 connected, not starting AP")
            return
        time.sleep(INTERVAL)
        elapsed += INTERVAL

    syslog.syslog(f"wifi_fallback: no wifi after {TIMEOUT}s, starting {AP_CONN}")
    subprocess.run(["nmcli", "con", "up", AP_CONN])


if __name__ == "__main__":
    main()

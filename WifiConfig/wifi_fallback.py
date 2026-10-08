#!/usr/bin/env python3
"""Old name, kept so the boot service that points here keeps working. The code is in wifi.py.
Delete this file after `sudo python3 wifi.py install` and disabling the old service."""

import wifi

if __name__ == "__main__":
    wifi.watch()

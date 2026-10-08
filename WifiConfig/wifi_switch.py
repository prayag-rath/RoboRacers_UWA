#!/usr/bin/env python3
"""Interactive switcher between saved NetworkManager connection profiles."""

import subprocess
import sys


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


def get_wifi_connections():
    out = run(["nmcli", "-t", "-f", "NAME,TYPE", "con", "show"]).stdout.strip()
    conns = []
    for line in out.splitlines():
        name, ctype = line.rsplit(":", 1)
        if ctype == "802-11-wireless":
            conns.append(name)
    return conns


def get_active_connections():
    return run(["nmcli", "-t", "-f", "NAME", "con", "show", "--active"]).stdout.strip().splitlines()


def switch_to(name):
    print(f"Switching to '{name}'...")
    result = subprocess.run(["nmcli", "con", "up", name])
    print(f"Connected to {name}." if result.returncode == 0 else "Failed — see nmcli output above.")


def main():
    conns = get_wifi_connections()
    if not conns:
        print("No wifi/AP connection profiles found.")
        sys.exit(1)

    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg in ("ap", "hotspot"):
            switch_to("Hotspot")
            return
        matches = [c for c in conns if arg in c.lower()]
        if len(matches) == 1:
            switch_to(matches[0])
            return
        print(f"No unique match for '{arg}'. Run with no arguments to pick from a list.")
        sys.exit(1)

    active = get_active_connections()
    print("Saved networks:\n")
    for i, name in enumerate(conns, 1):
        marker = " (active)" if name in active else ""
        print(f"  {i}) {name}{marker}")

    choice = input("\nSwitch to # (or q to quit): ").strip()
    if choice.lower() == "q":
        return
    try:
        switch_to(conns[int(choice) - 1])
    except (ValueError, IndexError):
        print("Invalid choice.")


if __name__ == "__main__":
    main()

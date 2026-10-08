#!/usr/bin/env python3
"""wifi.py - known wifi networks in a file, tried in order, AP hotspot as fallback.

networks.txt (next to this file, not in git), one network per line, top = tried first:
    on, HomeWifi, password123
    off, iPhone, hunter22
    on, eduroam                  no password = use the profile already saved in NetworkManager

Commands (or edit the file by hand and run `wifi apply`):
    wifi                      list the networks and what is connected
    wifi add SSID [PASSWORD]  add a network at the bottom (asks for the password if left out)
    wifi remove NAME
    wifi on NAME              try this network
    wifi off NAME             do not try this network
    wifi move NAME N          put the network at place N (1 = tried first)
    wifi connect NAME         connect now
    wifi ap                   start the hotspot now
    wifi apply                push the file to NetworkManager
NAME can be a part of the name, as long as only one network matches.

NetworkManager does the connecting: on/off is its autoconnect setting, the order is its
autoconnect priority. `wifi watch` (the wifi-fallback service) starts the hotspot when wlan0
has been without a connection for AP_AFTER_S seconds, at boot and later. It does not leave
the hotspot by itself when a network comes back, use `wifi connect NAME` for that.

Setup on the pi, once:
    sudo python3 wifi.py install      makes the `wifi` command and the wifi-fallback service
The hotspot profile must exist:
    sudo nmcli dev wifi hotspot ifname wlan0 con-name Hotspot ssid <name> password <password>
"""
import os
import subprocess
import sys
import time

# configs
AP_CONN = "Hotspot"    # name of the hotspot profile in NetworkManager
IFACE = "wlan0"
AP_AFTER_S = 30        # this long without wifi = start the hotspot
CHECK_S = 3            # how often the watcher looks
SERVICE = "wifi-fallback"
FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "networks.txt")

HEADER = "# on/off, ssid, password      top = tried first. After editing by hand: wifi apply\n"
UNIT = """[Unit]
Description=Start the wifi hotspot when there is no wifi
After=NetworkManager.service

[Service]
ExecStart=/usr/bin/python3 {me} watch
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
"""


# ---------------------------------------------------------------------------
# networks.txt
# ---------------------------------------------------------------------------

def parse(text):
    # ponytail: split on commas, so an ssid with a comma in it does not work (a password with one does)
    nets = []
    for no, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",", 2)]
        if len(parts) < 2 or parts[0] not in ("on", "off") or not parts[1]:
            raise SystemExit(f"networks.txt line {no}: expected  on/off, ssid, password")
        nets.append({"on": parts[0] == "on", "ssid": parts[1], "password": parts[2] if len(parts) > 2 else ""})
    return nets


def dump(nets):
    lines = [", ".join(filter(None, ["on" if n["on"] else "off", n["ssid"], n["password"]])) for n in nets]
    return HEADER + "".join(line + "\n" for line in lines)


def load():
    try:
        with open(FILE) as f:
            return parse(f.read())
    except FileNotFoundError:
        return []


def save(nets):
    with open(FILE, "w") as f:
        f.write(dump(nets))


def find(nets, name):
    # the network with this name, or the only one that has it as a part of its name
    low = name.lower()
    hits = [n for n in nets if n["ssid"].lower() == low] or [n for n in nets if low in n["ssid"].lower()]
    if len(hits) != 1:
        raise SystemExit(f"'{name}' matches {len(hits)} networks: " + ", ".join(n["ssid"] for n in hits or nets))
    return hits[0]


def move(nets, net, place):
    nets.remove(net)
    nets.insert(max(0, place - 1), net)


# ---------------------------------------------------------------------------
# NetworkManager
# ---------------------------------------------------------------------------

def nmcli(*args):
    r = subprocess.run(["nmcli", *args], capture_output=True, text=True)
    if r.returncode:    # only the first words, the rest can hold a password
        print(f"nmcli {' '.join(args[:3])} failed: {r.stderr.strip()}  (try with sudo)", flush=True)
    return r


def profiles():
    # names of the saved wifi profiles
    out = nmcli("-t", "-f", "NAME,TYPE", "con", "show").stdout
    return [line.rsplit(":", 1)[0] for line in out.splitlines() if line.endswith(":802-11-wireless")]


def active():
    return nmcli("-t", "-f", "NAME", "con", "show", "--active").stdout.splitlines()


def apply(nets):
    known = profiles()
    for i, n in enumerate(nets):
        auto = ["connection.autoconnect", "yes" if n["on"] else "no",
                "connection.autoconnect-priority", str(len(nets) - i)]
        # ponytail: wpa2 passwords only. Anything else (eduroam, wpa3 only): make the profile in
        # NetworkManager and list it here without a password
        sec = ["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", n["password"]] if n["password"] else []
        if n["ssid"] in known:
            nmcli("con", "mod", n["ssid"], *sec, *auto)
        else:
            nmcli("con", "add", "type", "wifi", "ifname", IFACE, "con-name", n["ssid"], "ssid", n["ssid"], *sec, *auto)
    # the hotspot must not start by itself, it would win over the networks at boot. The watcher starts it
    nmcli("con", "mod", AP_CONN, "connection.autoconnect", "no")


def show(nets):
    up = active()
    mark = lambda name: "   <- connected" if name in up else ""
    for i, n in enumerate(nets, 1):
        print(f"{i:>2}  {'on ' if n['on'] else 'off'}  {n['ssid']}{mark(n['ssid'])}")
    print(f"    ap   {AP_CONN} (after {AP_AFTER_S} s without wifi){mark(AP_CONN)}")
    ours = [n["ssid"] for n in nets] + [AP_CONN]
    other = [p for p in profiles() if p not in ours]
    if other:
        print("\nsaved in NetworkManager but not in networks.txt: " + ", ".join(other))


def connected():
    return f"{IFACE}:connected" in nmcli("-t", "-f", "DEVICE,STATE", "dev", "status").stdout.splitlines()


def watch():
    # runs as a service. The hotspot counts as connected, so this does nothing while it is up
    down = 0
    while True:
        down = 0 if connected() else down + CHECK_S
        if down >= AP_AFTER_S:
            print(f"no wifi for {down} s, starting {AP_CONN}", flush=True)
            nmcli("con", "up", AP_CONN)
            down = 0
        time.sleep(CHECK_S)


def install():
    me = os.path.abspath(__file__)
    try:
        with open("/usr/local/bin/wifi", "w") as f:
            f.write(f'#!/bin/sh\nexec python3 "{me}" "$@"\n')
        os.chmod("/usr/local/bin/wifi", 0o755)
        with open(f"/etc/systemd/system/{SERVICE}.service", "w") as f:
            f.write(UNIT.format(me=me))
    except PermissionError:
        raise SystemExit("run with sudo")
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    subprocess.run(["systemctl", "enable", "--now", SERVICE], check=True)
    print(f"installed: `wifi` command and {SERVICE} service")


# ---------------------------------------------------------------------------

def main(argv):
    cmd, args = (argv[0], argv[1:]) if argv else ("list", [])
    if cmd == "watch":
        return watch()
    if cmd == "install":
        return install()
    if cmd == "ap":
        return print(nmcli("con", "up", AP_CONN).stdout.strip())

    nets = load()
    if cmd == "list":
        return show(nets)
    if cmd == "connect" and args:
        return print(nmcli("con", "up", find(nets, args[0])["ssid"]).stdout.strip())

    if cmd == "add" and args:
        password = args[1] if len(args) > 1 else input("password (empty = none / already saved): ").strip()
        old = [n for n in nets if n["ssid"] == args[0]]
        if old:
            old[0]["password"] = password
        else:
            nets.append({"on": True, "ssid": args[0], "password": password})
    elif cmd == "remove" and args:
        net = find(nets, args[0])
        nets.remove(net)
        if net["password"]:    # without a password the profile was not made here, leave it
            nmcli("con", "delete", net["ssid"])
        else:
            nmcli("con", "mod", net["ssid"], "connection.autoconnect", "no")
    elif cmd in ("on", "off") and args:
        find(nets, args[0])["on"] = cmd == "on"
    elif cmd == "move" and len(args) == 2 and args[1].isdigit():
        move(nets, find(nets, args[0]), int(args[1]))
    elif cmd != "apply":
        return print(__doc__)

    save(nets)
    apply(nets)
    show(nets)


if __name__ == "__main__":
    main(sys.argv[1:])

# WiFi

The Pi tries the networks in `networks.txt` from the top down. With no WiFi for 30 s it starts its own hotspot (`Hotspot`, reach the Pi at `pi@10.42.0.1`).

## Commands

| Command | Does |
|---|---|
| `wifi` | list the networks and what is connected |
| `wifi add "SSID" "password"` | add a network at the bottom |
| `wifi remove NAME` | remove a network |
| `wifi on NAME` / `wifi off NAME` | try / do not try this network |
| `wifi move NAME N` | put the network at place N (1 = tried first) |
| `wifi connect NAME` | connect now |
| `wifi ap` | start the hotspot now |
| `wifi apply` | push `networks.txt` after editing it by hand |

`NAME` can be part of the name, as long as only one network matches.

## networks.txt

Lives next to `wifi.py`, only on the Pi (not in git, it holds passwords). It is created by the first `wifi add`.

```
on, HomeWifi, password123
off, iPhone, hunter22
on, eduroam
```

A line without a password uses the login already saved in NetworkManager. Use that for eduroam and anything that is not a normal WPA2 password.

## Good to know

- The Pi does not leave the hotspot by itself when a network comes back. Join the hotspot and run `wifi connect NAME`.
- `wifi connect` and `wifi ap` drop your current SSH/VNC session.
- If a command says "try with sudo", run `sudo wifi apply`.
- Hotspot not starting: `journalctl -u wifi-fallback -n 20`

## Setup on a new Pi

```bash
sudo nmcli dev wifi hotspot ifname wlan0 con-name Hotspot ssid <name> password <password>
sudo python3 WifiConfig/wifi.py install
```

The second line makes the `wifi` command and the `wifi-fallback` service. `python3 test_wifi.py` checks the file handling.

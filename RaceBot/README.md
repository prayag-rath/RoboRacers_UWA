# RaceBot

One window with a camera panel and a lidar panel, plus keyboard driving.

## Run

```bash
cd RaceBot/Main
python3 main.py                                # on the Pi, from the VNC desktop
python main.py --no-motors --sim --cam 0       # on a PC: simulated lidar, webcam (or --cam video.mp4)
```

Other options: `--lidar /dev/ttyACM1`, `--no-lidar`, `--no-dash`.

## Touch display

On the Pi, `main.py` also opens a fullscreen dashboard on the touch display (`:0`). It shows one view at a time, with buttons along the bottom: Raw, Threshold, Contours, Radar, and Points (lidar points on/off). Its view is separate from the main window; the settings are shared. `--no-dash` turns it off.

## Your own driving algorithm

Write it in `Main/pilot.py`, in the function `step()`. It returns throttle and steer, both -1..1 (steer -1 = left, 1 = right). `main.py` calls it about 30 times a second while auto is on (Space).

```python
import lidar
import vision

def step():
    points = lidar.points()          # [(angle_deg, range_mm), ...]  0 = ahead, + = left
    walls = lidar.planes()           # straight walls: ends, distance and angle
    hit = lidar.nearest(-30, 30)     # closest point in a sector -> (range_mm, angle_deg) or None
    img = vision.frame()             # latest camera picture or None
    offset = vision.line_offset()    # px from the picture centre to the biggest contour or None
    return 0.5, 0.0                  # throttle, steer
```

- Throttle 1 is the max speed set with Up/Down, so start with that low.
- Space, or any drive key, gives control back to the keyboard.
- `main.py` blocks forward driving when something is closer than `STOP_MM` ahead, or the lidar is silent. `STOP_MM` and `STOP_DEG` are at the top of `main.py`.
- The lidar gives 10 scans a second. `lidar.scan_count()` goes up by one per scan, use it to act on new scans only.
- An error in `step()` is printed in the terminal and switches back to manual.
- Another algorithm: copy `pilot.py` and change the `import pilot` line in `main.py`.

## Keys

| Key | Does |
|---|---|
| W / S, A / D | drive, steer |
| Space | auto on/off (`pilot.py` drives) |
| Up / Down | max speed |
| Left / Right | steer factor |
| 1 2 3 | camera view: raw, threshold, contours |
| 4 / 5 | lidar points / planes on the camera picture |
| + / - | lidar zoom |
| L | lock camera exposure |
| Esc | quit |

The car only drives while the window is active.

## Settings

Pick a menu tab, click a box, type a number, Enter. A 0/1 box switches on click. Everything is saved to `Main/settings.json` and loaded at the next start.

| Menu | For |
|---|---|
| Camera | crop, blur, threshold, contour size |
| Overlay | where the camera sits, so the lidar dots land on the right spot in the picture |
| Lidar points | range limits, scan window, mount angle, flip |
| Lidar planes | how points are grouped into straight walls |

First time on the car: set `mount_angle` and `flip` until the lidar panel matches reality, then tune Overlay until the green dots sit on the objects.

## Files (`Main/`)

- `main.py`: window, menus, keys
- `pilot.py`: the driving algorithm, empty for now
- `vision.py`: camera and image pipeline
- `lidar.py`: Hokuyo reader and plane detection
- `control.py`: ESC and steering servo
- `dashboard.py`: the window on the touch display, started by `main.py`
- `test_planes.py`: check of the plane detection, `python3 test_planes.py`

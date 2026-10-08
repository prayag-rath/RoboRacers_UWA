"""lidar.py - Hokuyo URG-04LX-UG01 reader (own thread). Started by main.py.

The sensor is fixed at 10 scans/s and 0.3516 deg per step, only the window,
cluster and filters below can be set.

Everything the other files need is in `state` plus a few helpers:
    state["points"]  [(angle_deg, range_mm), ...]   angle 0 = ahead, + = left, range 0 = invalid
    fresh()          True while scans keep arriving
    state["lines"]   the scan joined into lines: [{"verts": [(angle_deg, range_mm, x_mm, y_mm), ...], "n": points}, ...]
                     x = ahead, y = left. Points that are close together become one polyline
    nearest(lo, hi)  closest valid point between two angles -> (range_mm, angle_deg) or None
    edges()          angles of the first and last step of the scan window

Standalone check:  python3 lidar.py [/dev/ttyACM1]     (prints the closest point)
Needs pyserial:    sudo apt install python3-serial
"""
import math
import random
import sys
import threading
import time

try:
    import serial
except ImportError:
    serial = None

# configs
PORT = "/dev/ttyACM0"
BAUD = 115200          # ignored over usb
TIMEOUT_S = 1.0        # serial read timeout

START_STEP = 44        # first step of the scan (44 = about -120 deg, right side)
END_STEP = 725         # last step (725 = about +120 deg, left side)
CLUSTER = 1            # merge N neighbouring steps into one point, 1 = off

MIN_RANGE_MM = 30      # closer than this = invalid
MAX_RANGE_MM = 4000    # farther than this = invalid

MOUNT_ANGLE_DEG = 0.0  # lidar turned on the car, + = turned left
FLIP = False           # True if left/right is mirrored (check with the hand test)

# joining points into lines
LINE_GAP_MM = 50       # neighbours further apart than this (+ LINE_GAP_FRAC * range) start a new line
LINE_GAP_FRAC = 0.05
LINE_MIN_POINTS = 4    # fewer points than this stay single dots
LINE_TOL_MM = 30       # a bend smaller than this is straightened out (bigger = fewer corners)

STALE_S = 0.5          # no new scan for this long = NO DATA
RETRY_S = 1.0          # wait before reconnecting

# fixed by the sensor
FRONT_STEP = 384
DEG_PER_STEP = 360 / 1024

state = {"points": [], "lines": [], "time": None, "hz": 0.0, "last": 0.0, "error": "starting", "run": False}
_thread = None


# ---------------------------------------------------------------------------
# helpers for the other files
# ---------------------------------------------------------------------------

def fresh():
    return state["time"] is not None and time.time() - state["time"] < STALE_S


def nearest(lo_deg, hi_deg):
    best = None
    for a, r in state["points"]:
        if r and lo_deg <= a <= hi_deg and (best is None or r < best[0]):
            best = (r, a)
    return best


def edges():
    n = (END_STEP - START_STEP + 1) // CLUSTER
    return _point(0, 1)[0], _point(n - 1, 1)[0]


def _point(n, d):
    # n-th point of a scan -> (angle_deg, range_mm), range 0 = invalid
    if not MIN_RANGE_MM <= d <= MAX_RANGE_MM:
        d = 0
    step = START_STEP + n * CLUSTER + (CLUSTER - 1) / 2
    angle = (step - FRONT_STEP) * DEG_PER_STEP
    if FLIP:
        angle = -angle
    return (angle + MOUNT_ANGLE_DEG, d)


def _rdp(pts, tol):
    # keeps only the points that matter for the shape (Ramer-Douglas-Peucker), returns their indices
    keep = {0, len(pts) - 1}
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        (x1, y1), (x2, y2) = pts[i], pts[j]
        dx, dy = x2 - x1, y2 - y1
        norm = math.hypot(dx, dy) or 1e-9
        far, far_d = None, tol
        for k in range(i + 1, j):
            d = abs(dy * (pts[k][0] - x1) - dx * (pts[k][1] - y1)) / norm
            if d > far_d:
                far, far_d = k, d
        if far is not None:
            keep.add(far)
            stack += [(i, far), (far, j)]
    return sorted(keep)


def find_lines(points):
    # joins neighbouring points into lines. a gap or an invalid point ends a line
    lines, cur, prev = [], [], None

    def close():
        if len(cur) >= LINE_MIN_POINTS:
            idx = _rdp([(p[2], p[3]) for p in cur], LINE_TOL_MM)
            lines.append({"verts": [cur[i] for i in idx], "n": len(cur)})

    for angle, r in points:
        if not r:
            close()
            cur, prev = [], None
            continue
        a = math.radians(angle)
        x, y = r * math.cos(a), r * math.sin(a)
        if prev and math.hypot(x - prev[0], y - prev[1]) > LINE_GAP_MM + LINE_GAP_FRAC * r:
            close()
            cur = []
        cur.append((angle, r, x, y))
        prev = (x, y)
    close()
    return lines


def _publish(points):
    now = time.time()
    state["lines"] = find_lines(points)
    state["hz"] = 0.8 * state["hz"] + 0.2 / max(now - state["last"], 1e-3)
    state["last"] = now
    state["points"] = points
    state["time"] = now


# ---------------------------------------------------------------------------
# SCIP protocol
# ---------------------------------------------------------------------------

def checked(line):
    # a scip line is data + 1 checksum char, returns the data
    body, check = line[:-1], line[-1:]
    if chr((sum(map(ord, body)) & 0x3F) + 0x30) != check:
        raise ValueError("checksum error")
    return body


def command(ser, cmd):
    # send a command, return the reply lines up to the empty line
    ser.reset_input_buffer()    # throw away old data, keeps us in sync
    ser.write((cmd + "\n").encode())
    lines = []
    while True:
        raw = ser.readline()
        if not raw:
            raise TimeoutError(f"no reply to {cmd}")
        line = raw.decode(errors="replace").rstrip("\r\n")
        if line == "":
            return lines
        lines.append(line)


def init_sensor(ser):
    # the sensor always starts in scip 1.1, switch to 2.0 and turn the laser on
    # replies are ignored: an error just means it already is in 2.0 / the laser is already on
    for cmd in ("SCIP2.0", "BM"):
        try:
            command(ser, cmd)
        except TimeoutError:
            pass


def read_scan(ser):
    # one scan: GD command with start step, end step, cluster. Returns [(angle_deg, range_mm), ...]
    cmd = f"GD{START_STEP:04d}{END_STEP:04d}{CLUSTER:02d}"
    lines = command(ser, cmd)
    if len(lines) < 4 or lines[0] != cmd:
        raise ValueError("bad reply")
    status = checked(lines[1])
    if status[:2] != "00":
        raise ValueError(f"sensor status {status}")
    checked(lines[2])    # timestamp, not used

    # distances are 3 chars each. a value can be split over two lines, so join first
    data = "".join(checked(line) for line in lines[3:])
    if len(data) % 3:
        raise ValueError("bad data length")

    points = []
    for i in range(0, len(data), 3):
        d = ((ord(data[i]) - 0x30) << 12) | ((ord(data[i + 1]) - 0x30) << 6) | (ord(data[i + 2]) - 0x30)
        points.append(_point(i // 3, d))
    return points


# ---------------------------------------------------------------------------
# threads
# ---------------------------------------------------------------------------

def _reader(port):
    # reconnects by itself if the sensor or cable drops out
    if serial is None:
        state["error"] = "pyserial missing (sudo apt install python3-serial)"
        return
    while state["run"]:
        ser = None
        try:
            ser = serial.Serial(port, BAUD, timeout=TIMEOUT_S)
            init_sensor(ser)
            state["error"] = ""
            while state["run"]:
                _publish(read_scan(ser))
        except Exception as err:
            state["error"] = str(err) or type(err).__name__
            time.sleep(RETRY_S)
        finally:
            if ser:
                try:
                    ser.write(b"QT\n")    # laser off
                    ser.close()
                except Exception:
                    pass


def _sim_scan(t):
    # a small room with a box that drives around, for testing without the sensor
    ox, oy, rr = 1300 + 500 * math.sin(t * 0.6), 380 * math.sin(t * 0.9), 170
    points = []
    for n in range((END_STEP - START_STEP + 1) // CLUSTER):
        angle = _point(n, 1)[0] - MOUNT_ANGLE_DEG
        a = math.radians(angle)
        dx, dy = math.cos(a), math.sin(a)
        hit = 1e9
        for wall, comp in ((3200 if dx > 0 else -600, dx), (1300 if dy > 0 else -1300, dy)):
            if abs(comp) > 1e-6:
                hit = min(hit, wall / comp)
        b = dx * ox + dy * oy
        disc = b * b - (ox * ox + oy * oy - rr * rr)
        if disc >= 0 and b - math.sqrt(disc) > 0:
            hit = min(hit, b - math.sqrt(disc))
        points.append(_point(n, int(hit + random.gauss(0, 5))))
    return points


def _simulator():
    state["error"] = ""
    while state["run"]:
        _publish(_sim_scan(time.time()))
        time.sleep(0.1)


def start(port=None, sim=False):
    global _thread
    state["run"] = True
    state["last"] = time.time()
    target = _simulator if sim else (lambda: _reader(port or PORT))
    _thread = threading.Thread(target=target, daemon=True)
    _thread.start()


def stop():
    state["run"] = False
    if _thread:
        _thread.join(3)


if __name__ == "__main__":
    start(sys.argv[1] if len(sys.argv) > 1 else None)
    try:
        while True:
            time.sleep(0.5)
            hit = nearest(-180, 180)
            print(f"{state['hz']:.1f} Hz  closest {hit}  {state['error']}")
    except KeyboardInterrupt:
        pass
    finally:
        stop()

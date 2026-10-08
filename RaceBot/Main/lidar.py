"""lidar.py - Hokuyo URG-04LX-UG01 reader (own thread). Started by main.py.

The sensor is fixed at 10 scans/s and 0.3516 deg per step, only the settings below can be changed.

Everything the other files need is in `state` plus a few helpers:
    state["points"]  [(angle_deg, range_mm), ...]   angle 0 = ahead, + = left, range 0 = invalid
    state["planes"]  straight walls / object sides found in the scan:
                     [{"a": (x, y), "b": (x, y), "dist": mm, "angle": deg, "n": points}, ...]
                     x = ahead, y = left (mm). a, b = the two ends, dist = from the lidar to the plane,
                     angle = direction of the plane, 0 = along the car, +-90 = across
    fresh()          True while scans keep arriving
    nearest(lo, hi)  closest valid point between two angles -> (range_mm, angle_deg) or None

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
STALE_S = 0.5          # no new scan for this long = NO DATA
RETRY_S = 1.0          # wait before reconnecting
GAP_FRAC = 0.05        # the gap setting grows by this much of the range, far points are further apart

# settings, changed live from the menus in main.py:  name: [value, min, max, note]
POINTS = {
    "min_range": [30, 20, 1000, "mm, closer = invalid"],
    "max_range": [4000, 500, 5600, "mm, farther = invalid"],
    "fov": [240, 20, 240, "deg, scan window around ahead"],
    "cluster": [1, 1, 8, "merge N steps into one point"],
    "mount_angle": [0, -180, 180, "deg, lidar turned left = +"],
    "flip": [0, 0, 1, "1 = left/right mirrored"],
    "view_range": [4000, 500, 6000, "mm shown ahead (+ / - keys)"],
}
PLANES = {
    "gap": [80, 10, 500, "mm, bigger jump = new object"],
    "tolerance": [40, 5, 200, "mm, bend allowed in a plane"],
    "min_points": [6, 2, 50, "fewer = no plane"],
    "min_length": [150, 0, 2000, "mm, shorter = no plane"],
    "shade": [1, 0, 1, "gray area behind planes"],
}

# fixed by the sensor
FRONT_STEP = 384
FIRST_STEP, LAST_STEP = 44, 725
DEG_PER_STEP = 360 / 1024

state = {"points": [], "planes": [], "time": None, "hz": 0.0, "last": 0.0, "error": "starting", "run": False}
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


def _window():
    # first step, last step and cluster of the scan, from the settings
    half = int(POINTS["fov"][0] / 2 / DEG_PER_STEP)
    return max(FIRST_STEP, FRONT_STEP - half), min(LAST_STEP, FRONT_STEP + half), POINTS["cluster"][0]


def _point(n, d, start, cluster):
    # n-th point of a scan -> (angle_deg, range_mm), range 0 = invalid
    if not POINTS["min_range"][0] <= d <= POINTS["max_range"][0]:
        d = 0
    step = start + n * cluster + (cluster - 1) / 2
    angle = (step - FRONT_STEP) * DEG_PER_STEP
    if POINTS["flip"][0]:
        angle = -angle
    return (angle + POINTS["mount_angle"][0], d)


# ---------------------------------------------------------------------------
# planes: split the scan into objects at jumps, cut every object into straight pieces, fit a line to each
# ---------------------------------------------------------------------------

def _fit(pts):
    # best straight line through the points -> centre x, y, direction x, y, biggest distance from the line
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    t = 0.5 * math.atan2(2 * sxy, sxx - syy)
    dx, dy = math.cos(t), math.sin(t)
    return mx, my, dx, dy, max(abs((p[1] - my) * dx - (p[0] - mx) * dy) for p in pts)


def _split(pts, tol):
    # cut where the points bend more than tol away from a straight line -> list of straight pieces
    cuts = {0, len(pts) - 1}
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
            cuts.add(far)
            stack += [(i, far), (far, j)]
    cuts = sorted(cuts)
    return [pts[i:j + 1] for i, j in zip(cuts, cuts[1:])]


def _merge(pieces, tol):
    # noise makes the split cut too often, join neighbours that are one straight line after all
    out = [pieces[0]]
    for piece in pieces[1:]:
        both = out[-1] + piece[1:]
        if _fit(both)[4] <= tol:
            out[-1] = both
        else:
            out.append(piece)
    return out


def find_planes(points):
    gap, tol = PLANES["gap"][0], PLANES["tolerance"][0]
    min_points, min_length = PLANES["min_points"][0], PLANES["min_length"][0]

    # objects: neighbouring points that are close together
    objects, cur = [], []
    for angle, r in points:
        if r:
            a = math.radians(angle)
            p = (r * math.cos(a), r * math.sin(a))
            if cur and math.dist(p, cur[-1]) > gap + GAP_FRAC * r:
                objects.append(cur)
                cur = []
            cur.append(p)
        elif cur:
            objects.append(cur)
            cur = []
    if cur:
        objects.append(cur)

    planes = []
    for obj in objects:
        if len(obj) < min_points:
            continue
        for pts in _merge(_split(obj, tol), tol):
            if len(pts) < min_points:
                continue
            mx, my, dx, dy, _ = _fit(pts)
            ends = []
            for x, y in (pts[0], pts[-1]):    # the first and last point, moved onto the line
                s = (x - mx) * dx + (y - my) * dy
                ends.append((mx + s * dx, my + s * dy))
            a, b = ends
            if math.dist(a, b) >= min_length:
                angle = math.degrees(math.atan2(dy, dx))
                planes.append({"a": a, "b": b, "dist": abs(my * dx - mx * dy),
                               "angle": (angle + 90) % 180 - 90, "n": len(pts)})
    return planes


def _publish(points):
    now = time.time()
    state["planes"] = find_planes(points)
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
    start, end, cluster = _window()
    cmd = f"GD{start:04d}{end:04d}{cluster:02d}"
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
        points.append(_point(i // 3, d, start, cluster))
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
    start, end, cluster = _window()
    points = []
    for n in range((end - start + 1) // cluster):
        angle = _point(n, 1, start, cluster)[0]
        a = math.radians((angle - POINTS["mount_angle"][0]) * (-1 if POINTS["flip"][0] else 1))
        dx, dy = math.cos(a), math.sin(a)
        hit = 1e9
        for wall, comp in ((3200 if dx > 0 else -600, dx), (1300 if dy > 0 else -1300, dy)):
            if abs(comp) > 1e-6:
                hit = min(hit, wall / comp)
        b = dx * ox + dy * oy
        disc = b * b - (ox * ox + oy * oy - rr * rr)
        if disc >= 0 and b - math.sqrt(disc) > 0:
            hit = min(hit, b - math.sqrt(disc))
        points.append(_point(n, int(hit + random.gauss(0, 5)), start, cluster))
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

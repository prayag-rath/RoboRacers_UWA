"""vision.py - camera + pipeline (own thread). Started by main.py.

pipeline: raw -> crop -> blur -> gray -> threshold -> clean -> contours -> result
each step works on the output of the step before, and has its own settings.

Shared with the other files:
    state["out"]   latest result: raw frame, image + info text per step, contours, line centre
    project(...), wall_quad(...)   lidar point / wall piece -> pixels in the camera picture (fusion view)

Standalone use is not needed, run main.py.
"""
import math
import threading
import time

import cv2
import numpy as np

# camera
CAM_SOURCE = "picam"   # "picam", a camera number (0) or a video file
CAM_SIZE = (640, 480)  # capture size
CAM_FPS = 15           # camera + pipeline rate

# geometry for the lidar overlay, measure these on the car (mm / deg)
CAM_HFOV_DEG = 62.2    # horizontal field of view: pi camera v2 = 62, v3 = 66, v3 wide = 102
CAM_PITCH_DEG = 0.0    # + = camera looks down
CAM_HEIGHT_MM = 150    # camera height above the ground
LIDAR_HEIGHT_MM = 90   # lidar scan plane height above the ground
CAM_FWD_MM = 0         # camera position relative to the lidar, + = ahead
CAM_LEFT_MM = 0        # + = left of the lidar

MASK = "gray"    # "gray" = gray + threshold, "hsv" = colour range (for coloured tape)

STEPS = ["Raw", "Crop", "Blur", "Gray", "Threshold", "Clean", "Contours", "Result"]

# settings per step:  name: [value, min, max, note]
# min-max is the recommended range, typed values are kept inside it
SETTINGS = {
    "Crop": {"keep_bottom": [60, 10, 100, "% of the image height"]},
    "Blur": {"kernel": [5, 1, 15, "odd numbers, 1 = off"]},
    "Clean": {"kernel": [3, 0, 15, "0 = off"]},
    "Contours": {"min_area": [200, 0, 5000, "px, smaller is ignored"]},
}

if MASK == "gray":
    SETTINGS["Threshold"] = {
        "value": [127, 0, 255, "ignored when otsu = 1"],
        "invert": [0, 0, 1, "1 = dark things become white"],
        "otsu": [0, 0, 1, "1 = automatic value"],
    }
else:
    SETTINGS["Threshold"] = {
        "h_min": [20, 0, 179, "yellow is about 20-35"],
        "h_max": [35, 0, 179, "if below h_min: wraps (red)"],
        "s_min": [100, 0, 255, ""],
        "s_max": [255, 0, 255, ""],
        "v_min": [100, 0, 255, ""],
        "v_max": [255, 0, 255, ""],
    }

state = {"step": 7, "lock_ae": False, "run": False, "out": None, "fps": 0.0, "error": ""}
_cam = None
_thread = None


# ---------------------------------------------------------------------------
# pipeline
# ---------------------------------------------------------------------------

def val(step, name):
    return SETTINGS[step][name][0]


def set_value(step, name, text):
    # keep the typed number inside the range, make it odd if needed
    _, lo, hi, note = SETTINGS[step][name]
    value = min(hi, max(lo, int(text)))
    if "odd" in note and value % 2 == 0:
        value += 1
    SETTINGS[step][name][0] = value


def to_bgr(gray):
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def gray_mask(gray):
    flag = cv2.THRESH_BINARY_INV if val("Threshold", "invert") else cv2.THRESH_BINARY
    if val("Threshold", "otsu"):
        flag |= cv2.THRESH_OTSU
    thr, mask = cv2.threshold(gray, val("Threshold", "value"), 255, flag)
    return mask, f"threshold {thr:.0f}"


def hsv_mask(img):
    t = {name: item[0] for name, item in SETTINGS["Threshold"].items()}
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lo = (t["h_min"], t["s_min"], t["v_min"])
    hi = (t["h_max"], t["s_max"], t["v_max"])
    if t["h_min"] <= t["h_max"]:
        mask = cv2.inRange(hsv, lo, hi)
    else:
        # hue wraps around 179 -> 0
        m1 = cv2.inRange(hsv, lo, (179, t["s_max"], t["v_max"]))
        m2 = cv2.inRange(hsv, (0, t["s_min"], t["v_min"]), hi)
        mask = m1 | m2
    return mask, f"{100 * cv2.countNonZero(mask) / mask.size:.1f}% of pixels in range"


def run(img):
    # img is bgr. returns the picture and an info text for every step, plus extras for the fusion view
    images = {}
    info = {step: "" for step in STEPS}

    images["Raw"] = img

    # crop: keep the bottom part (the track), drop the horizon
    top = int(img.shape[0] * (100 - val("Crop", "keep_bottom")) / 100)
    crop = img[top:]
    images["Crop"] = crop

    k = val("Blur", "kernel")
    blur = cv2.GaussianBlur(crop, (k, k), 0) if k > 1 else crop
    images["Blur"] = blur

    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    images["Gray"] = to_bgr(gray)

    if MASK == "gray":
        mask, info["Threshold"] = gray_mask(gray)
    else:
        mask, info["Threshold"] = hsv_mask(blur)
    images["Threshold"] = to_bgr(mask)

    # clean: remove small specks (open), fill small holes (close)
    k = val("Clean", "kernel")
    if k > 0:
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    images["Clean"] = to_bgr(mask)

    found, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = [c for c in found if cv2.contourArea(c) >= val("Contours", "min_area")]

    shape = to_bgr(mask)
    cv2.drawContours(shape, found, -1, (0, 255, 0), 2)
    images["Contours"] = shape
    info["Contours"] = f"{len(found)} contours"

    # result: colour picture with the contours on top
    result = crop.copy()
    cv2.drawContours(result, found, -1, (0, 255, 0), 2)
    info["Result"] = f"{len(found)} contours"
    center, offset = None, None
    if found:
        big = max(found, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(big)
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 255), 2)
        m = cv2.moments(big)
        if m["m00"] > 0:
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
            cv2.circle(result, (cx, cy), 6, (0, 0, 255), -1)
            offset = cx - crop.shape[1] // 2    # px from the image centre, + = right
            center = (cx, cy + top)
            info["Result"] += f"   biggest {cv2.contourArea(big):.0f}px   offset {offset:+d}px"
    images["Result"] = result

    # the same contours in full-picture coordinates, for the fusion view
    shift = np.array([0, top], np.int32)
    extras = {"contours": [c + shift for c in found], "center": center, "offset": offset, "top": top}
    return images, info, extras


# ---------------------------------------------------------------------------
# lidar -> camera
# ---------------------------------------------------------------------------

def project_xy(x, y, w, h, z_mm=None):
    """Pixel in a w x h camera picture for a point in lidar coordinates (x ahead, y left, mm),
    or None if it is behind the camera. z_mm = height above the ground (default: the lidar scan plane)."""
    z = LIDAR_HEIGHT_MM if z_mm is None else z_mm
    x -= CAM_FWD_MM                              # ahead of the camera
    y -= CAM_LEFT_MM                             # left of the camera
    up = z - CAM_HEIGHT_MM                       # above the camera
    p = math.radians(CAM_PITCH_DEG)
    depth = x * math.cos(p) - up * math.sin(p)   # along the viewing direction
    vert = x * math.sin(p) + up * math.cos(p)
    if depth < 50:
        return None
    f = (w / 2) / math.tan(math.radians(CAM_HFOV_DEG) / 2)
    return w / 2 - f * y / depth, h / 2 - f * vert / depth


def project(angle_deg, r_mm, w, h, z_mm=None):
    # same for a lidar point given as angle + range
    a = math.radians(angle_deg)
    return project_xy(r_mm * math.cos(a), r_mm * math.sin(a), w, h, z_mm)


NEAR_MM = 120    # parts of a wall closer than this to the camera plane are cut off


def _depth(x, z_mm):
    up = z_mm - CAM_HEIGHT_MM
    p = math.radians(CAM_PITCH_DEG)
    return (x - CAM_FWD_MM) * math.cos(p) - up * math.sin(p)


def wall_quad(a, b, w, h, top_mm):
    """A piece of wall between two lidar points a, b = (x, y) standing on the ground up to top_mm.
    Returns pixels (ground a, ground b, top b, top a, scan plane a, scan plane b) or None if not visible.
    A straight wall stays straight in the picture, so the two end points are enough."""
    d0, d1 = _depth(a[0], LIDAR_HEIGHT_MM), _depth(b[0], LIDAR_HEIGHT_MM)
    if d0 < NEAR_MM and d1 < NEAR_MM:
        return None
    if d0 < NEAR_MM:        # cut the part behind the camera
        t = (NEAR_MM - d0) / (d1 - d0)
        a = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
    elif d1 < NEAR_MM:
        t = (NEAR_MM - d1) / (d0 - d1)
        b = (b[0] + (a[0] - b[0]) * t, b[1] + (a[1] - b[1]) * t)
    pts = [project_xy(a[0], a[1], w, h, 0), project_xy(b[0], b[1], w, h, 0),
           project_xy(b[0], b[1], w, h, top_mm), project_xy(a[0], a[1], w, h, top_mm),
           project_xy(a[0], a[1], w, h), project_xy(b[0], b[1], w, h)]
    return pts if all(pts) else None


# ---------------------------------------------------------------------------
# camera
# ---------------------------------------------------------------------------

def open_camera():
    if CAM_SOURCE == "picam":
        from picamera2 import Picamera2
        picam = Picamera2()
        picam.configure(picam.create_preview_configuration(
            main={"size": CAM_SIZE, "format": "RGB888"},    # rgb888 arrays are bgr, as opencv wants
            controls={"FrameDurationLimits": (int(1e6 / CAM_FPS),) * 2}))
        picam.start()
        return picam
    src = int(CAM_SOURCE) if CAM_SOURCE.isdigit() else CAM_SOURCE
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {src}")
    return cap


def read_frame(cam):
    if CAM_SOURCE == "picam":
        return cam.capture_array()
    ok, img = cam.read()
    if not ok:    # video ended, start over
        cam.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ok, img = cam.read()
    return cv2.resize(img, CAM_SIZE) if ok else None


def lock_exposure(cam, on):
    # fixed exposure + white balance keeps the threshold stable (pi camera only)
    if on:
        md = cam.capture_metadata()
        cam.set_controls({"AeEnable": False, "AwbEnable": False,
                          "ExposureTime": md["ExposureTime"],
                          "AnalogueGain": md["AnalogueGain"],
                          "ColourGains": md["ColourGains"]})
    else:
        cam.set_controls({"AeEnable": True, "AwbEnable": True})


def _loop():
    # own thread so nothing else waits for the camera or opencv
    locked = False
    t_prev = time.time()
    while state["run"]:
        t0 = time.time()
        if CAM_SOURCE == "picam" and state["lock_ae"] != locked:
            locked = state["lock_ae"]
            try:
                lock_exposure(_cam, locked)
            except Exception as err:
                state["error"] = f"exposure: {err}"

        img = read_frame(_cam)
        if img is None:
            time.sleep(0.1)
            continue
        try:
            images, info, extras = run(img)
            state["out"] = {"raw": img, "images": images, "info": info, **extras}
            state["error"] = ""
        except Exception as err:    # keep going while settings are being changed
            state["error"] = str(err)
            state["out"] = {"raw": img, "images": {s: img for s in STEPS},
                            "info": {s: f"error: {err}" for s in STEPS},
                            "contours": [], "center": None, "offset": None, "top": 0}

        time.sleep(max(0, 1 / CAM_FPS - (time.time() - t0)))
        now = time.time()
        state["fps"] = 0.8 * state["fps"] + 0.2 / max(now - t_prev, 1e-3)
        t_prev = now


def start(source=None):
    global CAM_SOURCE, _cam, _thread
    if source is not None:
        CAM_SOURCE = str(source)
    state["run"] = True
    try:
        _cam = open_camera()
    except Exception as err:    # camera is optional, the rest keeps working
        state["error"] = f"camera: {err}"
        print("Camera failed:", err)
        return
    _thread = threading.Thread(target=_loop, daemon=True)
    _thread.start()


def stop():
    state["run"] = False
    if _thread:
        _thread.join(2)
    if _cam is not None:
        try:
            if CAM_SOURCE == "picam":
                _cam.stop()
                _cam.close()
            else:
                _cam.release()
        except Exception:
            pass
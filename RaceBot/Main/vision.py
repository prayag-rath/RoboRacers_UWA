"""vision.py - camera + pipeline (own thread). Started by main.py.

pipeline: crop -> blur -> threshold -> clean -> contours
main.py shows one of three views of it: Raw, Threshold, Contours.

Shared with the other files:
    state["out"]   latest result: raw frame, a picture per view, info text, line centre and offset
    project(...), project_line(...)   lidar point / plane -> pixels in the camera picture

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

MASK = "gray"    # "gray" = gray + threshold, "hsv" = colour range (for coloured tape)
NEAR_MM = 120    # parts of a plane closer than this to the camera are cut off

VIEWS = ["Raw", "Threshold", "Contours"]

# settings, changed live from the menus in main.py:  name: [value, min, max, note]
SETTINGS = {
    "keep_bottom": [60, 10, 100, "% of the image height"],
    "blur": [5, 1, 15, "kernel, 1 = off"],
    "clean": [3, 0, 15, "kernel, 0 = off"],
    "min_area": [200, 0, 5000, "px, smaller is ignored"],
}
if MASK == "gray":
    SETTINGS.update({
        "threshold": [127, 0, 255, "ignored when otsu = 1"],
        "invert": [0, 0, 1, "1 = dark things are white"],
        "otsu": [0, 0, 1, "1 = automatic threshold"],
    })
else:
    SETTINGS.update({
        "h_min": [20, 0, 179, "yellow is about 20-35"],
        "h_max": [35, 0, 179, "below h_min: wraps (red)"],
        "s_min": [100, 0, 255, ""],
        "s_max": [255, 0, 255, ""],
        "v_min": [100, 0, 255, ""],
        "v_max": [255, 0, 255, ""],
    })

# where the camera sits, for the lidar overlay. Measure on the car, then tune until the dots fit
OVERLAY = {
    "cam_hfov": [62, 30, 160, "deg, v2 = 62, v3 = 66, wide = 102"],
    "cam_pitch": [0, -45, 45, "deg, + = camera looks down"],
    "cam_above": [60, -300, 500, "mm above the lidar scan"],
    "cam_fwd": [0, -300, 300, "mm ahead of the lidar"],
    "cam_left": [0, -300, 300, "mm left of the lidar"],
}

state = {"lock_ae": False, "run": False, "out": None, "fps": 0.0, "error": ""}
_cam = None
_thread = None


# ---------------------------------------------------------------------------
# pipeline
# ---------------------------------------------------------------------------

def val(name):
    return SETTINGS[name][0]


def to_bgr(gray):
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def gray_mask(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flag = cv2.THRESH_BINARY_INV if val("invert") else cv2.THRESH_BINARY
    if val("otsu"):
        flag |= cv2.THRESH_OTSU
    thr, mask = cv2.threshold(gray, val("threshold"), 255, flag)
    return mask, f"threshold {thr:.0f}"


def hsv_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lo = (val("h_min"), val("s_min"), val("v_min"))
    hi = (val("h_max"), val("s_max"), val("v_max"))
    if lo[0] <= hi[0]:
        mask = cv2.inRange(hsv, lo, hi)
    else:
        # hue wraps around 179 -> 0
        mask = cv2.inRange(hsv, lo, (179, hi[1], hi[2])) | cv2.inRange(hsv, (0, lo[1], lo[2]), hi)
    return mask, f"{100 * cv2.countNonZero(mask) / mask.size:.1f}% in range"


def run(img):
    # img is bgr. returns a picture per view, all the size of img so the lidar overlay fits on each
    h, w = img.shape[:2]

    # crop: keep the bottom part (the track), drop the horizon
    top = min(h - 1, int(h * (100 - val("keep_bottom")) / 100))
    crop = img[top:]

    k = val("blur") | 1    # the kernel must be odd
    blur = cv2.GaussianBlur(crop, (k, k), 0) if k > 1 else crop

    mask, info = gray_mask(blur) if MASK == "gray" else hsv_mask(blur)

    # clean: remove small specks (open), fill small holes (close)
    k = val("clean")
    if k > 0:
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    found, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = [c + np.array([0, top], np.int32) for c in found if cv2.contourArea(c) >= val("min_area")]
    info += f"   {len(found)} contours"

    full = np.zeros((h, w), np.uint8)
    full[top:] = mask
    threshold = to_bgr(full)

    contours = img.copy()
    cv2.drawContours(contours, found, -1, (0, 255, 0), 2)
    for view in (threshold, contours):    # above this line nothing is looked at
        cv2.line(view, (0, top), (w, top), (120, 120, 120), 1)

    # centre of the biggest contour, offset = px from the image centre, + = right
    center, offset = None, None
    if found:
        m = cv2.moments(max(found, key=cv2.contourArea))
        if m["m00"] > 0:
            center = (int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]))
            offset = center[0] - w // 2
            cv2.circle(contours, center, 6, (0, 0, 255), -1)
            info += f"   offset {offset:+d} px"

    images = {"Raw": img, "Threshold": threshold, "Contours": contours}
    return {"raw": img, "images": images, "info": info, "center": center, "offset": offset}


# ---------------------------------------------------------------------------
# lidar -> camera. Everything is drawn at the height of the lidar scan
# ---------------------------------------------------------------------------

def _depth(x):
    # how far ahead of the camera a point is, x = ahead of the lidar
    p = math.radians(OVERLAY["cam_pitch"][0])
    return (x - OVERLAY["cam_fwd"][0]) * math.cos(p) + OVERLAY["cam_above"][0] * math.sin(p)


def project_xy(x, y, w, h):
    """Pixel in a w x h camera picture for a point in lidar coordinates (x ahead, y left, mm),
    or None if it is behind the camera."""
    x -= OVERLAY["cam_fwd"][0]                   # ahead of the camera
    y -= OVERLAY["cam_left"][0]                  # left of the camera
    up = -OVERLAY["cam_above"][0]                # above the camera
    p = math.radians(OVERLAY["cam_pitch"][0])
    depth = x * math.cos(p) - up * math.sin(p)   # along the viewing direction
    vert = x * math.sin(p) + up * math.cos(p)
    if depth < 50:
        return None
    f = (w / 2) / math.tan(math.radians(OVERLAY["cam_hfov"][0]) / 2)
    return w / 2 - f * y / depth, h / 2 - f * vert / depth


def project(angle_deg, r_mm, w, h):
    # same for a lidar point given as angle + range
    a = math.radians(angle_deg)
    return project_xy(r_mm * math.cos(a), r_mm * math.sin(a), w, h)


def project_line(a, b, w, h):
    # a plane from the lidar, a, b = (x, y) -> its two end pixels, or None if it is behind the camera
    d0, d1 = _depth(a[0]), _depth(b[0])
    if d0 < NEAR_MM and d1 < NEAR_MM:
        return None
    if d0 < NEAR_MM:        # cut the part behind the camera
        t = (NEAR_MM - d0) / (d1 - d0)
        a = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
    elif d1 < NEAR_MM:
        t = (NEAR_MM - d1) / (d0 - d1)
        b = (b[0] + (a[0] - b[0]) * t, b[1] + (a[1] - b[1]) * t)
    ends = project_xy(a[0], a[1], w, h), project_xy(b[0], b[1], w, h)
    return ends if all(ends) else None


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
            state["out"] = run(img)
            state["error"] = ""
        except Exception as err:    # keep going while settings are being changed
            state["error"] = str(err)
            state["out"] = {"raw": img, "images": {v: img for v in VIEWS}, "info": f"error: {err}",
                            "center": None, "offset": None}

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

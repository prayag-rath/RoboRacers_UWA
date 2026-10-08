# starts lidar.py, vision.py and control.py and shows a camera panel and a lidar panel in one window
# on the pi (vnc desktop):  python3 main.py
# on a pc without hardware: python main.py --no-motors --sim --cam 0     (or --cam video.mp4)
# other options:            --lidar /dev/ttyACM1    --no-lidar
# keys: W/S A/D drive, up/down max speed, left/right steer factor, 1-3 camera view,
#       4 / 5 lidar points / planes on the camera, L lock exposure, + / - lidar zoom, Esc quit
# settings: pick a menu, click a box, type a number, Enter to set (Esc cancels). A 0/1 box switches
#       on click. The car stops while you type. Everything is kept in settings.json

import argparse
import json
import math
import os

import cv2
import numpy as np
import pygame

import control
import lidar
import vision

# configs
FPS = 30
RING_MM = 1000             # distance between the rings in the lidar panel
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.json")

# layout
W, H = 1280, 650
PANEL_W, PANEL_H = 625, 400
TITLE_H = 24
PANELS = [pygame.Rect(10 + i * (PANEL_W + 10), 10, PANEL_W, PANEL_H) for i in range(2)]    # camera, lidar
BTN_Y, BTN_W, BTN_H = 418, 125, 28    # buttons under the camera
TAB_Y, TAB_W = 458, 150               # menu tabs
SET_Y, ROW_H, ROWS, COL_W = 496, 36, 4, 285
DRIVE_X = 890

BG = (25, 25, 30)
LINE = (70, 70, 80)
TEXT = (230, 230, 230)
GRAY = (150, 150, 160)
AMBER = (240, 200, 60)
BLUE = (60, 110, 200)
RED = (240, 80, 80)
GREEN = (90, 210, 120)     # lidar points
SHADE = (50, 50, 60)       # area behind a plane
RING = (60, 60, 72)

MENUS = {"Camera": vision.SETTINGS, "Overlay": vision.OVERLAY,
         "Lidar points": lidar.POINTS, "Lidar planes": lidar.PLANES}
SAVED_UI = ("view", "points", "planes", "menu")

ui = {"view": 0, "points": True, "planes": True, "menu": "Camera", "editing": None, "typed": ""}
screen = font = small = None


# ---------------------------------------------------------------------------
# settings
# ---------------------------------------------------------------------------

def set_value(items, key, value):
    # keep the number inside its range
    lo, hi = items[key][1:3]
    try:
        items[key][0] = min(hi, max(lo, int(value)))
    except ValueError:
        pass


def load_settings():
    try:
        with open(SETTINGS_FILE) as f:
            saved = json.load(f)
    except (OSError, ValueError):
        return
    for menu, items in MENUS.items():
        for key, value in saved.get(menu, {}).items():
            if key in items:
                set_value(items, key, value)
    for key in SAVED_UI:
        ui[key] = saved.get("ui", {}).get(key, ui[key])
    if ui["menu"] not in MENUS or ui["view"] not in range(len(vision.VIEWS)):
        ui["menu"], ui["view"] = "Camera", 0


def save_settings():
    data = {menu: {key: item[0] for key, item in items.items()} for menu, items in MENUS.items()}
    data["ui"] = {key: ui[key] for key in SAVED_UI}
    try:
        with open(SETTINGS_FILE, "w") as f:
            json.dump(data, f, indent=2)
    except OSError as err:
        print("could not save settings:", err)


# ---------------------------------------------------------------------------
# drawing helpers
# ---------------------------------------------------------------------------

def text(s, pos, color=TEXT, f=None, anchor="topleft", bg=None):
    img = (f or font).render(str(s), True, color, bg)
    screen.blit(img, img.get_rect(**{anchor: pos}))


def panel(i, title):
    # frame with a title line, returns the area inside
    r = PANELS[i]
    pygame.draw.rect(screen, LINE, r, 1)
    text(title, (r.x + 6, r.y + 5), GRAY)
    return pygame.Rect(r.x + 1, r.y + TITLE_H, r.w - 2, r.h - TITLE_H - 1)


def geom(iw, ih, size):
    # where a picture of iw x ih lands inside size, keeping the aspect ratio: offset x, offset y, scale
    s = min(size[0] / iw, size[1] / ih)
    return (size[0] - iw * s) / 2, (size[1] - ih * s) / 2, s


def show_image(img, area):
    # draw a bgr picture into the area with black borders
    ih, iw = img.shape[:2]
    ox, oy, s = geom(iw, ih, area.size)
    nw, nh = max(1, int(iw * s)), max(1, int(ih * s))
    view = np.zeros((area.h, area.w, 3), np.uint8)
    view[int(oy):int(oy) + nh, int(ox):int(ox) + nw] = cv2.resize(img, (nw, nh))
    view = np.ascontiguousarray(cv2.cvtColor(view, cv2.COLOR_BGR2RGB))
    screen.blit(pygame.image.frombuffer(view, area.size, "RGB"), area.topleft)


def buttons():
    # every button: where, label, lit, and the ui value a click sets
    out = [(i, BTN_Y, BTN_W, name, ui["view"] == i, "view", i) for i, name in enumerate(vision.VIEWS)]
    out += [(3 + i, BTN_Y, BTN_W, "Lidar " + key, ui[key], key, not ui[key]) for i, key in enumerate(("points", "planes"))]
    out += [(i, TAB_Y, TAB_W, name, ui["menu"] == name, "menu", name) for i, name in enumerate(MENUS)]
    return [(pygame.Rect(10 + i * w, y, w - 4, BTN_H), label, lit, key, value)
            for i, y, w, label, lit, key, value in out]


def box_rect(i):
    # number box of setting number i in the open menu
    return pygame.Rect(10 + (i // ROWS) * COL_W + 115, SET_Y + (i % ROWS) * ROW_H, 60, 22)


# ---------------------------------------------------------------------------
# the two panels
# ---------------------------------------------------------------------------

def draw_camera(out):
    view = vision.VIEWS[ui["view"]]
    area = panel(0, f"Camera - {view}   {vision.state['fps']:.0f} fps")
    if out is None:
        text("No camera", area.center, GRAY, anchor="center")
        text(vision.state["error"][:60], (area.centerx, area.centery + 24), RED, small, "midtop")
        return
    show_image(out["images"][view], area)
    if not lidar.fresh():
        text(out["info"], (area.x + 6, area.y + 4), AMBER, small, bg=BG)
        return

    # the lidar on top of the picture, at the height of the scan
    fh, fw = out["raw"].shape[:2]
    ox, oy, s = geom(fw, fh, area.size)

    def px(p):
        return (max(-30000, min(30000, int(area.x + ox + p[0] * s))),
                max(-30000, min(30000, int(area.y + oy + p[1] * s))))

    screen.set_clip(area)
    if ui["planes"]:
        for plane in lidar.state["planes"]:
            ends = vision.project_line(plane["a"], plane["b"], fw, fh)
            if ends:
                pygame.draw.line(screen, AMBER, px(ends[0]), px(ends[1]), 3)
    if ui["points"]:
        for a, r in lidar.state["points"]:
            p = vision.project(a, r, fw, fh) if r else None
            if p:
                pygame.draw.circle(screen, GREEN, px(p), 2)
    screen.set_clip(None)
    text(out["info"], (area.x + 6, area.y + 4), AMBER, small, bg=BG)


def draw_lidar():
    fresh = lidar.fresh()
    points = lidar.state["points"] if fresh else []
    planes = lidar.state["planes"] if fresh else []
    title = f"Lidar   {lidar.state['hz']:.1f} Hz   {len(planes)} planes"
    hit = lidar.nearest(-360, 360) if fresh else None
    if hit:
        title += f"   closest {hit[0]} mm at {hit[1]:+.0f} deg"
    area = panel(1, title)

    view = lidar.POINTS["view_range"][0]
    ox, oy = area.centerx, area.y + int(area.h * 0.7)    # the car
    z = view / (oy - area.y)                             # mm per px, view_range reaches the top

    def xy(x, y):    # x ahead = up, y left = left
        return int(ox - y / z), int(oy - x / z)

    def polar(angle, r):
        a = math.radians(angle)
        return xy(r * math.cos(a), r * math.sin(a))

    screen.set_clip(area)

    # gray area behind every plane, first so the rest stays visible on top
    if lidar.PLANES["shade"][0]:
        for plane in planes:
            a0 = math.degrees(math.atan2(plane["a"][1], plane["a"][0]))
            a1 = math.degrees(math.atan2(plane["b"][1], plane["b"][0]))
            span = (a1 - a0 + 180) % 360 - 180
            n = max(1, int(abs(span) / 4))
            arc = [polar(a0 + span * k / n, view * 2) for k in range(n, -1, -1)]    # outside the panel
            pygame.draw.polygon(screen, SHADE, [xy(*plane["a"]), xy(*plane["b"])] + arc)

    for r in range(RING_MM, int(view * 1.6), RING_MM):
        pygame.draw.circle(screen, RING, (ox, oy), int(r / z), 1)
        text(f"{r / 1000:g} m", (ox + 4, oy - int(r / z) - 14), (110, 110, 120), small)

    for a, r in points:
        if r:
            pygame.draw.circle(screen, GREEN, polar(a, r), 1)
    for plane in planes:
        pygame.draw.line(screen, AMBER, xy(*plane["a"]), xy(*plane["b"]), 3)
    if not fresh:
        text("NO DATA", (area.centerx, area.centery - 10), RED, anchor="center")
        text(lidar.state["error"][:60], (area.centerx, area.centery + 10), RED, small, "midtop")
    pygame.draw.polygon(screen, TEXT, [(ox, oy - 9), (ox - 6, oy + 6), (ox + 6, oy + 6)])
    screen.set_clip(None)


# ---------------------------------------------------------------------------
# buttons, menu and drive info
# ---------------------------------------------------------------------------

def draw_bottom():
    for r, label, lit, _, _ in buttons():
        pygame.draw.rect(screen, BLUE if lit else (45, 45, 55), r)
        text(label, r.center, TEXT, small, "center")

    # the open menu: name, number box, range and note
    for i, (key, (value, lo, hi, note)) in enumerate(MENUS[ui["menu"]].items()):
        box = box_rect(i)
        x = box.x - 115
        editing = ui["editing"] == key
        pygame.draw.rect(screen, AMBER if editing else LINE, box, 2)
        text(key, (x, box.y + 3))
        text(ui["typed"] + "_" if editing else value, (box.x + 6, box.y + 4))
        text(f"{lo}..{hi}", (box.right + 8, box.y + 4), GRAY, small)
        text(note, (x, box.y + 22), GRAY, small)

    # drive info
    cs = control.state
    x, y = DRIVE_X, TAB_Y
    text("Throttle", (x, y))
    text("Steering", (x, y + 26))
    for k, v in enumerate((cs["throttle"] / control.SPEED_MAX, cs["steer"])):
        bar = pygame.Rect(x + 90, y + 3 + k * 26, 200, 16)
        pygame.draw.rect(screen, LINE, bar, 1)
        mid = bar.centerx
        pygame.draw.line(screen, TEXT, (mid, bar.y - 2), (mid, bar.bottom + 1))
        width = int(abs(v) * bar.w / 2)
        if width:
            pygame.draw.rect(screen, AMBER, (mid if v > 0 else mid - width, bar.y + 2, width, bar.h - 4))
    text(f"{cs['throttle'] * 100:+.0f}%", (x + 300, y + 3), TEXT, small)
    text(f"{cs['steer']:+.2f}", (x + 300, y + 29), TEXT, small)
    text(f"Max speed {cs['max_speed'] * 100:.0f}%  (up/down)", (x, y + 56))
    text(f"Steer factor {cs['steer_factor']:.1f}  (left/right)", (x, y + 80))
    if not cs["motors"]:
        text("no motors", (x, y + 110), AMBER, small)
    elif not pygame.key.get_focused():
        text("window not active, no driving", (x, y + 110), AMBER, small)
    if vision.CAM_SOURCE == "picam":
        text("exposure locked (L)" if vision.state["lock_ae"] else "exposure auto (L)", (x, y + 132), GRAY, small)


# ---------------------------------------------------------------------------
# events and main loop
# ---------------------------------------------------------------------------

def handle(e):
    # returns False to quit
    items = MENUS[ui["menu"]]

    if e.type == pygame.QUIT:
        return False

    # typing a number into a setting
    if e.type == pygame.KEYDOWN and ui["editing"]:
        if e.key == pygame.K_RETURN:
            set_value(items, ui["editing"], ui["typed"])
            save_settings()
            ui["editing"] = None
        elif e.key == pygame.K_ESCAPE:
            ui["editing"] = None
        elif e.key == pygame.K_BACKSPACE:
            ui["typed"] = ui["typed"][:-1]
        elif (e.unicode.isdigit() or (e.unicode == "-" and not ui["typed"])) and len(ui["typed"]) < 5:
            ui["typed"] += e.unicode

    # key functions
    elif e.type == pygame.KEYDOWN:
        if e.key == pygame.K_ESCAPE:
            return False
        elif e.key == pygame.K_UP:
            control.change_speed(+1)
        elif e.key == pygame.K_DOWN:
            control.change_speed(-1)
        elif e.key == pygame.K_RIGHT:
            control.change_steer(+1)
        elif e.key == pygame.K_LEFT:
            control.change_steer(-1)
        elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            set_value(lidar.POINTS, "view_range", lidar.POINTS["view_range"][0] / 1.25)
            save_settings()
        elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            set_value(lidar.POINTS, "view_range", lidar.POINTS["view_range"][0] * 1.25)
            save_settings()
        elif pygame.K_1 <= e.key <= pygame.K_5:    # the five buttons under the camera
            key, value = buttons()[e.key - pygame.K_1][3:]
            ui[key] = value
            save_settings()
        elif e.key == pygame.K_l and vision.CAM_SOURCE == "picam":
            vision.state["lock_ae"] = not vision.state["lock_ae"]

    # click a button or a setting
    elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
        ui["editing"] = None
        for r, _, _, key, value in buttons():
            if r.collidepoint(e.pos):
                ui[key] = value
                save_settings()
                return True
        for i, (key, (value, lo, hi, _)) in enumerate(items.items()):
            if box_rect(i).collidepoint(e.pos):
                if (lo, hi) == (0, 1):
                    set_value(items, key, 1 - value)
                    save_settings()
                else:
                    ui["editing"], ui["typed"] = key, ""
    return True


def main(argv=None):
    global screen, font, small

    ap = argparse.ArgumentParser()
    ap.add_argument("--no-motors", action="store_true", help="do not touch the pwm (pc testing)")
    ap.add_argument("--cam", default=None, help="picam (default), a camera number or a video file")
    ap.add_argument("--lidar", default=None, help=f"lidar port (default {lidar.PORT})")
    ap.add_argument("--sim", action="store_true", help="simulated lidar (pc testing)")
    ap.add_argument("--no-lidar", action="store_true", help="run without lidar")
    args = ap.parse_args(argv)

    load_settings()

    # start the other files, lidar and vision run in their own threads
    if args.no_lidar:
        lidar.state["error"] = "disabled (--no-lidar)"
    else:
        lidar.start(args.lidar, sim=args.sim)
    vision.start(args.cam)
    control.install_signals()
    control.start(no_motors=args.no_motors)

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Racecar")
    font = pygame.font.SysFont(None, 24)
    small = pygame.font.SysFont(None, 20)
    clock = pygame.time.Clock()

    running = True
    try:
        while running:
            for e in pygame.event.get():
                running = handle(e) and running

            # no throttle/steering without window active, or while typing
            keys = pygame.key.get_pressed()
            fwd = keys[pygame.K_w] - keys[pygame.K_s]
            steer = keys[pygame.K_d] - keys[pygame.K_a]
            if not pygame.key.get_focused() or ui["editing"]:
                fwd = steer = 0
            control.drive(fwd, steer)

            screen.fill(BG)
            draw_camera(vision.state["out"])
            draw_lidar()
            draw_bottom()
            pygame.display.flip()
            clock.tick(FPS)
    finally:
        control.stop()    # neutral first
        vision.stop()
        lidar.stop()
        pygame.quit()


if __name__ == "__main__":
    main()

# starts lidar.py, vision.py and control.py and shows them in one window
# on the pi (vnc desktop):  python3 main.py
# on a pc without hardware: python main.py --no-motors --sim --cam 0     (or --cam video.mp4)
# other options:            --lidar /dev/ttyACM1    --no-lidar
# keys: W/S A/D drive, up/down max speed, left/right steer factor, 1-8 pipeline step, L lock exposure,
#       + / - lidar zoom, Esc quit
# settings: click a box, type a number, Enter to set (Esc cancels). The car stops while you type

import argparse
import math
import time

import cv2
import numpy as np
import pygame

import control
import lidar
import vision

# configs
FPS = 30
RADAR_MM_PER_PX = 16.0     # lidar zoom at start
RING_MM = 1000             # distance between the rings
DOT = 2                    # point size in px
LINE_W = 3                 # width of the lidar lines in px
SHADE = "behind"           # solid area: "behind" = hidden area behind each line, "front" = free area in front, "none"
WALL_H_MM = 300            # height of the walls drawn in the camera picture
WALL_ALPHA = 110           # 0-255, how solid the walls are in the camera picture

# layout
PANEL_W, PANEL_H = 410, 380
TITLE_H = 24
PANELS = [pygame.Rect(10 + i * 425, 10, PANEL_W, PANEL_H) for i in range(3)]   # vision, lidar, camera + lidar
STEP_Y, STEP_H = 398, 28
SET_Y, ROW_H, ROWS = 440, 36, 3
W, H = 1280, 575

BG = (25, 25, 30)
LINE = (70, 70, 80)
TEXT = (230, 230, 230)
GRAY = (150, 150, 160)
AMBER = (240, 200, 60)
BLUE = (60, 110, 200)
RED = (240, 80, 80)
SHADE_COLOR = {"behind": (46, 46, 62), "front": (32, 48, 44)}

ui = {"zoom": RADAR_MM_PER_PX, "editing": None, "typed": ""}
screen = font = small = None


def text(s, pos, color=TEXT, f=None, anchor="topleft"):
    img = (f or font).render(str(s), True, color)
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


def point_color(r):
    # near = red, far = green
    t = min(r / lidar.MAX_RANGE_MM, 1.0)
    return (int(255 * (1 - t)), int(255 * t), 80)


# ---------------------------------------------------------------------------
# the three views
# ---------------------------------------------------------------------------

def draw_vision(out):
    step = vision.STEPS[vision.state["step"]]
    area = panel(0, f"Vision - {step}   {vision.state['fps']:.0f} fps")
    if out is None:
        text("No camera", area.center, GRAY, anchor="center")
        text(vision.state["error"][:50], (area.centerx, area.centery + 24), RED, small, "midtop")
        return
    show_image(out["images"][step], area)
    text(out["info"][step], (area.x + 6, area.y + 4), AMBER, small)


def draw_radar():
    pts = lidar.state["points"]
    lines = lidar.state["lines"]
    fresh = lidar.fresh()
    valid = [(r, a) for a, r in pts if r] if fresh else []
    title = f"Lidar   {lidar.state['hz']:.1f} Hz"
    if valid:
        r, a = min(valid)
        title += f"   closest {r} mm at {a:+.0f} deg"
    area = panel(1, title)

    ox, oy = area.centerx, area.y + int(area.h * 0.62)    # lidar position
    z = ui["zoom"]
    origin = (ox, oy)

    def to_px(angle, r_mm):
        a = math.radians(angle)
        return int(ox - r_mm * math.sin(a) / z), int(oy - r_mm * math.cos(a) / z)

    screen.set_clip(area)

    # solid area for every line, drawn first so rings and points stay visible on top
    if fresh and SHADE != "none":
        for seg in lines:
            v = seg["verts"]
            poly = [to_px(a, r) for a, r, _, _ in v]
            if SHADE == "behind":    # from the line out to max range, along the same angles
                a0, a1 = v[-1][0], v[0][0]
                n = max(1, int(abs(a1 - a0) / 4))
                poly += [to_px(a0 + (a1 - a0) * k / n, lidar.MAX_RANGE_MM) for k in range(n + 1)]
            else:                    # from the lidar to the line
                poly = [origin] + poly
            pygame.draw.polygon(screen, SHADE_COLOR[SHADE], poly)

    for r in range(RING_MM, lidar.MAX_RANGE_MM + 1, RING_MM):
        radius = int(r / z)
        pygame.draw.circle(screen, (60, 60, 72), origin, radius, 1)
        text(f"{r / 1000:g} m", (ox + 4, oy - radius - 14), (110, 110, 120), small)
    pygame.draw.line(screen, (70, 70, 80), origin, to_px(0, lidar.MAX_RANGE_MM), 1)
    for a in lidar.edges():    # edges of the scan window, the rest is the blind sector
        pygame.draw.line(screen, (90, 70, 40), origin, to_px(a, lidar.MAX_RANGE_MM), 1)

    if fresh:
        for a, r in pts:    # every point, small
            if r:
                pygame.draw.circle(screen, point_color(r), to_px(a, r), 1)
        for seg in lines:   # joined into lines, colour = distance
            v = seg["verts"]
            for p, q in zip(v, v[1:]):
                pygame.draw.line(screen, point_color((p[1] + q[1]) / 2), to_px(p[0], p[1]), to_px(q[0], q[1]), LINE_W)
        if valid:
            r, a = min(valid)
            p = to_px(a, r)
            pygame.draw.line(screen, (90, 90, 100), origin, p, 1)
            pygame.draw.circle(screen, TEXT, p, 7, 1)
    else:
        text("NO DATA", (area.centerx, area.centery - 10), RED, anchor="center")
        text(lidar.state["error"][:50], (area.centerx, area.centery + 10), RED, small, "midtop")
    pygame.draw.polygon(screen, TEXT, [(ox, oy - 9), (ox - 6, oy + 6), (ox + 6, oy + 6)])
    screen.set_clip(None)


def draw_fusion(out):
    pts = lidar.state["points"] if lidar.fresh() else []
    lines = lidar.state["lines"] if lidar.fresh() else []
    area = panel(2, "Camera + lidar")
    pygame.draw.rect(screen, (0, 0, 0), area)
    if out is None:
        text("No camera", area.center, GRAY, anchor="center")
        return
    show_image(out["raw"], area)
    fh, fw = out["raw"].shape[:2]
    ox, oy, s = geom(fw, fh, area.size)

    def px(p):
        return max(-30000, min(30000, int(ox + p[0] * s))), max(-30000, min(30000, int(oy + p[1] * s)))

    ov = pygame.Surface(area.size, pygame.SRCALPHA)

    # walls: every line is a wall standing on the ground, far ones first
    for seg in sorted(lines, key=lambda g: -sum(p[1] for p in g["verts"]) / len(g["verts"])):
        v = seg["verts"]
        for p, q in zip(v, v[1:]):
            quad = vision.wall_quad(p[2:], q[2:], fw, fh, WALL_H_MM)
            if not quad:
                continue
            ga, gb, tb, ta, pa, pb = [px(c) for c in quad]
            col = point_color((p[1] + q[1]) / 2)
            pygame.draw.polygon(ov, col + (WALL_ALPHA,), [ga, gb, tb, ta])
            pygame.draw.line(ov, col + (200,), ga, ta, 1)
            pygame.draw.line(ov, col + (200,), gb, tb, 1)
            pygame.draw.line(ov, col + (255,), pa, pb, LINE_W)

    # the single points and the closest one
    closest = None
    for a, r in pts:
        if r:
            p = vision.project(a, r, fw, fh)
            if p:
                pygame.draw.circle(ov, point_color(r) + (255,), px(p), 1)
                if closest is None or r < closest[0]:
                    closest = (r, px(p))
    if closest:
        pygame.draw.circle(ov, TEXT + (255,), closest[1], 8, 1)
        label = small.render(f"{closest[0] / 1000:.2f} m", True, TEXT)
        ov.blit(label, (min(closest[1][0] + 12, area.w - label.get_width() - 4), max(closest[1][1] - 18, 4)))

    screen.blit(ov, area.topleft)


# ---------------------------------------------------------------------------
# steps, settings and drive info
# ---------------------------------------------------------------------------

def field_pos(i):
    # top left of setting number i
    return 10 + (i // ROWS) * 425, SET_Y + (i % ROWS) * ROW_H


def box_rect(i):
    x, y = field_pos(i)
    return pygame.Rect(x + 130, y, 70, 22)


def draw_bottom():
    step = vision.state["step"]
    name = vision.STEPS[step]

    # step bar
    w = W // len(vision.STEPS)
    for i, s in enumerate(vision.STEPS):
        r = pygame.Rect(i * w, STEP_Y, w, STEP_H)
        pygame.draw.rect(screen, BLUE if i == step else (45, 45, 55), r)
        pygame.draw.rect(screen, BG, r, 1)
        text(f"{i + 1} {s}", (r.x + 10, r.y + 7), TEXT, small)

    # settings of this step: name, number box, range and note
    items = vision.SETTINGS.get(name, {})
    if not items:
        text("no settings for this step", (10, SET_Y), GRAY, small)
    for i, (key, (value, lo, hi, note)) in enumerate(items.items()):
        x, y = field_pos(i)
        box = box_rect(i)
        editing = ui["editing"] == key
        pygame.draw.rect(screen, AMBER if editing else LINE, box, 2)
        text(key, (x, y + 3))
        text(ui["typed"] + "_" if editing else value, (box.x + 6, box.y + 4))
        text(f"{lo}-{hi}", (box.right + 10, y + 4), GRAY, small)
        text(note, (x, y + 22), GRAY, small)

    # drive info
    cs = control.state
    x, y = 870, SET_Y
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
        text("exposure locked (L)" if vision.state["lock_ae"] else "exposure auto (L)", (x + 200, y + 110), GRAY, small)


# ---------------------------------------------------------------------------
# events and main loop
# ---------------------------------------------------------------------------

def handle(e):
    # returns False to quit
    name = vision.STEPS[vision.state["step"]]
    items = vision.SETTINGS.get(name, {})

    if e.type == pygame.QUIT:
        return False

    # typing a number into a setting
    if e.type == pygame.KEYDOWN and ui["editing"]:
        if e.key == pygame.K_RETURN:
            if ui["typed"]:
                vision.set_value(name, ui["editing"], ui["typed"])
            ui["editing"] = None
        elif e.key == pygame.K_ESCAPE:
            ui["editing"] = None
        elif e.key == pygame.K_BACKSPACE:
            ui["typed"] = ui["typed"][:-1]
        elif e.unicode.isdigit() and len(ui["typed"]) < 5:
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
            ui["zoom"] = max(4.0, ui["zoom"] / 1.25)
        elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            ui["zoom"] = min(40.0, ui["zoom"] * 1.25)
        elif pygame.K_1 <= e.key < pygame.K_1 + len(vision.STEPS):
            vision.state["step"] = e.key - pygame.K_1
        elif e.key == pygame.K_l and vision.CAM_SOURCE == "picam":
            vision.state["lock_ae"] = not vision.state["lock_ae"]

    # click a step or a setting
    elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
        ui["editing"] = None
        w = W // len(vision.STEPS)
        if STEP_Y <= e.pos[1] < STEP_Y + STEP_H:
            vision.state["step"] = min(e.pos[0] // w, len(vision.STEPS) - 1)
        for i, key in enumerate(items):
            if box_rect(i).collidepoint(e.pos):
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

            out = vision.state["out"]
            screen.fill(BG)
            draw_vision(out)
            draw_radar()
            draw_fusion(out)
            draw_bottom()
            pygame.display.flip()
            clock.tick(FPS)
    finally:
        control.stop()    # neutral first
        vision.stop()
        lidar.stop()
        pygame.quit()

        # final settings, paste into the SETTINGS block in vision.py
        for step, items in vision.SETTINGS.items():
            print(step, {k: v[0] for k, v in items.items()})


if __name__ == "__main__":
     main()

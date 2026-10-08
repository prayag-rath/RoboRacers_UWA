"""pilot.py - the driving algorithm. main.py calls step() about 30 times a second while auto is on (Space).

What you can use:
    lidar.points()          [(angle_deg, range_mm), ...]   angle 0 = ahead, + = left
    lidar.planes()          straight walls: [{"a": (x, y), "b": (x, y), "dist": mm, "angle": deg, "n": points}, ...]
    lidar.nearest(lo, hi)   closest point between two angles -> (range_mm, angle_deg) or None
    lidar.scan_count()      goes up by one for every new scan (10 per second)
    vision.frame()          latest camera picture (bgr) or None
    vision.line_offset()    px from the picture centre to the biggest contour, + = right, or None

step() returns throttle and steer, both -1..1:
    throttle  1 = forward at the max speed set with up/down in main.py, -1 = reverse
    steer     -1 = full left, 1 = full right

main.py blocks forward driving when something is close ahead or the lidar is silent (STOP_MM in main.py).
Another algorithm: copy this file and change the `import pilot` line in main.py.
"""
import lidar
import vision

# configs: speeds, distances and gains of the algorithm go here


def step():
    # the algorithm goes here. For now the car stands still
    return 0, 0

# check of the plane detection in lidar.py, runs anywhere:  python test_planes.py
import math
import random

import lidar

random.seed(1)


def scan(hit):
    # hit(angle_deg) -> range in mm, with the noise of the real sensor
    return [(a, max(0, int(hit(a) + random.gauss(0, 10)))) for a in [i * 0.35 - 90 for i in range(515)]]


# a corner: wall 2 m ahead (across) and a wall 1 m to the left (along), nothing on the right
def corner(a):
    r = math.radians(a)
    ahead = 2000 / math.cos(r) if math.cos(r) > 1e-6 else 1e9
    left = 1000 / math.sin(r) if math.sin(r) > 1e-6 else 1e9
    hit = min(ahead, left)
    return hit if hit <= 2600 else 0    # 0 = out of range

planes = lidar.find_planes(scan(corner))
assert len(planes) == 2, planes
across, along = sorted(planes, key=lambda p: abs(p["angle"]), reverse=True)
assert abs(abs(across["angle"]) - 90) < 3 and abs(across["dist"] - 2000) < 30, across
assert abs(along["angle"]) < 3 and abs(along["dist"] - 1000) < 30, along

# a box in front of a wall is its own plane, and the wall is cut in two by the jump
def box(a):
    return 800 / math.cos(math.radians(a)) if abs(a) < 8 else 2000 / math.cos(math.radians(a)) if abs(a) < 50 else 0

planes = lidar.find_planes(scan(box))
assert sorted(round(p["dist"], -2) for p in planes) == [800, 2000, 2000], planes

# too few points or too short = no plane, and an empty scan is fine
assert lidar.find_planes(scan(lambda a: 500 if abs(a) < 0.6 else 0)) == []
assert lidar.find_planes([]) == []
print("ok")

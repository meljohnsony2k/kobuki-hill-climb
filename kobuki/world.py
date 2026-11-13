"""
Course: a walled arena with random obstacles, funnel walls into a ramp (up, plateau, down)
with open cliff edges on both sides, then a finish zone on the floor.

Layout follows the ELEN90066 course (3 m wide arena, ramp at the far end, stop about 40 cm
past the bottom of the down ramp). World frame: x to the right, y forward (the robot's
starting "ground orientation"), z up. Units: metres.
"""
from dataclasses import dataclass, field

import numpy as np

ARENA_HALF_W = 1.5
ARENA_LEN = 2.6          # straight part of the arena
FUNNEL_END = 3.4         # funnel walls meet the ramp mouth here
RAMP_HALF_W = 0.45       # ramp column is 0.9 m wide
UP = (3.4, 4.0)          # up ramp
FLAT = (4.0, 4.6)        # plateau
DOWN = (4.6, 5.2)        # down ramp
HEIGHT = 0.15            # plateau height -> about 14 deg slopes
FINISH = (5.2 + 0.05, 5.2 + 1.0)    # success: comes to rest within 1 m past the bottom of the down ramp


@dataclass
class Course:
    segments: np.ndarray                    # (n, 4): x1 y1 x2 y2 for every wall and obstacle edge
    obstacles: list = field(default_factory=list)   # list of (4, 2) corner arrays, for drawing


def ramp_height(y):
    if y < UP[0] or y > DOWN[1]:
        return 0.0
    if y < UP[1]:
        return HEIGHT * (y - UP[0]) / (UP[1] - UP[0])
    if y <= FLAT[1]:
        return HEIGHT
    return HEIGHT * (DOWN[1] - y) / (DOWN[1] - DOWN[0])


def height(x, y):
    """Floor height under a point. Off the sides of the ramp column is the floor (a cliff)."""
    if abs(x) > RAMP_HALF_W:
        return 0.0
    return ramp_height(y)


def slope(x, y):
    """Gradient of the surface (dz/dx, dz/dy) under the robot centre."""
    if abs(x) > RAMP_HALF_W:
        return 0.0, 0.0
    s = HEIGHT / (UP[1] - UP[0])
    if UP[0] <= y < UP[1]:
        return 0.0, s
    if DOWN[0] < y <= DOWN[1]:
        return 0.0, -s
    return 0.0, 0.0


def _rect(cx, cy, w, h, ang):
    c, s = np.cos(ang), np.sin(ang)
    pts = np.array([[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]])
    return pts @ np.array([[c, s], [-s, c]]) + [cx, cy]


def _poly_segments(p):
    return [[*p[i], *p[(i + 1) % len(p)]] for i in range(len(p))]


def walls():
    W, L, F, R = ARENA_HALF_W, ARENA_LEN, FUNNEL_END, RAMP_HALF_W
    return [
        [-W, -0.2, W, -0.2],          # back wall
        [-W, -0.2, -W, L], [W, -0.2, W, L],            # side walls
        [-W, L, -R, F], [W, L, R, F],  # funnel into the ramp mouth
    ]


def make_course(rng, n_obstacles=(4, 7), clear=0.42, tries=400):
    """Random obstacles in the arena. Keeps the start box clear and leaves gaps wide enough to pass."""
    obs = []
    n = rng.integers(n_obstacles[0], n_obstacles[1] + 1)
    for _ in range(tries):
        if len(obs) >= n:
            break
        w, h = rng.uniform(0.12, 0.35), rng.uniform(0.06, 0.15)
        cx, cy = rng.uniform(-1.2, 1.2), rng.uniform(0.9, 2.4)
        p = _rect(cx, cy, w, h, rng.uniform(0, np.pi))
        if np.any(np.abs(p[:, 0]) > ARENA_HALF_W - 0.05):
            continue
        ok = all(_poly_gap(p, q) > clear for q in obs)
        if ok:
            obs.append(p)
    segs = walls() + [s for p in obs for s in _poly_segments(p)]
    return Course(np.array(segs, float), obs)


def _poly_gap(p, q):
    """Smallest distance between two convex polygons that don't overlap (vertex to edge, both ways)."""
    sp, sq = np.array(_poly_segments(p)), np.array(_poly_segments(q))
    d1 = min(_seg_point_dist(sp, *pt)[0].min() for pt in q)
    d2 = min(_seg_point_dist(sq, *pt)[0].min() for pt in p)
    return min(d1, d2)


def _seg_point_dist(segs, px, py):
    """Distance from a point to each segment, and the closest points."""
    x1, y1, x2, y2 = segs.T
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy + 1e-12
    t = np.clip(((px - x1) * dx + (py - y1) * dy) / L2, 0, 1)
    cx, cy = x1 + t * dx, y1 + t * dy
    return np.hypot(px - cx, py - cy), cx, cy

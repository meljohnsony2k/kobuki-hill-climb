"""
Kobuki model: differential drive, bumpers, cliff sensors, wheel encoders, myRIO accelerometer
and the Kobuki's gyro.

The hardware problems from the real project are modelled on purpose:
  - wheels don't match, so the robot drifts (each robot gets its own mismatch)
  - in place turns slip, so encoder angle != real angle (errors pile up over many turns)
  - the accelerometer is noisy, has an offset, and feels speed changes and bumps as spikes
    (the front lifts on hard starts and stops: the "jolting" seen in the speed test)
  - on a slope a misaligned robot yaws downhill, so it drifts toward the cliff edges
"""
from dataclasses import dataclass

import numpy as np

from . import world

R = 0.175            # body radius (m)
B = 0.230            # wheelbase (m)
G = 9.81


@dataclass
class Sensors:
    bump_left: bool
    bump_centre: bool
    bump_right: bool
    cliff_left: bool
    cliff_centre: bool
    cliff_right: bool
    net_distance: float      # mm, from encoders (average of both wheels)
    net_angle: float         # deg, from encoders (difference / wheelbase), CCW positive
    accel_x: float           # g, forward
    accel_y: float           # g, left
    gyro_heading: float      # deg, CCW positive


class Kobuki:
    def __init__(self, rng, x=0.0, y=0.25, heading=0.0, wheel_mismatch=None, turn_slip_bias=None, accel_offset=None,
                 max_accel=0.8, max_decel=2.5, max_alpha=40.0, accel_noise=0.025):
        self.rng = rng
        self.x, self.y, self.psi = x, y, heading     # psi: CCW from +y (rad)
        self.vl = self.vr = 0.0                      # wheel speeds (m/s)
        self.mismatch = rng.normal(0, 0.006) if wheel_mismatch is None else wheel_mismatch
        self.slip_bias = rng.uniform(-0.03, 0.08) if turn_slip_bias is None else turn_slip_bias
        self.slip = self.slip_bias
        self.amax, self.decel, self.alpha, self.noise = max_accel, max_decel, max_alpha, accel_noise
        self.enc_dist = 0.0
        self.enc_angle = 0.0
        self.gyro_bias = rng.normal(0, 0.002)        # rad/s
        # flat ground reads about -0.035 to -0.015 g (varies per myRIO)
        self.accel_offset = rng.normal(-0.025, 0.005) if accel_offset is None else accel_offset
        self.gyro = heading
        self.a_fwd = 0.0
        self.v_body = 0.0
        self.impact = 0.0                            # decaying spike after a collision (g)
        self.bumps = (False, False, False)
        self.fallen = False
        self.z = 0.0
        self.turn_time = 0.0

    # ---------------- geometry helpers ----------------
    def heading_vec(self):
        return np.array([-np.sin(self.psi), np.cos(self.psi)])

    def _move_with_slide(self, dx, dy, segs):
        """Move the body; against a wall keep only the sliding part of the motion (with friction).
        Returns the distance actually moved along the heading."""
        x0, y0 = self.x, self.y
        nx, ny = x0 + dx, y0 + dy
        d, cx, cy = world._seg_point_dist(segs, nx, ny)
        if np.all(d >= R):
            self.x, self.y = nx, ny
        else:
            k = int(np.argmin(d))
            n = np.array([nx - cx[k], ny - cy[k]])
            n = n / (np.linalg.norm(n) + 1e-12)
            step = np.array([dx, dy])
            into = step @ n
            if into < 0:
                step = (step - into * n) * 0.5
            tx, ty = x0 + step[0], y0 + step[1]
            if np.all(world._seg_point_dist(segs, tx, ty)[0] >= R - 1e-4):
                self.x, self.y = tx, ty
        return float(np.array([self.x - x0, self.y - y0]) @ self.heading_vec())

    def _bumpers(self, segs):
        d, cx, cy = world._seg_point_dist(segs, self.x, self.y)
        near = d < R + 0.006
        left = centre = right = False
        for px, py in zip(cx[near], cy[near]):
            ang = np.arctan2(-(px - self.x), py - self.y) - self.psi     # contact angle, CCW from heading
            ang = (ang + np.pi) % (2 * np.pi) - np.pi
            if abs(ang) < np.radians(25):
                centre = True
            elif np.radians(25) <= ang < np.radians(95):
                left = True
            elif -np.radians(95) < ang <= -np.radians(25):
                right = True
        return left, centre, right

    def _cliffs(self):
        z0 = world.height(self.x, self.y)
        out = []
        for a in (np.radians(35), 0.0, -np.radians(35)):        # left, centre, right
            px = self.x - 0.16 * np.sin(self.psi + a)
            py = self.y + 0.16 * np.cos(self.psi + a)
            out.append(z0 - world.height(px, py) > 0.06)      # IR cliff sensor: needs a real drop, not a slope
        return out

    # ---------------- one physics step ----------------
    def step(self, cmd_l, cmd_r, segs, dt):
        """Wheel speed commands in mm/s."""
        gx, gy = world.slope(self.x, self.y)
        on_slope = abs(gy) > 0
        mismatch = self.mismatch * (3.0 if on_slope else 1.0)    # wheel mismatch shows up more under load
        tl, tr = cmd_l / 1000.0, cmd_r / 1000.0 * (1 + mismatch)

        # Kobuki base: forward speed ramps up gently and brakes hard; turning responds fast
        v_t, w_t = 0.5 * (tl + tr), (tr - tl) / B
        v0, w0 = 0.5 * (self.vl + self.vr), (self.vr - self.vl) / B
        braking = abs(v_t) < abs(v0) or np.sign(v_t) != np.sign(v0)
        a_lim = self.decel if braking else self.amax
        v1 = v0 + np.clip(v_t - v0, -a_lim * dt, a_lim * dt)
        w1 = w0 + np.clip(w_t - w0, -self.alpha * dt, self.alpha * dt)
        self.vl, self.vr = v1 - w1 * B / 2, v1 + w1 * B / 2
        v = 0.5 * (self.vl + self.vr)
        w = (self.vr - self.vl) / B

        in_place = abs(v) < 0.05 and abs(w) > 0.3
        if in_place:
            self.turn_time += dt
            if self.turn_time > 0.4:                       # new slip value for each turn
                self.slip = self.slip_bias + self.rng.normal(0, 0.04)
                self.turn_time = -1e9
        else:
            self.turn_time = 0.0
        w_true = w * (1 - self.slip) if in_place else w

        # slope: gravity yaws a misaligned robot further downhill, so it drifts toward an edge
        if on_slope and abs(v) > 0.02:
            left = np.array([-np.cos(self.psi), -np.sin(self.psi)])
            w_true -= 4.0 * (gx * left[0] + gy * left[1])
        nx = self.x - np.sin(self.psi) * v * dt
        ny = self.y + np.cos(self.psi) * v * dt
        v_body = self._move_with_slide(nx - self.x, ny - self.y, segs) / dt
        hit = abs(self.v_body) > 0.1 and abs(v_body) < 0.3 * abs(self.v_body)   # just hit something
        if hit:
            self.impact = -np.sign(self.v_body) * min(abs(self.v_body) * 1.2, 1.0)  # short sharp spike (g)
        self.psi += w_true * dt
        self.a_fwd = 0.0 if hit else float(np.clip((v_body - self.v_body) / dt, -3.0, 3.0))
        self.v_body = v_body
        self.impact *= np.exp(-dt / 0.08)

        # encoders count wheel turns: they see the wheels' rotation even when the wheels slip
        # (turning on the spot, or spinning against an obstacle)
        self.enc_dist += v * dt
        self.enc_angle += w * dt
        self.gyro += (w_true + self.gyro_bias) * dt

        self.bumps = self._bumpers(segs)
        self.z = world.height(self.x, self.y)
        if abs(self.x) > world.RAMP_HALF_W and world.ramp_height(self.y) > 0.04 and self.y > world.FUNNEL_END:
            self.fallen = True

    def read(self):
        gx, gy = world.slope(self.x, self.y)
        h = self.heading_vec()
        left = np.array([-np.cos(self.psi), -np.sin(self.psi)])
        s_fwd = gx * h[0] + gy * h[1]
        s_lat = gx * left[0] + gy * left[1]
        jolt = 0.10 * np.clip(self.a_fwd / G, -2, 2)           # front lifts on hard speed changes
        ax = s_fwd + self.a_fwd / G + jolt + self.impact + self.accel_offset + self.rng.normal(0, self.noise)
        ay = s_lat + self.rng.normal(0, self.noise)
        cl, cc, cr = self._cliffs()
        return Sensors(*self.bumps, cl, cc, cr, self.enc_dist * 1000, np.degrees(self.enc_angle),
                       float(ax), float(ay),
                       float(np.degrees(self.gyro) + self.rng.normal(0, 0.2)))

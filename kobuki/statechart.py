"""
Hill climb statechart (Run mode), stepped at 20 Hz.

Core states
  Drive --bump--> ReverseG (50 mm) --> Right/LeftTurn --> PDR/PDL side step --> turn back --> Drive
  Drive --slope up (x accel > 0.12 g), once--> Ramp: arc until sideways tilt < 0.05 g --> Drive
  Drive --slope down (x accel < -0.2 g)--> DownRamp: square up on the spot --> DownRampDrive
  DownRampDrive --floor detected--> GroundRunout --> Stop
  Drive / DownRampDrive --cliff L/C/R--> Reverse (100 mm) --> CliffTurn (15 / 25 / 15 deg) --> back
  The accelerometer is averaged over 20 samples (1 s) before any slope guard.

Features (each can be switched off for the design study in experiments/run_experiments.py)
  heading_hold     The Kobuki gyro keeps straight drives on their target heading, and turns end on
                   an absolute heading (shortest way round, slowing near the target). Without it,
                   both wheels get the same command and turns end on encoder angle (80 / 85 deg).
  motion_comp      The robot's own acceleration (from the encoders) is subtracted from the
                   accelerometer before the slope is judged, so starts, stops and bumps don't look
                   like slopes. The down ramp is only looked for after the climb.
  clean_stop       A short average spots the floor, hysteresis rejects noise, then the robot drives
                   a set distance on the encoders and stops for good. Without it, Stop is a guard on
                   the filtered accelerometer and can flip back to DownRampDrive.
  obstacle_escape  Three bumps within 8 s means a corner: flip the turn direction and side step
                   further. Bumps during a side step back off and turn straight back to heading.
"""
from collections import deque

import numpy as np

G = 9.81
SPEED_TEST = dict(drive=600, reverse=500, turn=500, pending=500, cliff_turn=100, ramp_fast=500,
                  ramp_slow=50, down_drive=500)
DEMO = dict(drive=200, reverse=200, turn=200, pending=200, cliff_turn=100, ramp_fast=200,
            ramp_slow=20, down_drive=200)
FEATURES = ("heading_hold", "motion_comp", "clean_stop", "obstacle_escape")


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class HillClimbStatechart:
    RAMP_UP, RAMP_DOWN, GROUND = 0.12, -0.2, -0.04
    Y_OK = 0.05
    KP = 10.0               # mm/s of wheel speed difference per degree of heading error
    TURN_EARLY = 9.0        # stop the turn command this many degrees early (the robot coasts)
    STOP_AFTER = 200        # mm driven after the floor is detected
    GROUND_ON, GROUND_OFF = -0.06, -0.15

    def __init__(self, speeds=SPEED_TEST, ctrl_dt=0.05, n_filter=20, use_filter=True,
                 heading_hold=True, motion_comp=True, clean_stop=True, obstacle_escape=True):
        self.v = dict(speeds)
        self.dt = ctrl_dt
        self.use_filter = use_filter
        self.heading_hold, self.motion_comp = heading_hold, motion_comp
        self.clean_stop, self.obstacle_escape = clean_stop, obstacle_escape
        self.buf_x = deque(maxlen=n_filter)
        self.buf_y = deque(maxlen=n_filter)
        self.state = "Drive"
        self.ramp_flag = 0
        self.flag = 0                # 0: first turn of a side step, 1: turning back
        self.return_to = "Drive"
        self.turn_dir = None
        self.d0 = self.a0 = 0.0
        self.log = []                # (time, state) on every change
        self.ground_heading = None
        self.target = 0.0
        self.prev_dist = None
        self.v_hist = deque(maxlen=3)
        self.bump_times = deque()
        self.flip = False
        self.side_step = 150
        self.cliff_hold = -1e9
        self.t = 0.0

    @property
    def name(self):
        off = [f for f in FEATURES if not getattr(self, f)]
        return "Full statechart" if not off else "Without " + ", ".join(off)

    # ---------- helpers ----------
    def _enter(self, state, s, t):
        self.state = state
        self.d0, self.a0 = s.net_distance, s.net_angle
        self.log.append((t, state))

    def dist(self, s):
        return abs(s.net_distance - self.d0)

    def angle(self, s):
        return abs(s.net_angle - self.a0)

    # ---------- sensing ----------
    def accel(self, s):
        ax = s.accel_x
        if self.motion_comp:
            if self.prev_dist is None:
                self.prev_dist = s.net_distance
            v = (s.net_distance - self.prev_dist) / 1000.0 / self.dt
            self.prev_dist = s.net_distance
            self.v_hist.append(v)
            a_own = (self.v_hist[-1] - self.v_hist[0]) / (self.dt * max(len(self.v_hist) - 1, 1))
            ax = ax - 1.10 * a_own / G            # 1.10: the jolt makes the spike about 10% bigger
        self.buf_x.append(ax)
        self.buf_y.append(s.accel_y)
        if self.use_filter:
            return float(np.mean(self.buf_x)), float(np.mean(self.buf_y))
        return ax, s.accel_y

    def short_ax(self):
        """5 sample average (0.25 s): notices the floor at the bottom of the ramp sooner."""
        return float(np.mean(list(self.buf_x)[-5:]))

    def heading(self, s):
        if self.ground_heading is None:
            self.ground_heading = s.gyro_heading
        return wrap(s.gyro_heading - self.ground_heading)

    # ---------- motion ----------
    def straight(self, speed, s):
        if not self.heading_hold:
            return speed, speed
        err = wrap(self.target - self.heading(s))
        d = float(np.clip(self.KP * err, -150, 150))
        return speed - d / 2, speed + d / 2

    def turned_to(self, s, target):
        return abs(wrap(target - self.heading(s))) <= self.TURN_EARLY

    def bump_turn(self, s):
        d = "Right" if (s.bump_left or s.bump_centre) else "Left"     # away from the bumper that hit
        if not self.obstacle_escape:
            return d
        self.bump_times.append(self.t)
        while self.bump_times and self.bump_times[0] < self.t - 8.0:
            self.bump_times.popleft()
        if len(self.bump_times) >= 3:              # boxed in: go the other way, step further
            self.flip = not self.flip
            self.side_step = 300
            self.bump_times.clear()
        else:
            self.side_step = 150
        if self.flip:
            d = "Left" if d == "Right" else "Right"
        return d

    # ---------- one controller tick ----------
    def step(self, s, t):
        self.t = t
        h = self.heading(s)
        ax, ay = self.accel(s)
        st = self.state
        cliff = s.cliff_left or s.cliff_centre or s.cliff_right
        bump = s.bump_left or s.bump_centre or s.bump_right
        bump_states = ("Drive", "PDL", "PDR") if self.obstacle_escape else ("Drive",)

        if st in ("Drive", "DownRampDrive") and cliff:
            self.return_to = st
            self.cliff_side = "Left" if s.cliff_left else ("Centre" if s.cliff_centre else "Right")
            self._enter("Reverse", s, t)
        elif st in bump_states and bump:
            self.turn_dir = self.bump_turn(s)
            if self.obstacle_escape:               # bumped mid side step: turn straight back to heading
                self.flag = 1 if st in ("PDL", "PDR") else 0
            self._enter("ReverseG", s, t)
        elif st == "Drive":
            if self.heading_hold and s.net_distance > self.cliff_hold:
                self.target = 0.0                  # back to the ground orientation
            if ax > self.RAMP_UP and self.ramp_flag == 0:
                self.ramp_flag = 1
                self._enter("Ramp", s, t)
            elif ax < self.RAMP_DOWN and (self.ramp_flag == 1 or not self.motion_comp):
                self._enter("DownRamp", s, t)
        elif st == "ReverseG":
            if self.dist(s) >= 50:
                self._enter(self.turn_dir + "Turn", s, t)
        elif st in ("RightTurn", "LeftTurn"):
            if self.heading_hold:
                if self.flag == 0:
                    tgt = -90.0 if st == "RightTurn" else 90.0
                    if self.turned_to(s, tgt):
                        self.flag, self.target = 1, tgt
                        self._enter("PDR" if st == "RightTurn" else "PDL", s, t)
                elif self.turned_to(s, 0.0):
                    self.flag, self.target = 0, 0.0
                    self._enter("Drive", s, t)
            else:
                if self.flag == 0 and self.angle(s) >= 80:
                    self.flag = 1
                    self._enter("PDR" if st == "RightTurn" else "PDL", s, t)
                elif self.flag == 1 and self.angle(s) >= 85:
                    self.flag = 0
                    self._enter("Drive", s, t)
        elif st in ("PDR", "PDL"):
            if self.dist(s) >= self.side_step:
                self._enter("LeftTurn" if st == "PDR" else "RightTurn", s, t)
        elif st == "Ramp":
            if abs(ay) <= self.Y_OK and ax > 0:
                self.target = 0.0
                self._enter("Drive", s, t)
        elif st == "DownRamp":
            if abs(ay) <= self.Y_OK:
                self.target = h                    # hold the heading it lined up on
                self._enter("DownRampDrive", s, t)
        elif st == "DownRampDrive":
            if self.clean_stop:
                if self.short_ax() > self.GROUND_ON:
                    self._enter("GroundRunout", s, t)
            elif ax > self.GROUND:
                self._enter("Stop", s, t)
        elif st == "GroundRunout":
            if self.short_ax() < self.GROUND_OFF:  # still on the slope after all
                self._enter("DownRampDrive", s, t)
            elif self.dist(s) >= self.STOP_AFTER:
                self._enter("Stop", s, t)
        elif st == "Stop":
            if not self.clean_stop and ax < self.GROUND:
                self._enter("DownRampDrive", s, t)
        elif st == "Reverse":
            if self.dist(s) >= 100:
                self._enter("CliffTurn" + self.cliff_side, s, t)
        elif st.startswith("CliffTurn"):
            target = 25 if st.endswith("Centre") else 15
            if self.angle(s) >= target:
                if self.heading_hold and self.return_to == "Drive":
                    self.target = h                # hold the new, safer heading for 30 cm
                    self.cliff_hold = s.net_distance + 300
                self._enter(self.return_to, s, t)
        return self.action(s, ay)

    def action(self, s, ay):
        st, v = self.state, self.v
        if st == "Drive":
            return self.straight(v["drive"], s)
        if st in ("ReverseG", "Reverse"):
            return -v["reverse"], -v["reverse"]
        if st in ("RightTurn", "LeftTurn"):
            if self.heading_hold:
                tgt = (-90.0 if st == "RightTurn" else 90.0) if self.flag == 0 else 0.0
                err_signed = wrap(tgt - self.heading(s))
                err = abs(err_signed)
                sp = v["turn"] if err > 30 else max(150.0, v["turn"] * err / 30)   # slow near the target
                return (-sp, sp) if err_signed > 0 else (sp, -sp)                  # shortest way round
            return (v["turn"], -v["turn"]) if st == "RightTurn" else (-v["turn"], v["turn"])
        if st in ("PDR", "PDL"):
            return self.straight(v["pending"], s)
        if st == "Ramp":
            if ay < -self.Y_OK:
                return v["ramp_fast"], v["ramp_slow"]      # arc right
            return v["ramp_slow"], v["ramp_fast"]          # arc left
        if st == "DownRamp":
            if ay > 0:
                return v["turn"], -v["turn"]               # turn right on the spot
            return -v["turn"], v["turn"]
        if st == "DownRampDrive":
            return self.straight(v["down_drive"], s)
        if st == "GroundRunout":
            return self.straight(v["down_drive"] * 0.6, s)
        if st in ("CliffTurnLeft", "CliffTurnCentre"):
            return v["cliff_turn"], -v["cliff_turn"]       # turn right
        if st == "CliffTurnRight":
            return -v["cliff_turn"], v["cliff_turn"]       # turn left
        return 0.0, 0.0                                    # Stop

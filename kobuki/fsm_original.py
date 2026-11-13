"""
Python port of our team's LabVIEW statechart (Run mode), as run in the final workshop.

States, guards and actions are taken from the statechart export and the report:
  Drive --BumpLeft|BumpCentre--> ReverseG(50 mm) --> RightTurn --angle>=80, flag=0--> PDR (flag=1)
        --150 mm--> LeftTurn --angle>=85, flag=1--> Drive (flag=0)          (mirror for BumpRight)
  Drive --x_accel>0.12, ramp_flag=0--> Ramp (ramp_flag=1): arc until |y_accel|<=0.05 --> Drive
  Drive --x_accel<-0.2--> DownRamp: turn on the spot until |y_accel|<=0.05 --> DownRampDrive
  DownRampDrive --x_accel>-0.04--> Stop --x_accel<-0.04--> DownRampDrive   (re-check, report 6.3)
  Drive / DownRampDrive --cliff L/C/R--> Reverse(100 mm) --> CliffTurn(15 / 25 / 15 deg) --> back
Accelerometer guards use a 20 sample moving average (report section 5).

Angles and distances come from the wheel encoders (net_angle, net_distance), measured from the
start of each manoeuvre, like StateData.angle / distance at manoeuvre start.
"""
from collections import deque

import numpy as np

SPEED_TEST = dict(drive=600, reverse=500, turn=500, pending=500, cliff_turn=100, ramp_fast=500,
                  ramp_slow=50, down_drive=500)
DEMO = dict(drive=200, reverse=200, turn=200, pending=200, cliff_turn=100, ramp_fast=200,
            ramp_slow=20, down_drive=200)


class OriginalFSM:
    name = "Original (team statechart)"
    RAMP_UP, RAMP_DOWN, GROUND = 0.12, -0.2, -0.04
    Y_OK = 0.05

    def __init__(self, speeds=SPEED_TEST, use_filter=True, use_flags=True, n_filter=20):
        self.v = dict(speeds)
        self.use_filter, self.use_flags = use_filter, use_flags
        self.buf_x = deque(maxlen=n_filter)
        self.buf_y = deque(maxlen=n_filter)
        self.state = "Drive"
        self.ramp_flag = 0
        self.flag = 0                # t_flag in the report
        self.return_to = "Drive"     # caller state for the cliff routines
        self.turn_dir = None
        self.d0 = self.a0 = 0.0
        self.log = []                # (time, state) on every change

    # -------- helpers --------
    def _enter(self, state, s, t):
        self.state = state
        self.d0, self.a0 = s.net_distance, s.net_angle
        self.log.append((t, state))

    def dist(self, s):
        return abs(s.net_distance - self.d0)

    def angle(self, s):
        return abs(s.net_angle - self.a0)

    def accel(self, s):
        self.buf_x.append(s.accel_x)
        self.buf_y.append(s.accel_y)
        if self.use_filter:
            return float(np.mean(self.buf_x)), float(np.mean(self.buf_y))
        return s.accel_x, s.accel_y

    def straight(self, speed, s):
        """Wheel speeds for driving straight. The original just sets both wheels equal."""
        return speed, speed

    def bump_turn(self, s):
        """Which way to turn after a bump: away from the bumper that hit."""
        if s.bump_left or s.bump_centre:
            return "Right"
        return "Left"

    # -------- one controller tick --------
    def step(self, s, t):
        ax, ay = self.accel(s)
        st = self.state
        cliff = s.cliff_left or s.cliff_centre or s.cliff_right
        bump = s.bump_left or s.bump_centre or s.bump_right

        if st in ("Drive", "DownRampDrive") and cliff:
            self.return_to = st
            self.cliff_side = "Left" if s.cliff_left else ("Centre" if s.cliff_centre else "Right")
            self._enter("Reverse", s, t)
        elif st == "Drive":
            if bump:
                self.turn_dir = self.bump_turn(s)
                self._enter("ReverseG", s, t)
            elif ax > self.RAMP_UP and (self.ramp_flag == 0 or not self.use_flags):
                self.ramp_flag = 1
                self._enter("Ramp", s, t)
            elif ax < self.RAMP_DOWN:
                self._enter("DownRamp", s, t)
        elif st == "ReverseG":
            if self.dist(s) >= 50:
                self._enter(self.turn_dir + "Turn", s, t)
        elif st in ("RightTurn", "LeftTurn"):
            first = self.flag == 0 or not self.use_flags
            if first and self.angle(s) >= 80:
                self.flag = 1
                self._enter("PDR" if st == "RightTurn" else "PDL", s, t)
            elif not first and self.angle(s) >= 85:
                self.flag = 0
                self._enter("Drive", s, t)
        elif st in ("PDR", "PDL"):
            if self.dist(s) >= 150:
                self._enter("LeftTurn" if st == "PDR" else "RightTurn", s, t)
        elif st == "Ramp":
            if abs(ay) <= self.Y_OK and ax > 0:
                self._enter("Drive", s, t)
        elif st == "DownRamp":
            if abs(ay) <= self.Y_OK:
                self._enter("DownRampDrive", s, t)
        elif st == "DownRampDrive":
            if ax > self.GROUND:
                self._enter("Stop", s, t)
        elif st == "Stop":
            if ax < self.GROUND:
                self._enter("DownRampDrive", s, t)
        elif st == "Reverse":
            if self.dist(s) >= 100:
                self._enter("CliffTurn" + self.cliff_side, s, t)
        elif st.startswith("CliffTurn"):
            target = 25 if st.endswith("Centre") else 15
            if self.angle(s) >= target:
                self._enter(self.return_to, s, t)
        return self.action(s, ay)

    def action(self, s, ay):
        st, v = self.state, self.v
        if st == "Drive":
            return self.straight(v["drive"], s)
        if st in ("ReverseG", "Reverse"):
            return -v["reverse"], -v["reverse"]
        if st == "RightTurn":
            return v["turn"], -v["turn"]
        if st == "LeftTurn":
            return -v["turn"], v["turn"]
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
        if st in ("CliffTurnLeft", "CliffTurnCentre"):
            return v["cliff_turn"], -v["cliff_turn"]       # turn right
        if st == "CliffTurnRight":
            return -v["cliff_turn"], v["cliff_turn"]       # turn left
        return 0.0, 0.0                                    # Stop

"""
Upgraded statechart. Same states and the same overall behaviour as the team's design, plus
fixes for the problems listed in the report's Limitations and Possible Improvements:

  1. Gyro heading hold   The Kobuki's gyro (unused in 2023) keeps every straight drive on its
                         target heading, and turns end on an absolute heading instead of
                         "80 / 85 deg from the encoders". Drift and turn errors no longer pile up.
  2. Motion compensated  The accelerometer also feels the robot's own speed changes (the jolts
     tilt sensing        that caused false Ramp / DownRamp / Stop transitions). The wheel encoders
                         say how fast the robot is speeding up, so that part is subtracted before
                         the slope is judged.
  3. Clean stop          Ground is detected with hysteresis, then the robot drives a set distance
                         on the encoders and stops for good. No Stop <-> DownRampDrive stutter.
  4. Corner escape       Three bump manoeuvres within 8 s means a corner: flip the turn direction
                         and side step further.
  5. Bumps in side steps PDL / PDR react to bumps instead of pushing into obstacles.
  6. Shortest way round  Turns always take the shorter direction (never more than 180 deg).
"""
from collections import deque

import numpy as np

from .fsm_original import SPEED_TEST, OriginalFSM

G = 9.81


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class UpgradedFSM(OriginalFSM):
    name = "Upgraded"
    KP = 10.0               # mm/s of wheel speed difference per degree of heading error
    TURN_EARLY = 9.0        # stop the turn command this many degrees early (the robot coasts)
    STOP_AFTER = 200        # mm driven after the ground is detected (lands about 40 cm past the bottom)
    GROUND_ON, GROUND_OFF = -0.06, -0.15

    def __init__(self, speeds=SPEED_TEST, ctrl_dt=0.05, **kw):
        super().__init__(speeds, **kw)
        self.dt = ctrl_dt
        self.ground_heading = None
        self.target = 0.0
        self.prev_dist = None
        self.v_hist = deque(maxlen=3)
        self.bump_times = deque()
        self.flip = False
        self.side_step = 150
        self.cliff_hold = -1e9
        self.t = 0.0

    # ---------- sensing ----------
    def accel(self, s):
        """Remove the robot's own acceleration (from the encoders) before filtering."""
        if self.prev_dist is None:
            self.prev_dist = s.net_distance
        v = (s.net_distance - self.prev_dist) / 1000.0 / self.dt
        self.prev_dist = s.net_distance
        self.v_hist.append(v)
        a_own = (self.v_hist[-1] - self.v_hist[0]) / (self.dt * max(len(self.v_hist) - 1, 1))
        s_ax = s.accel_x - 1.10 * a_own / G           # 1.10: the jolt makes the spike ~10% bigger
        self.buf_x.append(s_ax)
        self.buf_y.append(s.accel_y)
        return float(np.mean(self.buf_x)), float(np.mean(self.buf_y))

    def short_ax(self):
        """5 sample average (0.25 s): notices the floor at the bottom of the ramp sooner."""
        return float(np.mean(list(self.buf_x)[-5:]))

    def heading(self, s):
        if self.ground_heading is None:
            self.ground_heading = s.gyro_heading
        return wrap(s.gyro_heading - self.ground_heading)

    def straight(self, speed, s):
        err = wrap(self.target - self.heading(s))
        d = float(np.clip(self.KP * err, -150, 150))
        return speed - d / 2, speed + d / 2

    def turned_to(self, s, target):
        return abs(wrap(target - self.heading(s))) <= self.TURN_EARLY

    def bump_turn(self, s):
        now = self.t
        self.bump_times.append(now)
        while self.bump_times and self.bump_times[0] < now - 8.0:
            self.bump_times.popleft()
        if len(self.bump_times) >= 3:              # boxed in: go the other way, step further
            self.flip = not self.flip
            self.side_step = 300
            self.bump_times.clear()
        else:
            self.side_step = 150
        d = super().bump_turn(s)
        if self.flip:
            d = "Left" if d == "Right" else "Right"
        return d

    # ---------- statechart ----------
    def step(self, s, t):
        self.t = t
        h = self.heading(s)
        ax, ay = self.accel(s)
        st = self.state
        cliff = s.cliff_left or s.cliff_centre or s.cliff_right
        bump = s.bump_left or s.bump_centre or s.bump_right

        if st in ("Drive", "DownRampDrive") and cliff:
            self.return_to = st
            self.cliff_side = "Left" if s.cliff_left else ("Centre" if s.cliff_centre else "Right")
            self._enter("Reverse", s, t)
        elif st in ("Drive", "PDL", "PDR") and bump:
            self.turn_dir = self.bump_turn(s)
            # bumped while side stepping: back off and turn straight back to the ground heading
            self.flag = 1 if st in ("PDL", "PDR") else 0
            self._enter("ReverseG", s, t)
        elif st == "Drive":
            if s.net_distance > self.cliff_hold:
                self.target = 0.0                  # back to the ground orientation
            if ax > self.RAMP_UP and self.ramp_flag == 0:
                self.ramp_flag = 1
                self._enter("Ramp", s, t)
            elif ax < self.RAMP_DOWN and self.ramp_flag == 1:      # only after the climb
                self._enter("DownRamp", s, t)
        elif st == "ReverseG":
            if self.dist(s) >= 50:
                self._enter(self.turn_dir + "Turn", s, t)
        elif st in ("RightTurn", "LeftTurn"):
            if self.flag == 0:
                tgt = -90.0 if st == "RightTurn" else 90.0
                if self.turned_to(s, tgt):
                    self.flag, self.target = 1, tgt
                    self._enter("PDR" if st == "RightTurn" else "PDL", s, t)
            elif self.turned_to(s, 0.0):
                self.flag, self.target = 0, 0.0
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
            if self.short_ax() > self.GROUND_ON:
                self._enter("GroundRunout", s, t)
        elif st == "GroundRunout":
            if self.short_ax() < self.GROUND_OFF:  # still on the slope after all
                self._enter("DownRampDrive", s, t)
            elif self.dist(s) >= self.STOP_AFTER:
                self._enter("Stop", s, t)
        elif st == "Reverse":
            if self.dist(s) >= 100:
                self._enter("CliffTurn" + self.cliff_side, s, t)
        elif st.startswith("CliffTurn"):
            target = 25 if st.endswith("Centre") else 15
            if self.angle(s) >= target:
                if self.return_to == "Drive":
                    self.target = h                # hold the new, safer heading for 30 cm
                    self.cliff_hold = s.net_distance + 300
                self._enter(self.return_to, s, t)
        return self.action(s, ay)

    def action(self, s, ay):
        st, v = self.state, self.v
        if st == "GroundRunout":
            return self.straight(v["down_drive"] * 0.6, s)
        if st in ("RightTurn", "LeftTurn"):
            tgt = (-90.0 if st == "RightTurn" else 90.0) if self.flag == 0 else 0.0
            err_signed = wrap(tgt - self.heading(s))
            err = abs(err_signed)
            sp = v["turn"] if err > 30 else max(150.0, v["turn"] * err / 30)   # slow down near the target
            return (-sp, sp) if err_signed > 0 else (sp, -sp)                  # shortest way round
        return super().action(s, ay)

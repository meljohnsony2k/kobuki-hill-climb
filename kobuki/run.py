"""Run one course: physics at 200 Hz, statechart at 20 Hz."""
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from . import world
from .robot import Kobuki

PHYS_DT, CTRL_DT = 0.005, 0.05     # 20 Hz statechart: makes the 20 sample filter a 1 s average
IN_PLACE = {"RightTurn", "LeftTurn", "DownRamp", "CliffTurnLeft", "CliffTurnCentre", "CliffTurnRight"}


@dataclass
class Result:
    outcome: str                 # success | fell | stuck | stopped_early | stopped_late | overshot
    time: float
    states: list = field(default_factory=list)
    trace: np.ndarray = None     # t, x, y, psi, z per control tick
    max_spin_deg: float = 0.0    # largest single in-place rotation (turnabout requirement: <= 180)
    chatter: int = 0             # state changes less than 0.1 s after the previous one


def run_course(fsm, course, seed, t_max=120.0, keep_trace=False, robot_kw=None):
    rng = np.random.default_rng(seed + 10_000)
    bot = Kobuki(rng, **(robot_kw or {}))
    n_sub = int(round(CTRL_DT / PHYS_DT))
    t, cmd = 0.0, (0.0, 0.0)
    trace, hist = [], deque()
    spin, last_state, max_spin = 0.0, fsm.state, 0.0
    outcome = "stuck"
    while t < t_max:
        s = bot.read()
        cmd = fsm.step(s, t)
        if fsm.state != last_state:
            spin, last_state = 0.0, fsm.state
        for _ in range(n_sub):
            bot.step(cmd[0], cmd[1], course.segments, PHYS_DT)
            if fsm.state in IN_PLACE:
                spin += abs(bot.vr - bot.vl) / 0.230 * PHYS_DT
        max_spin = max(max_spin, np.degrees(spin))
        t += CTRL_DT
        if keep_trace:
            trace.append((t, bot.x, bot.y, bot.psi, bot.z))
        if bot.fallen:
            outcome = "fell"
            break
        if bot.y > world.FINISH[1] + 0.6:
            outcome = "overshot"
            break
        # finished = robot has come to rest (stayed within 3 cm for 2 s) after the ramp
        hist.append((t, bot.x, bot.y))
        while hist and hist[0][0] < t - 2.0:
            hist.popleft()
        if fsm.state in ("Stop", "DownRampDrive") and t - hist[0][0] > 1.9:
            if max(np.hypot(bot.x - x, bot.y - y) for _, x, y in hist) < 0.03:
                rest_t = hist[0][0]
                if world.FINISH[0] <= bot.y <= world.FINISH[1]:
                    outcome = "success"
                elif bot.y < world.FINISH[0]:
                    outcome = "stopped_early"
                else:
                    outcome = "stopped_late"
                t = rest_t
                break
    times = [a for a, _ in fsm.log]
    chatter = int(np.sum(np.diff(times) < 0.1)) if len(times) > 1 else 0
    return Result(outcome, t, list(fsm.log), np.array(trace) if keep_trace else None, max_spin, chatter)

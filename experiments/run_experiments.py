"""
Monte Carlo comparison: the 2023 statechart vs the upgraded one over random courses and random
"lab hardware" (every run gets its own wheel mismatch, turn slip and accelerometer offset).

Writes results/summary.json. About 30 minutes on 2 cores.
Run from the repo root:  python experiments/run_experiments.py
"""
import json
import os
import sys
from collections import Counter
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kobuki import world  # noqa: E402
from kobuki.fsm_original import SPEED_TEST, OriginalFSM  # noqa: E402
from kobuki.fsm_upgraded import UpgradedFSM  # noqa: E402
from kobuki.run import run_course  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
N = 200
TUNED = dict(wheel_mismatch=0.0, turn_slip_bias=0.10, accel_offset=-0.02)   # a robot like the one it was tuned on
FAST = dict(SPEED_TEST, drive=700, pending=600)

VARIANTS = {
    "2023 statechart, tuned robot": ("orig", SPEED_TEST, TUNED, {}),
    "2023 statechart, random lab robot": ("orig", SPEED_TEST, None, {}),
    "2023, random robot, no accel filter": ("orig", SPEED_TEST, None, dict(use_filter=False)),
    "Upgraded, random lab robot": ("up", SPEED_TEST, None, {}),
    "Upgraded, random robot, 700 mm/s": ("up", FAST, None, {}),
}
FAIL_GROUPS = {"success": "Success", "stuck": "Stuck / timed out", "fell": "Fell off",
               "stopped_early": "Wrong stop", "stopped_late": "Wrong stop", "overshot": "Wrong stop"}


def one(args):
    kind, speeds, robot_kw, fsm_kw, seed = args
    course = world.make_course(np.random.default_rng(seed))
    fsm = (UpgradedFSM if kind == "up" else OriginalFSM)(speeds, **fsm_kw)
    r = run_course(fsm, course, seed, keep_trace=True, robot_kw=robot_kw)
    states = [s for _, s in r.states]
    return dict(seed=seed, outcome=r.outcome, time=r.time, chatter=r.chatter, max_spin=r.max_spin_deg,
                stop_past_bottom=float(r.trace[-1, 2] - world.DOWN[1]),
                ramp_entries=states.count("Ramp"))


def summarise(rows):
    c = Counter(FAIL_GROUPS[r["outcome"]] for r in rows)
    ok = [r for r in rows if r["outcome"] == "success"]
    return dict(n=len(rows), outcomes=dict(c), success_pct=100 * len(ok) / len(rows),
                median_time_s=float(np.median([r["time"] for r in ok])) if ok else None,
                median_stop_past_bottom_m=float(np.median([r["stop_past_bottom"] for r in ok])) if ok else None,
                chatter_per_run=float(np.mean([r["chatter"] for r in rows])),
                max_spin_deg=float(max(r["max_spin"] for r in rows)),
                runs_with_repeat_ramp=int(sum(r["ramp_entries"] > 1 for r in rows)))


def main():
    out = dict(n_courses=N, variants={}, sweeps={})
    with Pool() as pool:
        for name, (kind, sp, rk, fk) in VARIANTS.items():
            rows = pool.map(one, [(kind, sp, rk, fk, s) for s in range(N)])
            out["variants"][name] = summarise(rows)
            print(f"{name:40s} {out['variants'][name]['success_pct']:5.1f}%  {out['variants'][name]['outcomes']}",
                  flush=True)

        # robustness: change one hardware property at a time, the rest as on the tuned robot
        sweeps = {"turn_slip_bias": [-0.05, 0.0, 0.05, 0.10, 0.15, 0.20],
                  "wheel_mismatch": [0.0, 0.004, 0.008, 0.012, 0.016],
                  "accel_offset": [-0.040, -0.035, -0.030, -0.025, -0.020, -0.015]}
        for key, vals in sweeps.items():
            res = {}
            for kind in ("orig", "up"):
                pct = []
                for v in vals:
                    rk = dict(TUNED, **{key: v})
                    rows = pool.map(one, [(kind, SPEED_TEST, rk, {}, s) for s in range(80)])
                    pct.append(100 * sum(r["outcome"] == "success" for r in rows) / len(rows))
                res[kind] = pct
                print(key, kind, pct, flush=True)
            out["sweeps"][key] = dict(values=vals, original=res["orig"], upgraded=res["up"])

    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    with open(os.path.join(ROOT, "results", "summary.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("saved results/summary.json", flush=True)


if __name__ == "__main__":
    main()

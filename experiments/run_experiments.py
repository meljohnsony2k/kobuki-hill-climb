"""
Monte Carlo design study over random courses and random "lab hardware" (every run gets its own
wheel mismatch, turn slip and accelerometer offset). The full statechart is run, then again with
each feature switched off, to measure what each one contributes.

Writes results/summary.json. About 35 minutes on 2 cores.
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
from kobuki.statechart import FEATURES, SPEED_TEST, HillClimbStatechart  # noqa: E402
from kobuki.run import run_course  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
N = 200
TYPICAL = dict(wheel_mismatch=0.0, turn_slip_bias=0.10, accel_offset=-0.02)   # mid range robot for sweeps
FAST = dict(SPEED_TEST, drive=700, pending=600)
CORE = {f: False for f in FEATURES}

VARIANTS = {
    "Full statechart": (SPEED_TEST, {}),
    "Full statechart, 700 mm/s": (FAST, {}),
    "Without heading hold": (SPEED_TEST, dict(heading_hold=False)),
    "Without motion compensation": (SPEED_TEST, dict(motion_comp=False)),
    "Without clean stop": (SPEED_TEST, dict(clean_stop=False)),
    "Without motion compensation or clean stop": (SPEED_TEST, dict(motion_comp=False, clean_stop=False)),
    "Without motion compensation, 700 mm/s": (FAST, dict(motion_comp=False)),
    "Without obstacle escape": (SPEED_TEST, dict(obstacle_escape=False)),
    "Core states only": (SPEED_TEST, CORE),
}
SWEEPS = {   # hardware property -> (values, feature that should handle it)
    "wheel_mismatch": ([0.0, 0.004, 0.008, 0.012, 0.016], ("heading_hold",)),
    "turn_slip_bias": ([-0.05, 0.0, 0.05, 0.10, 0.15, 0.20], ("heading_hold",)),
    "accel_offset": ([-0.040, -0.035, -0.030, -0.025, -0.020, -0.015], ("motion_comp", "clean_stop")),
}
FAIL_GROUPS = {"success": "Success", "stuck": "Stuck / timed out", "fell": "Fell off",
               "stopped_early": "Wrong stop", "stopped_late": "Wrong stop", "overshot": "Wrong stop"}


def one(args):
    speeds, robot_kw, fsm_kw, seed = args
    course = world.make_course(np.random.default_rng(seed))
    fsm = HillClimbStatechart(speeds, **fsm_kw)
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
        for name, (sp, fk) in VARIANTS.items():
            rows = pool.map(one, [(sp, None, fk, s) for s in range(N)])
            out["variants"][name] = summarise(rows)
            print(f"{name:32s} {out['variants'][name]['success_pct']:5.1f}%  {out['variants'][name]['outcomes']}",
                  flush=True)

        # change one hardware property at a time (the rest mid range), with and without the feature
        for key, (vals, feats) in SWEEPS.items():
            res = {}
            for label, fk in (("full", {}), ("without", {f: False for f in feats})):
                pct = []
                for v in vals:
                    rk = dict(TYPICAL, **{key: v})
                    rows = pool.map(one, [(SPEED_TEST, rk, fk, s) for s in range(80)])
                    pct.append(100 * sum(r["outcome"] == "success" for r in rows) / len(rows))
                res[label] = pct
                print(key, label, pct, flush=True)
            out["sweeps"][key] = dict(values=vals, features=list(feats), full=res["full"], without=res["without"])

    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    with open(os.path.join(ROOT, "results", "summary.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("saved results/summary.json", flush=True)


if __name__ == "__main__":
    main()

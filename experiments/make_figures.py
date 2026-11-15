"""
Figures for the README: design study outcomes, robustness sweeps, example paths and an animated
run. Needs results/summary.json from run_experiments.py (and ffmpeg for the GIF).

Run from the repo root:  python experiments/make_figures.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kobuki import world  # noqa: E402
from kobuki.statechart import SPEED_TEST, HillClimbStatechart  # noqa: E402
from kobuki.robot import R  # noqa: E402
from kobuki.run import run_course  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#e6e5e1", "#fcfcfb"
EXAMPLE_SEEDS = (0, 2, 4)
NICE = dict(heading_hold="heading hold", motion_comp="motion comp.", clean_stop="clean stop",
            obstacle_escape="obstacle escape")
GIF_SEED = 4
LABEL = {"success": "made it", "stuck": "stuck", "fell": "fell off", "stopped_early": "stopped in the wrong place",
         "stopped_late": "stopped too far", "overshot": "overshot"}

plt.rcParams.update({"font.size": 10, "axes.labelcolor": INK, "axes.titlecolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.edgecolor": MUTED})


def style(ax, grid=True):
    ax.set_facecolor(SURFACE)
    if grid:
        ax.grid(color=GRID, lw=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def outcomes(summary):
    cats = [("Success", BLUE), ("Stuck / timed out", ORANGE), ("Fell off", AQUA), ("Wrong stop", YELLOW)]
    names = list(summary["variants"])
    fig, ax = plt.subplots(figsize=(9.5, 4.2), facecolor=SURFACE)
    style(ax, grid=False)
    for i, name in enumerate(names):
        v = summary["variants"][name]
        left = 0
        for cat, col in cats:
            pct = 100 * v["outcomes"].get(cat, 0) / v["n"]
            if pct > 0:
                ax.barh(i, pct, left=left, color=col, height=0.62, edgecolor=SURFACE, linewidth=2)
                if pct >= 7:
                    ax.text(left + pct / 2, i, f"{pct:.1f}".rstrip("0").rstrip(".") + "%", ha="center", va="center", fontsize=9,
                            color="white" if col in (BLUE, ORANGE) else INK)
            left += pct
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names)
    ax.invert_yaxis()
    ax.set_xlim(0, 100)
    ax.set_xlabel(f"Share of {summary['n_courses']} random courses (%)")
    handles = [Rectangle((0, 0), 1, 1, color=c) for _, c in cats]
    ax.legend(handles, [c for c, _ in cats], loc="lower left", bbox_to_anchor=(0, 1.0), ncol=4, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, "media", "outcomes.png"), dpi=160, facecolor=SURFACE)
    plt.close(fig)


def robustness(summary):
    labels = {"turn_slip_bias": ("Turn slip (fraction of each turn lost)", 1),
              "wheel_mismatch": ("Wheel speed mismatch (%)", 100),
              "accel_offset": ("Accelerometer reading on flat ground (g)", 1)}
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.6), facecolor=SURFACE, sharey=True)
    for ax, (key, (lab, k)) in zip(axs, labels.items()):
        style(ax)
        sw = summary["sweeps"][key]
        x = np.array(sw["values"]) * k
        feat = "stop sensing" if len(sw["features"]) > 1 else NICE[sw["features"][0]]
        ax.plot(x, sw["without"], "-o", color=ORANGE, lw=2, ms=5, markeredgecolor=SURFACE, label=f"without {feat}")
        ax.plot(x, sw["full"], "-o", color=BLUE, lw=2, ms=5, markeredgecolor=SURFACE, label="full statechart")
        ax.legend(frameon=False, loc="lower right" if key == "accel_offset" else "lower left", fontsize=8.5)
        ax.set_xlabel(lab)
        ax.set_ylim(0, 105)
    axs[0].set_ylabel("Success (%)")
    fig.suptitle("Change one hardware property at a time (the rest mid range)", x=0.06, ha="left",
                 color=INK, fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(ROOT, "media", "robustness.png"), dpi=160, facecolor=SURFACE)
    plt.close(fig)


def draw_course(ax, course):
    ax.set_facecolor(SURFACE)
    ax.add_patch(Rectangle((-world.RAMP_HALF_W, world.UP[0]), 2 * world.RAMP_HALF_W, world.DOWN[1] - world.UP[0],
                           color="#e8e7e2", zorder=0))
    for (a, b), lab in ((world.UP, "up ramp"), (world.FLAT, "plateau"), (world.DOWN, "down ramp")):
        ax.text(0, (a + b) / 2, lab, ha="center", va="center", color=MUTED, fontsize=8, zorder=1)
    ax.add_patch(Rectangle((-0.6, world.FINISH[0]), 1.2, world.FINISH[1] - world.FINISH[0], color=AQUA, alpha=0.15,
                           lw=0, zorder=0))
    ax.text(0.62, world.FINISH[0] + 0.05, "finish", color=MUTED, fontsize=8, va="bottom")
    for x1, y1, x2, y2 in course.segments[:5]:
        ax.plot([x1, x2], [y1, y2], color=INK, lw=2.2, zorder=2)
    for p in course.obstacles:
        ax.add_patch(Polygon(p, closed=True, color=MUTED, zorder=2))
    ax.set_aspect("equal")
    ax.set_xlim(-1.65, 1.65)
    ax.set_ylim(-0.3, 6.5)
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def example_runs():
    out = []
    for seed in EXAMPLE_SEEDS:
        course = world.make_course(np.random.default_rng(seed))
        out.append((course, run_course(HillClimbStatechart(SPEED_TEST), course, seed, keep_trace=True, t_max=40)))
    return out


def paths(examples):
    fig, axs = plt.subplots(1, len(examples), figsize=(11, 7.4), facecolor=SURFACE)
    for ax, (course, r) in zip(axs, examples):
        draw_course(ax, course)
        tr = r.trace
        ax.plot(tr[:, 1], tr[:, 2], color=BLUE, lw=2, zorder=3)
        ax.add_patch(Circle((tr[-1, 1], tr[-1, 2]), R, fill=False, color=BLUE, lw=2, zorder=4))
        ax.set_title(f"{LABEL[r.outcome]}" + (f" in {r.time:.1f} s" if r.outcome == "success" else ""), fontsize=10)
    fig.suptitle("Three random courses, three random robots", x=0.05, ha="left", color=INK, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(ROOT, "media", "example_paths.png"), dpi=160, facecolor=SURFACE)
    plt.close(fig)


def state_at(log, t):
    s = "Drive"
    for tt, st in log:
        if tt > t:
            break
        s = st
    return s


def animation(seed=GIF_SEED, every=0.15):
    course = world.make_course(np.random.default_rng(seed))
    r = run_course(HillClimbStatechart(SPEED_TEST), course, seed, keep_trace=True, t_max=40)
    tr = r.trace
    t_end = tr[-1, 0] + 1.0
    tmp = tempfile.mkdtemp()
    k = 0
    for t in np.arange(0, t_end, every):
        fig, ax = plt.subplots(figsize=(3.6, 5.8), facecolor=SURFACE, dpi=80)
        draw_course(ax, course)
        i = min(np.searchsorted(tr[:, 0], t), len(tr) - 1)
        ax.plot(tr[:i + 1, 1], tr[:i + 1, 2], color=BLUE, lw=1.6, alpha=0.8, zorder=3)
        x, y, psi = tr[i, 1], tr[i, 2], tr[i, 3]
        ax.add_patch(Circle((x, y), R, color=BLUE, alpha=0.9, zorder=4))
        ax.plot([x, x - 0.3 * np.sin(psi)], [y, y + 0.3 * np.cos(psi)], color=INK, lw=1.5, zorder=5)
        done = t >= tr[-1, 0]
        st = LABEL[r.outcome] if done else state_at(r.states, t)
        ax.set_title(f"{min(t, tr[-1, 0]):4.1f} s  ·  {st}", fontsize=9)
        fig.tight_layout()
        fig.savefig(os.path.join(tmp, f"{k:04d}.png"), facecolor=SURFACE)
        plt.close(fig)
        k += 1
    out = os.path.join(ROOT, "media", "run.gif")
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", str(round(1 / every)), "-i",
                    os.path.join(tmp, "%04d.png"), "-vf",
                    "split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer:bayer_scale=5",
                    "-loop", "0", out], check=True)
    shutil.rmtree(tmp)
    print(f"{k} frames -> {out} ({os.path.getsize(out) / 1e6:.1f} MB)")


def main():
    with open(os.path.join(ROOT, "results", "summary.json")) as f:
        summary = json.load(f)
    outcomes(summary)
    robustness(summary)
    paths(example_runs())
    animation()


if __name__ == "__main__":
    main()

"""Run the VO on clean and degraded images, evaluate each run, and save a table + charts.

Usage (from the repo root):
    python run_experiments.py data/euroc/MH_01_easy/mav0
"""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from eval.evaluate import SEGMENTS_M, evaluate, plot  # noqa: E402
from vo.mono_vo import run_vo, write_gt_tum  # noqa: E402

# factor -> (keyword argument for run_vo, levels, the clean value)
FACTORS = {
    "brightness": ("brightness", [0.5, 0.3, 0.15], 1.0),
    "blur_px": ("blur", [5, 9, 15], 0),
    "noise_sigma": ("noise", [10, 20, 30], 0),
    "frame_skip": ("skip", [2, 3, 4], 1),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mav0")
    ap.add_argument("--out", default="results")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(exist_ok=True)

    gt = out / "gt.txt"
    write_gt_tum(args.mav0, gt)

    runs = [("baseline", "none", 0, {})]
    for factor, (kw, levels, _) in FACTORS.items():
        runs += [(f"{factor}_{lv}", factor, lv, {kw: lv}) for lv in levels]

    rows = []
    for name, factor, level, kwargs in runs:
        est = out / f"{name}.txt"
        print(f"[{name}] running VO...")
        stats = run_vo(args.mav0, est, **kwargs)
        row = {"run": name, "factor": factor, "level": level, **stats}
        try:
            metrics, ref, aligned, ape = evaluate(str(gt), str(est))
            row.update(metrics)
            plot(ref, aligned, ape, out / name, title=f"({name})")
        except Exception as e:  # a run can lose tracking so badly that evo can't compare it
            print(f"[{name}] evaluation failed: {e}")
        print(f"[{name}] {row}")
        rows.append(row)

    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(out / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    # Drift vs segment length for the clean run.
    base = rows[0]
    if "drift_10m_pct" in base:
        errs = [base[f"drift_{d}m_pct"] * d / 100 for d in SEGMENTS_M]
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot(SEGMENTS_M, errs, "o-")
        ax.set(xlabel="segment length [m]", ylabel="mean position error [m]", title="Drift grows with distance")
        fig.tight_layout()
        fig.savefig(out / "drift_vs_distance.png", dpi=120)
        plt.close(fig)

    # One chart per stress factor: drift per 10 m and tracking failures vs severity.
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for ax, (factor, (_, levels, clean)) in zip(axes.flat, FACTORS.items()):
        pts = [(clean, base)] + [(r["level"], r) for r in rows if r["factor"] == factor]
        xs = [x for x, r in pts if "drift_10m_pct" in r]
        ys = [r["drift_10m_pct"] for x, r in pts if "drift_10m_pct" in r]
        ax.plot(xs, ys, "o-", label="drift per 10 m [%]")
        ax2 = ax.twinx()
        ax2.bar([x for x, _ in pts], [r["failures"] + r["rejected"] for _, r in pts], alpha=0.25,
                color="red", width=0.03 if factor == "brightness" else 0.6)
        ax2.set_ylabel("lost + rejected steps", color="red")
        ax.set(xlabel=factor, ylabel="drift per 10 m [%]", title=factor)
    fig.tight_layout()
    fig.savefig(out / "stress_tests.png", dpi=120)
    plt.close(fig)
    print(f"\nDone. Table: {out / 'summary.csv'}  Charts: {out}/*.png")


if __name__ == "__main__":
    main()

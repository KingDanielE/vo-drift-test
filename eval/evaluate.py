"""Drift metrics for one estimated trajectory against ground truth (both TUM format).

- ATE: RMSE of position error after best-fit (SE3) alignment of the whole path.
- Drift %: mean translation error over segments of N metres, divided by N (RPE).
- Heading drift: mean rotation error (degrees) per 10 m segment.

Usage:
    python eval/evaluate.py results/gt.txt results/baseline.txt --plot results/baseline
"""

import argparse
import copy

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from evo.core import metrics, sync  # noqa: E402
from evo.tools import file_interface  # noqa: E402

SEGMENTS_M = (2, 5, 10, 20)


def evaluate(gt_path, est_path):
    ref = file_interface.read_tum_trajectory_file(gt_path)
    est = file_interface.read_tum_trajectory_file(est_path)
    ref, est = sync.associate_trajectories(ref, est, max_diff=0.01)

    aligned = copy.deepcopy(est)
    aligned.align(ref, correct_scale=False)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((ref, aligned))

    out = {
        "path_m": round(ref.path_length, 2),
        "ate_rmse_m": round(ape.get_statistic(metrics.StatisticsType.rmse), 3),
    }
    for d in SEGMENTS_M:
        rpe = metrics.RPE(
            metrics.PoseRelation.translation_part,
            delta=d,
            delta_unit=metrics.Unit.meters,
            all_pairs=True,
        )
        rpe.process_data((ref, est))
        out[f"drift_{d}m_pct"] = round(100 * rpe.get_statistic(metrics.StatisticsType.mean) / d, 2)

    rot = metrics.RPE(
        metrics.PoseRelation.rotation_angle_deg,
        delta=10,
        delta_unit=metrics.Unit.meters,
        all_pairs=True,
    )
    rot.process_data((ref, est))
    out["heading_deg_per_10m"] = round(rot.get_statistic(metrics.StatisticsType.mean), 2)
    return out, ref, aligned, ape


def plot(ref, aligned, ape, prefix, title=""):
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    ax[0].plot(ref.positions_xyz[:, 0], ref.positions_xyz[:, 1], "k--", label="ground truth")
    ax[0].plot(aligned.positions_xyz[:, 0], aligned.positions_xyz[:, 1], label="VO")
    ax[0].set(xlabel="x [m]", ylabel="y [m]", title=f"Trajectory (top view) {title}")
    ax[0].axis("equal")
    ax[0].legend()
    ax[1].plot(ref.distances, ape.error)
    ax[1].set(xlabel="distance travelled [m]", ylabel="position error [m]", title="Error along the path")
    fig.tight_layout()
    fig.savefig(f"{prefix}_trajectory.png", dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gt")
    ap.add_argument("est")
    ap.add_argument("--plot", help="save plots with this path prefix")
    args = ap.parse_args()
    out, ref, aligned, ape = evaluate(args.gt, args.est)
    for k, v in out.items():
        print(f"{k:22s} {v}")
    if args.plot:
        plot(ref, aligned, ape, args.plot)


if __name__ == "__main__":
    main()

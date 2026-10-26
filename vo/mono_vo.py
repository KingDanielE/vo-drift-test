"""Monocular visual odometry (KLT tracking + essential matrix) on an EuRoC sequence.

Scale comes from ground truth ("oracle scale"): a single camera cannot measure absolute
scale, so this isolates rotation / direction drift. Writes the trajectory in TUM format:
    timestamp tx ty tz qx qy qz qw

Usage:
    python vo/mono_vo.py data/euroc/MH_01_easy/mav0 results/baseline.txt
    python vo/mono_vo.py data/euroc/MH_01_easy/mav0 results/dark.txt --brightness 0.3
"""

import argparse
import csv
import re
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

MIN_FEATURES = 300  # re-detect when fewer tracked points remain
MAX_FEATURES = 1500
MIN_PARALLAX_PX = 10.0  # wait for this much image motion before a new keyframe
MIN_INLIER_RATIO = 0.3  # reject a motion estimate supported by fewer of the tracked points
MAX_STEP_ROT_DEG = 15.0  # reject implausible rotations between keyframes
MAX_REJECT_STREAK = 5  # after this many rejections in a row, restart from the current frame
MAX_POINT_DEPTH = 1000.0  # in step lengths; OpenCV's default (50) drops distant points

LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)


def load_calib(mav0):
    """Camera matrix K and distortion D from cam0/sensor.yaml."""
    text = (mav0 / "cam0" / "sensor.yaml").read_text()

    def arr(name):
        m = re.search(name + r":\s*\[([^\]]*)\]", text)
        return np.array([float(v) for v in m.group(1).split(",")])

    fu, fv, cu, cv_ = arr("intrinsics")
    K = np.array([[fu, 0, cu], [0, fv, cv_], [0, 0, 1]])
    return K, arr("distortion_coefficients")


def load_body_from_cam(mav0):
    """4x4 transform from camera to body (IMU) frame, T_BS in cam0/sensor.yaml.

    EuRoC ground truth is the body's pose, so camera poses are converted to body poses
    before comparing. Identity when the file has no T_BS (e.g. synthetic data).
    """
    text = (mav0 / "cam0" / "sensor.yaml").read_text()
    m = re.search(r"T_BS:.*?data:\s*\[([^\]]*)\]", text, re.S)
    if not m:
        return np.eye(4)
    return np.array([float(v) for v in m.group(1).split(",")]).reshape(4, 4)


def load_frames(mav0):
    """List of (time_s, image_path) for cam0."""
    frames = []
    with open(mav0 / "cam0" / "data.csv") as f:
        for row in csv.reader(f):
            if row[0].startswith("#"):
                continue
            frames.append((int(row[0]) * 1e-9, mav0 / "cam0" / "data" / row[1].strip()))
    return frames


def load_gt(mav0):
    """Ground truth: times (s), positions (N x 3), quaternions (N x 4, w x y z)."""
    d = np.loadtxt(
        mav0 / "state_groundtruth_estimate0" / "data.csv", delimiter=",", comments="#"
    )
    return d[:, 0] * 1e-9, d[:, 1:4], d[:, 4:8]


def write_gt_tum(mav0, out_path):
    """Convert EuRoC ground truth to TUM format so evo can compare it."""
    t, xyz, q = load_gt(Path(mav0))
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        for ti, p, (qw, qx, qy, qz) in zip(t, xyz, q):
            f.write(f"{ti:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")


def degrade(img, brightness=1.0, blur=0, noise=0.0, rng=None):
    """Simulate bad conditions: darker image, blur (kernel px), Gaussian noise (sigma)."""
    if brightness != 1.0:
        img = cv2.convertScaleAbs(img, alpha=brightness, beta=0)
    if blur > 0:
        k = blur | 1  # kernel size must be odd
        img = cv2.GaussianBlur(img, (k, k), 0)
    if noise > 0:
        img = np.clip(img + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img


def detect(img):
    pts = cv2.goodFeaturesToTrack(img, MAX_FEATURES, 0.01, 7)
    if pts is None:
        return np.empty((0, 2), np.float32)
    return pts.reshape(-1, 2)


def run_vo(mav0, out_path, brightness=1.0, blur=0, noise=0.0, skip=1, max_frames=None):
    mav0 = Path(mav0)
    K, D = load_calib(mav0)
    gt_t, gt_xyz, _ = load_gt(mav0)
    frames = load_frames(mav0)[::skip]
    frames = [f for f in frames if gt_t[0] <= f[0] <= gt_t[-1]]
    if max_frames:
        frames = frames[:max_frames]

    rng = np.random.default_rng(0)
    h, w = cv2.imread(str(frames[0][1]), cv2.IMREAD_GRAYSCALE).shape
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, (w, h), cv2.CV_32FC1)

    def load(path):
        img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        img = degrade(img, brightness, blur, noise, rng)
        return cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

    def gt_pos(t):
        return np.array([np.interp(t, gt_t, gt_xyz[:, i]) for i in range(3)])

    R, t = np.eye(3), np.zeros(3)  # current camera pose in the world (first camera) frame
    ref_time = frames[0][0]
    ref_img = load(frames[0][1])
    ref_pts = detect(ref_img)
    poses = [(ref_time, t.copy(), R.copy())]
    failures = 0
    rejected = 0
    streak = 0  # rejections in a row

    for time, path in frames[1:]:
        img = load(path)

        if len(ref_pts) >= 8:
            cur_pts, st, _ = cv2.calcOpticalFlowPyrLK(ref_img, img, ref_pts, None, **LK_PARAMS)
            ok = st.reshape(-1).astype(bool)
            p_ref, p_cur = ref_pts[ok], cur_pts.reshape(-1, 2)[ok]
        else:
            p_ref = p_cur = np.empty((0, 2), np.float32)

        E = None
        if len(p_cur) >= 8:
            if np.median(np.linalg.norm(p_cur - p_ref, axis=1)) < MIN_PARALLAX_PX:
                poses.append((time, t.copy(), R.copy()))  # barely moved: keep the keyframe
                continue
            E, mask = cv2.findEssentialMat(p_cur, p_ref, K, cv2.RANSAC, 0.999, 1.0)

        if E is None or E.shape != (3, 3):
            # Tracking lost: hold the pose and start again from this frame.
            failures += 1
            ref_img, ref_time, ref_pts = img, time, detect(img)
            poses.append((time, t.copy(), R.copy()))
            continue

        # R_rel, t_rel = pose of the current camera in the reference camera's frame.
        n_inliers, R_rel, t_rel, mask, _ = cv2.recoverPose(
            E, p_cur, p_ref, K, distanceThresh=MAX_POINT_DEPTH, mask=mask
        )
        angle = np.degrees(np.linalg.norm(Rotation.from_matrix(R_rel).as_rotvec()))
        if n_inliers < MIN_INLIER_RATIO * len(p_cur) or angle > MAX_STEP_ROT_DEG:
            # Unreliable estimate: keep the reference frame and try again with the next image,
            # unless that keeps failing, then start over from this frame.
            rejected += 1
            streak += 1
            if streak >= MAX_REJECT_STREAK:
                failures += 1
                streak = 0
                ref_img, ref_time, ref_pts = img, time, detect(img)
            poses.append((time, t.copy(), R.copy()))
            continue
        streak = 0
        scale = np.linalg.norm(gt_pos(time) - gt_pos(ref_time))
        t = t + scale * (R @ t_rel.ravel())
        R = R @ R_rel

        ref_img, ref_time = img, time
        ref_pts = p_cur[mask.ravel() > 0]
        if len(ref_pts) < MIN_FEATURES:
            ref_pts = detect(img)
        poses.append((time, t.copy(), R.copy()))

    # Camera poses -> body poses (world = first camera frame): T_w_b = T_w_c * inv(T_b_c).
    T_cb = np.linalg.inv(load_body_from_cam(mav0))
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        for time, p_c, R_c in poses:
            Rm = R_c @ T_cb[:3, :3]
            p = p_c + R_c @ T_cb[:3, 3]
            qx, qy, qz, qw = Rotation.from_matrix(Rm).as_quat()
            f.write(f"{time:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
    return {"frames": len(frames), "failures": failures, "rejected": rejected}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("mav0", help="path to the sequence's mav0 folder")
    ap.add_argument("out", help="output trajectory (TUM format)")
    ap.add_argument("--brightness", type=float, default=1.0)
    ap.add_argument("--blur", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--skip", type=int, default=1, help="use every Nth frame")
    ap.add_argument("--max-frames", type=int)
    ap.add_argument("--gt-out", help="also write the ground truth in TUM format here")
    args = ap.parse_args()

    if args.gt_out:
        write_gt_tum(args.mav0, args.gt_out)
    stats = run_vo(
        args.mav0, args.out, args.brightness, args.blur, args.noise, args.skip, args.max_frames
    )
    print(
        f"{stats['frames']} frames, {stats['failures']} tracking failures, "
        f"{stats['rejected']} rejected estimates -> {args.out}"
    )


if __name__ == "__main__":
    main()

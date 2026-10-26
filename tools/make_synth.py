"""Synthetic EuRoC-format sequence: a downward-looking camera flying a 24 m arc at ~5 m height.

The ground is a random texture with raised square blocks (0.8 to 2.4 m high), so the scene
has depth. --flat makes the ground perfectly flat, which shows the planar-scene ambiguity
of the essential matrix (a real risk for a drone looking straight down at flat terrain).

Usage:
    python tools/make_synth.py data/synth            # -> data/synth/mav0
    python tools/make_synth.py data/synth_flat --flat
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

W, H_IMG = 752, 480  # EuRoC image size
K = np.array([[458.654, 0, 367.215], [0, 457.296, 248.375], [0, 0, 1]])
N, RES = 4000, 100.0  # texture size (px) and pixels per metre: 40 m x 40 m of ground
CELL = 300  # block size in texture px (3 m)
FPS, SECONDS = 20, 20


def pose(t):
    """Camera position and rotation (world from camera) at time t: an arc, camera facing down."""
    th = 0.15 * t
    p = np.array([8 * np.cos(th) - 8, 8 * np.sin(th), 5 + 0.3 * np.sin(t)])
    yaw = th + np.pi / 2
    x = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    z = np.array([0, 0, -1.0])
    return p, np.column_stack([x, np.cross(z, x), z])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--flat", action="store_true", help="no raised blocks")
    args = ap.parse_args()

    out = Path(args.out) / "mav0"
    (out / "cam0" / "data").mkdir(parents=True, exist_ok=True)
    (out / "state_groundtruth_estimate0").mkdir(parents=True, exist_ok=True)
    (out / "cam0" / "sensor.yaml").write_text(
        "%YAML:1.0\nintrinsics: [458.654, 457.296, 367.215, 248.375]\n"
        "distortion_coefficients: [0.0, 0.0, 0.0, 0.0]\n"
    )

    rng = np.random.default_rng(1)
    tex = np.zeros((N, N), np.float32)
    for s in (3, 9, 27):  # mix of fine and coarse detail
        tex += cv2.GaussianBlur(rng.random((N, N)).astype(np.float32), (0, 0), s) * s
    tex = cv2.equalizeHist(cv2.normalize(tex, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8))

    levels = [0.0] if args.flat else [0.0, 0.8, 1.6, 2.4]
    cells = rng.choice(levels, size=(N // CELL + 1, N // CELL + 1))
    height = np.kron(cells, np.ones((CELL, CELL)))[:N, :N]
    tex_to_ground = np.array([[1 / RES, 0, -N / 2 / RES], [0, 1 / RES, -N / 2 / RES], [0, 0, 1]])

    with open(out / "cam0" / "data.csv", "w") as f:
        f.write("#timestamp [ns],filename\n")
        for i in range(FPS * SECONDS):
            t = i / FPS
            ns = int(1e9 + t * 1e9)
            p, Rwc = pose(t)
            Rcw, tcw = Rwc.T, -Rwc.T @ p
            img = np.zeros((H_IMG, W), np.uint8)
            for h in levels:  # ground first, then block tops (higher ones hide what's below)
                Hm = K @ np.column_stack([Rcw[:, 0], Rcw[:, 1], Rcw[:, 2] * h + tcw]) @ tex_to_ground
                layer = cv2.warpPerspective(tex, Hm, (W, H_IMG))
                if h == 0:
                    img = layer
                    continue
                mask = cv2.warpPerspective(
                    (height == h).astype(np.uint8), Hm, (W, H_IMG), flags=cv2.INTER_NEAREST
                )
                img[mask > 0] = layer[mask > 0]
            cv2.imwrite(str(out / "cam0" / "data" / f"{ns}.png"), img)
            f.write(f"{ns},{ns}.png\n")

    with open(out / "state_groundtruth_estimate0" / "data.csv", "w") as f:
        f.write("#timestamp,px,py,pz,qw,qx,qy,qz\n")
        for i in range(int((SECONDS + 0.5) * 200)):  # 200 Hz, like EuRoC
            t = i / 200 - 0.1
            p, Rwc = pose(t)
            qx, qy, qz, qw = Rotation.from_matrix(Rwc).as_quat()
            f.write(f"{int(1e9 + t * 1e9)},{p[0]},{p[1]},{p[2]},{qw},{qx},{qy},{qz}\n")
    print(f"wrote {FPS * SECONDS} frames to {out}")


if __name__ == "__main__":
    main()

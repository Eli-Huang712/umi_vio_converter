#!/usr/bin/env python3
"""prof_compare_poses.py — trajectory-level ATE/RPE between two poses.npy files.

Used to validate P1 direct-feed against stock under the handoff's TRAJECTORY-LEVEL
tolerance (not byte-identity). Both inputs are ``{timestamp_ns: 4x4}`` dicts (the
format build_map writes). We match keyframes by exact timestamp (both runs derive
stamps from the same source frames, so common keyframes share stamps), then report:

  * keyframe counts + common-timestamp count
  * ATE: Umeyama-aligned (sim3, no scale) RMSE of translation over common stamps
  * RPE: RMSE of relative-pose translation error over consecutive common stamps

Run stock 3x to get the self-jitter band, then check P1 falls inside it.
"""
from __future__ import annotations

import argparse
import numpy as np


def load_poses(path):
    d = np.load(path, allow_pickle=True).item()
    return {int(k): np.asarray(v, dtype=np.float64) for k, v in d.items()}


def align_umeyama(src, dst):
    """Rigid (rotation+translation, no scale) alignment src->dst, least squares.

    src, dst: (N,3). Returns (R, t) minimizing ||dst - (R@src + t)||.
    """
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src_c = src - mu_s
    dst_c = dst - mu_d
    H = src_c.T @ dst_c
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = mu_d - R @ mu_s
    return R, t


def ate(poses_a, poses_b, common):
    src = np.array([poses_a[t][:3, 3] for t in common])
    dst = np.array([poses_b[t][:3, 3] for t in common])
    R, t = align_umeyama(src, dst)
    aligned = (R @ src.T).T + t
    err = np.linalg.norm(aligned - dst, axis=1)
    return float(np.sqrt((err ** 2).mean())), float(err.max())


def rpe(poses_a, poses_b, common):
    """Relative pose error over consecutive common keyframes (translation part)."""
    errs = []
    for i in range(len(common) - 1):
        t0, t1 = common[i], common[i + 1]
        rel_a = np.linalg.inv(poses_a[t0]) @ poses_a[t1]
        rel_b = np.linalg.inv(poses_b[t0]) @ poses_b[t1]
        delta = np.linalg.inv(rel_a) @ rel_b
        errs.append(np.linalg.norm(delta[:3, 3]))
    if not errs:
        return 0.0, 0.0
    errs = np.array(errs)
    return float(np.sqrt((errs ** 2).mean())), float(errs.max())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("poses_a")
    ap.add_argument("poses_b")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    a = load_poses(args.poses_a)
    b = load_poses(args.poses_b)
    common = sorted(set(a) & set(b))
    tag = f"[{args.label}] " if args.label else ""
    print(f"{tag}kf_a={len(a)} kf_b={len(b)} common={len(common)} "
          f"(a_only={len(set(a)-set(b))} b_only={len(set(b)-set(a))})")
    if len(common) < 3:
        print(f"{tag}TOO_FEW_COMMON: cannot compute ATE/RPE")
        return
    ate_rmse, ate_max = ate(a, b, common)
    rpe_rmse, rpe_max = rpe(a, b, common)
    print(f"{tag}ATE_rmse_m={ate_rmse:.6f} ATE_max_m={ate_max:.6f} "
          f"RPE_rmse_m={rpe_rmse:.6f} RPE_max_m={rpe_max:.6f}")


if __name__ == "__main__":
    main()

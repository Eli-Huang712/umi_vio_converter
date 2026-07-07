#!/usr/bin/env python3
"""umi_vio_converter — single-episode UMI VIO converter with tactile + toggles.

Per raw episode, auto-dispatches one of three actions (``--mode auto``):

  * FULL     : no ``*_with_pose.mcap`` yet -> run build_map (VIO) for the enabled
               pose sensors, then merge into a with_pose bag WITH tactile.
  * BACKFILL : with_pose exists but has no tactile -> insert tactile from the raw
               bag only (no VIO rerun, pure CPU).
  * SKIP     : with_pose already has tactile -> nothing to do (idempotent).

Pose conversion is per-sensor toggleable via ``--pose-sensors`` (subset of
left_wrist,right_wrist,head). ``head`` is accepted but gated behind
``--allow-experimental-head`` (its VIO path is unvalidated; default off).

Runs inside the tinynav devcontainer. Use the ``umi_vio_converter.sh`` wrapper so
ROS overlays are sourced and python-mcap is present before this runs.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path


_TACT_RE = re.compile(r"^/observation/tactile_\w+/tactile/tactile_point_cloud2$")
SUPPORTED_SENSORS = ("left_wrist", "right_wrist", "head")
EXPERIMENTAL_SENSORS = ("head",)

SELF_DIR = Path(__file__).resolve().parent          # /tinynav/tool/umi
REPO_DIR = SELF_DIR.parent.parent                   # /tinynav
ONE_SENSOR_SH = SELF_DIR / "_build_map_one_sensor.sh"
MERGE_PY = SELF_DIR / "merge_sqlite_mcap.py"
BACKFILL_PY = SELF_DIR / "backfill_tactile.py"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input_mcap", type=Path, help="Raw UMI .mcap for one episode.")
    p.add_argument("--pose-sensors", default="left_wrist,right_wrist",
                   help="Comma list of pose sensors to VIO+merge (subset of "
                        "left_wrist,right_wrist,head; empty = convert only).")
    p.add_argument("--mode", choices=("auto", "full", "backfill", "check"), default="auto",
                   help="auto (default): decide FULL/BACKFILL/SKIP per episode state.")
    p.add_argument("--play-rate", default=None, help="build_map play rate (default env or 20).")
    p.add_argument("--ros-domain-id", default=None, help="ROS_DOMAIN_ID for isolated build_map.")
    p.add_argument("--output", type=Path, help="Override output with_pose path.")
    p.add_argument("--no-tcp-transform", dest="tcp_transform", action="store_false",
                   help="Emit raw camera poses (world_T_camera) instead of TCP poses.")
    p.add_argument("--force", action="store_true", help="Force FULL even if with_pose exists (overwrites VIO!).")
    p.add_argument("--allow-experimental-head", action="store_true",
                   help="Permit head as a VIO pose sensor (unvalidated; default refused).")
    p.add_argument("--build-parallel", dest="build_parallel", action="store_true", default=True,
                   help="Run the per-sensor build_maps concurrently (default). Each gets its own "
                        "ROS domain (base+i); space per-episode base domains by >= #sensors.")
    p.add_argument("--build-sequential", dest="build_parallel", action="store_false",
                   help="Run the per-sensor build_maps one at a time (the original behaviour).")
    p.add_argument("--wrist-gpus", default=None,
                   help="Comma list of GPU ids to pin concurrent sensors to (standalone use), "
                        "e.g. '0,1'. Default: inherit CUDA_VISIBLE_DEVICES (sensors share it).")
    p.set_defaults(tcp_transform=True)
    return p.parse_args()


def parse_pose_sensors(value: str):
    sensors = [s.strip() for s in value.split(",")] if value else []
    sensors = [s for s in sensors if s]
    for s in sensors:
        if s not in SUPPORTED_SENSORS:
            raise SystemExit(f"Unknown pose sensor {s!r}; supported: {', '.join(SUPPORTED_SENSORS)}")
    return sensors


def default_work_dir(input_mcap: Path) -> Path:
    return input_mcap.with_suffix("")


def default_output_mcap(input_mcap: Path) -> Path:
    return default_work_dir(input_mcap) / f"{input_mcap.stem}_with_pose.mcap"


def wp_exists(wp: Path) -> bool:
    return (wp / "metadata.yaml").exists()


def wp_has_tactile(wp: Path) -> bool:
    """True if the with_pose bag already contains a tactile channel.

    Reads the bag's inner MCAP file(s) directly (python-mcap), so it needs no ROS.
    """
    from mcap.reader import make_reader

    for mcap_file in sorted(wp.glob("*.mcap")):
        with open(mcap_file, "rb") as f:
            summary = make_reader(f).get_summary()
            channels = summary.channels.values() if summary is not None else []
            if any(_TACT_RE.match(c.topic) for c in channels):
                return True
    return False


def decide(mode: str, wp: Path, force: bool) -> str:
    if force or mode == "full":
        return "FULL"
    if mode == "backfill":
        if not wp_exists(wp):
            raise SystemExit(f"--mode backfill but no with_pose exists: {wp}")
        return "SKIP" if wp_has_tactile(wp) else "BACKFILL"
    # auto / check
    if not wp_exists(wp):
        return "FULL"
    return "SKIP" if wp_has_tactile(wp) else "BACKFILL"


def run(cmd, **kw):
    print("+ " + " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run([str(c) for c in cmd], check=True, **kw)


def do_full(args, wp: Path, pose_sensors):
    experimental = [s for s in pose_sensors if s in EXPERIMENTAL_SENSORS]
    if experimental and not args.allow_experimental_head:
        raise SystemExit(
            f"pose sensor(s) {experimental} are experimental (unvalidated VIO) and refused by default; "
            f"pass --allow-experimental-head to attempt (also needs the build_map head guard lifted)."
        )

    # _build_map_one_sensor.sh: <mcap> <sensor> [ros_domain_id] [play_rate].
    # Sequential (default): each sensor reuses the same domain, run one at a time.
    # Parallel: sensors run concurrently, so each needs its OWN ROS domain (else
    # their /clock + /camera/... topics collide). Sensor i -> base_domain + i.
    #   Caller must space per-episode base domains by >= len(pose_sensors) to
    #   avoid cross-episode collisions (e.g. batch stride 2 for the two wrists).
    #   Domains must stay <= 232 (FastDDS cap): base + max_slot*stride + i <= 232.
    base_domain = args.ros_domain_id
    if args.build_parallel and base_domain is None:
        base_domain = "90"  # need a base to derive distinct per-sensor domains

    def wrist_cmd(i, sensor):
        domain = str(int(base_domain) + i) if base_domain is not None else ""
        cmd = ["bash", ONE_SENSOR_SH, args.input_mcap, sensor, domain]
        if args.play_rate is not None:
            cmd += [args.play_rate]
        return cmd

    def wrist_env(i):
        # Optionally pin each concurrent sensor to its own GPU (standalone use);
        # under the batch driver CUDA_VISIBLE_DEVICES is already one GPU and both
        # sensors share it, which is fine.
        if not args.wrist_gpus:
            return None
        gpus = [g.strip() for g in args.wrist_gpus.split(",") if g.strip()]
        env = dict(os.environ)
        env["CUDA_VISIBLE_DEVICES"] = gpus[i % len(gpus)]
        return env

    if args.build_parallel and len(pose_sensors) > 1:
        procs = []
        for i, sensor in enumerate(pose_sensors):
            cmd = wrist_cmd(i, sensor)
            print("+ (parallel) " + " ".join(str(c) for c in cmd), flush=True)
            procs.append((sensor, subprocess.Popen(
                [str(c) for c in cmd], cwd=REPO_DIR, env=wrist_env(i))))
        failed = [s for s, p in procs if p.wait() != 0]
        if failed:
            raise SystemExit(f"build_map failed for: {', '.join(failed)}")
    else:
        for i, sensor in enumerate(pose_sensors):
            run(wrist_cmd(i, sensor), cwd=REPO_DIR, env=wrist_env(i))

    merge_cmd = [sys.executable, MERGE_PY, "--input_mcap", args.input_mcap,
                 "--pose-sensors", ",".join(pose_sensors)]
    if args.output is not None:
        merge_cmd += ["--output_mcap", args.output]
    if not args.tcp_transform:
        merge_cmd += ["--no-tcp-transform"]
    run(merge_cmd, cwd=REPO_DIR)


def do_backfill(args, wp: Path):
    cmd = [sys.executable, BACKFILL_PY, "--with-pose", wp, "--raw", args.input_mcap]
    run(cmd, cwd=REPO_DIR)


def main():
    args = parse_args()
    if not args.input_mcap.is_file() or args.input_mcap.suffix != ".mcap":
        raise SystemExit(f"input must be an existing .mcap file: {args.input_mcap}")

    pose_sensors = parse_pose_sensors(args.pose_sensors)
    wp = args.output if args.output is not None else default_output_mcap(args.input_mcap)

    action = decide(args.mode, wp, args.force)
    print(f"episode={args.input_mcap}")
    print(f"with_pose={wp} exists={wp_exists(wp)}")
    print(f"pose_sensors={','.join(pose_sensors) if pose_sensors else '(none)'}")
    print(f"action={action}")

    if args.mode == "check":
        return
    if action == "SKIP":
        return
    if action == "FULL":
        do_full(args, wp, pose_sensors)
    elif action == "BACKFILL":
        do_backfill(args, wp)
    print(f"done action={action} output={wp}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""persistent_worker.py — persistent VIO worker for high-throughput FULL conversion.

Amortizes the ~20s/build_map fixed overhead (TensorRT model load, process spawn,
CUDA/ROS init) by keeping two long-lived processes alive across episodes and
building a FRESH node per job (clean SLAM state). Models are loaded once per
process via a TRT-engine cache; the exact 2-process ROS concurrency profile of
the original pipeline is preserved, so poses are unchanged (verified by the
bit-identical equivalence gate — VIO is run-to-run deterministic).

Roles (selected by --role), all in this one file:
  * perception : long-lived; per job builds a fresh PerceptionNode and spins it
                 until told STOP. Stdin line protocol: RUN / STOP / QUIT.
  * buildmap   : long-lived; per job builds fresh BagPlayer+BuildMapNode, plays
                 the bag, saves poses, tears down. Stdin: RUN <bag> <mapdir>
                 <sensor> <play_rate> / QUIT.
  * (default)  : orchestrator; launches one perception + one buildmap child on a
                 chosen (ROS_DOMAIN_ID, GPU), feeds it a list of episodes (both
                 wrists each), then merges tactile. Launch several orchestrators
                 on distinct domains/GPUs for parallelism.

No edits to tinynav perception_node.py / build_map_node.py / models_trt.py.
"""
from __future__ import annotations

import argparse
import os
import select
import subprocess
import sys
import time
from pathlib import Path

SELF = Path(__file__).resolve()
SELF_DIR = SELF.parent
REPO_DIR = SELF_DIR.parent.parent            # /tinynav
MERGE_PY = SELF_DIR / "merge_sqlite_mcap.py"

WRISTS = ("left_wrist", "right_wrist")


# ---------------------------------------------------------------------------
# Model-engine cache: deserialize each .plan once per process. All model
# wrappers subclass TRTBase and load in TRTBase.__init__(engine_path). We cache
# ONLY the read-only, deserialized `engine` (the expensive ~5s part). Every
# wrapper instance still creates its OWN execution context + device buffers +
# CUDA graph — exactly like the baseline — so no MUTABLE TRT state is shared
# across jobs. Sharing context/buffers/graph (an earlier attempt) leaked state
# between jobs and corrupted poses; sharing only the stateless engine does not.
# ---------------------------------------------------------------------------
def install_engine_cache():
    import tinynav.core.models_trt as mt

    trt = mt.trt
    engine_cache = {}

    def cached_init(self, engine_path):
        eng = engine_cache.get(engine_path)
        if eng is None:
            logger = trt.Logger(trt.Logger.WARNING)
            with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
                eng = runtime.deserialize_cuda_engine(f.read())
            engine_cache[engine_path] = eng
        # Read-only engine reused; fresh mutable state per instance (== baseline).
        self.engine = eng
        self.context = self.engine.create_execution_context()
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers()
        self.graph_exec = self.capture_graph()

    mt.TRTBase.__init__ = cached_init
    return engine_cache


def clear_tinynav_caches():
    """Clear the process-level (a)lru_cache_numpy memoizations that persist across
    jobs. These are decorated at class/module definition, so a persistent process
    carries them between jobs; a fresh-process baseline fills them from empty per
    episode. Leaving them populated changes eviction/hits and hence perception's
    keyframe decisions (the cross-job pose drift). We clear the EXACT known set
    (not by getattr-discovery: iterating a ctypes-bearing module and getattr'ing
    'cache_clear' makes ctypes.LibraryLoader try to dlopen a lib named
    'cache_clear' and crash).
    """
    import importlib
    targets = [
        ("tinynav.core.models_trt", "SuperPointTRT", "infer"),
        ("tinynav.core.models_trt", "LightGlueTRT", "infer"),
        ("tinynav.core.math_utils", None, "estimate_pose"),
    ]
    cleared = 0
    for modname, clsname, funcname in targets:
        try:
            mod = importlib.import_module(modname)
            holder = getattr(mod, clsname) if clsname else mod
            fn = getattr(holder, funcname)
            cc = getattr(fn, "cache_clear", None)
            if callable(cc):
                cc()
                cleared += 1
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[cache_clear] {modname}.{clsname}.{funcname}: {exc}\n")
    return cleared


def _emit(tag):
    """Emit a control token on the dedicated control fd, NOT stdout.

    perception_node / build_map_node log copiously to stdout/stderr; if the
    protocol shared stdout, that log noise would corrupt line-based token reads.
    The orchestrator passes the control write-fd number via env UVC_CTRL_FD and
    the child writes tokens straight to it. Falls back to stdout if unset (e.g.
    running a child by hand for debugging).
    """
    line = tag + "\n"
    fd = os.environ.get("UVC_CTRL_FD")
    if fd is not None:
        try:
            os.write(int(fd), line.encode())
            return
        except OSError:
            pass
    sys.stdout.write(line)
    sys.stdout.flush()


def _next_cmd():
    line = sys.stdin.readline()
    if not line:
        return None
    return line.rstrip("\n")


# ---------------------------------------------------------------------------
# perception child
# ---------------------------------------------------------------------------
def role_perception():
    install_engine_cache()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    import tinynav.core.perception_node as pn
    from tinynav.core.perception_node import PerceptionNode

    # Pre-load models into the engine cache ONCE, before any job. Model wrappers
    # are pure TRT/CUDA objects (no rclpy), so they persist across the per-job
    # rclpy.init()/shutdown() cycles below. Re-initialising rclpy per job gives
    # every job a FRESH DDS participant, so no retained/queued messages leak from
    # one job into the next (the cross-job pose drift the shared-participant
    # design suffered). CUDA context + models stay warm; only ROS discovery is
    # re-incurred per job.
    for cls in (pn.SuperPointTRT, pn.LightGlueTRT, pn.StereoEngineTRT):
        try:
            cls()
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[perception] preload {cls.__name__} failed: {exc}\n")
    _emit("BOOT")

    quit_after = False
    while not quit_after:
        cmd = _next_cmd()
        if cmd is None or cmd == "QUIT":
            break
        if cmd != "RUN":
            continue
        rclpy.init()
        node = PerceptionNode(verbose_timer=False)
        ex = MultiThreadedExecutor()
        ex.add_node(node)
        _emit("READY")  # subscriptions created; buildmap barrier can pass
        stop = False
        while not stop and rclpy.ok():
            ex.spin_once(timeout_sec=0.02)
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                c2 = _next_cmd()
                if c2 is None or c2 in ("STOP", "QUIT"):
                    stop = True
                    quit_after = c2 is None or c2 == "QUIT"
        ex.remove_node(node)
        node.destroy_node()
        del node, ex
        rclpy.shutdown()  # tear down participant -> clean DDS state next job
        clear_tinynav_caches()  # clear process-level memoizations -> clean next job
        _emit("STOPPED")


# ---------------------------------------------------------------------------
# buildmap child
# ---------------------------------------------------------------------------
def role_buildmap():
    install_engine_cache()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from tinynav.core.build_map_node import BagPlayer, BuildMapNode, ImageTransportsNode
    import tinynav.core.build_map_node as bm

    # Pre-load build_map's models into the engine cache once, before jobs (see
    # role_perception for why rclpy is re-initialised per job).
    for cls in (bm.SuperPointTRT, bm.LightGlueTRT, bm.Dinov2TRT):
        try:
            cls()
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[buildmap] preload {cls.__name__} failed: {exc}\n")
    _emit("BOOT")

    while True:
        cmd = _next_cmd()
        if cmd is None or cmd == "QUIT":
            break
        if not cmd.startswith("RUN\t"):
            continue
        _, bag, mapdir, sensor, play_rate = cmd.split("\t")
        status = 0
        rclpy.init()
        try:
            ex = SingleThreadedExecutor()
            player = BagPlayer(bag, play_rate=float(play_rate), sensor=sensor)
            mapper = BuildMapNode(mapdir, verbose_timer=False, global_frames_ratio=1.1)
            itn = ImageTransportsNode()
            for n in (player, mapper, itn):
                ex.add_node(n)
            player.wait_for_perception_subscribers(ex)
            while rclpy.ok() and player.play_next():
                ex.spin_once(timeout_sec=0.001)
            player._publish_percent(100.0)
            mapper.save_mapping()
            for n in (player, mapper, itn):
                ex.remove_node(n)
                n.destroy_node()
            del player, mapper, itn, ex
        except Exception as exc:  # noqa: BLE001 — report, keep the worker alive
            status = 1
            sys.stderr.write(f"[buildmap] job failed: {exc}\n")
            sys.stderr.flush()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            clear_tinynav_caches()  # clear process-level memoizations -> clean next job
        _emit(f"DONE {status}")


# ---------------------------------------------------------------------------
# orchestrator
# ---------------------------------------------------------------------------
class Child:
    """A persistent child process with a line protocol.

    Commands go to the child's stdin; control tokens come back on a dedicated
    control pipe wired to the child's fd 3 (so ROS/Python log noise on the
    child's stdout/stderr can never corrupt the protocol). The child's stdout +
    stderr are inherited by the parent's stderr for logging.
    """

    def __init__(self, role, env, log_file=None):
        self.role = role
        ctrl_r, ctrl_w = os.pipe()  # child writes ctrl_w; parent reads ctrl_r
        os.set_inheritable(ctrl_w, True)
        log_dst = open(log_file, "ab") if log_file else sys.stderr
        child_env = dict(env)
        child_env["UVC_CTRL_FD"] = str(ctrl_w)  # child writes tokens to this exact fd
        self.p = subprocess.Popen(
            [sys.executable, str(SELF), "--role", role],
            cwd=str(REPO_DIR), env=child_env,
            stdin=subprocess.PIPE,
            stdout=log_dst, stderr=log_dst,
            pass_fds=(ctrl_w,),
            text=True, bufsize=1,
        )
        os.close(ctrl_w)  # parent keeps only the read end
        self._ctrl = os.fdopen(ctrl_r, "r")
        if log_file:
            log_dst.close()

    def send(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()

    def wait_for(self, token, timeout):
        """Read control-pipe lines until one starts with token; return it. None on timeout/EOF."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            r, _, _ = select.select([self._ctrl], [], [], min(1.0, max(0.0, deadline - time.monotonic())))
            if not r:
                if self.p.poll() is not None:
                    return None
                continue
            line = self._ctrl.readline()
            if not line:
                return None
            line = line.rstrip("\n")
            if line.startswith(token):
                return line
        return None

    def close(self):
        try:
            self.send("QUIT")
        except Exception:
            pass
        try:
            self.p.wait(timeout=20)
        except Exception:
            self.p.kill()
        try:
            self._ctrl.close()
        except Exception:
            pass


def default_work_dir(mcap: Path) -> Path:
    return mcap.with_suffix("")


def default_output_mcap(mcap: Path) -> Path:
    return default_work_dir(mcap) / f"{mcap.stem}_with_pose.mcap"


def run_merge(mcap: Path, pose_sensors, tcp_transform, output):
    cmd = [sys.executable, str(MERGE_PY), "--input_mcap", str(mcap),
           "--pose-sensors", ",".join(pose_sensors)]
    if output is not None:
        cmd += ["--output_mcap", str(output)]
    if not tcp_transform:
        cmd += ["--no-tcp-transform"]
    print("+ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=str(REPO_DIR), check=True)


def orchestrate(args):
    episodes = [Path(e) for e in args.episodes]
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    if args.gpu is not None:
        env["CUDA_VISIBLE_DEVICES"] = str(args.gpu)
    env.setdefault("PYTHONPATH", str(REPO_DIR))

    print(f"[orch] launching persistent pair on domain={args.ros_domain_id} gpu={args.gpu}", flush=True)
    logdir = args.log_dir
    perc_log = str(Path(logdir) / f"perception_d{args.ros_domain_id}.log") if logdir else None
    bm_log = str(Path(logdir) / f"buildmap_d{args.ros_domain_id}.log") if logdir else None
    if logdir:
        Path(logdir).mkdir(parents=True, exist_ok=True)
    perc = Child("perception", env, log_file=perc_log)
    bm = Child("buildmap", env, log_file=bm_log)
    # Wait for both to load models.
    if perc.wait_for("BOOT", args.boot_timeout) is None:
        raise SystemExit("perception child failed to boot")
    if bm.wait_for("BOOT", args.boot_timeout) is None:
        raise SystemExit("buildmap child failed to boot")
    print("[orch] pair booted (models loaded)", flush=True)

    ok = fail = 0
    try:
        for ep in episodes:
            work = default_work_dir(ep)
            done_sensors = []
            for sensor in WRISTS:
                mapdir = work / f"{sensor}_db"
                mapdir.parent.mkdir(parents=True, exist_ok=True)
                t0 = time.monotonic()
                # protocol: perception RUN -> READY ; buildmap RUN -> DONE ; perception STOP -> STOPPED
                perc.send("RUN")
                if perc.wait_for("READY", args.job_timeout) is None:
                    print(f"[orch] FAIL {ep.parent.name}/{sensor}: perception no READY", flush=True)
                    fail += 1
                    break
                bm.send(f"RUN\t{ep}\t{mapdir}\t{sensor}\t{args.play_rate}")
                done = bm.wait_for("DONE", args.job_timeout)
                perc.send("STOP")
                perc.wait_for("STOPPED", args.job_timeout)
                if done is None or done.split()[1] != "0":
                    print(f"[orch] FAIL {ep.parent.name}/{sensor}: buildmap {done}", flush=True)
                    fail += 1
                    break
                dt = time.monotonic() - t0
                print(f"[orch] ok {ep.parent.name}/{sensor} {dt:.1f}s", flush=True)
                done_sensors.append(sensor)
            if len(done_sensors) == len(WRISTS):
                run_merge(ep, WRISTS, args.tcp_transform, args.output)
                ok += 1
                print(f"[orch] DONE episode {ep.parent.name}", flush=True)
    finally:
        perc.close()
        bm.close()
    print(f"[orch] finished ok={ok} fail={fail}", flush=True)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--role", choices=("perception", "buildmap"), default=None,
                   help="Internal: run as a persistent child. Omit to run the orchestrator.")
    p.add_argument("episodes", nargs="*", help="Raw episode.mcap paths (orchestrator mode).")
    p.add_argument("--ros-domain-id", type=int, default=90)
    p.add_argument("--gpu", default=None, help="CUDA_VISIBLE_DEVICES for the pair.")
    p.add_argument("--play-rate", default="20")
    p.add_argument("--no-tcp-transform", dest="tcp_transform", action="store_false")
    p.add_argument("--output", type=Path, default=None, help="Override output (single-episode only).")
    p.add_argument("--boot-timeout", type=float, default=120.0)
    p.add_argument("--job-timeout", type=float, default=300.0)
    p.add_argument("--log-dir", default=None, help="Dir for per-child perception/buildmap logs.")
    p.set_defaults(tcp_transform=True)
    return p.parse_args()


def main():
    args = parse_args()
    if args.role == "perception":
        role_perception()
    elif args.role == "buildmap":
        role_buildmap()
    else:
        if not args.episodes:
            raise SystemExit("orchestrator mode needs episode.mcap paths")
        orchestrate(args)


if __name__ == "__main__":
    main()

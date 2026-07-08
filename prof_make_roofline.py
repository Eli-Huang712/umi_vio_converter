#!/usr/bin/env python3
"""prof_make_roofline.py — assemble roofline.json from measured footprint +
concurrency numbers, for prof_plots.py chart #1.

The roofline expresses each resource's ceiling as an episode/hour throughput:
  CPU-compute : N_cores * 3600 / C           (C = median CPU core-s/episode)
  GPU-compute : NGPU * 3600 / G_per_ep        (G = clean GPU-busy-s/episode/card)
  IO-bandwidth: disk_write_MBps * 3600 / write_MB_per_ep
Observed plateau is the measured PAR-saturation throughput.

Feed measured values as flags; prints roofline.json.
Usage:
  prof_make_roofline.py --cores 192 --C 234.4 --ngpu 8 --gpu-sec-per-ep 11.0 \
     --disk-write-mbps 900 --write-mb-per-ep 249 --observed 1190 --out roofline.json
"""
import argparse, json

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cores", type=float, default=192)
    ap.add_argument("--C", type=float, required=True, help="median CPU core-seconds/episode")
    ap.add_argument("--ngpu", type=float, default=8)
    ap.add_argument("--gpu-sec-per-ep", type=float, default=None, help="clean GPU-busy s/episode/card")
    ap.add_argument("--disk-write-mbps", type=float, default=None)
    ap.add_argument("--write-mb-per-ep", type=float, default=249)
    ap.add_argument("--observed", type=float, default=1190)
    ap.add_argument("--out", default="roofline.json")
    a = ap.parse_args()

    R = {"observed_plateau_eph": a.observed}
    R["cpu_ceiling_eph"] = round(a.cores * 3600 / a.C) if a.C else None
    if a.gpu_sec_per_ep:
        R["gpu_ceiling_eph"] = round(a.ngpu * 3600 / a.gpu_sec_per_ep)
    if a.disk_write_mbps and a.write_mb_per_ep:
        R["io_ceiling_eph"] = round(a.disk_write_mbps * 3600 / a.write_mb_per_ep)
    R["_inputs"] = {
        "cores": a.cores, "C_core_s_per_ep": a.C, "ngpu": a.ngpu,
        "gpu_sec_per_ep": a.gpu_sec_per_ep, "disk_write_mbps": a.disk_write_mbps,
        "write_mb_per_ep": a.write_mb_per_ep,
    }
    with open(a.out, "w") as f:
        json.dump(R, f, indent=2)
    print(json.dumps(R, indent=2))

if __name__ == "__main__":
    main()

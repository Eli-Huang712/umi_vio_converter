#!/usr/bin/env python3
"""prof_parse_gil.py — from `pidstat -t` output, quantify GIL-serialization:
the top thread's %CPU vs the sum across all threads. GIL-bound => top ~= sum
(one thread does all the work, process capped near 1 core).

pidstat -t columns: time UID TGID TID %usr %system %guest %wait %CPU CPU Command
Process rows have TID='-'; thread rows have TGID='-' + a real TID. %CPU is col 8.

Usage: prof_parse_gil.py <pidstat_threads.txt>
"""
import sys, statistics
f = sys.argv[1]
proc_cpu, top_thread, sum_thread = [], [], []
cur_ts = None
threads = []
pc = None

def flush():
    global threads, pc
    if threads:
        top_thread.append(max(threads)); sum_thread.append(sum(threads))
    if pc is not None:
        proc_cpu.append(pc)
    threads = []; pc = None

for ln in open(f, errors="replace"):
    t = ln.split()
    if len(t) < 10 or not t[0][0:2].isdigit():
        continue
    ts = t[0]
    if ts != cur_ts:
        flush(); cur_ts = ts
    try:
        if t[2] != '-' and t[3] == '-':      # process row
            pc = float(t[8])
        elif t[3] != '-':                     # thread row
            threads.append(float(t[8]))
    except (ValueError, IndexError):
        pass
flush()

def med(x):
    return statistics.median(x) if x else 0

print(f"# {f}")
print(f"proc_%CPU median={med(proc_cpu):.1f}  (n={len(proc_cpu)})")
print(f"top_thread_%CPU median={med(top_thread):.1f}")
print(f"sum_all_threads_%CPU median={med(sum_thread):.1f}")
if sum_thread and med(sum_thread):
    print(f"top/sum ratio = {med(top_thread)/med(sum_thread):.2f}  "
          f"(=>1.0 means GIL-serial: one thread does ~all the work)")

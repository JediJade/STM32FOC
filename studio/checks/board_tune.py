#!/usr/bin/env python3
"""
Auto tune on the board, without the window, as text.

    python board_tune.py COM13 [runs]

Runs boardtune.BoardTune against a board with console version 2 and prints
each step, what was identified beside what vmotor.c's constants work out to,
the gains before and after, and the two recorded speed steps. The result is
discarded at the end of each run, so the board is left on the gains it had.

Exit code 0: every run finished. 1: one did not, or the board has the
original console.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import boardlink  # noqa: E402
import boardtune  # noqa: E402


def one(link, n):
    tune = boardtune.BoardTune(link, {"cbw": 400.0, "sbw": 20.0})
    if not tune.start():
        print(f"run {n}: did not start: {tune.message}")
        return False
    start = time.perf_counter()
    lowest = 999.0
    while tune.active and time.perf_counter() - start < 90:
        tune.advance()
        time.sleep(0.025)
        if time.perf_counter() - start > 1.0:
            lowest = min(lowest, link.rate)
    took = time.perf_counter() - start
    print(f"run {n}: {tune.message}  ({took:.1f} s, link never below {lowest:.0f} lines/s)")
    for step in tune.steps:
        print(f"  {step['state']:8s} {step['name']:26s} {step['value']}")
    if not tune.pending:
        return False
    r = tune.result
    print("  identified            on the board   vmotor.c   difference")
    for key in ("gain", "tau_ms", "accel", "tau_m_ms"):
        ref = boardtune.MODEL[key]
        if r.get(key) is None:
            print(f"  {key:20s} not measured")
            continue
        print(f"  {key:20s} {r[key]:12.4f} {ref:10.4f} {(r[key] - ref) / ref * 100:+10.1f} %")
    print("  gains before", r["before"])
    print("  gains after ", r["after"])
    for tag, m, resp in (("before", r["metrics_before"], tune.resp_before),
                         ("after ", r["metrics_after"], tune.resp_after)):
        settle = f"{m['settle_ms']} ms" if m["settle_ms"] is not None else "not within 512 ms"
        print(f"  step {tune.STEP} units/ms {tag}: {m['overshoot']:.0f} % overshoot, "
              f"settles in {settle}; every 20 ms: {resp[::10][:12]}")
    tune.discard()
    time.sleep(0.8)
    back = all(link.params.get(k) == v for k, v in r["before"].items())
    print(f"  discarded: gains back to what they were: {back}; loop {link.mode}")
    return back


def main(argv):
    port = argv[0] if argv else "COM13"
    runs = int(argv[1]) if len(argv) > 1 else 1
    link = boardlink.BoardLink(port)
    time.sleep(1.0)
    print(f"console {link.fw}, loop {link.mode}, {link.rate:.0f} state lines a second, "
          f"{link.latency_ms:.1f} ms to answer")
    ok = link.fw == 2
    if not ok:
        print("this board has the original console: nothing to tune")
    for n in range(1, runs + 1):
        if not ok:
            break
        ok = one(link, n)
        time.sleep(0.5)
    link.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

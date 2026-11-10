#!/usr/bin/env python3
"""
Send `align` and record what the board does next.

    python align_reset.py COM13

THIS RESETS THE BOARD. That is the point: the console's `align` command
waits 300 ms inside delay_ms(), which does not refresh a watchdog set to
100 ms. Run it only on a bench, with nothing armed.

It reads the reset cause before, sends `align`, records the reply, then
reopens the port as often as it takes and reads the reset cause again.

Exit code 1: the board reset (the reset cause now reads "watchdog").
0: it did not - which is what a fixed firmware should give.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import boardlink  # noqa: E402


def talk(ser, cmd, quiet=0.3, cap=3.0):
    ser.write_line(cmd)
    out = b""
    start = last = time.perf_counter()
    while time.perf_counter() - start < cap:
        chunk = ser.read()
        if chunk:
            out += chunk
            last = time.perf_counter()
        elif time.perf_counter() - last > quiet:
            break
        time.sleep(0.005)
    return [line.strip() for line in out.decode("ascii", "replace").splitlines() if line.strip()]


def cause(lines):
    for line in lines:
        if line.startswith("reset "):
            return line
    return "(no reset line)"


def main(argv):
    port = argv[0] if argv else "COM13"
    ser = boardlink.Serial(port)
    talk(ser, "off")
    before = talk(ser, "t")
    print("before:", cause(before))
    print("state :", (talk(ser, "s") or ["(none)"])[-1])

    sent = time.perf_counter()
    try:
        reply = talk(ser, "align", quiet=1.0, cap=4.0)
    except OSError as err:
        reply = [f"(port error: {err})"]
    print(f"align : reply in the first {time.perf_counter() - sent:.1f} s: {reply}")
    try:
        after_direct = talk(ser, "t")
    except OSError as err:
        after_direct = [f"(port error: {err})"]
    print("then  : `t` on the same handle:", after_direct or "(nothing)")
    try:
        ser.close()
    except OSError:
        pass

    after = []
    for attempt in range(1, 16):
        time.sleep(1.0)
        try:
            ser = boardlink.Serial(port)
        except OSError:
            continue
        try:
            after = talk(ser, "t")
            state = talk(ser, "s")
        except OSError:
            after, state = [], []
        ser.close()
        if any(line.startswith("ok_flags") for line in after):
            print(f"reopen: answered again {attempt} s later")
            print("after :", cause(after))
            print("state :", (state or ["(none)"])[-1])
            break
    else:
        print("reopen: the console did not come back in 15 s")
        return 1

    if "watchdog" in cause(after) and "watchdog" not in cause(before):
        print("THE BOARD RESET: the reset cause went from",
              cause(before).split(",")[0], "to", cause(after).split(",")[0])
        return 1
    if "watchdog" in cause(after):
        print("reset cause reads watchdog, as it did before: this run cannot tell. "
              "Reset the board another way first.")
        return 2
    print("no reset: align completed")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

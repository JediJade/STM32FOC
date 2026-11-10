#!/usr/bin/env python3
"""
One request at a time: does every reply arrive when it is sent, whatever its
length?

    python usb_fullpacket.py COM13

The bulk IN endpoint is 64 bytes. A reply of exactly 64 bytes, sent as one
packet, does not end the transfer, and the host holds it until the next
packet - so it arrives with the reply to the *next* request. The state line
is 64 bytes for some values, so the loop is run at a spread of speeds and in
idle to get lines of every length.

Each reply is matched to the request it answers, oldest first, and timed
from that request. A reply that takes more than 100 ms is counted late. The
usual time is 2 ms.

Exit code 0: nothing was late. 1: at least one reply was.
"""

import collections
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import boardlink  # noqa: E402

LATE = 0.1
TARGETS = (1000, 300, 60, -40, 5, 1500, 120, -7, 900, 33, 2500, -640)


def main(argv):
    port = argv[0] if argv else "COM13"
    ser = boardlink.Serial(port)
    asked = collections.deque()         # send times of requests not yet answered
    prompt = collections.Counter()
    late = collections.Counter()
    slowest = 0.0
    rx = b""

    def take(wait):
        """Read for up to `wait` seconds or until nothing is outstanding."""
        nonlocal rx, slowest
        end = time.perf_counter() + wait
        while time.perf_counter() < end:
            chunk = ser.read()
            if not chunk:
                if not asked:
                    return
                time.sleep(0.0005)
                continue
            rx += chunk
            while b"\n" in rx:
                line, rx = rx.split(b"\n", 1)
                size = len(line) + 1
                if not line.startswith(b"ang "):
                    continue            # a mode reply, not a state line
                if not asked:
                    continue
                age = time.perf_counter() - asked.popleft()
                if age > LATE:
                    late[size] += 1
                else:
                    prompt[size] += 1
                    slowest = max(slowest, age)

    def command(text):
        """A mode change. Its reply is a short line, which also frees
        anything the host is holding."""
        ser.write_line(text)
        take(0.25)
        asked.clear()

    try:
        command("off")
        for target in TARGETS:
            command(f"spd {target}")
            time.sleep(0.5)
            for phase in ("running", "idle"):
                if phase == "idle":
                    command("off")
                    time.sleep(0.4)
                for _ in range(100):
                    ser.write_line("s")
                    asked.append(time.perf_counter())
                    take(0.3)
        command("off")
    finally:
        try:
            ser.write_line("off")
            time.sleep(0.2)
            ser.close()
        except OSError:
            pass

    total = sum(prompt.values()) + sum(late.values())
    print(f"{total} state requests, one at a time, at {len(TARGETS)} speeds and in idle")
    print("reply bytes   on time   late")
    for size in sorted(set(prompt) | set(late)):
        mark = "   <- one full 64-byte packet" if size == 64 else ""
        print(f"  {size:5d}      {prompt[size]:6d}   {late[size]:4d}{mark}")
    print(f"slowest reply that was on time: {slowest * 1000:.1f} ms")
    n_late = sum(late.values())
    if n_late:
        sizes = ", ".join(str(s) for s in sorted(late))
        print(f"LATE: {n_late} replies arrived only with a later one; their sizes: {sizes} bytes")
        return 1
    print("every reply arrived on time")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

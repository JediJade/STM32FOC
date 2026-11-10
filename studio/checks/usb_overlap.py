#!/usr/bin/env python3
"""
Put a packet in each direction on the wire at once, on purpose, and see
whether the console survives it.

    python usb_overlap.py COM13 [seconds]

Requests are written without waiting for their replies, so an OUT packet is
regularly completing while an IN packet is. That is the condition under which
the original USB driver lost a CTR_RX and left the bulk OUT endpoint at NAK
for good; a console answering one request at a time almost never creates it.

Exit code 0: still answering at the end. 1: the console went silent, or a
write to it failed - on the original driver, within seconds.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import boardlink  # noqa: E402


def main(argv):
    port = argv[0] if argv else "COM13"
    limit = float(argv[1]) if len(argv) > 1 else 60.0
    ser = boardlink.Serial(port)
    start = time.perf_counter()
    last_rx = start
    next_report = start + 10.0
    sent = lines = 0
    why = None
    try:
        while time.perf_counter() - start < limit:
            ser.write_line("s")
            sent += 1
            if sent % 3 == 0:
                ser.write_line("x")     # the original console answers "? for help"
                sent += 1
            chunk = ser.read()
            now = time.perf_counter()
            if chunk:
                lines += chunk.count(b"\n")
                last_rx = now
            elif now - last_rx > 1.5:
                why = "no reply for 1.5 s"
                break
            if now >= next_report:
                next_report += 10.0
                print(f"  {now - start:5.1f} s  {sent} requests written, {lines} reply lines read",
                      flush=True)
            time.sleep(0.0015)
    except OSError as err:
        why = str(err)
    elapsed = time.perf_counter() - start
    try:
        ser.close()
    except OSError:
        pass
    print(f"{sent} requests written, {lines} reply lines read in {elapsed:.1f} s")
    if why:
        print(f"CONSOLE WENT SILENT after {elapsed:.1f} s: {why}")
        return 1
    print("still answering")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

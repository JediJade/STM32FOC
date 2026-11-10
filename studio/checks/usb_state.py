#!/usr/bin/env python3
"""
Read the USB driver's state off the running board over SWD, without halting.

    python usb_state.py [--elf ..\\..\\build\\STM32FOC.elf]

Prints the bulk endpoint register (EP1R) decoded, the driver's own variables,
and what the combination means. The addresses of the variables come out of
the ELF every time, because they move from build to build - give the ELF of
the firmware that is actually on the board.

Halting is not an option on this board: the independent watchdog keeps
counting while the core is halted and resets it on resume, so anything seen
afterwards is the power-on self-test. OpenOCD's read_memory works with the
core running.
"""

import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))

NM = [
    r"C:\Users\USER\AppData\Local\UnityMbed\tools\arm-none-eabi\bin\arm-none-eabi-nm.exe",
    "arm-none-eabi-nm",
]
OPENOCD = [
    r"C:\Users\USER\AppData\Local\UnityMbed\tools\openocd-xpack\bin\openocd.exe",
    "openocd",
]
STAT = ("DISABLED", "STALL", "NAK", "VALID")
EP1R = 0x40005C04


def first(candidates):
    for c in candidates:
        if os.path.sep not in c or os.path.exists(c):
            return c
    return candidates[-1]


def symbols(elf):
    out = subprocess.run([first(NM), elf], capture_output=True, text=True).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table[parts[2]] = int(parts[0], 16)
    return table


def read(requests):
    """requests: (label, address, width, count). One OpenOCD session."""
    args = [first(OPENOCD), "-f", "openocd.cfg", "-c", "init"]
    for label, addr, width, count in requests:
        args += ["-c", f"echo \"@@ {label} [read_memory 0x{addr:08x} {width} {count}]\""]
    args += ["-c", "exit"]
    out = subprocess.run(args, capture_output=True, text=True, cwd=REPO)
    values = {}
    for line in (out.stdout + out.stderr).splitlines():
        m = re.match(r"@@ (\S+) (.*)", line.strip())
        if m:
            values[m.group(1)] = [int(v, 16) for v in m.group(2).split()]
    return values


def main(argv):
    elf = os.path.join(REPO, "build", "STM32FOC.elf")
    if "--elf" in argv:
        elf = argv[argv.index("--elf") + 1]
    sym = symbols(elf)
    need = ("configured", "tx_busy", "rx_len", "rx_pos", "tick_ms")
    missing = [n for n in need if n not in sym]
    if missing:
        print("symbols not in", elf, ":", ", ".join(missing))
        return 2

    requests = [("ep1r", EP1R, 16, 1), ("tick", sym["tick_ms"], 32, 1),
                ("configured", sym["configured"], 8, 1),
                ("tx_busy", sym["tx_busy"], 8, 1),
                ("rx_len", sym["rx_len"], 16, 1), ("rx_pos", sym["rx_pos"], 16, 1)]
    a = read(requests)
    b = read(requests)
    if "ep1r" not in a or "ep1r" not in b:
        print("could not read the board over SWD")
        return 2

    for name, v in (("first read ", a), ("second read", b)):
        r = v["ep1r"][0]
        print(f"{name}: EP1R 0x{r:04x}  "
              f"CTR_RX {r >> 15 & 1}  STAT_RX {STAT[r >> 12 & 3]:5s} DTOG_RX {r >> 14 & 1}  |  "
              f"CTR_TX {r >> 7 & 1}  STAT_TX {STAT[r >> 4 & 3]:5s} DTOG_TX {r >> 6 & 1}  |  "
              f"configured {v['configured'][0]} tx_busy {v['tx_busy'][0]} "
              f"rx_len {v['rx_len'][0]} rx_pos {v['rx_pos'][0]}  tick_ms {v['tick'][0]}")

    alive = b["tick"][0] != a["tick"][0]
    print("main loop:", "running (tick_ms advanced between the two reads)" if alive
          else "NOT RUNNING (tick_ms did not advance)")

    def stuck(v):
        r = v["ep1r"][0]
        return (r >> 12 & 3) == 2 and (r >> 15 & 1) == 0 and v["rx_len"][0] == 0

    if stuck(a) and stuck(b):
        print("RECEIVE ENDPOINT DEAD: STAT_RX is NAK, CTR_RX is clear and the "
              "receive buffer is empty, twice. A packet arrived, the hardware "
              "set the endpoint to NAK, and the firmware never saw the event "
              "that would have made it take the packet and re-arm.")
        return 1
    print("receive endpoint: armed or mid-packet, as it should be")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

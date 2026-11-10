#!/usr/bin/env python3
"""
Read the self-test results off the board and print them with names.

The field list is parsed out of inc/selftest.h every time, and the address out
of the ELF every time. Nothing about the layout is written down here.

That is deliberate. Keeping the field order in a reader by hand went wrong
repeatedly: adding one field to the struct shifts every index after it, the
read still succeeds, and the output is neighbouring fields under the wrong
names - numbers that look entirely plausible. Adding one static variable
elsewhere moves the struct's address and does the same thing. Both failure
modes are silent, so neither the address nor the layout is allowed to be a
constant in this script.

Usage:
    python read-selftest.py [--elf build/STM32FOC.elf] [--wait 4]
"""

import argparse
import os
import re
import struct
import subprocess
import sys

NM_CANDIDATES = [
    r"C:\Program Files (x86)\ArmGNUToolchainarm-none-eabi\14.3 rel1\bin\arm-none-eabi-nm.exe",
    r"C:\Users\USER\AppData\Local\UnityMbed\tools\arm-none-eabi\bin\arm-none-eabi-nm.exe",
    "arm-none-eabi-nm",
]
OPENOCD_CANDIDATES = [
    r"C:\Users\USER\AppData\Local\UnityMbed\tools\openocd-xpack\bin\openocd.exe",
    "openocd",
]

MAGIC = 0xF0C00001


def first_available(candidates):
    for c in candidates:
        if os.path.sep in c or "/" in c:
            if os.path.exists(c):
                return c
        else:
            return c
    return None


def parse_fields(header):
    """Field order and signedness, straight out of the struct definition."""
    text = open(header, "r", encoding="utf-8", errors="replace").read()
    body = text[text.index("typedef struct"):text.index("} selftest_results;")]

    # strip comments so a type name inside one cannot be picked up
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)

    fields = []
    for m in re.finditer(r"\b(u?int32_t)\s+([A-Za-z_][A-Za-z0-9_]*)\s*;", body):
        fields.append((m.group(2), m.group(1) == "int32_t"))
    return fields


def symbol_address(nm, elf, name):
    out = subprocess.run([nm, elf], capture_output=True, text=True).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == name and parts[1] in ("b", "B", "d", "D"):
            return int(parts[0], 16)
    raise SystemExit("symbol %s not found in %s" % (name, elf))


def read_words(openocd, cfg, addr, count, wait):
    cmd = "init; sleep %d; set v [read_memory 0x%08x 32 %d]; echo \"OUT:$v\"; exit" % (
        int(wait * 1000), addr, count)
    out = subprocess.run([openocd, "-f", cfg, "-c", cmd],
                         capture_output=True, text=True)
    blob = out.stdout + out.stderr
    for line in blob.splitlines():
        if line.startswith("OUT:"):
            return [int(x, 16) for x in line[4:].split()]
    raise SystemExit("no data from openocd:\n" + blob)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=os.path.join(here, "build", "STM32FOC.elf"))
    ap.add_argument("--header", default=os.path.join(here, "inc", "selftest.h"))
    ap.add_argument("--cfg", default=os.path.join(here, "openocd.cfg"))
    ap.add_argument("--wait", type=float, default=4.0,
                    help="seconds to let the self-tests finish before reading")
    args = ap.parse_args()

    nm = first_available(NM_CANDIDATES)
    openocd = first_available(OPENOCD_CANDIDATES)

    fields = parse_fields(args.header)
    addr = symbol_address(nm, args.elf, "selftest")
    words = read_words(openocd, args.cfg, addr, len(fields), args.wait)

    print("selftest at 0x%08x, %d fields" % (addr, len(fields)))

    values = {}
    for (name, signed), raw in zip(fields, words):
        values[name] = struct.unpack("<i", struct.pack("<I", raw))[0] if signed else raw

    if values.get("magic") != MAGIC:
        print("magic is 0x%08x, not 0x%08x: the tests have not finished, so"
              % (values.get("magic", 0), MAGIC))
        print("every field below is mid-flight. Re-run with a longer --wait.")
        print()

    width = max(len(n) for n, _ in fields)
    for name, _ in fields:
        print("  %-*s %12d" % (width, name, values[name]))

    flags = values.get("ok_flags", 0)
    names = ["dead-time", "adc", "math", "loop", "align", "cascade", "break", "frontend"]
    print()
    print("ok_flags 0x%x:" % flags, ", ".join(
        "%s %s" % (n, "pass" if (flags >> i) & 1 else "FAIL")
        for i, n in enumerate(names)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

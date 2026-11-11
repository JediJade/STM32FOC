#!/usr/bin/env python3
"""
The saved settings, against a board or the emulator.

    python store_flash.py COM13            the board
    python store_flash.py emulator         fwsim.py, in real time
    python store_flash.py COM13 --hall     also learn the Hall edges and
                                           check they come back (slow)

What is actually being asked here is one question: after the power has been
off, is this the board that was put away? So the checks do not stop at
`save` answering `ok`. Twice they make the board reset - `hang`, which stops
feeding the watchdog, is the only reset this console can ask for - wait for
the port to come back, and read the parameters out again.

A reset through the watchdog is the honest test. It cuts the firmware off
mid-instruction without giving it a chance to write anything on the way
down, which is what a vehicle's key switch does.

It leaves the board as it found it: the parameters it started with, and the
flash page erased, so the next boot comes up on the built-in defaults.

Exit code 0: every check passed. 1: one did not.
"""

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

import boardlink            # noqa: E402
import fwsim                # noqa: E402
from v3_features import Bench   # noqa: E402

# Values no default has, so that reading them back proves where they came
# from. Each is inside the parameter's own range; `set` would clip it
# otherwise and the check would be comparing against the clip.
MARKS = (("thmin", 137), ("thmax", 3911), ("drvspd", 4321),
         ("regen", 7777), ("canid", 41), ("ckp", 23456))


def reset_and_reopen(b, port, why, limit=25.0):
    """`hang`, then wait for the console to come back. Seconds, or None."""
    b.ser.write_line("hang")
    time.sleep(0.2)
    try:
        b.ser.close()
    except OSError:
        pass
    gone = time.perf_counter()

    # The watchdog has to time out, the firmware then holds D+ low for
    # 100 ms so the host sees an unplug, and the host needs a moment to
    # agree. Opening the port before all that succeeds against the handle
    # of the device that is about to vanish, and the first write to it
    # fails with a semaphore timeout - so the port is not just opened, it
    # is asked something, and an answer is what counts as back.
    time.sleep(2.0)
    while time.perf_counter() - gone < limit:
        try:
            b.ser = (fwsim.open_emulator() if b.emulated
                     else boardlink.Serial(port))
        except OSError:
            time.sleep(0.5)
            continue
        try:
            if any(line.startswith("fw ") for line in b.talk("ver")):
                back = time.perf_counter() - gone
                b.talk("off")
                print(f"       {why}: back after {back:.1f} s")
                return back
        except OSError:
            pass
        try:
            b.ser.close()
        except OSError:
            pass
        time.sleep(0.5)
    print(f"       {why}: the port never came back")
    return None


def main(argv):
    port = argv[0] if argv and not argv[0].startswith("--") else "COM13"
    do_hall = "--hall" in argv

    b = Bench(port)
    started_with = b.params()

    ver = b.talk("ver")
    b.check("console version 4", ver == ["fw 4 cap 256"], ver)

    # ---- an erased page, and a board that says so
    b.talk("sto clear")
    sto = b.fields("sto")
    b.check("`sto` on an erased page: nothing stored",
            sto.get("state") == "empty" and sto.get("p") == 0, sto)

    # ---- save
    for name, value in MARKS:
        b.set(name, value)
    marked = b.params()
    b.check("the marked values went in",
            all(marked.get(n) == v for n, v in MARKS),
            ", ".join(f"{n}={marked.get(n)}" for n, v in MARKS if marked.get(n) != v)
            or "all six")

    reply = b.talk("save")
    b.check("`save` writes the page", reply == ["save ok"], reply)
    sto = b.fields("sto")
    b.check("`sto`: a record this console wrote",
            sto.get("state") == "ok" and sto.get("console") == 4
            and sto.get("p") == len(started_with), sto)

    # ---- load puts back what was changed since
    for name, _value in MARKS:
        b.set(name, started_with[name])
    reply = b.talk("load")
    b.check("`load` answers ok", reply == ["load ok"], reply)
    back = b.params()
    b.check("`load` puts the saved values back",
            all(back.get(n) == v for n, v in MARKS),
            ", ".join(f"{n}={back.get(n)}" for n, v in MARKS if back.get(n) != v)
            or "all six")

    # ---- the whole point: a reset
    if reset_and_reopen(b, port, "after save") is not None:
        tests = b.talk("t")
        # The emulator has no watchdog: a new FwSim is a power-on, and it
        # says so. Either way what matters is that the firmware restarted
        # rather than the host merely reopening the port.
        was = "power-on" if b.emulated else "watchdog"
        b.check(f"  the reset was the {was}, not a reconnection",
                any(was in t for t in tests), tests[2:3])
        after = b.params()
        b.check("  the saved values are there without being asked for",
                all(after.get(n) == v for n, v in MARKS),
                ", ".join(f"{n}={after.get(n)}" for n, v in MARKS
                          if after.get(n) != v) or "all six")
        sto = b.fields("sto")
        b.check("  `sto` says the board came up on the stored record",
                sto.get("boot") == "ok", sto)
        rest = [n for n in started_with
                if n not in dict(MARKS) and after.get(n) != started_with[n]]
        b.check("  and nothing else moved", not rest, ", ".join(rest) or "none")

    # ---- refused while the drive is running
    b.talk("iq 2000")
    reply = b.talk("save")
    b.check("`save` is refused while something is being driven",
            reply and reply[0].startswith("save refused"), reply)
    b.talk("off")
    reply = b.talk("save")
    b.check("  and allowed again once it is idle", reply == ["save ok"], reply)

    # ---- the Hall table
    if do_hall:
        # The order here is the firmware's, not a preference: `sens 1` is
        # refused until a table has been learned, so the calibration comes
        # first and the selection after it. Getting that round the wrong way
        # is what made the check fail against a board that had just erased
        # its page, and it is the same order store_load() has to use.
        b.talk("sto clear")             # so `hall 1` can only be this run
        b.set("hsrc", 0)
        b.talk("hallcal")
        end = time.perf_counter() + 30.0
        while time.perf_counter() < end:
            if b.fields("v").get("cal") in (1, 2, 3, 4, 9):
                break
            time.sleep(0.25)
        b.check("Hall calibration ran", b.fields("v").get("cal") == 1,
                b.fields("v"))

        edges = [line for line in b.talk("hall") if line.startswith("hall ")]
        learned = [int(v) for v in (edges[0].split()[6:] if edges else [])]
        # Six of the eight states occur; the two that cannot - all-off and
        # all-on - print as -1. A table that was never learned prints eight
        # of them, which would make a comparison after the reset pass
        # without proving anything.
        b.check("  it learned six edges",
                len(learned) == 8 and sum(1 for e in learned if e >= 0) == 6,
                edges[0] if edges else "no hall line")

        b.set("sens", 1)
        b.check("  with a table, the Hall sensors can be selected",
                b.fields("v").get("sens") == 1, b.fields("v"))
        b.talk("save")
        sto = b.fields("sto")
        b.check("  the record carries the table", sto.get("hall") == 1, sto)

        if reset_and_reopen(b, port, "after the hall save") is not None:
            again = [line for line in b.talk("hall") if line.startswith("hall ")]
            b.check("  the learned edges came back from flash",
                    again == edges and bool(edges),
                    (again[0] if again else "no hall line")
                    + ("" if again == edges else f"  was: {edges[0] if edges else ''}"))
            b.check("  on a board that had just restarted",
                    any(("power-on" if b.emulated else "watchdog") in t
                        for t in b.talk("t")), "")
            # The selection, not just the table. `sens 1` is refused while
            # no table has been learned, so a load that put the parameters
            # back before the table left the board on the encoder.
            b.check("  and it came up on the Hall sensors, as it was saved",
                    b.fields("v").get("sens") == 1, b.fields("v"))
        b.set("sens", 0)

    # ---- leave it as it was found
    reply = b.talk("sto clear")
    b.check("`sto clear` erases the page", reply == ["sto clear ok"], reply)
    sto = b.fields("sto")
    b.check("  and the page reads empty again", sto.get("state") == "empty", sto)
    for name, value in started_with.items():
        if b.params().get(name) != value:
            b.set(name, value)
    left = b.params()
    b.check("the board is left on the parameters it was found with",
            left == started_with,
            ", ".join(n for n in started_with if left.get(n) != started_with[n])
            or "all of them")

    b.ser.close()
    print(f"\n{sum(b.rows)} of {len(b.rows)} checks passed")
    return 0 if all(b.rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

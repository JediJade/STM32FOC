#!/usr/bin/env python3
"""
What the speed loop does on Hall sensors, measured rather than argued about.

    python hall_speed.py COM13            the board
    python hall_speed.py emulator         fwsim.py, in real time

The complaint is that with `sens 1` the speed loop hunts at low speed while
the same loop on the encoder is quiet. This prints the numbers behind that:
for each reference, the speed as the firmware's own recorder saw it over
half a second of steady running - mean, peak to peak, and the standard
deviation - on the encoder and on the Hall sensors, one after the other, on
the same motor with the same gains.

Peak to peak is the number that matters. A loop that hunts swings; a loop
that is merely coarse sits still at a wrong value.

The ripple bounds are asserted against a board only. The emulator keeps
time by the host's clock, so its sector times carry whatever the host was
doing; it runs the same arithmetic and its numbers are printed, but they
are not evidence about the firmware. The same goes for the lowest speed on
either: the model's wide sector lasts 180 ms there, so the loop is told the
speed five times a second and what comes out is the sector geometry.

It leaves the board idle, on the encoder, with the parameters it started
with.
"""

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from v3_features import Bench, hub_setup   # noqa: E402

REFS = (300, 600, 1000, 2000, 4000)

# 256 samples, one every 40 control periods: 512 ms of steady running.
DIV = 40


def stats(data):
    if not data:
        return (0.0, 0, 0.0)
    mean = sum(data) / len(data)
    p2p = max(data) - min(data)
    var = sum((v - mean) ** 2 for v in data) / len(data)
    return (mean, p2p, var ** 0.5)


def run_one(b, ref, settle=2.5):
    """Hold a speed, then record half a second of it in steady state."""
    b.talk(f"spd {ref}")
    time.sleep(settle)
    # Re-sending the same reference re-arms the recorder without changing
    # what the loop is being asked for, so what comes back is the steady
    # state and not the step into it.
    data = b.record("spd", DIV, f"spd {ref}", wait=2.0)
    return stats(data)


def main(argv):
    port = argv[0] if argv else "COM13"
    b = Bench(port)
    started_with = b.params()

    hub_setup(b)                      # the hub motor and gains that suit it
    print(f"motor: hub model (mot 1), recorder {DIV * 50} us per sample, "
          f"{DIV * 256 * 50 / 1000:.0f} ms per row\n")

    rows = {}
    for kind, sens in (("encoder", 0), ("hall", 1)):
        if sens == 1:
            # The Hall table has to exist before `sens 1` is allowed.
            b.talk("hallcal")
            end = time.perf_counter() + 30.0
            while time.perf_counter() < end:
                if b.fields("v").get("cal") in (1, 2, 3, 4, 9):
                    break
                time.sleep(0.25)
            cal = b.fields("v").get("cal")
            b.check("hall calibration ran", cal == 1, f"cal {cal}")
        if b.set("sens", sens) != sens:
            b.check(f"select {kind}", False, "refused")
            continue

        print(f"  {kind}")
        print(f"    {'asked':>7} {'mean':>9} {'p2p':>7} {'sd':>7}   units/ms")
        for ref in REFS:
            mean, p2p, sd = run_one(b, ref)
            rows[(kind, ref)] = (mean, p2p, sd)
            print(f"    {ref:>7} {mean:>9.0f} {p2p:>7.0f} {sd:>7.1f}")
        b.talk("off")
        print()

    # ---- what the numbers have to be
    #
    # Not "as quiet as the encoder": the encoder gives the loop a fresh
    # angle every 50 us, and six Hall edges a turn give it a speed six
    # times a turn. The ripple that is left after that is the sensor doing
    # its best, and these bounds are where it sits with the per-sector
    # correction working - a third of what it was without it.
    #
    # The model's 3144-like sectors are 7600 and 55000 units wide, so at
    # 300 units/ms the wide one lasts 180 ms: the loop is told the speed
    # five times a second and nothing in the firmware can make that
    # oftener. That row is held to a looser bound and says so.
    for ref in REFS:
        hall = rows.get(("hall", ref))
        enc = rows.get(("encoder", ref))
        if not hall or not enc:
            continue
        mean, _p2p, sd = hall
        ripple = (sd / mean * 100.0) if mean else 999.0
        detail = (f"sd {sd:.0f} on a mean of {mean:.0f} = {ripple:.1f}%"
                  f"   (encoder: {enc[2] / enc[0] * 100:.1f}%)")
        if ref < 600 or b.emulated:
            # Reported, not asserted. Here the wide sector lasts 180 ms and
            # the loop is told the speed five times a second; what comes out
            # is the model's sector geometry, not the firmware's arithmetic,
            # and the board and the emulator differ for the same reason
            # they differ on any step - the emulator has no SVPWM. Asserting
            # on it would be asserting on the wrong thing.
            why = ("one speed update per 180 ms" if ref < 600
                   else "the emulator runs on the host's clock")
            print(f"    {ref:>5} units/ms: ripple {ripple:.1f}% "
                  f"(not a pass or fail: {why})")
        else:
            b.check(f"{ref:>5} units/ms: ripple within 8% of the speed",
                    ripple <= 8.0, detail)
        b.check(f"      and it holds the speed it was asked for",
                abs(mean - ref) <= max(ref * 0.025, 15),
                f"{mean:.0f} against {ref}")

    b.set("sens", 0)
    b.talk("off")
    for name, value in started_with.items():
        if b.params().get(name) != value:
            b.set(name, value)
    b.check("the board is left on the parameters it was found with",
            b.params() == started_with, "")
    b.ser.close()
    print(f"\n{sum(b.rows)} of {len(b.rows)} checks passed")
    return 0 if all(b.rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

#!/usr/bin/env python3
"""
Console version 3, feature by feature, against a board or the emulator.

    python v3_features.py COM13            the board
    python v3_features.py emulator         fwsim.py, in real time
    python v3_features.py COM13 --can      also change over to CAN and back
                                           (the port goes away for a while)

Each line is one check: what was asked, what came back. Everything is done
through the console, the way a person would, and takes as long as the
motor model takes - about a minute.

It leaves the board idle, on the encoder, with the first motor model and
the gains it found there. It never arms the outputs, and it does not send
`clear`: the throttle-fault check, which needs one to undo, runs on the
emulator only.

Exit code 0: every check passed. 1: one did not.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import boardlink  # noqa: E402
import fwsim      # noqa: E402
import protocols  # noqa: E402


def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


class Bench:
    def __init__(self, port):
        self.port = port
        self.emulated = port == boardlink.EMULATOR_PORT
        self.ser = fwsim.FwSim() if self.emulated else boardlink.Serial(port)
        self.rows = []
        self.last_v = {}
        self.talk("off")
        if "telemetry on" in self.talk("m"):
            self.talk("m")

    def talk(self, cmd, quiet=0.05, cap=3.0):
        self.ser.write_line(cmd)
        out = b""
        start = last = time.perf_counter()
        while time.perf_counter() - start < cap:
            chunk = self.ser.read()
            if chunk:
                out += chunk
                last = time.perf_counter()
            elif time.perf_counter() - last > quiet:
                break
            time.sleep(0.002)
        return [line.strip() for line in out.decode("ascii", "replace").splitlines()
                if line.strip()]

    def fields(self, cmd):
        """A reply of `word name value name value ...` as a dict."""
        word = cmd.split()[0]
        for line in self.talk(cmd):
            parts = line.split()
            if parts and parts[0] == "v" and word != "v":
                self.last_v = {k: int(x) for k, x in zip(parts[1::2], parts[2::2])}
                continue
            if word == "s" and parts and parts[0] == "ang":
                parts = ["s"] + parts       # the state line has no word in front
            if parts and parts[0] == word:
                out = {}
                for k, v in zip(parts[1::2], parts[2::2]):
                    try:
                        out[k] = int(v)
                    except ValueError:
                        out[k] = v
                return out
        return {}

    def set(self, name, value):
        reply = self.talk(f"set {name} {value}")
        return int(reply[-1].split()[2]) if reply and reply[-1].startswith("p ") else None

    def params(self):
        return {line.split()[1]: int(line.split()[2])
                for line in self.talk("get", quiet=0.2) if line.startswith("p ")}

    def record(self, channel, divider, command, wait):
        """Arm the recorder, give the command that starts it, read it out."""
        self.talk(f"cap {channel} {divider}")
        if command:
            self.talk(command)
        end = time.perf_counter() + wait
        while time.perf_counter() < end:
            if self.fields("x").get("cap") == 3:
                break
            time.sleep(0.05)
        data = []
        for line in self.talk("dump", quiet=0.3, cap=5.0):
            if line.startswith("d "):
                data += [int(v) for v in line.split()[1:]]
        return data

    def speed(self, seconds=0.3):
        """Mean of the reported speed over a short while."""
        vals = []
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            st = self.fields("s")
            if "spd" in st:
                vals.append(st["spd"])
        return sum(vals) / len(vals) if vals else 0.0

    def check(self, name, ok, detail=""):
        self.rows.append(bool(ok))
        print(("ok    " if ok else "FAIL  ") + f"{name:56s} {detail}")


def hub_setup(b):
    """The second motor and gains that suit it. Auto tune finds much the
    same; they are written out here so this check stands on its own."""
    for name, value in (("mot", 1), ("sla", 328), ("cemf", 32800), ("cdec", 2),
                        ("ccross", 65535), ("ckp", 30000), ("cki", 300),
                        ("skp", 1000000), ("ski", 12000), ("slcur", 6000),
                        ("slmin", 150)):
        b.set(name, value)


def main(argv):
    port = argv[0] if argv else "COM13"
    b = Bench(port)
    saved = b.params()

    # ---- what is there
    ver = b.talk("ver")
    b.check("console version 3", ver == ["fw 3 cap 256"], ver)
    b.check("every parameter listed, defaults as the firmware is built",
            len(saved) == len(fwsim.PARAMS)
            and all(saved.get(n) == d for n, d, _lo, _hi in fwsim.PARAMS),
            f"{len(saved)} parameters; differ: "
            + (", ".join(n for n, d, _l, _h in fwsim.PARAMS if saved.get(n) != d) or "none"))
    can = b.fields("can")
    b.check("CAN controller passed its loopback test at power-on",
            can.get("test") == 7 and can.get("mode") == 0, can)
    v = b.fields("v")
    b.check("`v` line: idle, encoder, throttle at rest",
            v.get("veh") == 0 and v.get("sens") == 0 and v.get("th") == saved.get("thmin")
            and v.get("idfw") == 0, v)
    b.check("angle the loop uses against the model's rotor (encoder)",
            abs(v.get("aerr", 99999)) < 300, f"{v.get('aerr', 0) * 360 / 65536:.2f} deg")

    # ---- Hall sensors
    if b.talk("hall")[0].startswith("hall ok 0"):
        refused = b.set("sens", 1)
        b.check("Hall sensors refused before they are calibrated", refused == 0,
                f"sens {refused}")
    else:
        print("      (already calibrated since its last reset: not checking the refusal)")
    for width, label in ((32768, "latch-like"), (27000, "3144-like, on for 148 deg")):
        b.set("sens", 0)
        b.set("hwid", width)
        started = b.talk("hallcal")
        end = time.perf_counter() + 6.0
        while time.perf_counter() < end and b.fields("v").get("ph") == 4:
            time.sleep(0.1)
        hall = b.talk("hall")
        edges = [int(x) for x in hall[0].split()[6:]] if hall else []
        truth = fwsim.true_edges(width)
        worst = max((abs(s16(edges[c] - a)) for c, a in truth.items()), default=99999) \
            if len(edges) == 8 else 99999
        b.check(f"hall calibration ({label})",
                started == ["hall calibration started"] and b.fields("v").get("cal") == 1
                and worst < 500,
                f"worst edge {worst * 360 / 65536:.2f} deg from where the model has it")
        found = sorted(e for e in edges if e >= 0)
        widths = [((found[(i + 1) % len(found)] - a) & 0xFFFF) * 360 / 65536
                  for i, a in enumerate(found)] if found else []
        print("      sectors it found: " + " ".join(f"{w:.0f}" for w in widths) + " deg")
        b.check("  set sens 1 accepted", b.set("sens", 1) == 1)
        b.talk("spd 3000")
        time.sleep(1.5)
        err = b.record("aerr", 20, "id 0", 1.0)
        spd = b.speed()
        b.check("  speed loop holds 3000 units/ms on the Hall angle",
                abs(spd - 3000) < 100 and err and max(abs(e) for e in err) < 1500,
                f"speed {spd:.0f}, angle error {min(err) * 360 / 65536:+.1f} to "
                f"{max(err) * 360 / 65536:+.1f} deg over {len(err)} ms")
        b.talk("spd -2000")
        time.sleep(2.0)
        spd = b.speed()
        b.check("  and reverses", abs(spd + 2000) < 100, f"speed {spd:.0f}")
        b.talk("off")
        time.sleep(0.3)
    b.set("sens", 0)
    b.set("hwid", 32768)

    # ---- observer
    b.check("observer selected", b.set("sens", 2) == 2)
    b.talk("spd 3000")
    t0 = time.perf_counter()
    closed = None
    while time.perf_counter() - t0 < 3.0:
        if b.fields("v").get("ph") == 3:
            closed = time.perf_counter() - t0
            break
    time.sleep(1.0)
    err = b.record("aerr", 20, "id 0", 1.0)
    spd = b.speed()
    b.check("observer takes over from the forced angle and holds 3000",
            closed is not None and abs(spd - 3000) < 100 and err
            and max(abs(e) for e in err) < 600,
            f"closed after {0 if closed is None else closed * 1000:.0f} ms, speed {spd:.0f}, "
            f"angle error {min(err) * 360 / 65536:+.1f} to {max(err) * 360 / 65536:+.1f} deg")
    b.talk("spd 300")
    time.sleep(2.5)
    v = b.fields("v")
    spd = b.speed(1.0)
    b.check("below slmin it is back on the forced angle, still turning",
            v.get("ph") == 2 and abs(spd - 300) < 60, f"ph {v.get('ph')}, speed {spd:.0f}")
    b.talk("spd -3000")
    time.sleep(3.0)
    spd = b.speed()
    b.check("and it runs backwards", b.fields("v").get("ph") == 3 and abs(spd + 3000) < 100,
            f"speed {spd:.0f}")
    b.talk("off")
    time.sleep(0.3)
    b.set("sens", 0)

    # ---- throttle and brake
    b.talk("th 2500")
    on = b.talk("drv 1")
    time.sleep(0.3)
    v = b.fields("v")
    b.check("drive waits for the throttle to be released",
            on == ["drive on: release the throttle to start"] and v.get("veh") == 1
            and b.fields("x").get("loop") == 0, f"veh {v.get('veh')}")
    b.talk(f"th {saved['thmin']}")
    time.sleep(0.2)
    b.talk(f"th {saved['thmax']}")
    time.sleep(0.1)
    v = b.fields("v")
    b.check("throttle ramps up instead of stepping",
            v.get("veh") == 2 and 2000 < v.get("thr", 0) < 20000,
            f"{v.get('thr')} of 32767 a tenth of a second after full throttle")
    time.sleep(2.0)
    v = b.fields("v")
    spd = b.speed()
    b.check("full throttle runs up to the speed limit and is held there",
            v.get("thr") == 32767 and saved["drvspd"] - 100 < spd < saved["drvspd"] + 300,
            f"speed {spd:.0f}, limit {saved['drvspd']}")
    # The first motor stops in a fraction of a second, so the three requests
    # go out together and the answers are read as they come.
    b.ser.write_line("brk 1")
    time.sleep(0.012)
    b.ser.write_line("v")
    v = b.fields("s")
    v, st = b.last_v, v
    b.check("brake cuts the throttle and regenerates",
            v.get("veh") == 3 and v.get("thr") == 0 and st.get("iq", 0) < -1000,
            f"iq {st.get('iq')} at {st.get('spd')} units/ms")
    time.sleep(2.5)
    spd = b.speed()
    b.check("  down to a crawl, without reversing", -30 <= spd < 250, f"speed {spd:.0f}")
    b.talk("brk 0")
    b.talk(f"th {saved['thmin']}")
    b.set("thmode", 1)
    b.talk("drv 0")
    b.talk("drv 1")
    time.sleep(0.2)
    b.talk(f"th {(saved['thmin'] + saved['thmax']) // 2}")
    time.sleep(2.0)
    spd = b.speed()
    b.check("throttle as speed: half throttle is half the top speed",
            abs(spd - 0.48 * saved["drvspd"]) < 0.06 * saved["drvspd"], f"speed {spd:.0f}")
    b.talk("drv 0")
    b.set("thmode", 0)
    b.talk(f"th {saved['thmin']}")
    if b.emulated:
        b.talk("drv 1")
        time.sleep(0.2)
        b.talk("th 100")
        time.sleep(0.3)
        v = b.fields("v")
        refused = b.talk("drv 1")
        b.check("a throttle wire off its range is a fault, and stays one",
                v.get("veh") == 4 and v.get("vflt") == 1
                and refused == ["refused: throttle fault, clear it first"], v)
        b.talk("clear")
        b.talk(f"th {saved['thmin']}")
    time.sleep(0.5)

    # ---- one line
    b.set("ol", 1)
    b.talk("spd 3000")
    time.sleep(1.0)
    runs = b.record("ol", 1, None, 1.0)
    d = protocols.ol_decode(runs)
    frame = b.talk("olf")
    b.check("one-line frame decodes from the waveform recorded at the pin",
            d["ok"] and abs(d["t"] - saved["olt"]) < 0.05 and abs(d.get("speed", 0) - 3000) < 120
            and d.get("turning") and not d.get("brake"),
            f"{len(runs)} level changes, T {d['t'] * 50:.0f} us, "
            f"speed {d.get('speed')}, bytes {' '.join(f'{x:02x}' for x in d['bytes'])} {d['why']}")
    b.check("  sync is 31 T low and 1 T high", len(runs) > 2
            and runs[0] == -31 * saved["olt"] and runs[1] == saved["olt"], runs[:2])
    b.check("  `olf` prints a frame with a good checksum",
            bool(frame) and len(frame[0].split()) == 13
            and eval("^".join("0x" + x for x in frame[0].split()[1:])) == 0, frame)
    b.talk("off")
    b.set("ol", 0)

    # ---- field weakening, on the motor where it can work
    time.sleep(0.5)
    hub_setup(b)
    b.set("drvspd", 16000)
    tops = []
    for most in (0, 8192):
        b.set("fw", most)
        b.talk("drv 1")
        time.sleep(0.2)
        b.talk(f"th {saved['thmax']}")
        time.sleep(5.0)
        tops.append((b.speed(), b.fields("v").get("idfw")))
        b.talk(f"th {saved['thmin']}")
        b.talk("brk 1")
        time.sleep(3.0)
        b.talk("brk 0")
        b.talk("drv 0")
        time.sleep(0.5)
    b.check("field weakening raises the hub motor's top speed",
            tops[1][0] > 1.2 * tops[0][0] and (tops[1][1] or 0) < -1000 and tops[0][1] == 0,
            f"{tops[0][0]:.0f} units/ms without, {tops[1][0]:.0f} with (id {tops[1][1]})")

    # ---- observer on the hub motor
    b.set("fw", 0)
    b.set("sens", 2)
    b.talk("spd 3000")
    time.sleep(1.5)
    err = b.record("aerr", 20, "id 0", 1.0)
    spd = b.speed()
    b.check("observer on the hub motor holds 3000",
            b.fields("v").get("ph") == 3 and abs(spd - 3000) < 100 and err
            and max(abs(e) for e in err) < 600,
            f"speed {spd:.0f}, angle error {min(err) * 360 / 65536:+.1f} to "
            f"{max(err) * 360 / 65536:+.1f} deg")
    b.talk("off")
    time.sleep(1.0)

    # ---- back to how it was found
    b.set("sens", 0)
    for name, value in saved.items():
        if name not in ("sens",):
            b.set(name, value)
    after = b.params()
    b.check("parameters back to what they were", after == saved,
            ", ".join(n for n in saved if after.get(n) != saved[n]) or "all of them")
    x = b.fields("x")
    b.check("left idle", x.get("loop") == 0 and x.get("outer") == 0, x)

    if "--can" in argv:
        b.set("canback", 3)
        reply = b.talk("can on")
        print("      ", reply)
        try:
            b.ser.close()
        except OSError:
            pass
        gone = time.perf_counter()
        back = None
        while time.perf_counter() - gone < 20.0:
            time.sleep(0.5)
            try:
                b.ser = (fwsim.open_emulator() if b.emulated else boardlink.Serial(port))
            except OSError:
                continue
            back = time.perf_counter() - gone
            break
        can = b.fields("can") if back is not None else {}
        b.check("`can on`: the port goes away and comes back by itself",
                back is not None and 2.5 < back < 12.0 and can.get("mode") == 0,
                f"back after {0 if back is None else back:.1f} s; {can}")
        if back is not None:
            tests = b.talk("t")
            b.check("  it was a change-over, not a reset",
                    b.params().get("canback") == 3 and not any("watchdog" in t for t in tests[2:3]),
                    tests[2] if len(tests) > 2 else tests)
            b.set("canback", saved["canback"])

    b.ser.close()
    return 0 if all(b.rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

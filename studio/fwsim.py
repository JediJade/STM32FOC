"""
The STM32FOC firmware's control arithmetic and console, in Python.

This is a stand-in for the board, for when there is none on the desk: it
answers the console the way console.c (version 2) does, and behind the
console it runs the same integer arithmetic as pi.c, loop.c, outer.c,
vmotor.c and capture.c, one 50 us period at a time, in real time.

What is the same as the firmware: every gain, limit and shift; the Q15 and
Q30 formats; the order things happen in within a period; the 20:1 ratio
between the loops; the console's commands and the exact text of its replies.

What is left out: Clarke, Park and SVPWM. The firmware turns the rotor-frame
voltage into three duties and the motor model turns them back, through a
256-entry sine table; here the rotor-frame voltage goes to the model
directly. That drops a little table rounding and the 0.7 degree alignment
error the board measured, and nothing else - the model keeps its state in
the rotor frame anyway. There are no faults to trip, no watchdog and no
self-tests to run, so `t` reports a board on which everything passed.

It is a test double, not evidence. A result from here says the host side and
the arithmetic agree with each other; only the board says they are right.
"""

import time

OUT_MAX = 29491
SPEED_MAX = 6560
CAPTURE_LEN = 256
OUTER_DIVIDER = 20

VM_A = 1638
VM_KE = 20
VM_ACCEL = 336
VM_FRICTION = 30

LOOP_IDLE, LOOP_ALIGN, LOOP_CURRENT, LOOP_VOLTAGE = 0, 1, 2, 3
OUTER_OFF, OUTER_SPEED, OUTER_POSITION = 0, 1, 2
CAP_IDLE, CAP_ARMED, CAP_RUNNING, CAP_DONE = 0, 1, 2, 3
CAP_NAMES = ("id", "iq", "vd", "vq", "spd", "ang")

PARAMS = (          # name, default, lo, hi - as console.c has them
    ("ckp", 6000, 0, 262144),
    ("cki", 900, 0, 32767),
    ("cdec", 1, 0, 1),
    ("ccross", 20580, 0, 32767),
    ("cemf", 6554, 0, 32767),
    ("skp", 32768, 0, 1048576),
    ("ski", 6144, 0, 131072),
    ("simax", 16384, 256, 29491),
    ("pkp", 328, 0, 32768),
)


def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _parse_int(text):
    """parse_int() in console.c: optional minus, digits, stop at anything else."""
    text = text.lstrip(" \t")
    neg = text.startswith("-")
    if neg:
        text = text[1:]
    digits = ""
    for ch in text:
        if not ch.isdigit():
            break
        digits += ch
    v = int(digits) if digits else 0
    return -v if neg else v


def _after_word(text, word):
    if not text.startswith(word):
        return None
    rest = text[len(word):]
    if rest and not rest.startswith(" "):
        return None
    return rest.lstrip(" ")


class FwSim:
    """The firmware. Also quacks like boardlink.Serial: read, write_line,
    close - so a BoardLink can be opened on it."""

    DT = 50e-6

    def __init__(self, realtime=True):
        self.realtime = realtime
        self.p = {name: default for name, default, _lo, _hi in PARAMS}
        # vmotor
        self.i_d = self.i_q = 0
        self.omega_fx = 0
        self.theta = 20000
        # loop
        self.loop = LOOP_IDLE
        self.target_q = self.target_d = 0
        self.v_cmd = 0
        self.int_d = self.int_q = 0
        self.last_d = self.last_q = 0
        self.last_vd = self.last_vq = 0
        # outer
        self.outer = OUTER_OFF
        self.int_s = 0
        self.speed = 0
        self.speed_ref = 0
        self.iq_ref = 0
        self.pos_ref = 0
        self.prev_angle = 0
        self.have_prev = False
        self.div = 0
        # recorder
        self.cap_state = CAP_IDLE
        self.cap_ch = 0
        self.cap_div = 1
        self.cap_phase = 0
        self.cap = []
        # console
        self.moe = 0
        self.telemetry = False
        self.next_telem = 0.0
        self.t = 0.0
        self._out = bytearray()
        self._rx = ""
        self._wall = time.perf_counter()
        self.closed = False

    # ---- the control interrupt, n periods of it

    def run(self, n):
        p = self.p
        kp_c = p["ckp"]; ki_c = p["cki"]; dec = p["cdec"]
        ff_cross = p["ccross"]; ff_emf = p["cemf"]
        kp_s = p["skp"]; ki_s = p["ski"]; iq_max = p["simax"]; pos_kp = p["pkp"]
        imax_c = OUT_MAX << 15
        imax_s = iq_max << 15

        i_d = self.i_d; i_q = self.i_q; omega_fx = self.omega_fx; theta = self.theta
        loop = self.loop; target_q = self.target_q; target_d = self.target_d
        v_cmd = self.v_cmd
        int_d = self.int_d; int_q = self.int_q; int_s = self.int_s
        outer = self.outer; speed = self.speed; speed_ref = self.speed_ref
        iq_ref = self.iq_ref; pos_ref = self.pos_ref
        prev_angle = self.prev_angle; have_prev = self.have_prev; div = self.div
        cap_state = self.cap_state; cap_ch = self.cap_ch; cap_div = self.cap_div
        cap_phase = self.cap_phase; cap = self.cap
        vd = vq = 0
        md = mq = 0

        for _ in range(n):
            # ---- foc_loop_step: measure, control
            md = i_d
            mq = i_q
            if loop == LOOP_CURRENT:
                e = target_d - md
                int_d += ki_c * e
                if int_d > imax_c:
                    int_d = imax_c
                elif int_d < -imax_c:
                    int_d = -imax_c
                vd = ((kp_c * e) >> 15) + (int_d >> 15)
                if vd > OUT_MAX:
                    vd = OUT_MAX
                elif vd < -OUT_MAX:
                    vd = -OUT_MAX
                e = target_q - mq
                int_q += ki_c * e
                if int_q > imax_c:
                    int_q = imax_c
                elif int_q < -imax_c:
                    int_q = -imax_c
                vq = ((kp_c * e) >> 15) + (int_q >> 15)
                if vq > OUT_MAX:
                    vq = OUT_MAX
                elif vq < -OUT_MAX:
                    vq = -OUT_MAX
                if dec:
                    vd -= (mq * ff_cross) >> 15
                    vq += (md * ff_cross) >> 15
                    vq += ff_emf
                if vd > OUT_MAX:
                    vd = OUT_MAX
                elif vd < -OUT_MAX:
                    vd = -OUT_MAX
                if vq > OUT_MAX:
                    vq = OUT_MAX
                elif vq < -OUT_MAX:
                    vq = -OUT_MAX
            elif loop == LOOP_VOLTAGE:
                vd = v_cmd
                vq = 0
            else:
                vd = vq = 0

            # ---- vmotor_step
            omega_step = omega_fx >> 8
            c = (omega_step * 3217) >> 10
            d_new = i_d + ((VM_A * (vd - i_d)) >> 15) + ((c * i_q) >> 15)
            q_new = (i_q + ((VM_A * (vq - i_q)) >> 15) - ((c * i_d) >> 15)
                     - ((VM_A * (VM_KE * omega_step)) >> 15))
            i_d = 32767 if d_new > 32767 else (-32768 if d_new < -32768 else d_new)
            i_q = 32767 if q_new > 32767 else (-32768 if q_new < -32768 else q_new)
            omega_fx += (VM_ACCEL * i_q) >> 15
            omega_fx -= (VM_FRICTION * omega_fx) >> 15
            theta = (theta + (omega_fx >> 8)) & 0xFFFF

            # ---- outer_tick
            div += 1
            if div >= OUTER_DIVIDER:
                div = 0
                if have_prev:
                    speed = _s16(theta - prev_angle)
                prev_angle = theta
                have_prev = True
                if outer != OUTER_OFF:
                    if outer == OUTER_POSITION:
                        sref = (pos_kp * _s16(pos_ref - theta)) >> 15
                        if sref > SPEED_MAX:
                            sref = SPEED_MAX
                        elif sref < -SPEED_MAX:
                            sref = -SPEED_MAX
                        speed_ref = sref
                    e = speed_ref - speed
                    int_s += ki_s * e
                    if int_s > imax_s:
                        int_s = imax_s
                    elif int_s < -imax_s:
                        int_s = -imax_s
                    u = ((kp_s * e) >> 15) + (int_s >> 15)
                    if u > iq_max:
                        u = iq_max
                    elif u < -iq_max:
                        u = -iq_max
                    iq_ref = u
                    target_q = u

            # ---- capture_tick
            if cap_state == CAP_RUNNING:
                if cap_phase:
                    cap_phase -= 1
                else:
                    cap_phase = cap_div - 1
                    if cap_ch == 0:
                        v = md
                    elif cap_ch == 1:
                        v = mq
                    elif cap_ch == 2:
                        v = vd
                    elif cap_ch == 3:
                        v = vq
                    elif cap_ch == 4:
                        v = max(-32768, min(32767, speed))
                    else:
                        v = _s16(theta)
                    cap.append(v)
                    if len(cap) >= CAPTURE_LEN:
                        cap_state = CAP_DONE

        self.i_d = i_d; self.i_q = i_q; self.omega_fx = omega_fx; self.theta = theta
        self.target_q = target_q
        self.int_d = int_d; self.int_q = int_q; self.int_s = int_s
        self.speed = speed; self.speed_ref = speed_ref; self.iq_ref = iq_ref
        self.prev_angle = prev_angle; self.have_prev = have_prev; self.div = div
        self.cap_state = cap_state; self.cap_phase = cap_phase
        self.last_d = md; self.last_q = mq
        self.last_vd = vd if loop in (LOOP_CURRENT, LOOP_VOLTAGE) else 0
        self.last_vq = vq if loop in (LOOP_CURRENT, LOOP_VOLTAGE) else 0
        self.t += n * self.DT

    # ---- console.c

    def _line(self, text):
        self._out += (text + "\r\n").encode("ascii")

    def _state(self):
        self._line(f"ang {self.theta}  spd {self.speed}  id {self.last_d}  "
                   f"iq {self.last_q}  iqref {self.iq_ref}  moe {self.moe}  "
                   "fault none")

    def _trigger(self):
        if self.cap_state == CAP_ARMED:
            self.cap.clear()
            self.cap_phase = 0
            self.cap_state = CAP_RUNNING

    def _dispatch(self, cmd):
        if not cmd:
            return
        if cmd[0] == "?" or cmd.startswith("help"):
            self._line("commands:")
            for text in ("s t m iq spd pos align off on clear hang",
                         "ver x get set id vd cap dump"):
                self._line("  " + text)
        elif cmd.startswith("iq "):
            v = max(-16384, min(16384, _parse_int(cmd[3:])))
            self.outer = OUTER_OFF
            self.int_s = 0
            self.loop = LOOP_CURRENT
            self.target_q = v
            self._trigger()
            self._line("current mode")
        elif cmd.startswith("spd "):
            v = max(-SPEED_MAX, min(SPEED_MAX, _parse_int(cmd[4:])))
            self.target_d = 0
            self.loop = LOOP_CURRENT
            self.outer = OUTER_SPEED
            self.int_s = 0
            self.speed_ref = v
            self._trigger()
            self._line("speed mode")
        elif cmd.startswith("pos "):
            v = _parse_int(cmd[4:])
            self.target_d = 0
            self.loop = LOOP_CURRENT
            self.outer = OUTER_POSITION
            self.int_s = 0
            self.pos_ref = (self.theta + v) & 0xFFFF
            self._line("position mode")
        elif cmd.startswith("align"):
            # On the board this ends in a watchdog reset. A test double that
            # silently did something kinder would hide that.
            self._line("aligning...")
        elif cmd.startswith("off"):
            self.outer = OUTER_OFF
            self.int_s = 0
            self.loop = LOOP_IDLE
            self.target_q = self.target_d = 0
            self.v_cmd = 0
            self.moe = 0
            self._line("idle, outputs off")
        elif cmd.startswith("on"):
            if self.loop != LOOP_IDLE or self.outer != OUTER_OFF:
                self._line("refused: not idle")
            else:
                self.moe = 1
                self._line("armed, outputs enabled")
        elif cmd.startswith("clear"):
            self._line("fault cleared")
        elif cmd.startswith("hang"):
            self._line("hanging: the watchdog should reset the board")
        elif (arg := _after_word(cmd, "set")) is not None:
            for name, _default, lo, hi in PARAMS:
                value = _after_word(arg, name)
                if value:
                    self.p[name] = max(lo, min(hi, _parse_int(value)))
                    self._line(f"p {name} {self.p[name]}")
                    break
            else:
                self._line("? unknown parameter, or no value")
        elif _after_word(cmd, "get") is not None:
            for name, *_ in PARAMS:
                self._line(f"p {name} {self.p[name]}")
            self._line("get end")
        elif _after_word(cmd, "ver") is not None:
            self._line(f"fw 2 cap {CAPTURE_LEN}")
        elif (arg := _after_word(cmd, "id")) is not None:
            self.target_d = max(-16384, min(16384, _parse_int(arg)))
            self._trigger()
            self._line("id target")
        elif (arg := _after_word(cmd, "vd")) is not None:
            self.outer = OUTER_OFF
            self.int_s = 0
            self.v_cmd = max(-16384, min(16384, _parse_int(arg)))
            self.loop = LOOP_VOLTAGE
            self._trigger()
            self._line("voltage mode")
        elif (arg := _after_word(cmd, "cap")) is not None:
            for i, name in enumerate(CAP_NAMES):
                rest = _after_word(arg, name)
                if rest is not None:
                    self.cap_state = CAP_IDLE
                    self.cap_ch = i
                    self.cap_div = max(1, min(2000, _parse_int(rest) if rest else 1))
                    self.cap.clear()
                    self.cap_phase = 0
                    self.cap_state = CAP_ARMED
                    self._line("cap armed")
                    break
            else:
                self._line("? cap <id|iq|vd|vq|spd|ang> <divider>")
        elif _after_word(cmd, "dump") is not None:
            n = len(self.cap)
            self._line(f"cap {CAP_NAMES[self.cap_ch]} div {self.cap_div} "
                       f"n {n} state {self.cap_state}")
            for i in range(0, n, 8):
                self._line("d " + " ".join(str(v) for v in self.cap[i:i + 8]))
            self._line("cap end")
        elif cmd[0] == "x":
            self._line(f"x vd {self.last_vd}  vq {self.last_vq}  "
                       f"loop {self.loop}  outer {self.outer}  "
                       f"cap {self.cap_state}  n {len(self.cap)}")
        elif cmd[0] == "s":
            self._state()
        elif cmd[0] == "t":
            self._line("ok_flags 0xff of 0xff")
            self._line("deadtime 7200 cx100 ticks, adc 20000 Hz")
            self._line("reset power-on, wdg 1, fault none")
            self._line("align err 0, loop settle 85, ctrl 1024 cyc")
        elif cmd[0] == "m":
            self.telemetry = not self.telemetry
            self._line("telemetry on" if self.telemetry else "telemetry off")
        else:
            self._line("? for help")

    # ---- time

    def _catch_up(self):
        """Run the periods that real time says have passed."""
        if not self.realtime:
            return
        now = time.perf_counter()
        n = int(min(now - self._wall, 0.25) / self.DT)
        if n > 0:
            self._wall = now if now - self._wall > 0.25 else self._wall + n * self.DT
            self.run(n)
            if self.telemetry and self.t >= self.next_telem:
                self.next_telem = self.t + 0.1
                self._state()

    # ---- what boardlink.Serial offers

    def write_line(self, text):
        if self.closed:
            raise OSError("write failed, the emulator is closed")
        self._catch_up()
        self._rx += text + "\r"
        while "\r" in self._rx:
            cmd, self._rx = self._rx.split("\r", 1)
            self._dispatch(cmd[:63])

    def read(self, n=4096):
        if self.closed:
            raise OSError("read failed, the emulator is closed")
        self._catch_up()
        out = bytes(self._out[:n])
        del self._out[:n]
        return out

    def close(self):
        self.closed = True


# ---------------------------------------------------------------- checks ---

def selfcheck():
    """Does the arithmetic above reproduce what the firmware's own
    documentation says the model does? (name, passed, detail) rows."""
    rows = []

    def check(name, ok, detail):
        rows.append((name, bool(ok), detail))

    def talk(fw, cmd):
        fw.read()
        fw.write_line(cmd)
        return fw.read().decode("ascii").strip().splitlines()

    fw = FwSim(realtime=False)
    check("unknown command", talk(fw, "nonsense") == ["? for help"], "")
    check("version line", talk(fw, "ver") == ["fw 2 cap 256"], "")
    check("set clamps to the firmware's range",
          talk(fw, "set cki 99999") == ["p cki 32767"], "")
    fw.write_line("set cki 900")
    check("'set' is not mistaken for 's'", talk(fw, "set ckp 6000") == ["p ckp 6000"], "")
    check("get lists nine parameters", len(talk(fw, "get")) == 10, "")

    # selftest.c: iq 8192 with the shaft forced at 328 units per period
    # settles in 85 periods. The shaft cannot be forced from the console, so
    # check the free-running case the console can reach instead.
    fw = FwSim(realtime=False)
    fw.write_line("iq 800")
    fw.run(40000)
    rpm = fw.speed * 60000.0 / 65536.0
    check("iq 800 runs up to the speed the model's constants give",
          abs(rpm - 625) < 15, f"{rpm:.0f} rpm, 625 by hand, 637 on the board")

    fw = FwSim(realtime=False)
    fw.write_line("spd 1638")
    peak = 0
    for _ in range(200):
        fw.run(100)
        peak = max(peak, fw.speed)
    check("spd 1638 overshoots the way the board does",
          1.25 * 1638 < peak < 1.6 * 1638,
          f"peak {peak * 60000 / 65536:.0f} rpm on a step to 1500; "
          "the board showed 2181 to 2234")
    fw.run(20000)
    check("and settles on it", abs(fw.speed - 1638) <= 2, f"{fw.speed} units/ms")

    fw = FwSim(realtime=False)
    fw.write_line("cap id 1")
    fw.write_line("vd 3000")
    fw.run(300)
    check("recorder fills 256 samples and stops", fw.cap_state == CAP_DONE
          and len(fw.cap) == 256, f"{len(fw.cap)} samples")
    check("sample 0 is the instant before the step", fw.cap[0] == 0, fw.cap[:3])
    final = sum(fw.cap[-20:]) / 20
    area = sum(final - v for v in fw.cap) - 0.5 * (final - fw.cap[0])
    tau = area / final * 50e-6
    check("a voltage step shows the model's 1 ms time constant",
          abs(tau - 1.0e-3) < 0.05e-3, f"{tau * 1e3:.3f} ms, gain {final / 3000:.3f}")

    fw = FwSim(realtime=False)
    fw.write_line("set skp 1048576")
    fw.write_line("spd 6000")
    worst = 0
    for _ in range(100):
        fw.run(20)
        worst = max(worst, abs(fw.iq_ref))
    check("a large speed gain saturates instead of wrapping",
          worst == 16384 and fw.speed > 0, f"iq_ref peak {worst}")
    return rows


if __name__ == "__main__":
    failed = 0
    for name, ok, detail in selfcheck():
        print(("ok    " if ok else "FAIL  ") + f"{name:52s} {detail}")
        failed += not ok
    raise SystemExit(1 if failed else 0)

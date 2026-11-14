"""
The bench board's console, read from this window's own tick.

The drive gets `boardlink.BoardLink`: a thread, a hundred requests a
second, a history of every channel. The bench needs none of that. It
answers one line that carries everything it knows, and twenty of those a
second is already faster than anything on it moves.

So this is deliberately small: open the port, ask `s`, parse the pairs,
and let the window read `state` whenever it draws. The drive's link and
this one never touch each other - two ports, two devices, one process -
which is the whole reason the two can be watched together at all.

    link = benchlink.BenchLink("COM15")
    link.poll()                      # from the window's tick
    link.state["spd"]                # units per ms, the model's true speed
    link.send("set load 11")
"""

import time

from boardlink import Serial, list_ports     # noqa: F401  (re-exported)

POLL_HZ = 20.0

# What a line may start with and still be a line of values. The bench's
# state line begins with `s`; the rest are replies to a `set`.
VALUE_WORDS = ("s", "load", "lock", "iout", "ispan", "mot", "hwid")


class BenchLink:
    """One console on one port. Nothing threaded, nothing clever."""

    def __init__(self, port):
        self.port = port
        self.ser = Serial(port)          # raises OSError with the reason
        self.state = {}
        self.log = []
        self.error = None
        self.version = None
        self.answers = 0                 # state lines seen, for a liveness check
        # A speed with the ripple taken off. Both boards report an
        # instant, and both instants ride a hunting loop: comparing two
        # of them says more about when they were sampled than about
        # whether the boards agree.
        self.spd_mean = None
        self._buf = ""
        self._asked = 0.0
        self.send("ver")

    # ---- talking

    def send(self, text):
        try:
            self.ser.write_line(text)
            self._note("> " + text)
        except OSError as exc:
            self.error = str(exc)

    def poll(self):
        """Read whatever has arrived, and ask again when it is time."""
        try:
            chunk = self.ser.read()
        except OSError as exc:
            self.error = str(exc)
            return
        if chunk:
            self._buf += chunk.decode("ascii", "replace")
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                self._take(line.strip())

        now = time.perf_counter()
        if now - self._asked >= 1.0 / POLL_HZ:
            self._asked = now
            self.send("s")

    def close(self):
        try:
            self.ser.close()
        except OSError:
            pass

    # ---- reading

    def _note(self, line):
        self.log.append(line)
        del self.log[:-80]

    def _take(self, line):
        if not line:
            return
        self._note(line)
        parts = line.split()
        if not parts:
            return
        if parts[0] == "hil":
            self.version = line
            return
        if parts[0] not in VALUE_WORDS:
            return
        if parts[0] == "s":
            self.answers += 1
        for key, value in zip(parts[1::2], parts[2::2]):
            try:
                self.state[key] = int(value)
            except ValueError:
                self.state[key] = value
        spd = self.state.get("spd")
        if isinstance(spd, int):
            self.spd_mean = (float(spd) if self.spd_mean is None
                             else self.spd_mean + 0.12 * (spd - self.spd_mean))

    # ---- what the window asks of it

    @property
    def live(self):
        return self.error is None and self.answers > 0

    def get(self, key, default=None):
        return self.state.get(key, default)


# ------------------------------------------------------------- the checks --

class Check:
    """One thing that can be right or wrong, in a sentence.

    The bench board can tell most of this on its own. Only the last one
    needs the drive as well, which is why it is the one that goes grey
    rather than red when the drive is not connected.
    """

    def __init__(self, key, title, explain, wrong):
        self.key = key
        self.title = title
        self.explain = explain
        self.wrong = wrong
        self.ok = None
        self.detail = ""


def build_checks():
    return [
        Check("link", "bench board answering",
              "the bench board is on its port and replying",
              "the bench board is not answering"),
        Check("pwm", "drive is switching",
              "pulses are arriving on all three phase pins",
              "the drive is not switching"),
        Check("period", "carrier is 20 kHz",
              "every pulse period measures 50 us, as the drive intends",
              "the carrier is not 20 kHz"),
        Check("bridge", "bridge pattern safe",
              "no sample caught a phase with both transistors on",
              "a phase had both transistors on"),
        Check("realtime", "model keeping up",
              "every 50 us tick finished before the next one arrived",
              "the model is not keeping up"),
        Check("hall", "hall pattern valid",
              "never all-on or all-off: those two mean a broken sensor",
              "the hall pattern is impossible"),
        Check("sync", "both boards agree",
              "the drive's measured speed and the model's true speed match",
              "the two boards do not agree"),
    ]


def run_checks(checks, bench, drive_spd):
    """Fill in every check from the bench's last state line.

    drive_spd is the drive's speed in the same units, or None when this
    window is not holding the drive's console.
    """
    st = bench.state if bench is not None else {}
    by = {c.key: c for c in checks}

    c = by["link"]
    if bench is None:
        c.ok, c.detail = None, "no bench board connected yet"
    elif bench.error:
        c.ok, c.detail = False, bench.error
    else:
        c.ok = True
        c.detail = f"{bench.port}, firmware {bench.version or '?'}"

    c = by["pwm"]
    live, moe = st.get("live"), st.get("moe")
    if live is None:
        c.ok, c.detail = None, c.explain
    elif live and moe:
        c.ok, c.detail = True, "all three phases are switching"
    else:
        c.ok, c.detail = False, "the drive's outputs are off"

    c = by["period"]
    per = st.get("per") or 0
    if not per:
        c.ok, c.detail = None, c.explain
    else:
        khz = 72000.0 / per
        c.ok = abs(khz - 20.0) < 0.3
        c.detail = f"{per} ticks = {per / 72.0:.2f} us ({khz:.2f} kHz)"

    c = by["bridge"]
    ovl = st.get("ovl")
    if ovl is None:
        c.ok, c.detail = None, c.explain
    elif ovl == 0:
        c.ok, c.detail = True, "no overlap seen on any phase"
    else:
        c.ok, c.detail = False, f"{ovl} samples caught both transistors on"

    c = by["realtime"]
    late = st.get("late")
    if late is None:
        c.ok, c.detail = None, c.explain
    elif late == 0:
        c.ok, c.detail = True, "no tick has overrun its 50 us"
    else:
        c.ok, c.detail = False, f"{late} ticks arrived late"

    c = by["hall"]
    hall = st.get("hall")
    if hall is None:
        c.ok, c.detail = None, c.explain
    elif hall in (0, 7):
        c.ok, c.detail = False, f"code {hall:03b}: all on or all off"
    else:
        c.ok, c.detail = True, f"code {hall:03b}, one of the six valid states"

    c = by["sync"]
    bench_spd = bench.spd_mean if bench is not None else None
    if drive_spd is None or bench_spd is None:
        c.ok, c.detail = None, "needs both boards on their ports"
    else:
        span = max(abs(drive_spd), abs(bench_spd), 1.0)
        err = abs(drive_spd - bench_spd) / span
        c.ok = err < 0.08 and abs(bench_spd) > 20
        c.detail = (f"drive {drive_spd:.0f}, model {bench_spd:.0f} "
                    f"units/ms on average, {err * 100:.1f} % apart")
    return checks

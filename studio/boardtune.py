"""
Auto tune for the firmware on the board (console version 2).

The same idea as focsim.AutoTune - measure the plant, then design both
controllers from the measurements - in the firmware's own units, because
those are the units its gains are in:

    current   Q15 counts          voltage   Q15 counts
    speed     angle units per ms  time      seconds

so a "resistance" here is a pure number (counts of voltage per count of
current) and it is its inverse, the gain, that is reported.

  baseline    a 1000 units/ms speed step, recorded on the board
  gain        two d-axis current levels held by the current loop;
              gain = change in current / change in voltage
  time const  a voltage step on the d axis with no controller, recorded at
              20 kHz; the area above the curve is the time constant
  accel       a torque-current step with the speed recorded at 1 kHz;
              the final speed and the area give the acceleration constant
              and the mechanical time constant
  gains       current PI:  Kp = tau wc / gain,  Ki = wc / gain
              speed PI:    Kp = ws / accel,     Ki = Kp ws / 5
  verify      the same speed step, recorded again

Everything on the d axis makes no torque, so the rotor stays where it is for
the first three measurements; only the last two turn it.

It needs nothing the board does not report. Nothing here reads vmotor.c: the
plant is whatever answers the console, which today is the model inside the
firmware and later is a motor.

The new gains live in the board's RAM. apply() keeps them until the next
reset; the firmware has nowhere to store them yet.
"""

import math
import time

TS = 50e-6                      # one control period
TICK = 1e-3                     # one speed-loop period


class TuneError(Exception):
    pass


def _mean(values):
    return sum(values) / len(values)


def _first_order(samples, dt):
    """(final value, time constant, settled?) of a rise that starts at
    sample 0. The area between the curve and its final value, by the
    trapezoid rule, is the final value times the time constant."""
    n = len(samples)
    k = max(8, n // 10)
    final = _mean(samples[-k:])
    before = _mean(samples[-2 * k:-k])
    if final == 0:
        return 0.0, 0.0, False
    settled = abs(final - before) < 0.004 * abs(final)
    area = sum(final - v for v in samples)
    area -= 0.5 * (final - samples[0]) + 0.5 * (final - samples[-1])
    return final, area * dt / final, settled


def _slope(samples, dt):
    n = len(samples)
    mt = (n - 1) / 2.0
    my = _mean(samples)
    den = sum((i - mt) ** 2 for i in range(n))
    return sum((i - mt) * (y - my) for i, y in enumerate(samples)) / den / dt


def step_metrics(resp, step, dt_ms):
    if not resp:
        return {"overshoot": 0.0, "settle_ms": None}
    band = 0.05 * abs(step)
    last = 0
    for i, v in enumerate(resp):
        if abs(v - step) > band:
            last = i + 1
    return {"overshoot": max(0.0, (max(resp) - step) / step * 100.0),
            "settle_ms": int(last * dt_ms) if last < len(resp) else None}


class BoardTune:
    STEP = 1000                 # units per ms, about 915 rpm
    STEP_DIV = 40               # one sample every 2 ms: 512 ms of response
    ID_LEVELS = (2000, 4000)
    V_STEP = 3000
    IQ_STEP = 2000
    EXPECTED_SECONDS = 9.0
    KEYS = ("ckp", "cki", "skp", "ski")

    STEPS = (
        ("baseline", "baseline response", "speed step with the present gains"),
        ("gain", "winding gain", "two d-axis current levels"),
        ("tau", "electrical time constant", "voltage step, recorded at 20 kHz"),
        ("accel", "acceleration constant", "torque step, speed at 1 kHz"),
        ("gains", "controller gains", "from the bandwidth targets"),
        ("verify", "verification", "speed step with the new gains"),
    )

    def __init__(self, link, targets):
        self.link = link
        self.targets = targets          # {"cbw": Hz, "sbw": Hz}
        self.active = False
        self.pending = False
        self.message = "put the loop in idle, then start."
        self.result = {}
        self.resp_before = []
        self.resp_after = []
        self.progress = 0.0
        self.steps = []
        self._gen = None
        self._snapshot = None
        self._t0 = 0.0
        self._reset_steps()

    def _reset_steps(self):
        self.steps = [{"key": k, "name": n, "detail": d, "state": "pending",
                       "value": ""} for k, n, d in self.STEPS]

    def _step(self, key):
        return next(s for s in self.steps if s["key"] == key)

    # ---- control

    def start(self):
        link = self.link
        if self.active:
            return False
        if (link.fw or 0) < 2:
            self.message = ("this firmware has no tuning commands. "
                            "it needs console version 2.")
            return False
        if link.mode != "idle":
            self.message = "put the loop in idle first (stop)."
            return False
        if self.pending:
            self.discard()
        self._reset_steps()
        self.result = {}
        self.resp_before = []
        self.resp_after = []
        self.progress = 0.0
        self._snapshot = None
        self._t0 = time.perf_counter()
        self._gen = self._procedure()
        self.active = True
        link.x_every = 1                # modes and voltage at full rate
        self.message = "identifying. the rotor will turn."
        return True

    def advance(self, _n=0):
        if not self.active:
            return
        try:
            if self.link.error:
                raise TuneError("the board went away.")
            next(self._gen)
            self.progress = min(0.98, (time.perf_counter() - self._t0)
                                / self.EXPECTED_SECONDS)
        except StopIteration:
            self.active = False
            self.pending = True
            self.progress = 1.0
            self.link.x_every = 5
            self.message = ("finished. apply to keep the new gains until the "
                            "next reset, or discard.")
        except TuneError as err:
            self._fail("failed: " + str(err))

    def _restore(self):
        link = self.link
        link.x_every = 5
        if link.error:
            return
        link.send("off")
        if self._snapshot:
            for name, value in self._snapshot.items():
                if link.params.get(name) != value:
                    link.set_param(name, value)

    def _fail(self, why):
        for s in self.steps:
            if s["state"] == "running":
                s["state"] = "failed"
        self.active = False
        self.pending = False
        self._restore()
        self.message = why + " the previous gains are back."

    def abort(self):
        if self.active:
            self._fail("aborted.")

    def apply(self):
        if self.pending:
            self.pending = False
            self.message = ("applied. the gains are in the board's memory "
                            "until it is reset.")

    def discard(self):
        if self.pending:
            self.pending = False
            self._restore()
            self.message = "discarded. the previous gains are back."

    # ---- building blocks. Each yields once per frame of the window.

    def _wait(self, seconds):
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            yield

    def _until(self, done, timeout, why):
        end = time.perf_counter() + timeout
        while not done():
            if time.perf_counter() > end:
                raise TuneError(why)
            yield

    def _set(self, name, value):
        """Set one parameter and wait until the board confirms it."""
        link = self.link
        link.params.pop(name, None)
        link.set_param(name, value)
        yield from self._until(lambda: name in link.params, 1.5,
                               f"the board did not confirm {name}.")

    def _idle(self):
        """Idle, and wait for the rotor. The firmware's model never quite
        stops - it creeps at a few units per ms - so 'at rest' is a band."""
        link = self.link
        link.send("off")
        yield from self._until(lambda: link.mode == "idle", 1.5,
                               "the loop did not go idle.")
        quiet = [0.0]

        def at_rest():
            if abs(link.units) < 80.0:
                quiet[0] = quiet[0] or time.perf_counter()
                return time.perf_counter() - quiet[0] > 0.15
            quiet[0] = 0.0
            return False

        yield from self._until(at_rest, 6.0, "the rotor did not come to rest.")

    def _record(self, channel, divider, command):
        """Arm the recorder, send the command that starts it, wait, read."""
        link = self.link
        serial = link.cap_serial
        link.send(f"cap {channel} {divider}")
        link.send(command)
        length = 256 * divider * TS
        yield from self._until(lambda: link.cap_state == 3, length + 2.0,
                               "the recorder did not finish.")
        link.send("dump")
        yield from self._until(lambda: link.cap_serial != serial, 3.0,
                               "the record did not arrive.")
        if len(link.cap_data) < 200:
            raise TuneError("the record came back short.")
        return list(link.cap_data)

    def _step_test(self):
        samples = yield from self._record("spd", self.STEP_DIV,
                                          f"spd {self.STEP}")
        yield from self._idle()
        return samples

    def _begin(self, key):
        self._step(key)["state"] = "running"

    def _done(self, key, value):
        s = self._step(key)
        s["state"] = "done"
        s["value"] = value

    # ---- the procedure

    def _procedure(self):
        link = self.link
        res = self.result
        dt_ms = self.STEP_DIV * TS * 1000.0

        serial = link.params_serial
        link.send("get")
        yield from self._until(lambda: link.params_serial != serial, 2.0,
                               "the board did not list its parameters.")
        self._snapshot = dict(link.params)
        res["before"] = {k: self._snapshot[k] for k in self.KEYS}
        yield from self._idle()

        # 0 --- baseline
        self._begin("baseline")
        self.resp_before = yield from self._step_test()
        m = step_metrics(self.resp_before, self.STEP, dt_ms)
        res["metrics_before"] = m
        self._done("baseline", f"{m['overshoot']:.0f} % overshoot")

        # The feed-forward terms add voltage of their own. While the plant
        # is being measured they are switched off, so that the voltage seen
        # is the controllers' and nothing else.
        yield from self._set("cdec", 0)

        # 1 --- gain: two currents on the d axis, where they make no torque.
        self._begin("gain")
        points = []
        for level in self.ID_LEVELS:
            link.send(f"id {level}")
            link.send("iq 0")
            yield from self._wait(0.35)
            vs, cs = [], []
            end = time.perf_counter() + 0.3
            while time.perf_counter() < end:
                vs.append(link.vd)
                cs.append(link.state["id"])
                yield
            v, i = _mean(vs), _mean(cs)
            if abs(i - level) > 0.1 * level:
                raise TuneError("the d-axis current did not reach its target.")
            points.append((v, i))
        link.send("off")
        (v1, i1), (v2, i2) = points
        if abs(v2 - v1) < 50:
            raise TuneError("the voltage hardly changed between the two currents.")
        gain = (i2 - i1) / (v2 - v1)
        if not 0.01 < gain < 100.0:
            raise TuneError("the gain measurement makes no sense.")
        res["gain"] = gain
        self._done("gain", f"{gain:.3f}")

        # 2 --- time constant: a voltage step, no controller, 20 kHz record.
        #       If the current is still rising at the end of the record,
        #       the winding is slower than the window: thin the record out.
        self._begin("tau")
        divider = 1
        while True:
            yield from self._wait(0.06 + 256 * divider * TS)
            samples = yield from self._record("id", divider,
                                              f"vd {self.V_STEP}")
            link.send("off")
            final, tau, settled = _first_order(samples, divider * TS)
            if final <= 0:
                raise TuneError("no current flowed during the voltage step.")
            if settled or divider >= 64:
                break
            divider *= 2
        if not 5e-6 < tau < 1.0:
            raise TuneError("the time-constant measurement makes no sense.")
        res["tau_ms"] = tau * 1e3
        self._done("tau", f"{tau * 1e3:.3f} ms")

        # The current loop can be designed now, and the torque step that
        # follows is cleaner with it: Kp = tau wc / gain puts the zero on
        # the winding's pole, Ki = wc / gain sets the bandwidth.
        wc = 2.0 * math.pi * self.targets["cbw"]
        yield from self._set("ckp", tau * wc / gain * 32768.0)
        yield from self._set("cki", wc / gain * TS * 32768.0)

        # 3 --- acceleration: a known torque current, speed recorded at
        #       1 kHz. With friction the speed levels off and the curve is
        #       first order; without it the speed is a ramp.
        self._begin("accel")
        yield from self._idle()
        samples = yield from self._record("spd", 20, f"iq {self.IQ_STEP}")
        yield from self._idle()
        final, tau_m, settled = _first_order(samples, TICK)
        if settled and final > 20 and tau_m > 0:
            accel = final / (tau_m * self.IQ_STEP)
            res["tau_m_ms"] = tau_m * 1e3
        else:
            accel = _slope(samples, TICK) / self.IQ_STEP
            res["tau_m_ms"] = None
        if not 0.01 < accel < 1e5:
            raise TuneError("the rotor did not accelerate.")
        res["accel"] = accel
        self._done("accel", f"{accel:.2f}")

        # 4 --- speed loop
        self._begin("gains")
        ws = 2.0 * math.pi * self.targets["sbw"]
        kp = ws / accel
        yield from self._set("skp", kp * 32768.0)
        yield from self._set("ski", kp * ws / 5.0 * TICK * 32768.0)
        yield from self._set("cdec", self._snapshot["cdec"])
        res["after"] = {k: link.params[k] for k in self.KEYS}
        self._done("gains", f"{self.targets['cbw']:.0f} Hz / "
                            f"{self.targets['sbw']:.0f} Hz")

        # 5 --- verify
        self._begin("verify")
        self.resp_after = yield from self._step_test()
        m = step_metrics(self.resp_after, self.STEP, dt_ms)
        res["metrics_after"] = m
        self._done("verify", f"{m['overshoot']:.0f} % overshoot")


# What vmotor.c's constants work out to, for comparison while the plant is
# that model: the pole of i += a (v - i) with a = 1638/32768 per 50 us, and
# of the friction term 30/32768 per period.
MODEL = {
    "gain": 1.0,
    "tau_ms": -TS / math.log(1.0 - 1638.0 / 32768.0) * 1e3,
    "accel": 336.0 / 32768.0 / 256.0 * 20.0 / TS,
    "tau_m_ms": -TS / math.log(1.0 - 30.0 / 32768.0) * 1e3,
}

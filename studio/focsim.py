"""
A permanent-magnet motor and the FOC drive that runs it, simulated in
physical units. No window and no dependencies: foc.py is the front end, and
this file can be imported and tested on its own.

What is modelled

  motor    dq model with cross-coupling and back-EMF, viscous friction and a
           passive load torque (a brake: it opposes motion, it cannot drive)
  drive    10 kHz current loop (PI on d and q, feed-forward decoupling,
           circular voltage limit, anti-windup) and a 1 kHz speed loop (PI,
           ramp generator, current limit), with over-current and over-speed
           trips
  sensors  current noise and a little speed noise

The drive never reads the motor's real constants. It has its own copy in
parameter group 1 ("motor data"), which starts out as a nameplate guess, and
the real ones live in group 9 ("simulated motor"). Auto tune is what closes
the gap, by measurement, the way a real drive has to.

Same structure as the STM32 firmware next to it (current loop inside a speed
loop, decoupling, ramped reference); different units and rate. The firmware
works in Q15 counts at 20 kHz against its own built-in model; this works in
volts, amps and rad/s at 10 kHz so the numbers on screen mean something.
"""

import math
import random
import time
from array import array

RPM = 30.0 / math.pi          # rad/s -> rpm
SQRT3 = math.sqrt(3.0)

MODE_SPEED = 0
MODE_CURRENT = 1
MODE_VOLTAGE = 2


# ------------------------------------------------------------ parameters ---

class Param:
    __slots__ = ("number", "group", "key", "name", "unit", "default", "lo",
                 "hi", "step", "decimals", "help", "choices", "runlock")

    def __init__(self, number, key, name, unit, default, lo, hi, step,
                 decimals, help, choices=None, runlock=False):
        self.number = number
        self.group = int(number.split(".")[0])
        self.key = key
        self.name = name
        self.unit = unit
        self.default = default
        self.lo = lo
        self.hi = hi
        self.step = step
        self.decimals = decimals
        self.help = help
        self.choices = choices
        self.runlock = runlock

    def fmt(self, value):
        if self.choices:
            return self.choices[int(round(value))]
        return f"{value:.{self.decimals}f}"


GROUPS = [
    (1, "motor data", "what the drive believes about the motor"),
    (2, "current controller", "inner loop, 10 kHz"),
    (3, "speed controller", "outer loop, 1 kHz"),
    (4, "limits and protection", "what the drive refuses to exceed"),
    (9, "simulated motor", "the motor on the bench - the drive cannot read these"),
]

PARAMS = [
    Param("1.01", "pole_pairs", "pole pairs", "", 4, 1, 20, 1, 0,
          "Number of magnetic pole pairs. Electrical speed is this many times "
          "the shaft speed. A nameplate fact, shared with the simulated motor.",
          runlock=True),
    Param("1.02", "rated_current", "rated current", "A", 5.0, 0.5, 20.0, 0.1, 1,
          "Continuous q-axis current. Auto tune injects fractions of it."),
    Param("1.03", "rated_speed", "rated speed", "rpm", 3000, 100, 6000, 50, 0,
          "Nameplate speed. Auto tune runs at 40 % of it to measure flux."),
    Param("1.04", "m_rs", "stator resistance", "Ω", 0.600, 0.010, 10.0, 0.005, 3,
          "Phase resistance the drive assumes. Sets the current controller's "
          "integral gain. Measured by auto tune."),
    Param("1.05", "m_ls", "stator inductance", "mH", 0.600, 0.010, 20.0, 0.005, 3,
          "Phase inductance the drive assumes. Sets the current controller's "
          "proportional gain and the decoupling term. Measured by auto tune."),
    Param("1.06", "m_flux", "flux linkage", "mWb", 10.00, 0.50, 100.0, 0.05, 2,
          "Permanent-magnet flux linkage. Sets the back-EMF feed-forward and "
          "the torque constant. Measured by auto tune."),
    Param("1.07", "m_j", "inertia", "kg·cm²", 2.000, 0.010, 100.0, 0.01, 3,
          "Rotor plus load inertia the drive assumes. Sets the speed "
          "controller's gain. Measured by auto tune."),

    Param("2.01", "c_kp", "proportional gain", "V/A", 0.300, 0.001, 50.0, 0.01, 3,
          "Current controller Kp. Auto tune sets it to L × bandwidth."),
    Param("2.02", "c_ki", "integral gain", "V/As", 150.0, 0.0, 100000.0, 10, 1,
          "Current controller Ki. Auto tune sets it to R × bandwidth, which "
          "cancels the motor's electrical pole."),
    Param("2.03", "c_bw", "bandwidth target", "Hz", 300, 50, 800, 10, 0,
          "Closed-loop bandwidth auto tune designs the current loop for. "
          "Keep it well under a tenth of the 10 kHz loop rate."),
    Param("2.04", "c_dec", "decoupling", "", 1, 0, 1, 1, 0,
          "Feed-forward of the cross-coupling and back-EMF terms. Off, the "
          "integrators have to remove them after they appear as error.",
          choices=("off", "on")),

    Param("3.01", "s_kp", "proportional gain", "A·s/rad", 0.060, 0.001, 20.0, 0.005, 3,
          "Speed controller Kp. Auto tune sets it from inertia and the "
          "torque constant."),
    Param("3.02", "s_ki", "integral gain", "A/rad", 6.00, 0.0, 2000.0, 0.5, 2,
          "Speed controller Ki. Too much against too little Kp is what "
          "makes a speed step overshoot."),
    Param("3.03", "s_bw", "bandwidth target", "Hz", 25, 2, 80, 1, 0,
          "Closed-loop bandwidth auto tune designs the speed loop for. It "
          "has to stay several times below the current loop's."),
    Param("3.04", "accel", "acceleration ramp", "rpm/s", 3000, 10, 100000, 100, 0,
          "How fast the speed reference may move away from zero."),
    Param("3.05", "decel", "deceleration ramp", "rpm/s", 3000, 10, 100000, 100, 0,
          "How fast the speed reference may move towards zero."),

    Param("4.01", "i_limit", "current limit", "A", 10.0, 0.5, 40.0, 0.5, 1,
          "Ceiling on the q-axis current the speed controller may ask for."),
    Param("4.02", "max_speed", "maximum speed", "rpm", 3000, 100, 6000, 100, 0,
          "Largest setpoint, in either direction. Rescales the speed slider.",
          runlock=True),
    Param("4.03", "oc_trip", "over-current trip", "A", 15.0, 1.0, 60.0, 0.5, 1,
          "Phase current magnitude that trips the drive."),
    Param("4.04", "os_trip", "over-speed trip", "rpm", 4200, 200, 9000, 100, 0,
          "Shaft speed that trips the drive."),
    Param("4.05", "vbus", "dc bus voltage", "V", 24.0, 5.0, 60.0, 0.5, 1,
          "Supply to the inverter. The largest phase voltage is this over √3."),
    Param("4.06", "stop_mode", "stop mode", "", 0, 0, 1, 1, 0,
          "Ramp: follow the deceleration ramp to zero, then switch off. "
          "Coast: switch off at once and let friction stop the shaft.",
          choices=("ramp", "coast")),

    Param("9.01", "x_rs", "stator resistance", "Ω", 0.450, 0.010, 10.0, 0.005, 3,
          "The real phase resistance. Change it and re-run auto tune to see "
          "the drive find the new value."),
    Param("9.02", "x_ls", "stator inductance", "mH", 0.450, 0.010, 20.0, 0.005, 3,
          "The real phase inductance."),
    Param("9.03", "x_flux", "flux linkage", "mWb", 8.50, 0.50, 100.0, 0.05, 2,
          "The real flux linkage. With four pole pairs, 8.5 mWb is a torque "
          "constant of 51 mNm/A."),
    Param("9.04", "x_j", "inertia", "kg·cm²", 1.200, 0.010, 100.0, 0.01, 3,
          "The real inertia of rotor and load."),
    Param("9.05", "x_b", "viscous friction", "mNm/krpm", 2.00, 0.0, 100.0, 0.1, 2,
          "Friction torque that grows with speed."),
    Param("9.06", "x_load", "load torque", "mNm", 0, 0, 400, 5, 0,
          "A brake on the shaft: it opposes motion and cannot drive it. Also "
          "on the drive page as a slider."),
    Param("9.07", "x_noise", "current sensor noise", "mA", 15, 0, 200, 1, 0,
          "RMS noise on the measured currents. Zero switches the speed "
          "noise off as well."),
]

BY_KEY = {p.key: p for p in PARAMS}

CHANNELS = ("rpm", "ref", "iq", "iqref", "id", "ia", "ib")


# ------------------------------------------------------------------ drive ---

class Drive:
    DT = 1.0e-4                 # control period, 10 kHz
    OUTER = 10                  # speed loop every 10th period, 1 kHz
    REC = 2                     # history sample every 2nd period, 5 kHz
    HIST_SECONDS = 20.0

    def __init__(self, seed=1):
        self.rng = random.Random(seed)
        self.p = {q.key: float(q.default) for q in PARAMS}
        self.rec_hz = 1.0 / (self.DT * self.REC)
        self.hist_n = int(self.HIST_SECONDS * self.rec_hz)
        self.hist = {c: array("f", bytes(4 * self.hist_n)) for c in CHANNELS}
        self.h_idx = 0
        self.h_count = 0

        self.t = 0.0
        self.i_d = self.i_q = 0.0       # true motor currents
        self.w = 0.0                    # true shaft speed, rad/s
        self.theta = 0.0                # electrical angle
        self.idm = self.iqm = 0.0       # what the drive measured
        self.w_meas = 0.0
        self.vd = self.vq = 0.0
        self.int_d = self.int_q = self.int_s = 0.0
        self.id_ref = self.iq_ref = 0.0
        self.vd_cmd = self.vq_cmd = 0.0
        self.ref = 0.0                  # ramped speed reference, rad/s
        self.target = 0.0               # where the ramp is heading
        self.enabled = False
        self.mode = MODE_SPEED
        self.ramp_bypass = False
        self.stopping = False
        self.tuning = False
        self.fault = None
        self.ilim_hit = False
        self.vlim_hit = False
        self.ticks = 0                  # speed-loop updates so far
        self._div = 0
        self._rdiv = 0
        self._still = 0                 # speed-loop updates spent at rest
        self.apply_params()

    # ---- parameters

    def apply_params(self):
        p = self.p
        self.pp = int(round(p["pole_pairs"]))
        self.i_rated = p["rated_current"]
        self.w_rated = p["rated_speed"] / RPM
        self.Rm = p["m_rs"]
        self.Lm = p["m_ls"] * 1e-3
        self.lam_m = p["m_flux"] * 1e-3
        self.Jm = p["m_j"] * 1e-4
        self.kp_c = p["c_kp"]
        self.ki_c = p["c_ki"]
        self.dec = p["c_dec"] >= 0.5
        self.kp_s = p["s_kp"]
        self.ki_s = p["s_ki"]
        self.accel = p["accel"] / RPM
        self.decel = p["decel"] / RPM
        self.ilim = p["i_limit"]
        self.w_max = p["max_speed"] / RPM
        self.oc2 = p["oc_trip"] ** 2
        self.w_trip = p["os_trip"] / RPM
        self.vmax = p["vbus"] / SQRT3
        self.coast = p["stop_mode"] >= 0.5
        self.R = p["x_rs"]
        self.L = p["x_ls"] * 1e-3
        self.lam = p["x_flux"] * 1e-3
        self.J = p["x_j"] * 1e-4
        self.Bv = p["x_b"] * 1e-3 / (1000.0 / RPM)
        self.Tl = p["x_load"] * 1e-3
        self.noise = p["x_noise"] * 1e-3
        self.kt = 1.5 * self.pp * self.lam
        if self.target > self.w_max:
            self.target = self.w_max
        elif self.target < -self.w_max:
            self.target = -self.w_max

    def set_param(self, key, value):
        q = BY_KEY[key]
        value = round(min(q.hi, max(q.lo, float(value))), q.decimals)
        self.p[key] = value
        self.apply_params()
        return value

    def load_params(self, values):
        for key, value in values.items():
            if key in BY_KEY:
                q = BY_KEY[key]
                self.p[key] = round(min(q.hi, max(q.lo, float(value))), q.decimals)
        self.apply_params()

    # ---- commands

    def enable(self, mode, bypass=False):
        if self.fault:
            return False
        self.enabled = True
        self.mode = mode
        self.ramp_bypass = bypass
        self.stopping = False
        self.int_d = self.int_q = self.int_s = 0.0
        self.id_ref = self.iq_ref = 0.0
        self.vd_cmd = self.vq_cmd = 0.0
        self.ref = self.w_meas          # start the ramp from where the shaft is
        return True

    def disable(self):
        self.enabled = False
        self.stopping = False
        self.ilim_hit = self.vlim_hit = False

    def run(self):
        return self.enable(MODE_SPEED)

    def stop(self):
        if not self.enabled:
            return
        if self.coast or self.mode != MODE_SPEED:
            self.disable()
        else:
            self.stopping = True
            self.target = 0.0

    def set_target_rpm(self, rpm):
        w = rpm / RPM
        self.target = max(-self.w_max, min(self.w_max, w))

    def reset_fault(self):
        if not self.enabled:
            self.fault = None

    # ---- what the front end reads

    @property
    def rpm(self):
        return self.w_meas * RPM

    @property
    def ref_rpm(self):
        return self.ref * RPM

    @property
    def target_rpm(self):
        return self.target * RPM

    @property
    def torque_mnm(self):
        """The drive's own estimate: its flux value times measured i_q."""
        return 1.5 * self.pp * self.lam_m * self.iqm * 1000.0

    @property
    def vmag(self):
        return math.hypot(self.vd, self.vq)

    @property
    def power(self):
        return 1.5 * (self.vd * self.idm + self.vq * self.iqm)

    @property
    def freq_hz(self):
        return self.pp * abs(self.w_meas) / (2.0 * math.pi)

    @property
    def state(self):
        if self.fault:
            return "fault"
        if self.tuning:
            return "tuning"
        if self.stopping:
            return "stopping"
        return "run" if self.enabled else "ready"

    # ---- the simulation itself

    def advance(self, n):
        """Run n control periods."""
        if n <= 0:
            return
        dt = self.DT
        t_out = dt * self.OUTER
        gauss = self.rng.gauss
        sqrt = math.sqrt
        cos = math.cos
        sin = math.sin

        pp = self.pp
        R = self.R; L = self.L; lam = self.lam; J = self.J
        Bv = self.Bv; Tl = self.Tl; kt = self.kt
        noise = self.noise
        wnoise = (0.5 / RPM) if noise > 0.0 else 0.0
        Lm = self.Lm; lam_m = self.lam_m
        kp_c = self.kp_c; ki_dt = self.ki_c * dt; dec = self.dec
        kp_s = self.kp_s; ki_t = self.ki_s * t_out
        accel = self.accel; decel = self.decel
        ilim = self.ilim; vmax = self.vmax; vmax2 = vmax * vmax
        oc2 = self.oc2; w_trip = self.w_trip
        w_stop = 3.0 / RPM

        i_d = self.i_d; i_q = self.i_q; w = self.w; theta = self.theta
        idm = self.idm; iqm = self.iqm; w_meas = self.w_meas
        vd = self.vd; vq = self.vq
        int_d = self.int_d; int_q = self.int_q; int_s = self.int_s
        id_ref = self.id_ref; iq_ref = self.iq_ref
        ref = self.ref; target = self.target
        enabled = self.enabled; mode = self.mode
        bypass = self.ramp_bypass; stopping = self.stopping
        ilim_hit = self.ilim_hit; vlim_hit = self.vlim_hit
        div = self._div; rdiv = self._rdiv; ticks = self.ticks
        still = self._still
        fault = self.fault

        h_rpm = self.hist["rpm"]; h_ref = self.hist["ref"]
        h_iq = self.hist["iq"]; h_iqref = self.hist["iqref"]
        h_id = self.hist["id"]; h_ia = self.hist["ia"]; h_ib = self.hist["ib"]
        h_idx = self.h_idx; h_n = self.hist_n; h_count = self.h_count
        rec = self.REC; outer = self.OUTER

        for _ in range(n):
            # ---- what the sensors report at the start of this period
            if noise > 0.0:
                idm = i_d + noise * gauss(0.0, 1.0)
                iqm = i_q + noise * gauss(0.0, 1.0)
            else:
                idm = i_d
                iqm = i_q
            w_e = pp * w

            # ---- speed loop, every 10th period
            div += 1
            if div >= outer:
                div = 0
                ticks += 1
                w_meas = w + wnoise * gauss(0.0, 1.0) if wnoise else w
                if enabled and mode == 0:
                    if bypass:
                        ref = target
                    else:
                        # Away from zero uses the acceleration ramp, towards
                        # zero the deceleration ramp - so a reversal brakes
                        # first and then accelerates.
                        if ref * target < 0.0 or abs(target) < abs(ref):
                            lim = decel * t_out
                        else:
                            lim = accel * t_out
                        d = target - ref
                        if d > lim:
                            d = lim
                        elif d < -lim:
                            d = -lim
                        ref += d
                    e = ref - w_meas
                    u = kp_s * e + int_s
                    if u > ilim:
                        u = ilim
                        ilim_hit = True
                        if e < 0.0:
                            int_s += ki_t * e
                    elif u < -ilim:
                        u = -ilim
                        ilim_hit = True
                        if e > 0.0:
                            int_s += ki_t * e
                    else:
                        ilim_hit = False
                        int_s += ki_t * e
                    iq_ref = u
                    id_ref = 0.0
                    # A ramp stop holds zero speed until the shaft has
                    # really been at rest for 150 ms, and only then lets go.
                    # Switching off the moment the speed first dips under
                    # the threshold left it coasting at a few rpm.
                    if stopping and target == 0.0 and ref == 0.0:
                        still = still + 1 if abs(w_meas) < w_stop else 0
                        if still >= 150:
                            enabled = False
                            stopping = False
                            still = 0
                    else:
                        still = 0
                else:
                    ref = w_meas if enabled else 0.0
                    ilim_hit = False

            # ---- current loop
            if enabled:
                if mode == 2:
                    vd = self.vd_cmd
                    vq = self.vq_cmd
                    m2 = vd * vd + vq * vq
                    if m2 > vmax2:
                        s = vmax / sqrt(m2)
                        vd *= s
                        vq *= s
                else:
                    ed = id_ref - idm
                    eq = iq_ref - iqm
                    vd = kp_c * ed + int_d
                    vq = kp_c * eq + int_q
                    if dec:
                        vd -= w_e * Lm * iqm
                        vq += w_e * (Lm * idm + lam_m)
                    m2 = vd * vd + vq * vq
                    if m2 > vmax2:
                        s = vmax / sqrt(m2)
                        vd *= s
                        vq *= s
                        vlim_hit = True
                    else:
                        vlim_hit = False
                        int_d += ki_dt * ed
                        int_q += ki_dt * eq

                # ---- motor, electrical: midpoint rule over one period
                k1d = (vd - R * i_d + w_e * L * i_q) / L
                k1q = (vq - R * i_q - w_e * (L * i_d + lam)) / L
                md = i_d + 0.5 * dt * k1d
                mq = i_q + 0.5 * dt * k1q
                i_d += dt * (vd - R * md + w_e * L * mq) / L
                i_q += dt * (vq - R * mq - w_e * (L * md + lam)) / L
                te = kt * mq
            else:
                # Bridge off: the phases are open, so no current and no torque
                vd = vq = 0.0
                i_d = i_q = 0.0
                int_d = int_q = int_s = 0.0
                iq_ref = id_ref = 0.0
                te = 0.0

            # ---- motor, mechanical. The load is a brake: it can hold the
            #      shaft still and slow it down, never push it backwards.
            net = te - Bv * w
            if w == 0.0:
                if net > Tl:
                    w = dt * (net - Tl) / J
                elif net < -Tl:
                    w = dt * (net + Tl) / J
            else:
                w_new = w + dt * (net - (Tl if w > 0.0 else -Tl)) / J
                w = 0.0 if w_new * w < 0.0 else w_new
            theta += pp * w * dt
            if theta > 6.283185307179586:
                theta -= 6.283185307179586
            elif theta < 0.0:
                theta += 6.283185307179586

            # ---- protection
            if enabled:
                if i_d * i_d + i_q * i_q > oc2:
                    fault = "over-current"
                elif w > w_trip or w < -w_trip:
                    fault = "over-speed"
                if fault:
                    enabled = False
                    stopping = False
                    ilim_hit = vlim_hit = False

            # ---- history, 5 kHz
            rdiv += 1
            if rdiv >= rec:
                rdiv = 0
                c = cos(theta)
                s = sin(theta)
                ia = idm * c - iqm * s
                ibeta = idm * s + iqm * c
                h_rpm[h_idx] = w_meas * RPM
                h_ref[h_idx] = ref * RPM
                h_iq[h_idx] = iqm
                h_iqref[h_idx] = iq_ref
                h_id[h_idx] = idm
                h_ia[h_idx] = ia
                h_ib[h_idx] = -0.5 * ia + 0.8660254037844386 * ibeta
                h_idx += 1
                if h_idx >= h_n:
                    h_idx = 0
                if h_count < h_n:
                    h_count += 1

        self.t += n * dt
        self.i_d = i_d; self.i_q = i_q; self.w = w; self.theta = theta
        self.idm = idm; self.iqm = iqm; self.w_meas = w_meas
        self.vd = vd; self.vq = vq
        self.int_d = int_d; self.int_q = int_q; self.int_s = int_s
        self.id_ref = id_ref; self.iq_ref = iq_ref
        self.ref = ref
        self.enabled = enabled; self.stopping = stopping
        self.ilim_hit = ilim_hit; self.vlim_hit = vlim_hit
        self._div = div; self._rdiv = rdiv; self.ticks = ticks
        self._still = still
        self.fault = fault
        self.h_idx = h_idx; self.h_count = h_count

    def window(self, channel, seconds, max_points=1000):
        """The last `seconds` of one channel, oldest first, thinned to at
        most about max_points values."""
        n = min(self.h_count, int(seconds * self.rec_hz))
        if n <= 0:
            return []
        a = self.hist[channel]
        start = self.h_idx - n
        seg = a[start:] + a[:self.h_idx] if start < 0 else a[start:self.h_idx]
        step = max(1, n // max_points)
        return seg[::step] if step > 1 else seg


# ---------------------------------------------------------------- metrics ---

def step_metrics(resp, step):
    """Overshoot in percent and the 5 % settling time in ms, from a speed
    step response sampled at 1 kHz."""
    if not resp:
        return {"overshoot": 0.0, "settle_ms": None}
    peak = max(resp)
    band = 0.05 * abs(step)
    settle = 0
    for i, v in enumerate(resp):
        if abs(v - step) > band:
            settle = i + 1
    return {
        "overshoot": max(0.0, (peak - step) / step * 100.0),
        "settle_ms": settle if settle < len(resp) else None,
    }


def _slope(ts, ys):
    n = len(ts)
    mt = sum(ts) / n
    my = sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den


# -------------------------------------------------------------- auto tune ---

class TuneError(Exception):
    pass


class AutoTune:
    """
    Identifies the motor by measurement and designs both controllers from
    what it found. Seven steps, each a thing a real drive can do with only
    its own current sensors, its speed feedback and its own output voltage:

      baseline    a small speed step with the present gains, for comparison
      resistance  two dc current levels on the d axis; R = dV / dI
      inductance  a voltage step on the d axis; L = R x time constant
      flux        hold a constant speed; lambda = (vq - R iq) / w_e
      inertia     accelerate with a known torque, then coast;
                  J = torque / (acceleration - coasting deceleration)
      gains       current PI from R, L and a bandwidth; speed PI from J
      verify      the same speed step again, with the new gains

    The drive's parameters are changed as it goes, because later steps need
    the earlier results. Nothing is final until apply(): discard(), abort()
    and any failure put every parameter back.
    """

    STEP_RPM = 200.0
    EXPECTED_SECONDS = 6.3

    STEPS = (
        ("baseline", "baseline response", "speed step with the present gains"),
        ("resistance", "stator resistance", "two dc current levels"),
        ("inductance", "stator inductance", "voltage step, time constant"),
        ("flux", "flux linkage", "constant speed, back-emf"),
        ("inertia", "inertia", "accelerate, then coast"),
        ("gains", "controller gains", "from the bandwidth targets"),
        ("verify", "verification", "speed step with the new gains"),
    )

    def __init__(self, drive):
        self.d = drive
        self.active = False
        self.pending = False        # finished, waiting for apply or discard
        self.message = "stop the drive, then start."
        self.steps = []
        self.result = {}
        self.resp_before = []
        self.resp_after = []
        self.progress = 0.0
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
        d = self.d
        if self.active:
            return False
        if d.fault:
            self.message = "reset the fault first."
            return False
        if d.enabled:
            self.message = "stop the drive first."
            return False
        if abs(d.rpm) > 5.0:
            self.message = "wait for the shaft to come to rest."
            return False
        if self.pending:
            self.discard()
        self._snapshot = dict(d.p)
        self._reset_steps()
        self.result = {}
        self.resp_before = []
        self.resp_after = []
        self.progress = 0.0
        self._t0 = d.t
        self._gen = self._procedure()
        self.active = True
        d.tuning = True
        self.message = "identifying. keep the shaft free to turn."
        return True

    def advance(self, n):
        d = self.d
        try:
            for _ in range(n):
                next(self._gen)
            self.progress = min(0.98, (d.t - self._t0) / self.EXPECTED_SECONDS)
        except StopIteration:
            self._finish()
        except TuneError as err:
            self._fail("failed: " + str(err))

    def _finish(self):
        self.active = False
        self.pending = True
        self.progress = 1.0
        self.d.disable()
        self.d.tuning = False
        self.message = "finished. apply to keep the new values, or discard."

    def _restore(self):
        d = self.d
        d.disable()
        d.tuning = False
        if self._snapshot is not None:
            d.p = dict(self._snapshot)
            d.apply_params()
        d.target = 0.0

    def _fail(self, why):
        for s in self.steps:
            if s["state"] == "running":
                s["state"] = "failed"
        self.active = False
        self.pending = False
        self._restore()
        self.message = why + " nothing was changed."

    def abort(self):
        if self.active:
            self._fail("aborted.")

    def apply(self):
        if self.pending:
            self.pending = False
            self.message = "applied. the new values are in groups 1, 2 and 3."

    def discard(self):
        if self.pending:
            self.pending = False
            self._restore()
            self.message = "discarded. the previous values are back."

    # ---- building blocks

    def _run(self, seconds, each=None):
        d = self.d
        for _ in range(max(1, int(round(seconds / d.DT)))):
            d.advance(1)
            if d.fault:
                raise TuneError("the drive tripped on " + d.fault + ".")
            if each:
                each()
            yield

    def _wait_stop(self, timeout):
        d = self.d
        still = 0
        for _ in range(int(timeout / d.DT)):
            d.advance(1)
            if d.fault:
                raise TuneError("the drive tripped on " + d.fault + ".")
            still = still + 1 if abs(d.rpm) < 3.0 else 0
            if still > 500:
                return
            yield
        raise TuneError("the shaft did not come to rest.")

    def _step_test(self):
        d = self.d
        d.enable(MODE_SPEED, bypass=True)
        d.target = 0.0
        yield from self._run(0.05)
        d.target = self.STEP_RPM / RPM
        resp = []
        last = [d.ticks]

        def each():
            if d.ticks != last[0]:
                last[0] = d.ticks
                resp.append(d.rpm)

        yield from self._run(0.6, each)
        d.target = 0.0
        yield from self._wait_stop(3.0)
        d.disable()
        yield from self._run(0.02)
        return resp

    def _begin(self, key):
        self._step(key)["state"] = "running"

    def _done(self, key, value):
        s = self._step(key)
        s["state"] = "done"
        s["value"] = value

    # ---- the procedure

    def _procedure(self):
        d = self.d
        res = self.result
        snap = self._snapshot
        i_n = d.i_rated
        res["before"] = {k: snap[k] for k in
                         ("m_rs", "m_ls", "m_flux", "m_j",
                          "c_kp", "c_ki", "s_kp", "s_ki")}

        # 0 --- baseline
        self._begin("baseline")
        self.resp_before = yield from self._step_test()
        m = step_metrics(self.resp_before, self.STEP_RPM)
        res["metrics_before"] = m
        self._done("baseline", f"{m['overshoot']:.0f} % overshoot")

        # 1 --- resistance: two dc levels on the d axis. No torque, so the
        #       shaft stays put, and at standstill there is no back-EMF or
        #       cross-coupling in the voltage - it is all R.
        self._begin("resistance")
        d.enable(MODE_CURRENT)
        pts = []
        for frac in (0.3, 0.6):
            d.id_ref = frac * i_n
            d.iq_ref = 0.0
            yield from self._run(0.12)
            acc = [0.0, 0.0, 0]

            def each():
                acc[0] += d.vd
                acc[1] += d.idm
                acc[2] += 1

            yield from self._run(0.10, each)
            v = acc[0] / acc[2]
            i = acc[1] / acc[2]
            if abs(i - frac * i_n) > 0.1 * frac * i_n:
                raise TuneError("the current did not settle on its reference.")
            pts.append((v, i))
        (v1, i1), (v2, i2) = pts
        r_est = (v2 - v1) / (i2 - i1)
        if not 0.005 < r_est < 50.0:
            raise TuneError("the resistance measurement makes no sense.")
        res["m_rs"] = r_est
        self._done("resistance", f"{r_est:.3f} Ω")

        # 2 --- inductance: a voltage step straight onto the d axis, with no
        #       controller in the way. The current rises as a first-order
        #       lag, and the area between it and its final value is the time
        #       constant times that final value.
        self._begin("inductance")
        d.enable(MODE_VOLTAGE)
        v_step = r_est * 0.5 * i_n

        def pulse(length):
            d.vd_cmd = 0.0
            yield from self._run(max(0.01, length))
            d.vd_cmd = v_step
            samples = []
            yield from self._run(length, lambda: samples.append(d.idm))
            return samples

        def tau_of(samples):
            n = len(samples)
            tail = samples[-max(10, n // 10):]
            i_f = sum(tail) / len(tail)
            if i_f <= 0.0:
                raise TuneError("no current flowed during the voltage step.")
            area = sum(i_f - s for s in samples)
            area -= 0.5 * (i_f - samples[0]) + 0.5 * (i_f - samples[-1])
            return area * d.DT / i_f, i_f

        length = 0.012
        while True:
            samples = yield from pulse(length)
            tau, i_f = tau_of(samples)
            if i_f >= 0.995 * v_step / r_est or length >= 0.8:
                break
            length *= 2.0              # not settled: the motor is slower
        taus = [tau]
        for _ in range(3):
            samples = yield from pulse(length)
            taus.append(tau_of(samples)[0])
        d.vd_cmd = 0.0
        yield from self._run(max(0.01, length))
        l_est = r_est * sum(taus) / len(taus)
        if not 1e-6 < l_est < 0.1:
            raise TuneError("the inductance measurement makes no sense.")
        res["m_ls"] = l_est * 1e3
        self._done("inductance", f"{l_est * 1e3:.3f} mH")

        # The current loop can be designed now, and the remaining steps
        # need it: Kp = L wc and Ki = R wc put the controller's zero on the
        # motor's pole, leaving a first-order loop of bandwidth wc.
        wc = 2.0 * math.pi * d.p["c_bw"]
        d.set_param("m_rs", r_est)
        d.set_param("m_ls", l_est * 1e3)
        d.set_param("c_kp", l_est * wc)
        d.set_param("c_ki", r_est * wc)

        # 3 --- flux: at a steady speed with i_d = 0 the q-axis voltage is
        #       R i_q plus the back-EMF, w_e times the flux linkage.
        self._begin("flux")
        w_test = 0.4 * d.w_rated
        d.enable(MODE_SPEED)
        d.target = w_test
        recent = []
        last = d.ticks
        settled = False
        for k in range(int(6.0 / d.DT)):
            d.advance(1)
            if d.fault:
                raise TuneError("the drive tripped on " + d.fault + ".")
            if d.ticks != last:
                last = d.ticks
                recent.append(d.w_meas)
                if len(recent) > 200:
                    recent.pop(0)
                    if (d.ref == d.target and
                            max(recent) - min(recent) < 0.02 * w_test and
                            abs(recent[-1] - w_test) < 0.02 * w_test):
                        settled = True
                        break
            yield
        if not settled:
            raise TuneError("the speed did not settle - is the load too high?")
        acc = [0.0, 0.0, 0.0, 0]

        def each_flux():
            acc[0] += d.vq
            acc[1] += d.iqm
            acc[2] += d.pp * d.w_meas
            acc[3] += 1

        yield from self._run(0.3, each_flux)
        vq = acc[0] / acc[3]
        iq = acc[1] / acc[3]
        we = acc[2] / acc[3]
        lam_est = (vq - r_est * iq) / we
        if not 1e-4 < lam_est < 1.0:
            raise TuneError("the flux measurement makes no sense.")
        res["m_flux"] = lam_est * 1e3
        d.set_param("m_flux", lam_est * 1e3)
        kt_est = 1.5 * d.pp * lam_est
        self._done("flux", f"{lam_est * 1e3:.2f} mWb")

        # 4 --- inertia: a known torque accelerates the shaft, then no
        #       torque lets it coast. Friction and load act in both phases,
        #       so subtracting the two slopes removes them.
        self._begin("inertia")
        w0 = d.w_meas
        i_test = 0.5 * i_n
        d.mode = MODE_CURRENT
        d.id_ref = 0.0
        d.iq_ref = i_test
        yield from self._run(0.004)
        ts, ws, iqs = [], [], []
        last = d.ticks
        t_begin = d.t
        while d.w_meas < 1.5 * w0 or len(ts) < 30:
            d.advance(1)
            if d.fault:
                raise TuneError("the drive tripped on " + d.fault + ".")
            if d.ticks != last:
                last = d.ticks
                ts.append(d.t)
                ws.append(d.w_meas)
                iqs.append(d.iqm)
            if d.t - t_begin > 2.0:
                raise TuneError("the shaft would not accelerate - "
                                "is the load too high?")
            yield
        a_run = _slope(ts, ws)
        i_run = sum(iqs) / len(iqs)

        d.iq_ref = 0.0
        yield from self._run(0.004)
        ts, ws = [], []
        last = [d.ticks]

        def each_coast():
            if d.ticks != last[0]:
                last[0] = d.ticks
                ts.append(d.t)
                ws.append(d.w_meas)

        yield from self._run(0.25, each_coast)
        a_coast = _slope(ts, ws)
        if a_run - a_coast <= 1.0:
            raise TuneError("no usable acceleration was measured.")
        j_est = kt_est * i_run / (a_run - a_coast)
        if not 1e-8 < j_est < 1.0:
            raise TuneError("the inertia measurement makes no sense.")
        res["m_j"] = j_est * 1e4
        d.set_param("m_j", j_est * 1e4)
        self._done("inertia", f"{j_est * 1e4:.3f} kg·cm²")

        # 5 --- speed controller. With the plant taken as Kt / (J s), a PI
        #       gives s^2 + ws s + ws wi; wi = ws / 5 is a damping of 1.1.
        self._begin("gains")
        ws_bw = 2.0 * math.pi * d.p["s_bw"]
        kp_s = j_est * ws_bw / kt_est
        d.set_param("s_kp", kp_s)
        d.set_param("s_ki", kp_s * ws_bw / 5.0)
        res["after"] = {k: d.p[k] for k in ("c_kp", "c_ki", "s_kp", "s_ki")}
        d.enable(MODE_SPEED)
        d.target = 0.0
        yield from self._wait_stop(8.0)
        d.disable()
        yield from self._run(0.05)
        self._done("gains", f"{d.p['c_bw']:.0f} Hz / {d.p['s_bw']:.0f} Hz")

        # 6 --- verify
        self._begin("verify")
        self.resp_after = yield from self._step_test()
        m = step_metrics(self.resp_after, self.STEP_RPM)
        res["metrics_after"] = m
        self._done("verify", f"{m['overshoot']:.0f} % overshoot")


# ------------------------------------------------------------- self-check ---

def selfcheck():
    """Exercise the model end to end. Returns (name, passed, detail) rows."""
    rows = []

    def check(name, ok, detail):
        rows.append((name, bool(ok), detail))

    d = Drive(seed=7)
    d.run()
    d.set_target_rpm(1500)
    d.advance(2500)
    mid = d.rpm
    d.advance(12500)
    check("ramp reaches the setpoint", abs(d.rpm - 1500) < 8,
          f"{d.rpm:.1f} rpm after 1.5 s, {mid:.0f} rpm at 0.25 s")
    check("ramp limits the acceleration", 500 < mid < 900,
          f"{mid:.0f} rpm at 0.25 s on a 3000 rpm/s ramp")

    d.set_target_rpm(-1500)
    d.advance(20000)
    check("reverses through zero", abs(d.rpm + 1500) < 8, f"{d.rpm:.1f} rpm")

    d.set_param("x_load", 150)
    d.advance(15000)
    expect = (0.150 + d.Bv * abs(d.w)) / d.kt
    check("holds speed against a load", abs(d.rpm + 1500) < 8,
          f"{d.rpm:.1f} rpm with 150 mNm on the shaft")
    check("load shows up as torque current", abs(abs(d.i_q) - expect) < 0.15,
          f"i_q {d.i_q:.2f} A, {expect:.2f} A expected")

    d.set_param("stop_mode", 1)
    d.stop()
    d.advance(30000)
    check("a brake cannot drive the shaft", d.w == 0.0 and not d.enabled,
          f"{d.w * RPM:.2f} rpm after coasting against the load")

    d = Drive(seed=7)
    d.set_param("os_trip", 1000)
    d.run()
    d.set_target_rpm(1500)
    d.advance(10000)
    check("over-speed trips the drive", d.fault == "over-speed" and not d.enabled,
          f"fault {d.fault}, at {d.rpm:.0f} rpm")

    d = Drive(seed=7)
    tune = AutoTune(d)
    started = tune.start()
    guard = 0
    while tune.active and guard < 400:
        tune.advance(1000)
        guard += 1
    ok = started and tune.pending
    check("auto tune completes", ok, tune.message)
    if ok:
        r = tune.result
        for key, plant, unit in (("m_rs", "x_rs", "ohm"), ("m_ls", "x_ls", "mH"),
                                 ("m_flux", "x_flux", "mWb"), ("m_j", "x_j", "kg cm2")):
            err = (r[key] - d.p[plant]) / d.p[plant] * 100.0
            check(f"identifies {BY_KEY[key].name}", abs(err) < 3.0,
                  f"{r[key]:.4f} {unit} against {d.p[plant]:.4f}, {err:+.2f} %")
        mb = r["metrics_before"]
        ma = r["metrics_after"]
        check("tuning removes the overshoot",
              ma["overshoot"] < 15.0 and ma["overshoot"] < mb["overshoot"] / 2,
              f"{mb['overshoot']:.1f} % before, {ma['overshoot']:.1f} % after")
        check("tuning shortens the settling time",
              ma["settle_ms"] is not None and
              (mb["settle_ms"] is None or ma["settle_ms"] < mb["settle_ms"]),
              f"{mb['settle_ms']} ms before, {ma['settle_ms']} ms after")
        before = dict(d.p)
        tune.discard()
        check("discard restores every parameter",
              d.p == tune._snapshot and d.p != before,
              "parameters equal the snapshot taken at start")

    d = Drive(seed=7)
    d.run()
    d.set_target_rpm(2000)
    t0 = time.perf_counter()
    d.advance(20000)
    el = time.perf_counter() - t0
    check("runs faster than real time", el < 1.0,
          f"2.0 s of motor in {el:.2f} s, {2.0 / el:.1f}x real time")
    return rows


if __name__ == "__main__":
    failed = 0
    for name, ok, detail in selfcheck():
        print(("ok    " if ok else "FAIL  ") + f"{name:38s} {detail}")
        failed += not ok
    raise SystemExit(1 if failed else 0)

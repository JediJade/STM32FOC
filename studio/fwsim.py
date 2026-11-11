"""
The STM32FOC firmware's control arithmetic and console, in Python.

This is a stand-in for the board, for when there is none on the desk: it
answers the console the way console.c (version 3) does, and behind the
console it runs the same integer arithmetic as pi.c, loop.c, outer.c,
vmotor.c, sensor.c, drive.c, oneline.c and capture.c, one 50 us period at a
time, in real time.

What is the same as the firmware: every gain, limit and shift; the Q15 and
Q30 formats; the order things happen in within a period; the 20:1 ratio
between the loops; the console's commands and the exact text of its replies.

What is left out: Clarke, Park and SVPWM through the firmware's 256-entry
sine table. While the loop runs on the encoder the rotor-frame voltage goes
to the model directly; with the Hall sensors or the observer selected the
frames are rotated with floating-point sines, because then the angle the
loop uses and the rotor's own are different things and the difference is
the point. There are no faults to trip, no watchdog and no self-tests to
run, so `t` reports a board on which everything passed. There is no CAN
controller either: `can on` makes this port fall silent for as long as the
board would be away, and nothing more.

It is a test double, not evidence. A result from here says the host side and
the arithmetic agree with each other; only the board says they are right.
"""

import math
import time

import protocols

OUT_MAX = 29491
SPEED_MAX = 6560
CAPTURE_LEN = 256
OUTER_DIVIDER = 20
CONSOLE_VERSION = 3

# The virtual motor's constants: a, ke, accel, friction, load. Preset 0 is the
# gimbal motor the firmware was built around; preset 1 is shaped like a hub
# motor - five times the electrical time constant, five times the back-EMF,
# a heavier rotor with rolling resistance instead of viscous drag.
PRESETS = ((1638, 20, 336, 30, 0), (328, 100, 40, 1, 1))
VM_HALL_PHASE = 4000            # where the Hall sensors sit; the loop is not told

LOOP_IDLE, LOOP_ALIGN, LOOP_CURRENT, LOOP_VOLTAGE = 0, 1, 2, 3
OUTER_OFF, OUTER_SPEED, OUTER_POSITION = 0, 1, 2
CAP_IDLE, CAP_ARMED, CAP_RUNNING, CAP_DONE = 0, 1, 2, 3
CAP_NAMES = ("id", "iq", "vd", "vq", "spd", "ang", "ol", "hall", "aerr", "idfw")
CAP_OL = 6

SENS_ENCODER, SENS_HALL, SENS_OBSERVER = 0, 1, 2
VEH_OFF, VEH_WAIT, VEH_RUN, VEH_BRAKE, VEH_FAULT = 0, 1, 2, 3, 4
PH_NONE, PH_HALL, PH_OPEN, PH_CLOSED, PH_CAL = 0, 1, 2, 3, 4

LOOP_ALIGN_V = 6000
HALL_TIMEOUT = 4000             # periods without an edge: call it standing still
CAL_W = 8 << 8                  # the calibration field turns 8 units a period
CAL_TURN = 98304 << 8           # one and a half turns each way
PLL_KP = 4
PLL_KI = 10
BRAKE_FADE = 300                # regen fades out below this speed, units per ms
SPEED_REF = 6560                # the speed ccross and cemf are stated at

PARAMS = (          # name, default, lo, hi - as console.c has them
    ("ckp", 6000, 0, 262144),
    ("cki", 900, 0, 32767),
    ("cdec", 1, 0, 2),
    ("ccross", 20580, 0, 65535),
    ("cemf", 6554, 0, 131071),
    ("skp", 32768, 0, 1048576),
    ("ski", 6144, 0, 131072),
    ("simax", 16384, 256, 29491),
    ("pkp", 328, 0, 32768),
    ("vlim", 29491, 4096, 29491),
    ("fw", 0, 0, 16384),
    ("fwstart", 29491, 16384, 32767),
    ("sens", 0, 0, 2),
    ("hsrc", 0, 0, 1),
    ("hwid", 32768, 22000, 43000),
    ("sla", 1638, 128, 32767),
    ("slmin", 600, 100, 3000),
    ("slcur", 4000, 0, 16384),
    ("slramp", 2000, 100, 20000),
    ("insrc", 0, 0, 1),
    ("thmin", 900, 0, 4095),
    ("thmax", 3300, 0, 4095),
    ("thmode", 0, 0, 1),
    ("thramp", 66, 1, 32767),
    ("drvspd", 6560, 100, 16000),
    ("regen", 4096, 0, 16384),
    ("mot", 0, 0, 1),
    ("ol", 0, 0, 1),
    ("olt", 10, 5, 40),
    ("canid", 1, 1, 127),
    ("canbr", 500, 125, 1000),
    ("canper", 20, 5, 1000),
    ("canback", 10, 0, 600),
)
PARAM_INDEX = {name: i for i, (name, *_rest) in enumerate(PARAMS)}

K_ANGLE = 2.0 * math.pi / 65536.0


def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _tdiv(a, b):
    """Integer division the way C does it: towards zero."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _sign(v):
    return (v > 0) - (v < 0)


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


def hall_code(theta, width):
    """What three Hall switches 120 degrees apart read at rotor angle theta.
    width is how much of a turn each one stays on: 32768 is a latch's half
    turn, less is what a unipolar switch like the 3144 gives."""
    code = 0
    for k in range(3):
        if ((theta + VM_HALL_PHASE - k * 21845) & 0xFFFF) < width:
            code |= 1 << k
    return code


_parked = None      # an emulator that has gone to CAN and will come back


def open_emulator():
    """The emulator port. After `can on` it is the same board coming back,
    parameters and all, not a new one."""
    global _parked
    if _parked is not None:
        fw = _parked
        if fw.away_until > time.perf_counter():
            raise OSError("cannot open emulator: the port does not exist")
        _parked = None
        fw.closed = False
        fw.away_until = 0.0
        fw._wall = time.perf_counter()
        return fw
    return FwSim()


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
        self.vm = PRESETS[0]
        # loop
        self.loop = LOOP_IDLE
        self.target_q = self.target_d = 0
        self.v_cmd = 0
        self.int_d = self.int_q = 0
        self.last_d = self.last_q = 0
        self.last_vd = self.last_vq = 0
        self.fw_d = 0
        self.q_boost = 0
        self.cross_run = self.p["ccross"]
        self.emf_run = self.p["cemf"]
        self.ext = False            # the loop's angle comes from sensor code
        self.ang = self.theta       # the angle the loop used last
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
        # sensor: Hall
        self.h_valid = False
        self.h_edge = [0] * 8
        self.h_wid = [0] * 8
        self.h_next = [0] * 8
        self.h_code = 0
        self.h_cnt = 0
        self.h_cnt1 = 0
        self.h_wid1 = 0
        self.h_dir = 0
        self.h_ang_fx = 0
        self.h_spd_fx = 0
        self.h_rep_fx = 0
        self.h_trav = 0
        self.h_lim = 0
        self.h_err = 0
        # sensor: calibration
        self.cal = 0
        self.cal_result = 0
        self.cal_ms = 0
        self.cal_w_fx = 0
        self.cal_trav = 0
        self.phi_fx = 0
        self.f_edge = [0] * 8
        self.r_edge = [0] * 8
        self.f_next = [0] * 8
        self.f_mask = self.r_mask = 0
        # sensor: observer
        self.sl_closed = False
        self.wol_fx = 0
        self.pll_th_fx = 0
        self.pll_w_fx = 0
        self.pll_dir = 1
        self.sl_lock = 0
        self.pia = self.pib = self.pva = self.pvb = 0
        self.inva = (32768 << 8) // self.p["sla"]
        self.knorm = 0
        self._derive()
        # drive
        self.veh = VEH_OFF
        self.vflt = 0
        self.th_sim = self.p["thmin"]
        self.brk_sim = 0
        self.th_raw = self.th_sim
        self.brake = 0
        self.th_f = 0
        self.th_bad = 0
        self.limit = 0
        self.idfw_fx = 0
        # one line
        self.ol_runs = []
        self.ol_i = 0
        self.ol_left = 0
        self.ol_level = 0
        self.ol_run = 0
        self.ol_frame = [0] * protocols.OL_BYTES
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
        self.away_until = 0.0

    def _derive(self):
        """Numbers worked out from parameters once, not every period."""
        p = self.p
        self.inva = (32768 << 8) // p["sla"]
        cemf = max(p["cemf"], 64)
        self.knorm = min(32767, (328 << 15) // cemf)

    # ---- the control interrupt, n periods of it

    def run(self, n):
        p = self.p
        kp_c = p["ckp"]; ki_c = p["cki"]
        i_d = self.i_d; i_q = self.i_q; omega_fx = self.omega_fx; theta = self.theta
        int_d = self.int_d; int_q = self.int_q
        div = self.div
        vd = self.last_vd; vq = self.last_vq
        md = self.last_d; mq = self.last_q
        ang = self.ang
        reload = True

        for _ in range(n):
            if reload:
                reload = False
                vm_a, vm_ke, vm_accel, vm_fric, vm_load = self.vm
                loop = self.loop; v_cmd = self.v_cmd
                tq = self.target_q + self.q_boost
                td = self.target_d + self.fw_d
                dec = p["cdec"]; cross = self.cross_run; emf = self.emf_run
                vlim = p["vlim"]; imax_c = vlim << 15
                ext = self.ext
                ol_on = p["ol"]

            # ---- foc_loop_step: measure, control
            if ext:
                ang = self._sensor_angle()
                dl = ((theta - ang) & 0xFFFF) * K_ANGLE
                cs = math.cos(dl); sn = math.sin(dl)
                md = int(round(i_d * cs - i_q * sn))
                mq = int(round(i_d * sn + i_q * cs))
                th0 = theta * K_ANGLE
                ia = int(round(i_d * math.cos(th0) - i_q * math.sin(th0)))
                ib = int(round(i_d * math.sin(th0) + i_q * math.cos(th0)))
            else:
                ang = theta
                md = i_d
                mq = i_q
            if loop == LOOP_CURRENT:
                e = td - md
                int_d += ki_c * e
                if int_d > imax_c:
                    int_d = imax_c
                elif int_d < -imax_c:
                    int_d = -imax_c
                vd = ((kp_c * e) >> 15) + (int_d >> 15)
                if vd > vlim:
                    vd = vlim
                elif vd < -vlim:
                    vd = -vlim
                e = tq - mq
                int_q += ki_c * e
                if int_q > imax_c:
                    int_q = imax_c
                elif int_q < -imax_c:
                    int_q = -imax_c
                vq = ((kp_c * e) >> 15) + (int_q >> 15)
                if vq > vlim:
                    vq = vlim
                elif vq < -vlim:
                    vq = -vlim
                if dec:
                    vd -= (mq * cross) >> 15
                    vq += (md * cross) >> 15
                    vq += emf
                if vd > vlim:
                    vd = vlim
                elif vd < -vlim:
                    vd = -vlim
                if vq > vlim:
                    vq = vlim
                elif vq < -vlim:
                    vq = -vlim
            elif loop == LOOP_VOLTAGE:
                vd = v_cmd
                vq = 0
            else:
                vd = vq = 0

            # ---- vmotor_step, in the rotor's own frame
            if ext:
                vtd = int(round(vd * cs + vq * sn))
                vtq = int(round(-vd * sn + vq * cs))
            else:
                vtd = vd
                vtq = vq
            omega_step = omega_fx >> 8
            c = (omega_step * 3217) >> 10
            d_new = i_d + ((vm_a * (vtd - i_d)) >> 15) + ((c * i_q) >> 15)
            q_new = (i_q + ((vm_a * (vtq - i_q)) >> 15) - ((c * i_d) >> 15)
                     - ((vm_a * (vm_ke * omega_step)) >> 15))
            i_d = 32767 if d_new > 32767 else (-32768 if d_new < -32768 else d_new)
            i_q = 32767 if q_new > 32767 else (-32768 if q_new < -32768 else q_new)
            omega_fx += (vm_accel * i_q) >> 15
            omega_fx -= (vm_fric * omega_fx) >> 15
            if vm_load:
                if omega_fx > vm_load:
                    omega_fx -= vm_load
                elif omega_fx < -vm_load:
                    omega_fx += vm_load
                else:
                    omega_fx = 0
            theta = (theta + (omega_fx >> 8)) & 0xFFFF

            if ext:
                a0 = ang * K_ANGLE
                self.theta = theta
                self._sensor_update(ia, ib,
                                    int(round(vd * math.cos(a0) - vq * math.sin(a0))),
                                    int(round(vd * math.sin(a0) + vq * math.cos(a0))))

            # ---- outer_tick and drive_tick, every 20th period
            div += 1
            if div >= OUTER_DIVIDER:
                div = 0
                self.i_d = i_d; self.i_q = i_q; self.omega_fx = omega_fx
                self.theta = theta; self.ang = ang
                self.last_d = md; self.last_q = mq
                self.last_vd = vd; self.last_vq = vq
                self.int_d = int_d; self.int_q = int_q
                self._tick1k()
                int_d = self.int_d; int_q = self.int_q
                reload = True

            # ---- capture_tick, oneline_tick
            if self.cap_state == CAP_RUNNING and self.cap_ch != CAP_OL:
                self._capture(md, mq, vd, vq, ang, theta)
            if ol_on:
                self._oneline_tick()

        self.i_d = i_d; self.i_q = i_q; self.omega_fx = omega_fx; self.theta = theta
        self.int_d = int_d; self.int_q = int_q
        self.div = div
        self.ang = ang
        self.last_d = md; self.last_q = mq
        live = self.loop in (LOOP_CURRENT, LOOP_VOLTAGE)
        self.last_vd = vd if live else 0
        self.last_vq = vq if live else 0
        self.t += n * self.DT

    def _capture(self, md, mq, vd, vq, ang, theta):
        if self.cap_phase:
            self.cap_phase -= 1
            return
        self.cap_phase = self.cap_div - 1
        ch = self.cap_ch
        if ch == 0:
            v = md
        elif ch == 1:
            v = mq
        elif ch == 2:
            v = vd
        elif ch == 3:
            v = vq
        elif ch == 4:
            v = _clamp(self.speed, -32768, 32767)
        elif ch == 5:
            v = _s16(ang)
        elif ch == 7:
            v = self._hall_now(theta)
        elif ch == 8:
            v = _s16(ang - theta + (self.omega_fx >> 8))
        else:
            v = self.fw_d
        self.cap.append(v)
        if len(self.cap) >= CAPTURE_LEN:
            self.cap_state = CAP_DONE

    # ---- sensor.c: the angle, when it does not come from the encoder

    def _hall_now(self, theta=None):
        if self.p["hsrc"]:
            return 7                # the pins, pulled up, nothing on them
        return hall_code(self.theta if theta is None else theta, self.p["hwid"])

    def _sensor_angle(self):
        if self.cal:
            return (self.phi_fx >> 8) & 0xFFFF
        if self.p["sens"] == SENS_HALL:
            return (self.h_ang_fx >> 8) & 0xFFFF
        if self.sl_closed:
            return ((self.pll_th_fx + self.pll_w_fx) >> 8) & 0xFFFF
        return (self.phi_fx >> 8) & 0xFFFF

    def _sensor_speed(self):
        """Angle units per ms, from whatever is supplying the angle."""
        if self.p["sens"] == SENS_HALL:
            return (self.h_rep_fx * 20) >> 8
        if self.sl_closed:
            return (self.pll_w_fx * 20) >> 8
        return (self.wol_fx * 20) >> 8

    def _hall_seed(self):
        """Start from the middle of whatever sector the rotor is in."""
        code = self._hall_now()
        self.h_code = code
        self.h_cnt = 0
        self.h_cnt1 = 0
        self.h_dir = 0
        self.h_spd_fx = 0
        self.h_rep_fx = 0
        self.h_trav = 0
        self.h_lim = 0
        self.h_ang_fx = ((self.h_edge[code] + (self.h_wid[code] >> 1)) & 0xFFFF) << 8

    def _sensor_update(self, ia, ib, va, vb):
        if self.cal:
            self.phi_fx = (self.phi_fx + self.cal_w_fx) & 0xFFFFFF
            self.cal_trav += abs(self.cal_w_fx)
            code = self._hall_now()
            if code != self.h_code:
                phi = (self.phi_fx >> 8) & 0xFFFF
                if self.cal == 2 and self.cal_w_fx > 0:
                    self.f_edge[code] = phi
                    self.f_next[self.h_code] = code
                    self.f_mask |= 1 << code
                elif self.cal == 4 and self.cal_w_fx < 0:
                    self.r_edge[self.h_code] = phi
                    self.r_mask |= 1 << self.h_code
                self.h_code = code
            return

        if self.p["sens"] == SENS_HALL:
            code = self._hall_now()
            if code != self.h_code:
                old = self.h_code
                if self.h_wid[code] == 0:
                    # a state calibration never saw: hold the angle, count it
                    self.h_err += 1
                    self.h_spd_fx = 0
                    self.h_rep_fx = 0
                    self.h_dir = 0
                    self.h_trav = 0
                    self.h_lim = 0
                    self.h_cnt1 = 0
                else:
                    d = 0
                    if self.h_wid[old] != 0:
                        if self.h_next[old] == code:
                            d = 1
                            boundary = self.h_edge[code]
                        elif self.h_next[code] == old:
                            d = -1
                            boundary = self.h_edge[old]
                        else:
                            self.h_err += 1
                    if d != 0 and d == self.h_dir and 0 < self.h_cnt < HALL_TIMEOUT:
                        # the last two sectors together: one alone is too
                        # few periods to time, when it is a narrow one
                        wid = self.h_wid[old]
                        cnt = self.h_cnt
                        # the speed loop gets the last sector alone: it
                        # minds delay more than ripple
                        self.h_rep_fx = d * ((wid << 8) // cnt)
                        if self.h_cnt1:
                            wid += self.h_wid1
                            cnt += self.h_cnt1
                        self.h_spd_fx = d * ((wid << 8) // cnt)
                        # the edge fell somewhere in the last period: half a
                        # period's travel past it, on average
                        self.h_ang_fx = ((boundary << 8) + (self.h_spd_fx >> 1)) & 0xFFFFFF
                        self.h_cnt1 = self.h_cnt
                        self.h_wid1 = self.h_wid[old]
                    else:
                        self.h_spd_fx = 0
                        self.h_rep_fx = 0
                        self.h_ang_fx = ((self.h_edge[code] + (self.h_wid[code] >> 1))
                                         & 0xFFFF) << 8
                        self.h_cnt1 = 0
                    self.h_trav = 0
                    self.h_lim = self.h_wid[code] << 8
                    self.h_dir = d
                self.h_code = code
                self.h_cnt = 0
            else:
                if self.h_cnt < 60000:
                    self.h_cnt += 1
                if self.h_spd_fx != 0:
                    if self.h_cnt > HALL_TIMEOUT:
                        # standing still: back to the middle of the sector
                        self.h_spd_fx = 0
                        self.h_rep_fx = 0
                        self.h_ang_fx = ((self.h_edge[self.h_code]
                                          + (self.h_wid[self.h_code] >> 1)) & 0xFFFF) << 8
                    else:
                        # overdue: at this speed the next edge should have
                        # come by now, so the rotor is slowing down
                        step = abs(self.h_spd_fx)
                        if self.h_cnt > self.h_lim // step:
                            self.h_spd_fx = self.h_dir * (self.h_lim // self.h_cnt)
                            self.h_rep_fx = self.h_spd_fx
                            step = abs(self.h_spd_fx)
                        if self.h_trav + step <= self.h_lim:
                            self.h_ang_fx = (self.h_ang_fx + self.h_spd_fx) & 0xFFFFFF
                            self.h_trav += step
            return

        # the observer: back-EMF from the last period's voltage and the
        # current it produced, then a phase-locked loop on its direction
        da = _clamp(ia - self.pia, -16383, 16383)
        db = _clamp(ib - self.pib, -16383, 16383)
        ea = _clamp(self.pva - self.pia - ((da * self.inva) >> 8), -32767, 32767)
        eb = _clamp(self.pvb - self.pib - ((db * self.inva) >> 8), -32767, 32767)
        self.pia = ia; self.pib = ib; self.pva = va; self.pvb = vb
        th = ((self.pll_th_fx >> 8) & 0xFFFF) * K_ANGLE
        c = int(round(32767 * math.cos(th)))
        s = int(round(32767 * math.sin(th)))
        err = -((ea * c + eb * s) >> 15)
        if self.pll_dir < 0:
            err = -err
        errn = (_clamp(err, -32768, 32767) * self.knorm) >> 11
        self.pll_w_fx = _clamp(self.pll_w_fx + ((PLL_KI * errn) >> 4),
                               -(1000 << 8), 1000 << 8)
        self.pll_th_fx = (self.pll_th_fx + self.pll_w_fx + ((PLL_KP * errn) << 4)) & 0xFFFFFF
        if not self.sl_closed:
            self.phi_fx = (self.phi_fx + self.wol_fx) & 0xFFFFFF
        elif self.pll_w_fx > (5 << 8):
            self.pll_dir = 1
        elif self.pll_w_fx < -(5 << 8):
            self.pll_dir = -1

    def _sensor_select(self):
        """After `sens` changes, or calibration ends: start the estimators
        from scratch and tell the loop whose angle to use."""
        sens = self.p["sens"]
        self.ext = sens != SENS_ENCODER
        if sens == SENS_HALL:
            self._hall_seed()
        self.sl_closed = False
        self.wol_fx = 0
        self.phi_fx = self.ang << 8
        self.pll_th_fx = self.ang << 8
        self.pll_w_fx = 0
        self.sl_lock = 0
        self.q_boost = 0
        self.pia = self.pib = self.pva = self.pvb = 0

    def _cal_start(self):
        self.cal = 1
        self.cal_ms = 0
        self.cal_w_fx = 0
        self.cal_trav = 0
        self.cal_result = 0
        self.phi_fx = 0
        self.f_mask = self.r_mask = 0
        self.h_code = self._hall_now()
        self.v_cmd = LOOP_ALIGN_V
        self.loop = LOOP_VOLTAGE
        self.ext = True

    def _cal_finish(self):
        ok = 0
        mask = self.f_mask
        if mask != self.r_mask:
            ok = 2                  # the two directions saw different states
        elif bin(mask).count("1") != 6 or (mask & 0x81):
            ok = 3                  # not six states, or all-off / all-on among them
        else:
            first = next(c for c in range(8) if mask & (1 << c))
            c = first
            seen = 0
            for _ in range(6):
                seen |= 1 << c
                c = self.f_next[c]
            if c != first or seen != mask:
                ok = 4              # the states do not form one cycle of six
        if ok == 0:
            for c in range(8):
                if mask & (1 << c):
                    self.h_edge[c] = (self.f_edge[c]
                                      + (_s16(self.r_edge[c] - self.f_edge[c]) >> 1)) & 0xFFFF
                    self.h_next[c] = self.f_next[c]
                else:
                    self.h_next[c] = 0
                    self.h_wid[c] = 0
            for c in range(8):
                if mask & (1 << c):
                    self.h_wid[c] = (self.h_edge[self.h_next[c]] - self.h_edge[c]) & 0xFFFF
            self.h_valid = True
            self.h_err = 0
            ok = 1
        self.cal_result = ok
        self.cal = 0
        self.loop = LOOP_IDLE
        self.v_cmd = 0
        self._sensor_select()

    def _cal_tick(self):
        self.cal_ms += 1
        target = 0
        if self.cal == 1:
            if self.cal_ms >= 200:
                self.cal = 2
                self.cal_trav = 0
        elif self.cal == 2:
            target = CAL_W
            if self.cal_trav >= CAL_TURN:
                self.cal = 3
                self.cal_ms = 0
        elif self.cal == 3:
            if self.cal_ms >= 150:
                self.cal = 4
                self.cal_trav = 0
        elif self.cal == 4:
            target = -CAL_W
            if self.cal_trav >= CAL_TURN:
                self.cal = 5
                self.cal_ms = 0
        elif self.cal_ms >= 100:
            self._cal_finish()
            return
        if self.cal_w_fx < target:
            self.cal_w_fx = min(target, self.cal_w_fx + 64)
        elif self.cal_w_fx > target:
            self.cal_w_fx = max(target, self.cal_w_fx - 64)

    def _observer_tick(self):
        """Once a millisecond: who supplies the angle, the forced one or the
        observer's, and what the forced one should be doing."""
        p = self.p
        if self.loop != LOOP_CURRENT:
            self.sl_closed = False
            self.wol_fx = 0
            self.q_boost = 0
            self.sl_lock = 0
            return
        hand = (p["slmin"] << 8) // 20
        iq_cmd = self.target_q
        if not self.sl_closed:
            if self.outer != OUTER_OFF:
                tgt = _clamp(_tdiv(self.speed_ref << 8, 20), -hand, hand)
                dirn = _sign(tgt) or _sign(self.wol_fx)
            else:
                tgt = hand if iq_cmd > 200 else (-hand if iq_cmd < -200 else 0)
                dirn = _sign(tgt)
            inc = max(1, (p["slramp"] * 839) >> 16)
            if self.wol_fx < tgt:
                self.wol_fx = min(tgt, self.wol_fx + inc)
            elif self.wol_fx > tgt:
                self.wol_fx = max(tgt, self.wol_fx - inc)
            if self.outer != OUTER_OFF:
                self.int_s = 0      # the speed it sees is the forced one
            if dirn:
                self.q_boost = dirn * p["slcur"] - iq_cmd
                self.pll_dir = dirn
            else:
                self.q_boost = 0
            # hand over once the forced speed is up and the observer has
            # been following a rotor that turns the same way, fast enough
            if (tgt != 0 and self.wol_fx == tgt and abs(tgt) >= hand
                    and self.pll_w_fx * dirn > hand - (hand >> 2)):
                self.sl_lock += 1
            else:
                self.sl_lock = 0
            if self.sl_lock >= 20:
                self.sl_closed = True
                self.q_boost = 0
                self.sl_lock = 0
        elif abs(self.pll_w_fx) < (hand >> 1):
            self.sl_closed = False
            self.phi_fx = self.ang << 8
            self.wol_fx = self.pll_w_fx
            self.sl_lock = 0

    # ---- outer.c and drive.c, at 1 kHz

    def _go_idle(self):
        self.outer = OUTER_OFF
        self.int_s = 0
        self.loop = LOOP_IDLE
        self.target_q = self.target_d = 0
        self.v_cmd = 0

    def _veh_enter_run(self):
        self.veh = VEH_RUN
        self.th_f = 0
        self.target_d = 0
        self.target_q = 0
        self.loop = LOOP_CURRENT
        self.int_s = 0
        if self.p["thmode"]:
            self.speed_ref = 0
            self.outer = OUTER_SPEED
        else:
            self.outer = OUTER_OFF

    def _vehicle_tick(self):
        p = self.p
        if p["insrc"]:
            raw = 0                 # the pins: nothing is connected here
            brake = 0
        else:
            raw = self.th_sim
            brake = self.brk_sim
        self.th_raw = raw
        self.brake = brake
        if self.veh == VEH_OFF or self.veh == VEH_FAULT:
            return

        span = p["thmax"] - p["thmin"]
        dead = span >> 5
        den = span - dead
        num = raw - p["thmin"] - dead
        pos = 0
        if den >= 16 and num > 0:
            pos = min(32767, (num * 32767) // den)

        if raw < (p["thmin"] >> 1) or raw > p["thmax"] + ((4095 - p["thmax"]) >> 1):
            self.th_bad += 1
            if self.th_bad >= 50:
                self.veh = VEH_FAULT
                self.vflt = 1
                self.th_f = 0
                self._go_idle()
                return
        else:
            self.th_bad = 0

        self.limit = 0
        if self.veh == VEH_WAIT:
            if pos == 0:
                self._veh_enter_run()
        elif self.veh == VEH_RUN:
            if brake:
                self.veh = VEH_BRAKE
                self.th_f = 0
                self.outer = OUTER_OFF
                self.int_s = 0
                self.target_q = 0
            else:
                if pos > self.th_f:
                    self.th_f = min(pos, self.th_f + p["thramp"])
                else:
                    self.th_f = pos
                if p["thmode"]:
                    self.speed_ref = (self.th_f * p["drvspd"]) >> 15
                else:
                    iq = (self.th_f * p["simax"]) >> 15
                    over = self.speed - p["drvspd"]
                    if over > 0:
                        self.limit = 1
                        iq = (iq * (256 - over)) >> 8 if over < 256 else 0
                    self.target_q = iq
        elif self.veh == VEH_BRAKE:
            if not brake:
                self._veh_enter_run()
            else:
                a = min(abs(self.speed), BRAKE_FADE)
                iq = (p["regen"] * a) // BRAKE_FADE
                self.target_q = -iq if self.speed > 0 else iq

    def _tick1k(self):
        p = self.p
        # outer_tick
        if self.have_prev:
            self.speed = _s16(self.ang - self.prev_angle)
        self.prev_angle = self.ang
        self.have_prev = True
        if self.ext and not self.cal:
            self.speed = self._sensor_speed()
        if self.outer != OUTER_OFF:
            if self.outer == OUTER_POSITION:
                sref = (p["pkp"] * _s16(self.pos_ref - self.ang)) >> 15
                self.speed_ref = _clamp(sref, -SPEED_MAX, SPEED_MAX)
            iq_max = p["simax"]
            e = self.speed_ref - self.speed
            self.int_s = _clamp(self.int_s + p["ski"] * e, -(iq_max << 15), iq_max << 15)
            u = _clamp(((p["skp"] * e) >> 15) + (self.int_s >> 15), -iq_max, iq_max)
            self.iq_ref = u
            self.target_q = u

        # drive_tick
        self._vehicle_tick()
        if self.cal:
            self._cal_tick()
        elif p["sens"] == SENS_OBSERVER:
            self._observer_tick()

        if p["fw"] > 0 and self.loop == LOOP_CURRENT:
            mag = math.isqrt(self.last_vd * self.last_vd + self.last_vq * self.last_vq)
            thr = (p["fwstart"] * p["vlim"]) >> 15
            self.idfw_fx = _clamp(self.idfw_fx + ((thr - mag) << 5), -(p["fw"] << 8), 0)
        else:
            self.idfw_fx = 0
        self.fw_d = self.idfw_fx >> 8

        if p["cdec"] == 2:
            s = _clamp(self.speed, -16000, 16000)
            self.cross_run = _clamp(_tdiv(p["ccross"] * s, SPEED_REF), -65535, 65535)
            self.emf_run = _clamp(_tdiv(p["cemf"] * s, SPEED_REF), -32767, 32767)
        else:
            self.cross_run = p["ccross"]
            self.emf_run = p["cemf"]

    # ---- oneline.c

    def _ol_build(self):
        self.ol_frame = protocols.ol_frame(
            self.speed, self.last_q, self.th_f, self.brake,
            self.veh == VEH_BRAKE and self.p["regen"] > 0, self.limit,
            self.h_err > 0 and self.p["sens"] == SENS_HALL, self.vflt, False)
        t = self.p["olt"]
        self.ol_runs = [-protocols.OL_GAP * t] + protocols.ol_runs(self.ol_frame, t)
        self.ol_i = 0
        self.ol_left = 0

    def _oneline_tick(self):
        if self.ol_left == 0:
            if self.ol_i >= len(self.ol_runs):
                self._ol_build()
            if self.ol_i == 1:
                # the frame proper starts here, after the gap
                if self.cap_ch == CAP_OL:
                    if self.cap_state == CAP_RUNNING:
                        self.cap_state = CAP_DONE
                    elif self.cap_state == CAP_ARMED:
                        self.cap.clear()
                        self.ol_run = 0
                        self.cap_state = CAP_RUNNING
            run = self.ol_runs[self.ol_i]
            self.ol_i += 1
            level = 1 if run > 0 else 0
            if level != self.ol_level:
                if self.cap_state == CAP_RUNNING and self.cap_ch == CAP_OL:
                    if len(self.cap) < CAPTURE_LEN:
                        self.cap.append(self.ol_run if self.ol_level else -self.ol_run)
                self.ol_run = 0
                self.ol_level = level
            self.ol_left = abs(run)
        self.ol_left -= 1
        self.ol_run += 1

    # ---- console.c

    def _line(self, text):
        self._out += (text + "\r\n").encode("ascii")

    def _state(self):
        self._line(f"ang {self.ang}  spd {self.speed}  id {self.last_d}  "
                   f"iq {self.last_q}  iqref {self.iq_ref}  moe {self.moe}  "
                   "fault none")

    def _phase(self):
        if self.cal:
            return PH_CAL
        sens = self.p["sens"]
        if sens == SENS_HALL:
            return PH_HALL
        if sens == SENS_OBSERVER:
            return PH_CLOSED if self.sl_closed else PH_OPEN
        return PH_NONE

    def _vline(self):
        self._line(f"v th {self.th_raw} thr {self.th_f} brk {self.brake} "
                   f"veh {self.veh} vflt {self.vflt} sens {self.p['sens']} "
                   f"ph {self._phase()} hall {self._hall_now()} "
                   f"cal {self.cal_result} idfw {self.fw_d} "
                   f"aerr {_s16(self.ang - self.theta + (self.omega_fx >> 8))}")

    def _trigger(self):
        if self.cap_state == CAP_ARMED and self.cap_ch != CAP_OL:
            self.cap.clear()
            self.cap_phase = 0
            self.cap_state = CAP_RUNNING

    def _idle(self):
        return (self.loop == LOOP_IDLE and self.outer == OUTER_OFF
                and self.veh in (VEH_OFF, VEH_FAULT) and not self.cal)

    def _leave_vehicle(self):
        if self.veh != VEH_FAULT:
            self.veh = VEH_OFF
        self.th_f = 0
        if self.cal:
            self.cal = 0
            self.cal_result = 9     # interrupted
            self._sensor_select()

    def _set(self, name, value):
        """param_set() and what follows from it. Returns the stored value."""
        _i = PARAM_INDEX[name]
        _n, _default, lo, hi = PARAMS[_i]
        value = _clamp(value, lo, hi)
        p = self.p
        if name == "sens":
            if not self._idle() or (value == SENS_HALL and not self.h_valid):
                return p[name]
            p[name] = value
            self._sensor_select()
        elif name == "mot":
            if not self._idle():
                return p[name]
            p[name] = value
            self.vm = PRESETS[value]
        elif name == "canbr":
            p[name] = min((125, 250, 500, 1000), key=lambda r: abs(r - value))
        elif name == "thmin":
            p[name] = min(value, p["thmax"])
        elif name == "thmax":
            p[name] = max(value, p["thmin"])
        else:
            p[name] = value
        if name == "vlim":
            lim = value << 15
            self.int_d = _clamp(self.int_d, -lim, lim)
            self.int_q = _clamp(self.int_q, -lim, lim)
        self._derive()
        return p[name]

    def _dispatch(self, cmd):
        if not cmd:
            return
        if cmd[0] == "?" or cmd.startswith("help"):
            self._line("commands:")
            for text in ("s t m iq spd pos align off on clear hang",
                         "ver x get set id vd cap dump",
                         "v th brk drv hallcal hall olf can"):
                self._line("  " + text)
        elif cmd.startswith("iq "):
            v = _clamp(_parse_int(cmd[3:]), -16384, 16384)
            self._leave_vehicle()
            self.outer = OUTER_OFF
            self.int_s = 0
            self.loop = LOOP_CURRENT
            self.target_q = v
            self._trigger()
            self._line("current mode")
        elif cmd.startswith("spd "):
            v = _clamp(_parse_int(cmd[4:]), -SPEED_MAX, SPEED_MAX)
            self._leave_vehicle()
            self.target_d = 0
            self.loop = LOOP_CURRENT
            self.outer = OUTER_SPEED
            self.int_s = 0
            self.speed_ref = v
            self._trigger()
            self._line("speed mode")
        elif cmd.startswith("pos "):
            v = _parse_int(cmd[4:])
            self._leave_vehicle()
            self.target_d = 0
            self.loop = LOOP_CURRENT
            self.outer = OUTER_POSITION
            self.int_s = 0
            self.pos_ref = (self.ang + v) & 0xFFFF
            self._line("position mode")
        elif cmd.startswith("align"):
            # On the board this ends in a watchdog reset. A test double that
            # silently did something kinder would hide that.
            self._line("aligning...")
        elif cmd.startswith("off"):
            self._leave_vehicle()
            self._go_idle()
            self.moe = 0
            self._line("idle, outputs off")
        elif cmd.startswith("on"):
            if self.loop != LOOP_IDLE or self.outer != OUTER_OFF:
                self._line("refused: not idle")
            else:
                self.moe = 1
                self._line("armed, outputs enabled")
        elif cmd.startswith("clear"):
            self.vflt = 0
            self.th_bad = 0
            self.h_err = 0
            if self.veh == VEH_FAULT:
                self.veh = VEH_OFF
            self._line("fault cleared")
        elif _after_word(cmd, "hallcal") is not None:
            if not self._idle():
                self._line("? stop the drive first")
            else:
                self._cal_start()
                self._line("hall calibration started")
        elif _after_word(cmd, "hall") is not None:
            edges = " ".join(str(self.h_edge[c]) if self.h_valid and self.h_wid[c]
                             else "-1" for c in range(8))
            self._line(f"hall ok {int(self.h_valid)} err {self.h_err} e {edges}")
        elif cmd.startswith("hang"):
            self._line("hanging: the watchdog should reset the board")
        elif (arg := _after_word(cmd, "set")) is not None:
            for name, *_rest in PARAMS:
                value = _after_word(arg, name)
                if value:
                    self._line(f"p {name} {self._set(name, _parse_int(value))}")
                    break
            else:
                self._line("? unknown parameter, or no value")
        elif _after_word(cmd, "get") is not None:
            for name, *_ in PARAMS:
                self._line(f"p {name} {self.p[name]}")
            self._line("get end")
        elif _after_word(cmd, "ver") is not None:
            self._line(f"fw {CONSOLE_VERSION} cap {CAPTURE_LEN}")
        elif (arg := _after_word(cmd, "id")) is not None:
            self.target_d = _clamp(_parse_int(arg), -16384, 16384)
            self._trigger()
            self._line("id target")
        elif (arg := _after_word(cmd, "vd")) is not None:
            self._leave_vehicle()
            self.outer = OUTER_OFF
            self.int_s = 0
            self.v_cmd = _clamp(_parse_int(arg), -16384, 16384)
            self.loop = LOOP_VOLTAGE
            self._trigger()
            self._line("voltage mode")
        elif (arg := _after_word(cmd, "cap")) is not None:
            for i, name in enumerate(CAP_NAMES):
                rest = _after_word(arg, name)
                if rest is not None:
                    self.cap_state = CAP_IDLE
                    self.cap_ch = i
                    self.cap_div = _clamp(_parse_int(rest) if rest else 1, 1, 2000)
                    self.cap.clear()
                    self.cap_phase = 0
                    self.cap_state = CAP_ARMED
                    self._line("cap armed")
                    break
            else:
                self._line("? cap <id|iq|vd|vq|spd|ang|ol|hall|aerr|idfw> <divider>")
        elif _after_word(cmd, "dump") is not None:
            n = len(self.cap)
            self._line(f"cap {CAP_NAMES[self.cap_ch]} div {self.cap_div} "
                       f"n {n} state {self.cap_state}")
            for i in range(0, n, 8):
                self._line("d " + " ".join(str(v) for v in self.cap[i:i + 8]))
            self._line("cap end")
        elif (arg := _after_word(cmd, "th")) is not None:
            self.th_sim = _clamp(_parse_int(arg), 0, 4095)
            self._line(f"th {self.th_sim}")
        elif (arg := _after_word(cmd, "brk")) is not None:
            self.brk_sim = 1 if _parse_int(arg) else 0
            self._line(f"brk {self.brk_sim}")
        elif (arg := _after_word(cmd, "drv")) is not None:
            if _parse_int(arg):
                if self.cal:
                    self._line("? stop the drive first")
                elif self.veh == VEH_FAULT:
                    self._line("refused: throttle fault, clear it first")
                else:
                    self._go_idle()
                    self.veh = VEH_WAIT
                    self.th_f = 0
                    self.th_bad = 0
                    self._line("drive on: release the throttle to start")
            else:
                self._leave_vehicle()
                self._go_idle()
                self._line("drive off")
        elif _after_word(cmd, "olf") is not None:
            self._line("olf " + " ".join(f"{b:02x}" for b in self.ol_frame))
        elif (arg := _after_word(cmd, "can")) is not None:
            if arg.startswith("on"):
                back = self.p["canback"]
                self._line(f"can on: usb closes, back after {back} s without a frame")
                self._leave_vehicle()
                self._go_idle()
                global _parked
                self.away_until = time.perf_counter() + (back if back else 3600.0)
                _parked = self
            else:
                self._line(f"can test 7 mode 0 id {self.p['canid']} "
                           f"br {self.p['canbr']} rx 0 tx 0 drop 0")
        elif _after_word(cmd, "v") is not None:
            self._vline()
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

    def _gone(self, what):
        if self.closed:
            raise OSError(f"{what} failed, the emulator is closed")
        if self.away_until:
            raise OSError(f"{what} failed, win32 error 22")

    def write_line(self, text):
        self._gone("write")
        self._catch_up()
        self._rx += text + "\r"
        while "\r" in self._rx:
            cmd, self._rx = self._rx.split("\r", 1)
            self._dispatch(cmd[:63])

    def read(self, n=4096):
        if self.away_until and self._out:
            out = bytes(self._out[:n])      # the reply to `can on` still goes out
            del self._out[:n]
            return out
        self._gone("read")
        self._catch_up()
        out = bytes(self._out[:n])
        del self._out[:n]
        return out

    def close(self):
        self.closed = True


# ---------------------------------------------------------------- checks ---

def true_edges(width):
    """Where each Hall state really begins, going forward: {code: angle}."""
    out = {}
    prev = hall_code(65535, width)
    for theta in range(65536):
        code = hall_code(theta, width)
        if code != prev:
            out[code] = theta
            prev = code
    return out


def selfcheck():
    """Does the arithmetic above reproduce what the firmware's own
    documentation says the model does? (name, passed, detail) rows."""
    rows = []

    def check(name, ok, detail=""):
        rows.append((name, bool(ok), str(detail)))

    def talk(fw, cmd):
        fw.read()
        fw.write_line(cmd)
        return fw.read().decode("ascii").strip().splitlines()

    fw = FwSim(realtime=False)
    check("unknown command", talk(fw, "nonsense") == ["? for help"], "")
    check("version line", talk(fw, "ver") == ["fw 3 cap 256"], "")
    check("set clamps to the firmware's range",
          talk(fw, "set cki 99999") == ["p cki 32767"], "")
    fw.write_line("set cki 900")
    check("'set' is not mistaken for 's'", talk(fw, "set ckp 6000") == ["p ckp 6000"], "")
    check("get lists every parameter", len(talk(fw, "get")) == len(PARAMS) + 1, "")

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

    # ---- Hall sensors
    for width, label in ((32768, "latch-like"), (27000, "3144-like, on 148 deg")):
        fw = FwSim(realtime=False)
        fw.write_line(f"set hwid {width}")
        fw.write_line("hallcal")
        fw.run(60000)
        truth = true_edges(width)
        worst = max(abs(_s16(fw.h_edge[c] - a)) for c, a in truth.items()) if fw.h_valid else 99999
        check(f"hall calibration finds the six edges ({label})",
              fw.cal_result == 1 and worst < 500,
              f"result {fw.cal_result}, worst edge {worst * 360 / 65536:.2f} deg off")
        check("  set sens 1 is accepted afterwards",
              talk(fw, "set sens 1") == ["p sens 1"], "")
        fw.write_line("spd 3000")
        fw.run(20000)
        worst = 0
        speeds = []
        for _ in range(200):
            fw.run(20)
            worst = max(worst, abs(_s16(fw.ang - fw.theta)))
            speeds.append(fw.speed)
        true_speed = (fw.omega_fx >> 8) * 20
        check("  speed loop holds 3000 on the Hall angle",
              abs(true_speed - 3000) < 150 and worst < 2200,
              f"true speed {true_speed}, angle error up to {worst * 360 / 65536:.1f} deg, "
              f"reported {min(speeds)}..{max(speeds)}")
        fw.write_line("spd -2000")
        fw.run(50000)
        true_speed = (fw.omega_fx >> 8) * 20
        check("  and reverses", abs(true_speed + 2000) < 150, f"true speed {true_speed}")

    fw = FwSim(realtime=False)
    check("sens 1 is refused before calibration",
          talk(fw, "set sens 1") == ["p sens 0"], "")
    fw.write_line("set hsrc 1")
    fw.write_line("hallcal")
    fw.run(60000)
    check("pins with no sensors on them fail calibration",
          fw.cal_result > 1 and not fw.h_valid, f"result {fw.cal_result}")

    # ---- observer
    for mot in (0, 1):
        fw = FwSim(realtime=False)
        fw.write_line(f"set mot {mot}")
        if mot:
            # the hub motor's back-EMF is five times the gimbal's, so its
            # observer is good to a much lower speed
            for line in ("set sla 328", "set cemf 32800", "set cdec 2", "set ccross 65535",
                         "set ckp 30000", "set cki 300", "set skp 1000000",
                         "set ski 12000", "set slcur 6000", "set slmin 150"):
                fw.write_line(line)
        low = 300 if mot == 0 else 60
        fw.write_line("set sens 2")
        fw.write_line("spd 3000")
        opened = closed_at = None
        for ms in range(3000):
            fw.run(20)
            if fw.sl_closed and closed_at is None:
                closed_at = ms
        worst = 0
        for _ in range(300):
            fw.run(20)
            worst = max(worst, abs(_s16(fw.ang - fw.theta)))
        true_speed = (fw.omega_fx >> 8) * 20
        check(f"observer takes over and holds 3000 (motor {mot})",
              closed_at is not None and fw.sl_closed and abs(true_speed - 3000) < 100
              and worst < 400,
              f"closed after {closed_at} ms, true speed {true_speed}, "
              f"angle error up to {worst * 360 / 65536:.1f} deg")
        fw.write_line(f"spd {low}")
        fw.run(40000)
        turned = 0
        for _ in range(1000):
            before = fw.theta
            fw.run(20)
            turned += _s16(fw.theta - before)
        check("  below slmin it goes back to the forced angle and still turns",
              not fw.sl_closed and abs(turned / 1000 - low) < 0.2 * low,
              f"{turned / 1000:.0f} units/ms on average, asked for {low}")
        fw.write_line("spd -3000")
        fw.run(80000)
        true_speed = (fw.omega_fx >> 8) * 20
        check("  and it runs backwards", fw.sl_closed and abs(true_speed + 3000) < 100,
              f"true speed {true_speed}")

    # ---- field weakening, on the motor where it can work
    tops = []
    for fwmax in (0, 8192):
        fw = FwSim(realtime=False)
        for line in ("set mot 1", "set ckp 30000", "set cki 300", "set cdec 2",
                     "set ccross 65535", "set cemf 32800", "set drvspd 16000",
                     f"set fw {fwmax}", "drv 1"):
            fw.write_line(line)
        fw.run(100)
        fw.write_line("th 3300")
        fw.run(120000)
        tops.append(((fw.omega_fx >> 8) * 20, fw.fw_d))
    check("field weakening raises the top speed of the hub motor",
          tops[1][0] > 1.2 * tops[0][0] and tops[1][1] < -1000,
          f"{tops[0][0]} units/ms without, {tops[1][0]} with (id {tops[1][1]})")

    # ---- throttle and brake
    fw = FwSim(realtime=False)
    fw.write_line("th 2500")
    check("drive waits for a released throttle",
          talk(fw, "drv 1") == ["drive on: release the throttle to start"], "")
    fw.run(4000)
    check("  and does not start with it held", fw.veh == VEH_WAIT and fw.target_q == 0,
          f"state {fw.veh}")
    fw.write_line("th 900")
    fw.run(200)
    fw.write_line("th 3300")
    fw.run(2000)
    check("throttle ramps instead of stepping", 5000 < fw.th_f < 8000 and fw.veh == VEH_RUN,
          f"{fw.th_f} of 32767 after 100 ms")
    fw.run(40000)
    top = fw.speed
    check("full throttle is the full current limit", fw.target_q == 16384 or fw.limit,
          f"iq target {fw.target_q}, speed {top}, limit {fw.limit}")
    check("  and the speed limit holds it", top <= 6560 + 300, f"{top} units/ms")
    fw.write_line("brk 1")
    fw.run(400)
    check("brake cuts the throttle and regenerates",
          fw.veh == VEH_BRAKE and fw.target_q == -4096 and fw.th_f == 0,
          f"iq target {fw.target_q}")
    fw.run(40000)
    check("  down to a crawl, without reversing", 0 <= fw.speed < 200, f"{fw.speed} units/ms")
    fw.write_line("brk 0")
    fw.write_line("th 100")
    fw.run(2000)
    check("a throttle wire off its range is a fault", fw.veh == VEH_FAULT and fw.vflt == 1
          and fw.loop == LOOP_IDLE, f"state {fw.veh}")
    check("  which refuses a restart until cleared",
          talk(fw, "drv 1") == ["refused: throttle fault, clear it first"], "")

    # ---- one line
    fw = FwSim(realtime=False)
    for line in ("set ol 1", "spd 3000"):
        fw.write_line(line)
    fw.run(20000)
    fw.write_line("cap ol 1")
    fw.run(16000)
    d = protocols.ol_decode(fw.cap)
    check("one-line frame decodes from the recorded waveform",
          fw.cap_state == CAP_DONE and d["ok"] and abs(d["t"] - 10) < 0.01
          and abs(d["speed"] - 3000) < 60 and d["turning"],
          f"{len(fw.cap)} runs, T {d['t']:.1f}, speed {d.get('speed')}, {d['why']}")
    check("  and matches what `olf` prints",
          talk(fw, "olf")[0].split()[1:] == [f"{b:02x}" for b in fw.ol_frame], "")
    return rows


if __name__ == "__main__":
    failed = 0
    for name, ok, detail in selfcheck():
        print(("ok    " if ok else "FAIL  ") + f"{name:58s} {detail}")
        failed += not ok
    raise SystemExit(1 if failed else 0)

"""
The vehicle page of FOC studio: throttle, brake, sensor selector, field
weakening, the dashboard signal and CAN - everything console version 3 added
to the firmware, on one page.

It talks to a board, or to the firmware model (the port called "emulator"),
through boardlink. There is no third implementation for the built-in
simulator: these are properties of the firmware, and the firmware model runs
the firmware's own arithmetic.
"""

import time

import boardlink
import braun as B
import protocols

X0 = 44
UNITS_PER_RPM = boardlink.UNITS_PER_RPM

# Gains that suit each of the firmware model's two motors. The firmware does
# not change them when the motor is changed - on a bench it would not know.
MOTORS = (
    ("gimbal", (("mot", 0), ("sla", 1638), ("cemf", 6554), ("cdec", 1),
                ("ccross", 20580), ("ckp", 6000), ("cki", 900), ("skp", 32768),
                ("ski", 6144), ("slcur", 4000), ("slmin", 600), ("drvspd", 6560))),
    ("hub", (("mot", 1), ("sla", 328), ("cemf", 32800), ("cdec", 2),
             ("ccross", 65535), ("ckp", 30000), ("cki", 300), ("skp", 1000000),
             ("ski", 12000), ("slcur", 6000), ("slmin", 150), ("drvspd", 12000))),
)
HALL_KINDS = (("latch", 32768), ("3144 / 3143", 27000))
FW_MAX = 8192


def sectors(edges):
    """Six sector widths in degrees, from the eight edges `hall` reports."""
    found = sorted(e for e in edges if e >= 0)
    if len(found) != 6:
        return []
    return [((found[(i + 1) % 6] - a) & 0xFFFF) * 360.0 / 65536.0
            for i, a in enumerate(found)]


class VehiclePage:
    """Mixed into foc.Page by foc.py (which owns the canvas and the text
    helpers); kept here so the page can be read on its own."""

    key = "vehicle"
    title = "vehicle"

    def setup(self):
        self.ready = None
        self.msg = ""
        self._th = 0.0              # throttle, percent, as the fader has it
        self._th_sent = None
        self._th_at = 0.0
        self._brk = 0
        self._cal_wait = None       # when calibration was asked for
        self._cal_seen = False      # and whether it has been seen running
        self._labels = {}
        self._ol_at = 0.0
        self._pin = None            # result of "check at the pin"
        self._pin_wait = None
        self._can_at = 0.0

    def link(self):
        return self.app.board if self.app.board_has_vehicle() else None

    def sub(self):
        link = self.link()
        if link is None:
            return "throttle, brake, sensor selector, field weakening, dashboard, can"
        what = ("the firmware model" if link.port == boardlink.EMULATOR_PORT
                else f"the firmware on {link.port}")
        return f"{what}  ·  its motor is a model inside the firmware"

    # ---- building

    def build(self):
        cv, app = self.cv, self.app
        link = self.link()
        self.ready = link is not None
        if link is None:
            self._build_notice()
            return
        X1 = self.X1

        # -- throttle and brake
        xa, xb = X0, 500
        self.text(xa, 124, "throttle  %", 12, B.INK3)
        self.t_thr = self.text(xb, 112, "0", 44, B.INK, "ne", num=True)
        self.fader = B.Fader(cv, xa + 12, 196, xb - xa - 24, 0, 100, self._th,
                             self._set_throttle, step=1,
                             majors=(0, 25, 50, 75, 100), minors=(12.5, 37.5, 62.5, 87.5),
                             label=B.num)
        self.fader.wheel = 2
        self.t_raw = self.text(xa, 244, "", 11, B.INK3)
        self.b_drv = B.RoundButton(cv, xa + 40, 292, 64, B.GREEN, self._drive_on,
                                   label="drive", label_size=12)
        B.RoundButton(cv, xa + 130, 292, 64, B.RED, self._drive_off,
                      label="stop", label_size=12)
        self.b_brk = B.RoundButton(cv, xa + 220, 292, 64, B.DARK2, self._brake,
                                   label="brake", label_size=12)
        B.RoundButton(cv, xa + 300, 292, 40, B.DARK2, lambda: self._set_throttle(0, True),
                      glyph="0", glyph_size=15, label="release", label_size=11)
        self.b_mode = B.RoundButton(cv, xa + 372, 292, 40, B.DARK2, self._mode,
                                    glyph="T", glyph_size=15, label="torque",
                                    label_size=11)

        # -- what it is doing
        xa = 560
        self.text(xa, 124, "speed  rpm", 12, B.INK3)
        self.t_rpm = self.text(xa - 4, 136, "0", 76, B.INK, num=True)
        self.text(xa, 244, "torque current", 12, B.INK3)
        self.t_iq = self.text(xa + 150, 262, "0", 26, B.INK, "ne", num=True)
        self.text(xa + 180, 244, "field current", 12, B.INK3)
        self.t_id = self.text(xa + 320, 262, "0", 26, B.INK, "ne", num=True)
        self.t_state = self.text(xa, 312, "", 13, B.INK2, width=300)
        xl = 880
        self.l_on = B.Lamp(cv, xl, 136, "drive on")
        self.l_brk = B.Lamp(cv, xl, 162, "braking")
        self.l_lim = B.Lamp(cv, xl, 188, "at top speed")
        self.l_fw = B.Lamp(cv, xl, 214, "weakening")
        self.l_flt = B.Lamp(cv, xl, 240, "throttle fault")
        cv.create_line(X0, 356, X1, 356, fill=B.LINE)

        # -- where the angle comes from
        xa = X0
        self.text(xa, 370, "rotor angle from", 12, B.INK3)
        self.b_sens = []
        for i, name in enumerate(("encoder", "hall", "observer")):
            self.b_sens.append(B.RoundButton(
                cv, xa + 34 + i * 96, 424, 44, B.DARK2, lambda i=i: self._sensor(i),
                glyph=str(i), glyph_size=15, label=name, label_size=11))
        self.t_sens = self.text(xa, 478, "", 13, B.INK)
        self.t_aerr = self.text(xa, 500, "", 12, B.INK3)
        B.RoundButton(cv, xa + 18, 542, 30, B.ORANGE, self._calibrate,
                      label="calibrate hall sensors", label_side="e",
                      label_size=12)

        xa = 370
        cv.create_line(xa - 20, 372, xa - 20, 560, fill=B.LINE)
        self.text(xa, 370, "hall sensors", 12, B.INK3)
        self.l_hall = [B.Lamp(cv, xa + 8 + i * 60, 404, f"H{i + 1}") for i in range(3)]
        self.b_kind = []
        for i, (name, _w) in enumerate(HALL_KINDS):
            self.b_kind.append(B.RoundButton(
                cv, xa + 18 + i * 130, 444, 30, B.DARK2, lambda i=i: self._hall_kind(i),
                label=name, label_side="e", label_size=12))
        self.t_cal = self.text(xa, 478, "", 13, B.INK, width=300)
        self.t_sect = self.text(xa, 522, "", 12, B.INK3, width=300)

        xa = 720
        cv.create_line(xa - 20, 372, xa - 20, 560, fill=B.LINE)
        self.text(xa, 370, "motor model", 12, B.INK3)
        self.b_mot = []
        for i, (name, _set) in enumerate(MOTORS):
            self.b_mot.append(B.RoundButton(
                cv, xa + 18 + i * 120, 404, 30, B.DARK2, lambda i=i: self._motor(i),
                label=name, label_side="e", label_size=12))
        self.t_mot = self.text(xa, 430, "", 11, B.INK3, width=X1 - xa)
        self.text(xa, 478, "field weakening", 12, B.INK3)
        self.b_fw = B.RoundButton(cv, xa + 18, 516, 30, B.DARK2, self._weaken,
                                  label="off", label_side="e", label_size=12)
        self.t_fw = self.text(xa + 110, 508, "", 12, B.INK3, width=X1 - xa - 110)
        cv.create_line(X0, 572, X1, 572, fill=B.LINE)

        # -- dashboard and CAN
        xa, xb = X0, 640
        self.text(xa, 586, "dashboard  ·  one-line signal on PB10", 12, B.INK3)
        self.b_ol = B.RoundButton(cv, xa + 18, 630, 30, B.DARK2, self._oneline,
                                  label="off", label_side="e", label_size=12)
        B.RoundButton(cv, xa + 18, 674, 30, B.DARK2, self._pin_check,
                      label="check at the pin", label_side="e", label_size=12)
        cv.create_image(xa + 190, 610, anchor="nw",
                        image=B.rrect(xb - xa - 190, 162, 10, B.DARK))
        self.t_ol = cv.create_text(xa + 204, 622, anchor="nw", fill="#C9C6BC",
                                   font=("Consolas", -13), text="",
                                   width=xb - xa - 216)
        self.t_pin = self.text(xa, 712, "", 11, B.INK3, width=178)

        xa = 680
        cv.create_line(xa - 20, 588, xa - 20, 772, fill=B.LINE)
        self.text(xa, 586, "can bus  ·  PB8 / PB9", 12, B.INK3)
        self.l_can = B.Lamp(cv, xa + 8, 618, "controller passed its loopback test")
        self.t_can = self.text(xa, 636, "", 12, B.INK2, width=X1 - xa)
        B.RoundButton(cv, xa + 22, 716, 40, B.ORANGE, self._can_on,
                      label="change over to can", label_side="e", label_size=12)
        self.t_can2 = self.text(xa, 746, "", 11, B.INK3, width=X1 - xa)

        self.t_msg = self.text(X0, 778, "", 12, B.RED, anchor="sw")
        cv.bind("<MouseWheel>", self._wheel)
        self._labels.clear()
        self._set_throttle(self._th, True)

    def _build_notice(self):
        cv, app = self.cv, self.app
        link = app.board
        if app.rejoining():
            head = "the board is on can."
            body = ("its usb console is switched off while it is, so there is "
                    "nothing to show. it comes back by itself when no frame "
                    "has arrived for the time set in `canback`, and this page "
                    "reconnects when it does.")
        elif link is not None and link.fw is None:
            head = "asking the board which console it has."
            body = ""
        elif link is not None:
            head = f"the firmware on {link.port} has console {link.fw}."
            body = ("throttle, brake, field weakening, the sensor selector, "
                    "the dashboard signal and can came with console version 3. "
                    "flash the v0.3 firmware, or try them on the firmware "
                    "model: disconnect on the board page, then press below.")
        else:
            head = "this page drives the firmware."
            body = ("connect to a board on the board page - or press the "
                    "button, which opens the firmware model: the firmware's "
                    "own arithmetic running on this computer, no board needed. "
                    "everything on this page works on it.")
        self.text(X0, 150, head, 22, B.INK)
        self.text(X0, 196, body, 14, B.INK2, width=620)
        if link is None and not app.rejoining():
            B.RoundButton(cv, X0 + 36, 330, 70, B.ORANGE, self._open_model,
                          label="open the firmware model", label_side="e",
                          label_size=14)
        self.t_msg = self.text(X0, 420, "", 13, B.RED, width=620)

    # ---- actions

    def _say(self, text):
        self.msg = text
        self._msg_at = time.perf_counter()

    def _open_model(self):
        why = self.app.connect(boardlink.EMULATOR_PORT)
        if why:
            self._say(why)

    def _counts(self, pct):
        p = self.link().params
        lo, hi = p.get("thmin", 900), p.get("thmax", 3300)
        return int(round(lo + (hi - lo) * pct / 100.0))

    def _set_throttle(self, pct, move=False):
        self._th = max(0.0, min(100.0, float(pct)))
        if move and getattr(self, "fader", None) is not None and self.ready:
            self.fader.set(self._th)

    def _drive_on(self):
        link = self.link()
        link.notice = ""
        link.send("drv 1")
        if self._th > 0:
            self._say("release the throttle: the drive will not start with it open.")

    def _drive_off(self):
        self._set_throttle(0, True)
        self._brk = 0
        link = self.link()
        link.send("brk 0")
        link.send("drv 0")

    def _brake(self):
        self._brk ^= 1
        self.link().send(f"brk {self._brk}")

    def _busy(self):
        """Why the sensor or the motor cannot be changed right now."""
        link = self.link()
        if link.running or link.veh.get("veh") in (1, 2, 3):
            return "stop the drive first."
        if link.veh.get("ph") == 4:
            return "calibration is running."
        return ""

    def _mode(self):
        link = self.link()
        if link.veh.get("veh") in (1, 2, 3):
            self._say("stop the drive first.")
            return
        link.set_param("thmode", 0 if link.params.get("thmode") else 1)

    def _sensor(self, n):
        link = self.link()
        why = self._busy()
        if not why and n == 1 and not (link.hall and link.hall[0]):
            why = "calibrate the hall sensors first."
        if why:
            self._say(why)
        else:
            link.set_param("sens", n)

    def _calibrate(self):
        link = self.link()
        why = self._busy()
        if why:
            self._say(why)
            return
        if link.params.get("sens"):
            link.set_param("sens", 0)
        link.send("hallcal")
        self._cal_wait = time.perf_counter()
        self._cal_seen = False

    def _hall_kind(self, i):
        link = self.link()
        why = self._busy()
        if why:
            self._say(why)
            return
        if link.params.get("sens") == 1:
            link.set_param("sens", 0)
        link.set_param("hwid", HALL_KINDS[i][1])
        self._say("the sensors changed: calibrate again before using them.")

    def _motor(self, i):
        link = self.link()
        why = self._busy()
        if why:
            self._say(why)
            return
        if link.params.get("sens") == 1:
            link.set_param("sens", 0)       # the edges belong to the other rotor
        for name, value in MOTORS[i][1]:
            link.set_param(name, value)

    def _weaken(self):
        link = self.link()
        link.set_param("fw", 0 if link.params.get("fw") else FW_MAX)

    def _oneline(self):
        link = self.link()
        link.set_param("ol", 0 if link.params.get("ol") else 1)
        self._pin = None

    def _pin_check(self):
        link = self.link()
        if not link.params.get("ol"):
            self._say("switch the one-line output on first.")
            return
        link.send("cap ol 1")
        self._pin_wait = [time.perf_counter(), link.cap_serial, False]
        self._pin = "recording the pin ..."

    def _can_on(self):
        link = self.link()
        if link.port != boardlink.EMULATOR_PORT and link.can and link.can.get("test") != 7:
            self._say("the can controller did not pass its test: not changing over.")
            return
        self.msg = ""
        link.send("can on")

    def _wheel(self, e):
        if self.ready and self.fader.hit(e.x, e.y):
            self.fader.nudge((1 if e.delta > 0 else -1) * self.fader.wheel)

    # ---- drawing

    def update(self):
        app = self.app
        link = self.link()
        if (link is not None) != self.ready or (link is None and
                                                 self._notice_key() != getattr(self, "_nk", None)):
            self._nk = self._notice_key()
            self.rebuild()
        if link is None:
            self.set(self.t_msg, self.msg)
            return
        now = time.perf_counter()
        p, v = link.params, link.veh

        # the throttle goes out at most twenty times a second
        counts = self._counts(self._th)
        if counts != self._th_sent and now - self._th_at > 0.05:
            link.send(f"th {counts}")
            self._th_sent = counts
            self._th_at = now
        if not self.fader.dragging:
            self.fader.set(self._th)
        thr = v.get("thr", 0) * 100.0 / 32767.0
        self.set(self.t_thr, B.num(self._th))
        self.set(self.t_raw, f"sent as a reading of {counts} counts  ·  the firmware "
                 f"has it at {thr:.0f} % after its ramp")

        self.set(self.t_rpm, B.num(app.disp["rpm"]))
        self.set(self.t_iq, B.num(app.disp["iq"]))
        self.set(self.t_id, B.num(v.get("idfw", 0)))
        state = v.get("veh", 0)
        self.set(self.t_state, boardlink.VEHICLE_STATES[min(state, 4)] if state
                 else (f"loop {link.mode}" if link.mode else ""))
        self.b_drv.set_color(B.GREEN if state in (1, 2, 3) else B.DARK2)
        self.b_brk.set_color(B.RED if v.get("brk") else B.DARK2)
        speed_mode = bool(p.get("thmode"))
        self._label(self.b_mode, "speed" if speed_mode else "torque")
        self.set(self.b_mode.glyph, "S" if speed_mode else "T")
        self.l_on.set(B.GREEN if state in (2, 3) else (B.YELLOW if state == 1 else None))
        self.l_brk.set(B.ORANGE if state == 3 else None)
        top = p.get("drvspd", 6560)
        self.l_lim.set(B.YELLOW if state == 2 and not speed_mode
                       and link.units > top else None)
        self.l_fw.set(B.ORANGE if v.get("idfw", 0) < -50 else None)
        self.l_flt.set(B.RED if v.get("vflt") else None)

        sens, ph = v.get("sens", 0), v.get("ph", 0)
        for i, b in enumerate(self.b_sens):
            b.set_color(B.ORANGE if i == sens else B.DARK2)
        self.set(self.t_sens, boardlink.SENSOR_NAMES[min(sens, 2)]
                 + (f"  ·  {boardlink.SENSOR_PHASES[ph]}" if ph in (2, 3, 4) else ""))
        self.set(self.t_aerr, f"angle in use against the model's rotor  "
                 f"{v.get('aerr', 0) * 360.0 / 65536.0:+.1f}°")
        code = v.get("hall", 0)
        for i, lamp in enumerate(self.l_hall):
            lamp.set(B.GREEN if code & (1 << i) else None)
        wid = p.get("hwid", 32768)
        for i, (b, (_n, w)) in enumerate(zip(self.b_kind, HALL_KINDS)):
            b.set_color(B.ORANGE if abs(wid - w) < 1500 else B.DARK2)
        if self._cal_wait is not None:
            if ph == 4:
                self._cal_seen = True
            elif self._cal_seen or now - self._cal_wait > 1.5:
                self._cal_wait = None
                link.send("hall")
        hall = link.hall
        if ph == 4:
            self.set(self.t_cal, "turning the field one way, then the other ...", B.ORANGE)
        else:
            self.set(self.t_cal, boardlink.CAL_RESULTS.get(v.get("cal", 0), ""),
                     B.RED if v.get("cal", 0) > 1 else B.INK)
        sect = sectors(hall[2]) if hall and hall[0] else []
        self.set(self.t_sect, ("sectors  " + "  ".join(f"{w:.0f}°" for w in sect)
                               + (f"\nbad readings  {hall[1]}" if hall[1] else ""))
                 if sect else "the edges are not assumed to be 60° apart: they are measured.")

        mot = p.get("mot", 0)
        for i, b in enumerate(self.b_mot):
            b.set_color(B.ORANGE if i == mot else B.DARK2)
        self.set(self.t_mot, "light rotor, L/R 1 ms. nearly all resistance: "
                 "weakening buys little." if mot == 0 else
                 "heavy rotor, L/R 5 ms, five times the back-EMF: runs out of "
                 "voltage near 4800 rpm.")
        fw_on = p.get("fw", 0) > 0
        self.b_fw.set_color(B.ORANGE if fw_on else B.DARK2)
        self._label(self.b_fw, "on" if fw_on else "off")
        self.set(self.t_fw, f"up to {p.get('fw', 0)} counts of field current, once "
                 f"{p.get('fwstart', 29491) * 100 // 32768} % of the voltage is in use"
                 if fw_on else "the motor stops speeding up when the voltage runs out")

        # -- dashboard
        ol_on = bool(p.get("ol"))
        self.b_ol.set_color(B.ORANGE if ol_on else B.DARK2)
        self._label(self.b_ol, "on" if ol_on else "off")
        if ol_on and now > self._ol_at:
            self._ol_at = now + 0.5
            link.send("olf")
        if ol_on and link.ol_frame:
            d = protocols.ol_decode(protocols.ol_runs(link.ol_frame, 10))
            text = " ".join(f"{b:02x}" for b in link.ol_frame) + "\n\n"
            if d["ok"]:
                flags = [n for n, on in (("turning", d["turning"]), ("brake", d["brake"]),
                                         ("regen", d["regen"]), ("reverse", d["reverse"]),
                                         ("top speed", d["limit"]),
                                         ("throttle fault", d["throttle_fault"]),
                                         ("hall fault", d["hall_fault"]),
                                         ("controller fault", d["controller_fault"])) if on]
                text += (f"speed     {d['halls']} hall edges / 0.5 s"
                         f"  = {d['speed'] / UNITS_PER_RPM:.0f} rpm\n"
                         f"current   {d['current']} counts\n"
                         f"throttle  {d['throttle']} %\n"
                         f"flags     {', '.join(flags) or 'none'}\n"
                         f"checksum  ok")
            else:
                text += d["why"]
        elif ol_on:
            text = "waiting for a frame ..."
        else:
            text = ("off. twelve bytes, every 170 ms, for a display that takes "
                    "the one-line (SIF) signal.")
        self.set(self.t_ol, text)
        if self._pin_wait is not None:
            t0, serial, asked = self._pin_wait
            if not asked and link.cap_state == 3:
                link.send("dump")
                self._pin_wait[2] = True
            elif asked and link.cap_serial != serial:
                d = protocols.ol_decode(link.cap_data)
                self._pin = (f"read back from the pin: {len(link.cap_data)} level changes, "
                             f"T = {d['t'] * 50:.0f} µs, "
                             + ("checksum ok." if d["ok"] else d["why"]))
                self._pin_wait = None
            elif now - t0 > 3.0:
                self._pin = "no frame was recorded."
                self._pin_wait = None
        self.set(self.t_pin, self._pin or "")

        # -- CAN
        if now > self._can_at:
            self._can_at = now + 2.0
            link.send("can")
        can = link.can or {}
        self.l_can.set(B.GREEN if can.get("test") == 7 else
                       (B.RED if can else None))
        self.set(self.t_can, f"node {p.get('canid', 1)}  ·  {p.get('canbr', 500)} kbit/s  ·  "
                 f"status every {p.get('canper', 20)} ms\n"
                 "usb and can share memory inside this chip, so it is one or "
                 "the other: changing over closes this port.")
        self.set(self.t_can2, f"it comes back by itself after {p.get('canback', 10)} s "
                 "without a frame, and this window reconnects."
                 if p.get("canback", 10) else
                 "canback is 0: it stays on can until it is reset or told to return.")

        if self.msg and now - getattr(self, "_msg_at", now) > 6.0:
            self.msg = ""
        self.set(self.t_msg, self.msg or
                 (link.notice if link.notice.startswith(("refused", "?")) else ""))

    def _label(self, button, text):
        if self._labels.get(button) != text:
            self._labels[button] = text
            button.set_label(text=text)

    def _notice_key(self):
        link = self.app.board
        return (self.app.rejoining(), None if link is None else link.fw)

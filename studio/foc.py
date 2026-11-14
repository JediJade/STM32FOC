#!/usr/bin/env python3
"""
FOC studio: a drive front end for a permanent-magnet motor - the simulated
one built in, or the STM32FOC firmware on a board.

    python foc.py                    the window
    python foc.py --selftest FILE    run the model's self-check, write FILE
    python foc.py --shots DIR        drive the window through every page and
                                     save a picture of each into DIR
    python foc.py --shots DIR --port COM13
                                     the same, then again against the board
    python foc.py --vehicle-shots DIR [--port COM13]
                                     pictures of the vehicle page, on the
                                     firmware model or on a board

Built as a single FOC.exe by build-exe.cmd, it takes the same arguments.

Seven pages, keys 1 to 7:

  drive        speed slider, run / stop, an analogue dial
  display      the live values, large
  auto tune    measures R, L, flux and inertia, then designs both controllers
  parameters   every setting, numbered and grouped the way a drive lists them
  scope        speed and currents against time
  board        connect to the firmware, its self-test, its console
  vehicle      throttle, brake, sensor selector, field weakening, the
               dashboard signal and CAN (vehicle.py) - console version 3

Two sources of truth, one at a time. Until a board is connected every page
shows the simulated motor (focsim.py). Connecting on the board page switches
drive, display and scope over to the firmware (boardlink.py): the slider
sends `spd`, stop sends `off`, and the readings are the firmware's own, in
its own units.

What auto tune and parameters do then depends on the firmware. With console
version 2 they work on the board: the parameters page edits the gains in the
firmware's memory, and auto tune (boardtune.py) measures the plant through
the console and sets them. With the original console, which has no commands
for either, they stay with the simulator and say so.
"""

import ctypes
import json
import math
import os
import sys
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import filedialog

import benchlink
import boardlink
import boardtune
import braun as B
import vehicle
from focsim import (AutoTune, BY_KEY, Drive, GROUPS, MODE_SPEED, PARAMS,
                    Param, selfcheck)

VERSION = "3.1"
APP_ID = "unitymbed.stm32foc.studio"
ICON_FILE = "unitymbed.ico"

W, H = 1280, 800
RAIL = 220
CW = W - RAIL
X0, X1 = 44, CW - 44

NOTE_TUNE = ("this is the simulator's auto tune. the firmware on this "
             "board has the original console, with gains fixed when it was "
             "built and no command to change them. console version 2 has.")
NOTE_PARAMS = ("these are the simulator's parameters. the firmware on this "
               "board has the original console and its gains are constants. "
               "console version 2 makes them settable.")


def resource_path(name):
    """Beside the script, one folder up, or inside the one-file .exe."""
    here = os.path.dirname(os.path.abspath(__file__))
    for base in (getattr(sys, "_MEIPASS", here), here, os.path.dirname(here)):
        path = os.path.join(base, name)
        if os.path.exists(path):
            return path
    return os.path.join(here, name)


# ---------------------------------------------------------------- sources ---

class SimSource:
    """What drive, display and scope read while no board is connected."""

    kind = "sim"
    cur_unit = "A"
    cur_span = 1.0
    cur_dec = 1
    third_label = "limiting"
    scope_note = "the simulated motor, sampled at 5 kHz"

    def __init__(self, app):
        self.d = app.drive

    @property
    def vmax(self):
        return self.d.p["max_speed"]

    @property
    def enabled(self):
        return self.d.enabled

    @property
    def fault(self):
        return self.d.fault

    def window(self, channel, seconds, max_points):
        return self.d.window(channel, seconds, max_points)

    def at_speed(self, setpoint):
        d = self.d
        return (d.enabled and d.mode == MODE_SPEED and d.ref == d.target and
                abs(d.rpm - d.target_rpm) < max(5.0, 0.01 * abs(d.target_rpm)))

    def third_lamp(self):
        return B.YELLOW if (self.d.ilim_hit or self.d.vlim_hit) else None


class BoardSource:
    """The same questions, answered by the firmware."""

    kind = "board"
    cur_unit = "counts"
    cur_span = 400.0
    cur_dec = 0
    third_label = "outputs on"
    scope_note = "the firmware on the board, asked 100 times a second"
    vmax = float(boardlink.SPEED_MAX_RPM)

    def __init__(self, link):
        self.link = link

    @property
    def enabled(self):
        return self.link.running

    @property
    def fault(self):
        return self.link.fault

    def window(self, channel, seconds, max_points):
        return self.link.window(channel, seconds, max_points)

    def at_speed(self, setpoint):
        link = self.link
        return (link.mode == "speed" and
                abs(link.rpm - link.target_rpm) <
                max(10.0, 0.01 * abs(link.target_rpm)))

    def third_lamp(self):
        st = self.link.state
        return B.ORANGE if (st and st["moe"]) else None


# ------------------------------------------------------------------ pages ---

class Page:
    key = ""
    title = ""
    subtitle = ""

    def __init__(self, app):
        self.app = app
        self.d = app.drive
        self.cv = tk.Canvas(app, width=CW, height=H, bg=B.PAPER,
                            highlightthickness=0, bd=0)
        self._txt = {}
        self.setup()
        self.rebuild()

    def setup(self):
        pass

    def build(self):
        pass

    def update(self):
        pass

    def sub(self):
        return self.subtitle

    def rebuild(self):
        self.cv.delete("all")
        self._txt.clear()
        self.text(X0, 30, self.title, 30, B.INK)
        self.text(X0 + 2, 72, self.sub(), 12, B.INK3)
        self.cv.create_line(X0, 104, X1, 104, fill=B.LINE)
        self.build()

    def text(self, x, y, s, size=12, fill=B.INK2, anchor="nw", num=False,
             **kw):
        return self.cv.create_text(x, y, text=s, fill=fill, anchor=anchor,
                                   font=B.font(size, num=num), **kw)

    def set(self, item, s, fill=None):
        key = (s, fill)
        if self._txt.get(item) != key:
            self._txt[item] = key
            if fill is None:
                self.cv.itemconfigure(item, text=s)
            else:
                self.cv.itemconfigure(item, text=s, fill=fill)

    def legend(self, x, y, entries):
        """Small colour chips with their names, left to right."""
        for color, name in entries:
            self.cv.create_rectangle(x, y + 2, x + 9, y + 11, fill=color,
                                     outline=B.INK2)
            self.text(x + 15, y, name, 12, B.INK2)
            x += 15 + self.app.measure(name) + 18
        return x


class DrivePage(Page):
    key = "drive"
    title = "drive"

    def sub(self):
        if self.app.board:
            return (f"speed control of the firmware on {self.app.board.port}"
                    "  ·  its motor is a model inside the firmware")
        return "speed control of the simulated motor"

    def build(self):
        cv, d, app = self.cv, self.d, self.app
        src = app.src
        vmax = src.vmax
        on_board = src.kind == "board"

        self.dial = B.Dial(cv, 262, 366, 196, vmax)
        self.t_rpm = self.text(262, 452, "0", 46, B.INK, "center", num=True)
        self.text(262, 488, "rpm", 12, B.INK3, "center")

        xa, xb = 540, X1
        self.text(xa, 130, "setpoint  rpm", 12, B.INK3)
        self.t_set = self.text(xa - 4, 142, "0", 76, B.INK, num=True)
        for i, (glyph, label, fn) in enumerate((
                ("−", "−50", lambda: app.set_setpoint(app.setpoint - 50)),
                ("+", "+50", lambda: app.set_setpoint(app.setpoint + 50)),
                ("0", "zero", lambda: app.set_setpoint(0)),
                ("±", "reverse", lambda: app.set_setpoint(-app.setpoint)))):
            B.RoundButton(cv, xb - 156 + i * 46, 178, 34, B.DARK2, fn,
                          glyph=glyph, glyph_size=17, label=label,
                          label_size=10, label_color=B.INK3)

        major = self.dial.major
        n = int(vmax // major)
        majors = [i * major for i in range(-n, n + 1)]
        minors = [i * major / 2 for i in range(-2 * n, 2 * n + 1) if i % 2]
        self.fader = B.Fader(cv, xa + 12, 276, xb - xa - 24, -vmax, vmax,
                             app.setpoint, app.set_setpoint, step=10,
                             snap=vmax * 0.012, majors=majors, minors=minors,
                             label=B.num)
        self.fader.wheel = 5

        self.load = None
        if on_board:
            # The firmware's motor model has no load input, so in place of
            # the load slider: the torque current it is drawing.
            self.text(xa, 352, "torque current  Q15 counts", 12, B.INK3)
            self.t_load = self.text(xb, 338, "0", 32, B.INK, "ne", num=True)
            y = 404
            self.bar_y = y
            self.bar_c = (xa + xb) / 2
            self.bar_half = (xb - xa - 24) / 2
            cv.create_line(xa + 12, y, xb - 12, y, fill=B.LINE)
            for v in (-16384, -8192, 0, 8192, 16384):
                px = self.bar_c + v / 16384 * self.bar_half
                cv.create_line(px, y + 18, px, y + 27, fill=B.INK2)
                self.text(px, y + 31, B.num(v), 11, B.INK3, "n")
            self.bar = cv.create_rectangle(self.bar_c, y - 4, self.bar_c, y + 5,
                                           fill=B.ORANGE, width=0)
        else:
            self.text(xa, 352, "load torque  mNm", 12, B.INK3)
            self.t_load = self.text(xb, 338, "0", 32, B.INK, "ne", num=True)
            self.load = B.Fader(cv, xa + 12, 404, xb - xa - 24, 0, 400,
                                d.p["x_load"], self._set_load, step=5,
                                majors=(0, 100, 200, 300, 400),
                                minors=(50, 150, 250, 350), label=B.num)
            self.load.wheel = 1

        cv.create_line(xa, 470, xb, 470, fill=B.LINE)
        B.RoundButton(cv, xa + 40, 530, 70, B.GREEN, app.cmd_run, label="run",
                      label_size=12)
        B.RoundButton(cv, xa + 136, 530, 70, B.RED, app.cmd_stop, label="stop",
                      label_size=12)
        B.RoundButton(cv, xa + 222, 530, 44, B.YELLOW, app.cmd_reset,
                      label="reset", label_size=12)
        self.l_on = B.Lamp(cv, xa + 316, 498, "enabled")
        self.l_at = B.Lamp(cv, xa + 316, 522, "at speed")
        self.l_lim = B.Lamp(cv, xa + 316, 546, src.third_label)
        self.l_flt = B.Lamp(cv, xa + 316, 570, "fault")
        self.t_flt = self.text(xa + 316 + 62, 570, "", 12, B.RED, "w")

        self.text(X0, 608, "speed, last 10 s", 12, B.INK3)
        self.legend(X0 + 130, 608, ((B.WHITE, "actual"), (B.MUTE, "reference")))
        self.strip = B.Chart(cv, X0, 630, X1 - X0, 146, unit="rpm")
        cv.bind("<MouseWheel>", self._wheel)

    def _set_load(self, v):
        self.d.set_param("x_load", v)

    def _wheel(self, e):
        for f in (self.fader, self.load):
            if f is not None and f.hit(e.x, e.y):
                f.nudge((1 if e.delta > 0 else -1) * f.wheel)

    def update(self):
        d, app = self.d, self.app
        src = app.src
        self.dial.update(app.disp["rpm"], app.setpoint)
        self.set(self.t_rpm, B.num(app.disp["rpm"]))
        self.set(self.t_set, B.num(app.setpoint))
        if not self.fader.dragging:
            self.fader.set(app.setpoint)
        if self.load is not None:
            if not self.load.dragging:
                self.load.set(d.p["x_load"])
            self.set(self.t_load, B.num(d.p["x_load"]))
        else:
            iq = app.disp["iq"]
            self.set(self.t_load, B.num(iq))
            x = self.bar_c + max(-1.0, min(1.0, iq / 16384.0)) * self.bar_half
            self.cv.coords(self.bar, min(self.bar_c, x), self.bar_y - 4,
                           max(self.bar_c, x), self.bar_y + 5)

        fault = src.fault
        self.l_on.set(B.GREEN if src.enabled else None)
        self.l_at.set(B.GREEN if src.at_speed(app.setpoint) else None)
        self.l_lim.set(src.third_lamp())
        self.l_flt.set(B.RED if fault else None)
        self.set(self.t_flt, fault or "")

        rpm = src.window("rpm", 10.0, 480)
        ref = src.window("ref", 10.0, 480)
        if len(rpm) > 1 and len(ref) == len(rpm):
            lo, hi, step = B.axis(min(min(rpm), min(ref)),
                                  max(max(rpm), max(ref)), 200.0)
            self.strip.draw(((ref, B.MUTE, 1), (rpm, B.WHITE, 1.6)),
                            lo, hi, step, seconds=10.0)


class DisplayPage(Page):
    key = "display"
    title = "display"
    subtitle = "live values, readable from across the room"

    SIM = (
        ("torque", "mNm", 0, lambda a: a.disp["tq"]),
        ("torque current", "A", 2, lambda a: a.disp["iq"]),
        ("field current", "A", 2, lambda a: a.disp["id"]),
        ("voltage", "V", 1, lambda a: a.disp["v"]),
        ("power", "W", 0, lambda a: a.disp["p"]),
        ("frequency", "Hz", 1, lambda a: a.disp["f"]),
        ("setpoint", "rpm", 0, lambda a: a.setpoint),
        ("load", "%", 0,
         lambda a: abs(a.disp["iq"]) / a.drive.i_rated * 100.0),
    )
    BOARD = (
        ("torque current", "counts", 0, lambda a: a.disp["iq"]),
        ("field current", "counts", 0, lambda a: a.disp["id"]),
        ("current reference", "counts", 0, lambda a: a.disp["iqref"]),
        ("electrical angle", "°", 0,
         lambda a: a.board.state["ang"] * 360.0 / 65536.0 if a.board.state else 0),
        ("frequency", "Hz", 1, lambda a: abs(a.disp["rpm"]) / 60.0),
        ("setpoint", "rpm", 0, lambda a: a.setpoint),
        ("outputs", "", 0,
         lambda a: "on" if (a.board.state and a.board.state["moe"]) else "off"),
        ("self-test", "of 8", 0,
         lambda a: (str(bin(a.board.summary["flags"]).count("1"))
                    if "flags" in a.board.summary else "–")),
    )

    def build(self):
        cv = self.cv
        self.spec = self.BOARD if self.app.board else self.SIM
        self.t_label = self.text(X0, 128, "speed", 13, B.INK3)
        self.t_big = self.text(884, 430, "0", 250, B.INK, "se", num=True)
        self.text(900, 378, "rpm", 30, B.INK3, "sw")

        y = 458
        self.bar_y = y
        self.bar_c = (X0 + X1) / 2
        self.bar_half = (X1 - X0) / 2
        cv.create_line(X0, y, X1, y, fill=B.LINE)
        for x in (X0, self.bar_c, X1):
            cv.create_line(x, y - 6, x, y + 7, fill=B.INK3)
        self.bar = cv.create_rectangle(self.bar_c, y - 3, self.bar_c, y + 4,
                                       fill=B.ORANGE, width=0)
        self.bar_set = cv.create_rectangle(self.bar_c - 1, y - 14,
                                           self.bar_c + 1, y - 5,
                                           fill=B.INK, width=0)

        cw = (X1 - X0) / 4
        self.tiles = []
        for i, (label, unit, _dec, _get) in enumerate(self.spec):
            col, row = i % 4, i // 4
            x = X0 + col * cw
            ty = 500 + row * 142
            self.text(x + (14 if col else 0), ty, label, 13, B.INK2)
            self.text(x + cw - 14, ty, unit, 13, B.INK3, "ne")
            self.tiles.append(self.text(x + cw - 14, ty + 18, "0", 64, B.INK,
                                        "ne", num=True))
            if col:
                cv.create_line(x, ty - 2, x, ty + 112, fill=B.LINE)
        cv.create_line(X0, 628, X1, 628, fill=B.LINE)

    def update(self):
        app = self.app
        vmax = app.src.vmax
        link = app.board
        if link is None:
            self.set(self.t_label, "speed")
        elif link.speed_measured:
            self.set(self.t_label, "speed, as the firmware measures it")
        else:
            self.set(self.t_label, "speed, worked out from the angle  ·  "
                     "the firmware only measures it in speed and position mode")
        self.set(self.t_big, B.num(app.disp["rpm"]))
        x = self.bar_c + max(-1.0, min(1.0, app.disp["rpm"] / vmax)) * self.bar_half
        self.cv.coords(self.bar, min(self.bar_c, x), self.bar_y - 3,
                       max(self.bar_c, x), self.bar_y + 4)
        xs = self.bar_c + app.setpoint / vmax * self.bar_half
        self.cv.coords(self.bar_set, xs - 1, self.bar_y - 14, xs + 1,
                       self.bar_y - 5)
        for item, (_l, _u, dec, get) in zip(self.tiles, self.spec):
            value = get(app)
            self.set(item, value if isinstance(value, str) else B.num(value, dec))


# ------------------------------------------------- parameter sets to edit ---

BOARD_GROUPS = [
    (1, "current controller", "inner loop on the board, 20 kHz"),
    (2, "speed and position", "outer loops on the board, 1 kHz"),
    (3, "auto tune targets", "what auto tune designs for - kept here, not on the board"),
    (4, "voltage and weakening", "the voltage limit and what happens at it"),
    (5, "rotor angle", "encoder, hall sensors or the observer"),
    (6, "throttle and brake", "the vehicle side: inputs, limits, braking"),
    (7, "dashboard and can", "the one-line signal and the can bus"),
]
BOARD_GROUPS_V2 = BOARD_GROUPS[:3]      # what a console 2 board has

BOARD_PARAMS = [
    Param("1.01", "ckp", "proportional gain", "Q15", 6000, 0, 262144, 500, 0,
          "Current controller Kp, as the firmware stores it: 32768 is a gain "
          "of 1.0 count of voltage per count of current error."),
    Param("1.02", "cki", "integral gain", "Q15", 900, 0, 32767, 50, 0,
          "Current controller Ki per 50 µs period: 32768 would add the whole "
          "error to the output every period."),
    Param("1.03", "cdec", "decoupling", "", 1, 0, 2, 1, 0,
          "Feed-forward of the cross-coupling and back-EMF terms. Fixed uses "
          "the two values below as they stand, which is right at 6000 rpm; "
          "with speed scales both by the measured speed (console 3). Auto "
          "tune switches it off while it measures and puts it back.",
          choices=("off", "fixed", "with speed")),
    Param("1.04", "ccross", "cross-coupling term", "Q15", 20580, 0, 65535, 500, 0,
          "How much of each axis's current is fed forward into the other "
          "axis's voltage, at 6000 rpm."),
    Param("1.05", "cemf", "back-emf term", "Q15", 6554, 0, 131071, 100, 0,
          "Voltage added to the q axis to cancel the back-EMF, at 6000 rpm. "
          "The observer also uses it, to turn a back-EMF into a speed."),
    Param("2.01", "skp", "speed proportional gain", "Q15", 32768, 0, 1048576, 2000, 0,
          "Speed controller Kp: 32768 is one count of torque current per "
          "angle unit per ms of speed error."),
    Param("2.02", "ski", "speed integral gain", "Q15", 6144, 0, 131072, 100, 0,
          "Speed controller Ki per 1 ms update."),
    Param("2.03", "simax", "torque current limit", "counts", 16384, 256, 29491, 256, 0,
          "The largest torque current the speed controller may ask for. "
          "32767 is full scale."),
    Param("2.04", "pkp", "position gain", "Q15", 328, 0, 32768, 8, 0,
          "Position controller gain. 328 gives a time constant of about "
          "100 ms; it has to stay slower than the speed loop."),
    Param("3.01", "cbw", "current loop bandwidth", "Hz", 400, 50, 1500, 10, 0,
          "The bandwidth auto tune designs the current loop for. Keep it "
          "well under a tenth of the 20 kHz loop rate."),
    Param("3.02", "sbw", "speed loop bandwidth", "Hz", 20, 2, 60, 1, 0,
          "The bandwidth auto tune designs the speed loop for. It has to "
          "stay several times below the current loop's."),
    Param("4.01", "vlim", "voltage limit", "Q15", 29491, 4096, 29491, 500, 0,
          "The voltage either axis may use. 29491 is 0.9 of the supply. "
          "Lower it and the motor behaves as on a lower supply."),
    Param("4.02", "fw", "field weakening current", "counts", 0, 0, 16384, 512, 0,
          "The most negative d-axis current field weakening may add. 0 "
          "switches it off."),
    Param("4.03", "fwstart", "weakening starts at", "Q15", 29491, 16384, 32767, 500, 0,
          "The fraction of the voltage limit in use at which weakening "
          "starts: 29491 is 90 %."),
    Param("4.04", "mot", "motor model", "", 0, 0, 1, 1, 0,
          "Which motor the firmware's model is. Changing it is changing the "
          "motor on the bench: the gains do not follow. The vehicle page "
          "changes both together.", choices=("gimbal", "hub")),
    Param("5.01", "sens", "rotor angle from", "", 0, 0, 2, 1, 0,
          "Where the loop gets the rotor angle. Only while the drive is "
          "stopped, and hall only once calibrated.",
          choices=("encoder", "hall", "observer")),
    Param("5.02", "hsrc", "hall signals from", "", 0, 0, 1, 1, 0,
          "The model's own sensors, or the pins PB5, PB6, PB7 (pulled up, "
          "for open-collector sensors such as the 3144).",
          choices=("model", "pins")),
    Param("5.03", "hwid", "model sensor on-width", "units", 32768, 22000, 43000, 500, 0,
          "How much of a turn a model Hall sensor stays on. 32768 is a "
          "latch; a unipolar switch such as the 3144 gives less."),
    Param("5.04", "sla", "observer winding constant", "Q15", 1638, 128, 32767, 10, 0,
          "The control period over the winding's L/R: 1638 is 1 ms, 328 is "
          "5 ms. What the observer takes the winding to be."),
    Param("5.05", "slmin", "observer takes over at", "units/ms", 600, 100, 3000, 50, 0,
          "Above this speed the observer supplies the angle. Below it the "
          "rotor is dragged round at a forced angle."),
    Param("5.06", "slcur", "starting current", "counts", 4000, 0, 16384, 250, 0,
          "The current used while the rotor is on the forced angle."),
    Param("5.07", "slramp", "starting ramp", "units/ms/s", 2000, 100, 20000, 100, 0,
          "How fast the forced angle speeds up."),
    Param("6.01", "insrc", "throttle and brake from", "", 0, 0, 1, 1, 0,
          "This window, or the pins: throttle on PA2 (0 to 3.3 V), brake "
          "switch from PB11 to ground.", choices=("window", "pins")),
    Param("6.02", "thmin", "throttle released", "counts", 900, 0, 4095, 10, 0,
          "The reading with the throttle released. A reading under half of "
          "this is taken for a broken wire."),
    Param("6.03", "thmax", "throttle fully open", "counts", 3300, 0, 4095, 10, 0,
          "The reading at full throttle. A reading more than half way from "
          "here to 4095 is taken for a short to the supply."),
    Param("6.04", "thmode", "throttle is", "", 0, 0, 1, 1, 0,
          "Torque: the throttle sets the torque current, up to the limit in "
          "2.03. Speed: it sets the speed, up to the top speed.",
          choices=("torque", "speed")),
    Param("6.05", "thramp", "throttle ramp", "/ms", 66, 1, 32767, 5, 0,
          "How fast the throttle may rise, out of 32767 per ms. 66 is half "
          "a second from closed to open. It falls at once."),
    Param("6.06", "drvspd", "top speed", "units/ms", 6560, 100, 16000, 100, 0,
          "The speed at which throttle torque is taken away. 6560 is 6000 rpm."),
    Param("6.07", "regen", "braking current", "counts", 4096, 0, 16384, 256, 0,
          "The current the brake regenerates with. 0: the brake only cuts "
          "the throttle."),
    Param("7.01", "ol", "one-line output", "", 0, 0, 1, 1, 0,
          "The dashboard signal on PB10.", choices=("off", "on")),
    Param("7.02", "olt", "one-line time unit", "x 50 µs", 10, 5, 40, 1, 0,
          "The unit every pulse of the signal is a multiple of. 10 is 0.5 ms."),
    Param("7.03", "canid", "can node number", "", 1, 1, 127, 1, 0,
          "Added to each frame's identifier."),
    Param("7.04", "canbr", "can bit rate", "kbit/s", 500, 125, 1000, 125, 0,
          "125, 250, 500 or 1000."),
    Param("7.05", "canper", "can status period", "ms", 20, 5, 1000, 5, 0,
          "How often the two status frames go out."),
    Param("7.06", "canback", "back to usb after", "s", 10, 0, 600, 1, 0,
          "On CAN, with no frame arriving for this long, the board goes back "
          "to its USB console. 0: it stays."),
]
BOARD_V3_KEYS = {q.key for q in BOARD_PARAMS if q.group >= 4}


class SimParams:
    """The parameters page, editing the simulator."""

    kind = "sim"
    groups = GROUPS
    params = PARAMS
    note = ""

    def __init__(self, app):
        self.app = app
        self.by_key = BY_KEY

    def value(self, q):
        return self.app.drive.p[q.key]

    def set(self, q, value):
        return self.app.set_param(q.key, value)

    def load(self, values):
        app = self.app
        d = app.drive
        if app.tune.active or app.tune.pending:
            return "finish auto tune first."
        if d.enabled:
            return "stop the drive before loading a parameter set."
        if abs(d.rpm) > 5.0:
            return "wait for the shaft to come to rest, then load."
        known = {k: v for k, v in values.items() if k in BY_KEY}
        d.load_params(known)
        app._rescale()
        return f"loaded {len(known)} parameters"


class BoardParams:
    """The parameters page, editing the board (console version 2)."""

    kind = "board"

    @property
    def note(self):
        # Console 4 keeps them; anything older cannot, and saying so is the
        # whole difference between a board you can tune and one you have to
        # tune again at every switch-on.
        link = self.app.board
        if link is not None and link.store_supported:
            st = (link.store or {}).get("state")
            if st == "ok":
                return ("these are in the board's memory. its flash page "
                        "already holds a set - keep these in the board to "
                        "replace it, or take the board's own set to go back "
                        "to it.")
            return ("these are in the board's memory and go back to the "
                    "built-in values when it is reset, until keep these in "
                    "the board writes them to its flash page.")
        return ("these are in the board's memory and go back to the built-in "
                "values when it is reset. this console cannot store them; "
                "console 4 added save and load.")

    @property
    def groups(self):
        return BOARD_GROUPS if self.app.board_has_vehicle() else BOARD_GROUPS_V2

    @property
    def params(self):
        if self.app.board_has_vehicle():
            return BOARD_PARAMS
        return [q for q in BOARD_PARAMS if q.key not in BOARD_V3_KEYS]

    def __init__(self, app):
        self.app = app
        self.by_key = {q.key: q for q in BOARD_PARAMS}

    def value(self, q):
        if q.key in self.app.board_targets:
            return self.app.board_targets[q.key]
        return float(self.app.board.params.get(q.key, q.default))

    def set(self, q, value):
        app = self.app
        tune = app.btune
        if tune is not None and tune.active:
            return False, "auto tune is running."
        if tune is not None and tune.pending:
            return False, "apply or discard the auto tune result first."
        value = round(min(q.hi, max(q.lo, float(value))))
        if q.key in app.board_targets:
            app.board_targets[q.key] = float(value)
        else:
            # Shown at once; the board's own reply then confirms or corrects it.
            app.board.params[q.key] = int(value)
            app.board.set_param(q.key, value)
        return True, ""

    def load(self, values):
        n = 0
        for q in self.params:
            if q.key in values:
                ok, why = self.set(q, values[q.key])
                if not ok:
                    return why
                n += 1
        return f"sent {n} parameters to the board"


class TunePage(Page):
    key = "tune"
    title = "auto tune"

    MOTOR = (("m_rs", "x_rs"), ("m_ls", "x_ls"), ("m_flux", "x_flux"),
             ("m_j", "x_j"))
    GAINS = (("c_kp", "current  Kp"), ("c_ki", "current  Ki"),
             ("s_kp", "speed  Kp"), ("s_ki", "speed  Ki"))
    PLANT = (("gain", "winding gain", 3), ("tau_ms", "electrical time constant  ms", 3),
             ("accel", "acceleration constant", 2),
             ("tau_m_ms", "mechanical time constant  ms", 1))
    BOARD_GAINS = (("ckp", "current  Kp  Q15"), ("cki", "current  Ki  Q15"),
                   ("skp", "speed  Kp  Q15"), ("ski", "speed  Ki  Q15"))
    LAMP = {"pending": B.Lamp.OFF, "running": B.ORANGE, "done": B.INK,
            "failed": B.RED}

    def setup(self):
        self.kind = None

    def want(self):
        link = self.app.board
        return "board" if (link is not None and (link.fw or 0) >= 2) else "sim"

    def sub(self):
        if self.kind == "board":
            return ("measure the plant through the board's console, then "
                    "design both controllers from what was measured")
        return "measure the motor, then design both controllers from what was measured"

    def rebuild(self):
        self.kind = self.want()
        super().rebuild()

    def build(self):
        cv, app = self.cv, self.app
        tune = app.active_tune()
        xa, xb = X0, 492
        cv.create_rectangle(xa, 126, xb, 129, fill=B.LINE, width=0)
        self.prog = cv.create_rectangle(xa, 126, xa, 129, fill=B.ORANGE, width=0)
        self.rows = []
        for i, step in enumerate(tune.steps):
            y = 148 + i * 58
            lamp = cv.create_image(xa + 8, y + 12)
            self.text(xa + 28, y + 3, str(i + 1), 12, B.INK3)
            self.text(xa + 50, y - 1, step["name"], 17, B.INK)
            self.text(xa + 50, y + 24, step["detail"], 12, B.INK3)
            value = self.text(xb, y + 2, "", 16, B.INK, "ne", num=True)
            cv.create_line(xa, y + 50, xb, y + 50, fill=B.LINE)
            self.rows.append((lamp, value))
        self._lamps = [None] * len(self.rows)
        self.t_msg = self.text(xa, 566, "", 13, B.INK2, width=xb - xa)

        self.b_start = B.RoundButton(cv, xa + 46, 664, 84, B.ORANGE,
                                     app.tune_start, label="start", label_size=12)
        self.b_abort = B.RoundButton(cv, xa + 152, 664, 48, B.RED,
                                     lambda: app.active_tune().abort(),
                                     label="abort")
        self.b_apply = B.RoundButton(cv, xa + 240, 664, 48, B.GREEN,
                                     lambda: app.active_tune().apply(),
                                     label="apply")
        self.b_disc = B.RoundButton(cv, xa + 328, 664, 48, B.DARK2,
                                    lambda: app.active_tune().discard(),
                                    label="discard")

        xa, xb = 548, X1
        cols = (742, 838, 930, xb)
        self.cols = cols
        if self.kind == "board":
            self.text(xa, 128, "plant", 12, B.INK3)
            for x, head in zip(cols[1:], ("identified", "vmotor.c", "difference")):
                self.text(x, 128, head, 12, B.INK3, "ne")
            self.plant = []
            for i, (_key, label, _dec) in enumerate(self.PLANT):
                y = 158 + i * 32
                self.text(xa, y + 2, label, 13, B.INK)
                self.plant.append([self.text(x, y, "", 17, B.INK, "ne", num=True)
                                   for x in cols[1:]])
            gains = self.BOARD_GAINS
            self.step_text = (f"speed step, "
                              f"{boardtune.BoardTune.STEP / boardlink.UNITS_PER_RPM:.0f} rpm")
        else:
            self.text(xa, 128, "motor data", 12, B.INK3)
            for x, head in zip(cols, ("before", "identified", "motor", "error")):
                self.text(x, 128, head, 12, B.INK3, "ne")
            self.motor = []
            for i, (key, _plant) in enumerate(self.MOTOR):
                y = 158 + i * 32
                q = BY_KEY[key]
                self.text(xa, y + 2, f"{q.name}  {q.unit}", 13, B.INK)
                self.motor.append([self.text(x, y, "", 17, B.INK, "ne", num=True)
                                   for x in cols])
            gains = tuple((key, f"{name}  {BY_KEY[key].unit}")
                          for key, name in self.GAINS)
            self.step_text = "speed step, 200 rpm"
        cv.create_line(xa, 148, xb, 148, fill=B.LINE)

        self.text(xa, 306, "controller gains", 12, B.INK3)
        for x, head in zip((cols[1], xb), ("before", "after")):
            self.text(x, 306, head, 12, B.INK3, "ne")
        cv.create_line(xa, 326, xb, 326, fill=B.LINE)
        self.gains = []
        for i, (key, label) in enumerate(gains):
            y = 336 + i * 32
            self.text(xa, y + 2, label, 13, B.INK)
            self.gains.append((key, [self.text(x, y, "", 17, B.INK, "ne", num=True)
                                     for x in (cols[1], xb)]))

        self.text(xa, 490, self.step_text, 12, B.INK3)
        self.legend(xa + 150, 490, ((B.MUTE, "before"), (B.ORANGE, "after")))
        self.screen = B.Chart(cv, xa, 512, xb - xa, 194, unit="counts")
        self.t_mb = self.text(xa, 716, "", 13, B.INK3)
        self.t_ma = self.text(xa, 738, "", 13, B.INK)
        self._plotted = None

    @staticmethod
    def _metrics(tag, m, window_ms):
        if not m:
            return f"{tag}   –"
        settle = (f"{m['settle_ms']} ms to settle" if m["settle_ms"] is not None
                  else f"not settled in {window_ms} ms")
        return f"{tag}   {m['overshoot']:.0f} % overshoot   {settle}"

    def update(self):
        if self.want() != self.kind:
            self.rebuild()
        cv, d, app = self.cv, self.d, self.app
        tune = app.active_tune()
        res = tune.result
        on_board = self.kind == "board"

        cv.coords(self.prog, X0, 126, X0 + (492 - X0) * tune.progress, 129)
        for i, (step, (lamp, value)) in enumerate(zip(tune.steps, self.rows)):
            color = self.LAMP[step["state"]]
            if self._lamps[i] != color:
                self._lamps[i] = color
                cv.itemconfigure(lamp, image=B.disc(14, color, flat=True))
            self.set(value, step["value"])
        self.set(self.t_msg, NOTE_TUNE if (app.board and not on_board)
                 else tune.message)
        self.b_start.set_color(B.ORANGE if not tune.active else B.SLOT)
        self.b_abort.set_color(B.RED if tune.active else B.SLOT)
        self.b_apply.set_color(B.GREEN if tune.pending else B.SLOT)
        self.b_disc.set_color(B.DARK2 if tune.pending else B.SLOT)

        before = res.get("before", {})
        after = res.get("after", {})
        if on_board:
            for (key, _label, dec), cells in zip(self.PLANT, self.plant):
                ref = boardtune.MODEL[key]
                got = res.get(key)
                self.set(cells[1], B.num(ref, dec))
                if got is None:
                    self.set(cells[0], "–")
                    self.set(cells[2], "–")
                else:
                    self.set(cells[0], B.num(got, dec))
                    self.set(cells[2], B.num((got - ref) / ref * 100.0, 1) + " %")
            params = app.board.params
            for key, cells in self.gains:
                shown = before.get(key, params.get(key))
                self.set(cells[0], "–" if shown is None else B.num(shown))
                self.set(cells[1], B.num(after[key]) if key in after else "–")
            step = boardtune.BoardTune.STEP / boardlink.UNITS_PER_RPM
            scale = 1.0 / boardlink.UNITS_PER_RPM
            window = 512
        else:
            idle = not tune.active and not tune.pending
            for (key, plant), cells in zip(self.MOTOR, self.motor):
                q = BY_KEY[key]
                shown = before.get(key, d.p[key]) if not idle or before else d.p[key]
                self.set(cells[0], B.num(shown, q.decimals))
                if key in res:
                    err = (res[key] - d.p[plant]) / d.p[plant] * 100.0
                    self.set(cells[1], B.num(res[key], q.decimals))
                    self.set(cells[3], B.num(err, 1) + " %")
                else:
                    self.set(cells[1], "–")
                    self.set(cells[3], "–")
                self.set(cells[2], B.num(d.p[plant], q.decimals))
            for key, cells in self.gains:
                q = BY_KEY[key]
                self.set(cells[0], B.num(before.get(key, d.p[key]), q.decimals))
                self.set(cells[1], B.num(after[key], q.decimals) if key in after
                         else "–")
            step = tune.STEP_RPM
            scale = 1.0
            window = 600

        sig = (len(tune.resp_before), len(tune.resp_after), self.kind)
        if sig != self._plotted:
            self._plotted = sig
            series = []
            if tune.resp_before:
                series.append(([v * scale for v in tune.resp_before], B.MUTE, 1.5))
            if tune.resp_after:
                series.append(([v * scale for v in tune.resp_after], B.ORANGE, 2))
            top = max([1.3 * step] + [max(s[0]) * 1.08 for s in series])
            grid = B.nice_step(top, 4)
            hi = math.ceil(top / grid) * grid
            series.insert(0, ([step] * 2, B.DARK3, 1))
            lo, top, step = B.axis(-hi / 8.0, hi, 200.0)
            self.screen.draw(series, lo, top, step, seconds=float(window),
                             xlabel="milliseconds")
        self.set(self.t_mb, self._metrics("before", res.get("metrics_before"), window))
        self.set(self.t_ma, self._metrics("after", res.get("metrics_after"), window))


class ParamPage(Page):
    key = "params"
    title = "parameters"
    ROWS = 7

    def setup(self):
        self.ps = self.app.param_source()
        self.group = self.ps.groups[0][0]
        self.sel = self.ps.params[0].key
        self.msg = ""
        self.entry = tk.Entry(self.cv, font=B.font(20, num=True), bd=0,
                              relief="flat", bg=B.PANEL, fg=B.INK,
                              insertbackground=B.INK, justify="right",
                              highlightthickness=1, highlightbackground=B.LINE,
                              highlightcolor=B.ORANGE)
        self.entry.bind("<Return>", self._typed)
        self.entry.bind("<Escape>", lambda e: self.cv.focus_set())
        self._entry_for = None

    def sub(self):
        if self.ps.kind == "board":
            return (f"the gains in the firmware on {self.app.board.port}, "
                    "and what auto tune aims for")
        return "every setting of the drive, and of the motor it is connected to"

    def build(self):
        cv = self.cv
        self.g_items = []
        for i, (gno, name, _desc) in enumerate(self.ps.groups):
            y = 132 + i * 50
            tag = f"grp{gno}"
            cv.create_rectangle(X0 - 4, y - 8, 262, y + 34, fill=B.PAPER,
                                outline="", tags=(tag,))
            bar = cv.create_rectangle(X0, y, X0 + 3, y + 24, fill=B.ORANGE,
                                      width=0, tags=(tag,))
            self.text(X0 + 14, y + 5, f"{gno:02d}", 12, B.INK3, tags=(tag,))
            label = self.text(X0 + 42, y + 1, name, 16, B.INK2, tags=(tag,))
            count = self.text(262, y + 24, "", 11, B.ORANGE, "ne", tags=(tag,))
            cv.tag_bind(tag, "<ButtonPress-1>", lambda e, g=gno: self.pick_group(g))
            self.g_items.append((gno, bar, label, name, count))

        buttons = [("restore this default", self._default),
                   ("reset this group", self._reset_group),
                   ("reset everything", self._reset_all),
                   ("save to a file", self._save),
                   ("load from a file", self._load)]
        # A file is the host's copy; the flash page is the board's own, and
        # it is the one that decides what the board comes up as. Only a
        # board that can keep them gets the buttons.
        if self.ps.kind == "board":
            buttons += [("keep these in the board", self._store_save),
                        ("take the board's own set", self._store_load)]
        step = 44 if len(buttons) <= 5 else 38
        for i, (label, fn) in enumerate(buttons):
            B.RoundButton(cv, X0 + 14, 520 + i * step, 28, B.DARK2, fn,
                          label=label, label_side="e", label_size=13,
                          label_color=B.INK2)

        xa = 300
        self.cols = dict(no=xa + 14, name=xa + 70, value=760, unit=774,
                         range=850, default=X1)
        for key, head, anchor in (("no", "no.", "nw"), ("name", "parameter", "nw"),
                                  ("value", "value", "ne"), ("unit", "unit", "nw"),
                                  ("range", "range", "nw"),
                                  ("default", "default", "ne")):
            self.text(self.cols[key], 128, head, 12, B.INK3, anchor)
        cv.create_line(xa, 148, X1, 148, fill=B.LINE)
        self.rows = []
        for i in range(self.ROWS):
            y = 154 + i * 44
            tag = f"row{i}"
            bg = cv.create_rectangle(xa, y, X1 + 8, y + 40, fill=B.PAPER,
                                     outline="", tags=(tag,))
            bar = cv.create_rectangle(xa, y, xa + 3, y + 40, fill=B.ORANGE,
                                      width=0, tags=(tag,))
            dot = cv.create_rectangle(xa + 5, y + 18, xa + 10, y + 23,
                                      fill=B.ORANGE, width=0, tags=(tag,))
            c = self.cols
            items = dict(
                bg=bg, bar=bar, dot=dot,
                no=self.text(c["no"], y + 12, "", 13, B.INK3, tags=(tag,)),
                name=self.text(c["name"], y + 10, "", 16, B.INK, tags=(tag,)),
                value=self.text(c["value"], y + 6, "", 22, B.INK, "ne",
                                num=True, tags=(tag,)),
                unit=self.text(c["unit"], y + 13, "", 12, B.INK3, tags=(tag,)),
                range=self.text(c["range"], y + 13, "", 12, B.INK3, tags=(tag,)),
                default=self.text(c["default"], y + 13, "", 12, B.INK3, "ne",
                                  tags=(tag,)))
            cv.tag_bind(tag, "<ButtonPress-1>", lambda e, i=i: self.pick_row(i))
            self.rows.append(items)
        cv.create_line(xa, 472, X1, 472, fill=B.LINE)

        self.e_no = self.text(xa, 490, "", 13, B.INK3)
        self.e_name = self.text(xa, 510, "", 26, B.INK)
        self.e_help = self.text(xa, 552, "", 13, B.INK2, width=330)
        self.e_val = self.text(X1 - 62, 484, "", 72, B.INK, "ne", num=True)
        self.e_unit = self.text(X1 - 54, 544, "", 14, B.INK3, "sw")
        B.RoundButton(cv, 700, 650, 46, B.DARK2, lambda: self._bump(-1),
                      glyph="−", glyph_size=22)
        B.RoundButton(cv, 760, 650, 46, B.DARK2, lambda: self._bump(1),
                      glyph="+", glyph_size=22)
        cv.create_window(808, 632, anchor="nw", window=self.entry,
                         width=X1 - 808, height=36)
        self.text(X1, 676, "type a value, then enter", 11, B.INK3, "ne")
        self.e_msg = self.text(xa, 716, "", 13, B.ORANGE, width=X1 - xa)
        self._entry_for = None

    # ---- selection

    def _params(self):
        return [q for q in self.ps.params if q.group == self.group]

    def pick_group(self, gno):
        self.group = gno
        self.sel = self._params()[0].key
        self.msg = ""

    def pick_row(self, i):
        params = self._params()
        if i < len(params):
            self.sel = params[i].key
            self.msg = ""

    def select(self, key):
        self.group = self.ps.by_key[key].group
        self.sel = key

    # ---- editing

    def _apply(self, value):
        ok, why = self.ps.set(self.ps.by_key[self.sel], value)
        self.msg = "" if ok else why
        self._entry_for = None

    def _bump(self, direction):
        q = self.ps.by_key[self.sel]
        self._apply(self.ps.value(q) + direction * q.step)

    def _typed(self, _e):
        q = self.ps.by_key[self.sel]
        raw = self.entry.get().strip().lower().replace(",", ".")
        raw = raw.replace("−", "-")
        if q.choices and raw in q.choices:
            self._apply(q.choices.index(raw))
        else:
            try:
                self._apply(float(raw))
            except ValueError:
                self.msg = f"“{raw}” is not a number."
        self.cv.focus_set()

    def _default(self):
        self._apply(self.ps.by_key[self.sel].default)

    def _reset(self, params):
        """Put parameters back to their defaults. One that cannot be changed
        right now is skipped and named, not allowed to stop the rest."""
        blocked = []
        reason = ""
        for q in params:
            if self.ps.value(q) == q.default:
                continue
            ok, why = self.ps.set(q, q.default)
            if not ok:
                blocked.append(q.number)
                reason = why
        if not blocked:
            self.msg = ""
        elif len(blocked) <= 3:
            self.msg = f"{', '.join(blocked)} not reset: {reason}"
        else:
            self.msg = f"{len(blocked)} parameters not reset: {reason}"
        self._entry_for = None

    def _reset_group(self):
        self._reset(self._params())

    def _reset_all(self):
        self._reset(self.ps.params)

    def save_to(self, path):
        ps = self.ps
        doc = {"application": "FOC studio", "version": VERSION,
               "source": ps.kind,
               "parameters": {q.number: {"key": q.key, "name": q.name,
                                         "value": ps.value(q), "unit": q.unit}
                              for q in ps.params}}
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(doc, f, indent=2, ensure_ascii=False)
        except OSError as err:
            return f"could not save: {err}"
        return f"saved {len(ps.params)} parameters to {os.path.basename(path)}."

    def load_from(self, path):
        ps = self.ps
        try:
            with open(path, encoding="utf-8") as f:
                doc = json.load(f)
            source = doc.get("source", "sim")
            values = {v["key"]: float(v["value"])
                      for v in doc["parameters"].values()}
        except (OSError, ValueError, KeyError, TypeError, AttributeError) as err:
            return f"could not load: {err}"
        if source != ps.kind:
            other = "the simulator" if source == "sim" else "a board"
            return (f"{os.path.basename(path)} holds parameters for {other}. "
                    "nothing was loaded.")
        self._entry_for = None
        return ps.load(values) + f" from {os.path.basename(path)}."

    def _save(self):
        path = filedialog.asksaveasfilename(
            parent=self.app, title="Save parameters", defaultextension=".json",
            initialfile=("foc-board-gains.json" if self.ps.kind == "board"
                         else "foc-parameters.json"),
            filetypes=(("Parameter set", "*.json"),))
        if path:
            self.msg = self.save_to(path)

    def _load(self):
        path = filedialog.askopenfilename(
            parent=self.app, title="Load parameters",
            filetypes=(("Parameter set", "*.json"),))
        if path:
            self.msg = self.load_from(path)


    def _store_save(self):
        link = self.app.board
        if link is None:
            self.msg = "connect to a board first."
        elif not link.store_supported:
            self.msg = ("this firmware cannot keep its settings. "
                        "console 4 added save and load.")
        else:
            link.store_save()
            self.msg = "writing the flash page..."

    def _store_load(self):
        link = self.app.board
        if link is None:
            self.msg = "connect to a board first."
        elif not link.store_supported:
            self.msg = ("this firmware cannot keep its settings. "
                        "console 4 added save and load.")
        else:
            link.store_load()
            self.msg = "reading the flash page..."

    def _store_msg(self):
        """What the board last answered about its flash page, in words."""
        link = self.app.board
        if link is None or not link.store_supported:
            return None
        reply = link.store_reply
        if reply is None:
            return None
        what = reply.split()[0]
        if not reply.endswith(" ok"):
            return reply                    # it says why itself
        st = link.store or {}
        if what == "save":
            n = st.get("params", 0)
            hall = " and the hall table" if st.get("hall") else ""
            return (f"kept in the board: {n} parameters{hall}. "
                    "it comes up on these after a power cycle.")
        if what == "load":
            return "the board's own set is back."
        return "the flash page is erased. the board comes up on its defaults."
    # ---- drawing

    def update(self):
        cv, app = self.cv, self.app
        ps = app.param_source()
        if ps.kind != self.ps.kind or len(ps.groups) != len(self.g_items):
            self.ps = ps
            self.group = ps.groups[0][0]
            self.sel = ps.params[0].key
            self.msg = ""
            self.rebuild()
        ps = self.ps
        params = self._params()
        for gno, bar, label, name, count in self.g_items:
            on = gno == self.group
            changed = sum(1 for q in ps.params
                          if q.group == gno and ps.value(q) != q.default)
            cv.itemconfigure(bar, state="normal" if on else "hidden")
            self.set(label, name, B.INK if on else B.INK2)
            self.set(count, f"{changed} changed" if changed else "")

        for i, items in enumerate(self.rows):
            if i >= len(params):
                for key in ("no", "name", "value", "unit", "range", "default"):
                    self.set(items[key], "")
                cv.itemconfigure(items["bar"], state="hidden")
                cv.itemconfigure(items["dot"], state="hidden")
                cv.itemconfigure(items["bg"], fill=B.PAPER)
                continue
            q = params[i]
            on = q.key == self.sel
            value = ps.value(q)
            cv.itemconfigure(items["bg"], fill=B.PANEL if on else B.PAPER)
            cv.itemconfigure(items["bar"], state="normal" if on else "hidden")
            cv.itemconfigure(items["dot"], state="normal"
                             if (value != q.default and not on) else "hidden")
            self.set(items["no"], q.number)
            self.set(items["name"], q.name)
            self.set(items["value"], B.num(value, q.decimals)
                     if not q.choices else q.fmt(value),
                     B.ORANGE if value != q.default else B.INK)
            self.set(items["unit"], q.unit)
            self.set(items["range"], " / ".join(q.choices) if q.choices else
                     f"{B.num(q.lo, q.decimals)} … {B.num(q.hi, q.decimals)}")
            self.set(items["default"], q.fmt(q.default))

        q = ps.by_key[self.sel]
        value = ps.value(q)
        shown = q.fmt(value) if q.choices else B.num(value, q.decimals)
        self.set(self.e_no, f"parameter {q.number}" +
                 ("   ·   locked while running" if q.runlock else ""))
        self.set(self.e_name, q.name)
        self.set(self.e_help, q.help)
        self.set(self.e_val, shown)
        self.set(self.e_unit, q.unit)
        note = ps.note
        if app.board is not None and ps.kind == "sim":
            note = NOTE_PARAMS
        # A flash save or load is answered by the board, a moment later and
        # on the reader thread; the "..." stands until the answer arrives.
        if self.msg.endswith("flash page..."):
            answered = self._store_msg()
            if answered:
                self.msg = answered
        self.set(self.e_msg, self.msg or note, B.ORANGE if self.msg else B.INK3)
        if self._entry_for != (q.key, value) and app.focus_get() is not self.entry:
            self._entry_for = (q.key, value)
            self.entry.delete(0, "end")
            self.entry.insert(0, q.fmt(value))


class ScopePage(Page):
    key = "scope"
    title = "scope"
    SPANS = (0.2, 1.0, 5.0, 20.0)
    PHASE_MS = 50.0                  # the simulator's own phase-current window
    CAP_EVERY = 0.4                  # seconds between records asked of the board
    CAP_PAIRS = 128                  # what one iab record holds

    def setup(self):
        self.span = 2
        self.hold = False
        self.cap_seen = -1
        self.cap_sent = 0.0
        self.cap_due = 0.0           # when the armed record will be full
        self.cap_div = 1
        self.cap_fails = 0
        self.cap_wait = False        # armed, not yet asked for
        self.phase = None            # (ia, ib, milliseconds) read off the board

    def sub(self):
        return self.app.src.scope_note

    def build(self):
        cv = self.cv
        src = self.app.src
        self.b_hold = B.RoundButton(cv, 760, 50, 30, B.DARK2, self._toggle_hold,
                                    label="hold", label_size=10,
                                    label_color=B.INK3)
        self.b_span = []
        for i, s in enumerate(self.SPANS):
            self.b_span.append(B.RoundButton(
                cv, 840 + i * 52, 50, 30, B.DARK2,
                lambda i=i: setattr(self, "span", i), label=f"{s:g} s",
                label_size=10, label_color=B.INK3))

        self.text(X0, 112, "speed", 12, B.INK3)
        self.legend(X0 + 60, 112, ((B.WHITE, "actual"), (B.MUTE, "reference")))
        self.c_speed = B.Chart(cv, X0, 132, X1 - X0, 178, unit="rpm")

        self.text(X0, 326, "current", 12, B.INK3)
        self.legend(X0 + 70, 326, ((B.ORANGE, "torque, i q"),
                                   (B.MUTE, "its reference"),
                                   (B.YELLOW, "field, i d")))
        self.c_cur = B.Chart(cv, X0, 346, X1 - X0, 178, unit=src.cur_unit)

        self.text(X0, 540, "phase currents", 12, B.INK3)
        self.legend(X0 + 110, 540, ((B.WHITE, "a"), (B.ORANGE, "b"),
                                    (B.GREEN, "c")))
        self.c_abc = B.Chart(cv, X0, 560, 664, 204, unit=src.cur_unit,
                             fill=False)
        self.text(736, 540, "current vector", 12, B.INK3)
        self.t_xy = self.text(848, 540, "", 12, B.INK3)
        self.c_xy = B.Chart(cv, 736, 560, X1 - 736, 204, pad_right=56)

    def _toggle_hold(self):
        self.hold = not self.hold

    # ---- the board's own recorder -----------------------------------------

    def _ask_record(self, link, rpm_elec_hz):
        """One record of both phase currents, thinned so that a couple of
        electrical cycles fill the window however fast the motor is going.

        Arming and reading out are two steps with a wait between them: the
        record is not full until 128 periods of its divider have gone by,
        and a dump sent before then returns the handful of samples written
        so far.
        """
        now = time.perf_counter()
        if link.cap_serial != self.cap_seen:
            self.cap_seen = link.cap_serial
            self.cap_wait = False
            if getattr(link, "cap_chan", None) == "iab":
                d = link.cap_data
                if len(d) >= 16:
                    ms = self.CAP_PAIRS * link.cap_div / 20.0
                    self.phase = (d[0::2], d[1::2], ms)
                    self.cap_fails = 0

        if self.cap_wait:
            if now >= self.cap_due:
                link.send("dump")
                self.cap_wait = False
                self.cap_sent = now
            return
        if now - self.cap_sent < self.CAP_EVERY:
            return
        if self.cap_sent and self.phase is None:
            # Asked, and nothing came back: an older firmware without the
            # channel answers the question with a refusal, not a record.
            self.cap_fails += 1
        want_ms = (2000.0 / rpm_elec_hz) if rpm_elec_hz > 1.0 else 12.8
        div = int(round(want_ms * 20.0 / self.CAP_PAIRS))
        self.cap_div = max(1, min(40, div))
        link.send(f"cap iab {self.cap_div}")
        self.cap_sent = now
        self.cap_due = now + self.CAP_PAIRS * self.cap_div / 20000.0 + 0.03
        self.cap_wait = True

    def _phase_from_board(self):
        src = self.app.src
        link = src.link
        tune = self.app.active_tune()
        if getattr(tune, "active", False):
            return None                      # the recorder is tuning, not ours
        st = link.state or {}
        hz = abs(st.get("spd", 0)) * 1000.0 / 65536.0
        self._ask_record(link, hz)
        if self.phase is None and self.cap_fails >= 3:
            return False                     # firmware without the channel
        return self.phase

    # ---- drawing -----------------------------------------------------------

    def update(self):
        src = self.app.src
        self.b_hold.set_color(B.YELLOW if self.hold else B.DARK2)
        for i, b in enumerate(self.b_span):
            b.set_color(B.ORANGE if i == self.span else B.DARK2)
        if self.hold:
            return
        span = self.SPANS[self.span]

        rpm = src.window("rpm", span, 960)
        ref = src.window("ref", span, 960)
        iq = src.window("iq", span, 960)
        iqref = src.window("iqref", span, 960)
        idd = src.window("id", span, 960)
        n = min(len(rpm), len(ref), len(iq), len(iqref), len(idd))
        if n < 2:
            return
        # a sample may arrive between two of the reads above
        rpm, ref, iq, iqref, idd = (v[-n:] for v in (rpm, ref, iq, iqref, idd))

        lo, hi, step = B.axis(min(min(rpm), min(ref)),
                              max(max(rpm), max(ref)), 100.0)
        self.c_speed.draw(((ref, B.MUTE, 1), (rpm, B.WHITE, 1.6)),
                          lo, hi, step, seconds=span)

        lo, hi, step = B.axis(min(min(iq), min(idd), min(iqref)),
                              max(max(iq), max(idd), max(iqref)), src.cur_span)
        self.c_cur.draw(((iqref, B.MUTE, 1), (idd, B.YELLOW, 1.2),
                         (iq, B.ORANGE, 1.6, True)), lo, hi, step,
                        seconds=span, dec=src.cur_dec)

        if src.kind == "board":
            got = self._phase_from_board()
            if got is False:
                self.c_abc.note("this firmware does not record the phase "
                                "currents")
                self.c_xy.note("needs console 4 with the iab channel")
                return
            if not got:
                self.c_abc.note("reading the recorder…")
                self.c_xy.note("")
                return
            ia, ib, ms = got
        else:
            ia = src.window("ia", self.PHASE_MS / 1000.0, 400)
            ib = src.window("ib", self.PHASE_MS / 1000.0, 400)
            ms = self.PHASE_MS
        if len(ia) < 2 or len(ib) < 2:
            return
        n = min(len(ia), len(ib))
        ia, ib = ia[-n:], ib[-n:]
        self.c_abc.note("")
        self.c_xy.note("")

        ic = [-a - b for a, b in zip(ia, ib)]
        peak = max(max(abs(v) for v in ia), max(abs(v) for v in ib),
                   src.cur_span * 0.05)
        # a finer target here: a current that peaks at 1900 counts on an
        # axis that stops at 4000 is drawn half size for no reason
        lo, hi, step = B.axis(-peak * 1.15, peak * 1.15,
                              src.cur_span * 0.1, target=6)
        self.c_abc.draw(((ic, B.GREEN, 1.4), (ib, B.ORANGE, 1.4),
                         (ia, B.WHITE, 1.4)), lo, hi, step, seconds=ms,
                        xlabel="milliseconds", dec=src.cur_dec)

        beta = [(a + 2.0 * b) * 0.5773502691896258 for a, b in zip(ia, ib)]
        self.c_xy.xy(ia, beta, hi, B.ORANGE,
                     label=f"α β   full scale {B.num(hi, src.cur_dec)}"
                           f" {src.cur_unit}")
        self.set(self.t_xy,
                 f"{ms:.1f} ms of it, at {20.0 / max(1, self.cap_div):.0f} kHz"
                 if src.kind == "board" else "")


class BoardPage(Page):
    key = "board"
    title = "board"
    subtitle = "the stm32 foc firmware, over its usb console"
    PORT_ROWS = 4
    LOG_LINES = 13

    SUMMARY = (
        ("adc_hz", "adc trigger rate", "Hz"),
        ("reset", "last reset", ""),
        ("watchdog", "watchdog", ""),
        ("align_err", "alignment error", "angle units"),
        ("settle", "current loop settling", "periods"),
        ("ctrl_cycles", "control path", "cpu cycles"),
        ("moe", "outputs", ""),
        ("fault", "fault", ""),
    )

    def setup(self):
        self.ports = []
        self.sel = None
        self.msg = ""
        self._scan_at = 0.0
        self._log_serial = None
        self.entry = tk.Entry(self.cv, font=("Consolas", -15), bd=0,
                              relief="flat", bg=B.PANEL, fg=B.INK,
                              insertbackground=B.INK, highlightthickness=1,
                              highlightbackground=B.LINE,
                              highlightcolor=B.ORANGE)
        self.entry.bind("<Return>", self._typed)
        self.entry.bind("<Escape>", lambda e: self.cv.focus_set())

    def build(self):
        cv = self.cv
        xa, xb = X0, 470
        self.text(xa, 128, "serial port", 12, B.INK3)
        cv.create_line(xa, 148, xb, 148, fill=B.LINE)
        self.rows = []
        for i in range(self.PORT_ROWS):
            y = 154 + i * 44
            tag = f"port{i}"
            bg = cv.create_rectangle(xa, y, xb, y + 40, fill=B.PAPER,
                                     outline="", tags=(tag,))
            bar = cv.create_rectangle(xa, y, xa + 3, y + 40, fill=B.ORANGE,
                                      width=0, tags=(tag,))
            name = self.text(xa + 16, y + 8, "", 20, B.INK, num=True, tags=(tag,))
            kind = self.text(xb - 10, y + 13, "", 12, B.INK3, "ne", tags=(tag,))
            cv.tag_bind(tag, "<ButtonPress-1>", lambda e, i=i: self.pick(i))
            self.rows.append((bg, bar, name, kind))
        cv.create_line(xa, 336, xb, 336, fill=B.LINE)

        self.b_conn = B.RoundButton(cv, xa + 40, 394, 70, B.ORANGE,
                                    self.toggle, label="connect", label_size=12)
        B.RoundButton(cv, xa + 130, 394, 44, B.DARK2, self.rescan,
                      label="rescan", label_size=12)
        self.t_status = self.text(xa + 180, 362, "", 13, B.INK2,
                                  width=xb - xa - 180)

        cv.create_line(xa, 470, xb, 470, fill=B.LINE)
        self.text(xa, 486, "firmware commands", 12, B.INK3)
        # No button for `align`: in this firmware it ends in a watchdog
        # reset. It blocks for 300 ms inside delay_ms(), which does not
        # refresh a watchdog set to 100 ms. Seen on the board: the reply
        # stops after "aligning...", the port drops, and the next `t` reads
        # "reset watchdog". It can still be typed below, like anything else.
        for i, (label, cmd, color) in enumerate((
                ("self-test", "t", B.DARK2), ("idle", "off", B.DARK2),
                ("clear fault", "clear", B.YELLOW),
                ("arm outputs", "on", B.RED))):
            B.RoundButton(cv, xa + 30 + i * 100, 544, 44, color,
                          lambda c=cmd: self.command(c), label=label,
                          label_size=11)
        self.text(xa, 626, "console command", 12, B.INK3)
        cv.create_window(xa, 646, anchor="nw", window=self.entry,
                         width=xb - xa, height=34)
        self.text(xa, 690, "what you would type in a terminal, then enter.  "
                  "?  lists the firmware's commands.  align and hang both end "
                  "in a watchdog reset.", 11, B.INK3, width=xb - xa)

        xa, xb = 520, X1
        self.text(xa, 128, "power-on self-test", 12, B.INK3)
        self.t_flags = self.text(xb, 124, "", 17, B.INK, "ne", num=True)
        cv.create_line(xa, 148, xb, 148, fill=B.LINE)
        self.flags = []
        for i, (bit, name) in enumerate(boardlink.FLAGS):
            lamp = B.Lamp(cv, xa + 8 + (i % 2) * 250, 168 + (i // 2) * 26, name)
            self.flags.append((bit, lamp))
        cv.create_line(xa, 276, xb, 276, fill=B.LINE)
        self.values = []
        for i, (key, label, unit) in enumerate(self.SUMMARY):
            y = 288 + i * 27
            self.text(xa, y + 2, label + ("  " + unit if unit else ""), 13, B.INK)
            self.values.append((key, self.text(xb, y, "", 17, B.INK, "ne",
                                               num=True)))
        self.text(xa, 520, "console", 12, B.INK3)
        cv.create_image(xa, 540, anchor="nw", image=B.rrect(xb - xa, 232, 10, B.DARK))
        self.t_log = cv.create_text(xa + 14, 540 + 232 - 12, anchor="sw",
                                    fill="#C9C6BC", font=("Consolas", -12),
                                    text="")
        self._log_serial = None
        self.rescan()

    # ---- actions

    @staticmethod
    def _flash_words(link):
        """What the board's own settings page holds, in a few words."""
        st = link.store
        hall = " and the hall table" if st.get("hall") else ""
        return {
            "ok": f"{st.get('params', 0)} parameters{hall} stored",
            "empty": "nothing stored; it comes up on its defaults",
            "version": "a record another console version wrote; not used",
            "damaged": "a half-written record; not used",
        }.get(st.get("state"), st.get("state", "?"))

    def rescan(self):
        self.ports = boardlink.list_ports()[:self.PORT_ROWS]
        names = [p for p, _k in self.ports]
        link = self.app.board
        if link is not None:
            self.sel = link.port
        elif self.sel not in names:
            # a real port before the firmware model, which is always listed
            real = [n for n in names if n != boardlink.EMULATOR_PORT]
            self.sel = real[-1] if real else (names[-1] if names else None)

    def pick(self, i):
        if self.app.board is None and i < len(self.ports):
            self.sel = self.ports[i][0]
            self.msg = ""

    def toggle(self):
        app = self.app
        if app.board is not None:
            app.disconnect()
            self.msg = ""
        elif self.sel is None:
            self.msg = "no serial port to connect to. plug the board in, then rescan."
        else:
            self.msg = app.connect(self.sel)

    def command(self, text):
        link = self.app.board
        if link is None:
            self.msg = "connect first."
        else:
            link.notice = ""
            link.send(text)

    def _typed(self, _e):
        text = self.entry.get().strip()
        self.entry.delete(0, "end")
        if text:
            self.command(text)

    # ---- drawing

    def update(self):
        cv, app = self.cv, self.app
        link = app.board
        now = time.perf_counter()
        if link is None and now > self._scan_at:
            self._scan_at = now + 1.0
            self.rescan()

        for i, (bg, bar, name, kind) in enumerate(self.rows):
            if i < len(self.ports):
                port, what = self.ports[i]
                on = port == self.sel
                cv.itemconfigure(bg, fill=B.PANEL if on else B.PAPER)
                cv.itemconfigure(bar, state="normal" if on else "hidden")
                self.set(name, port)
                self.set(kind, what)
            else:
                cv.itemconfigure(bg, fill=B.PAPER)
                cv.itemconfigure(bar, state="hidden")
                self.set(name, "")
                self.set(kind, "no serial ports found" if (i == 0 and not self.ports) else "")

        self.b_conn.set_color(B.DARK2 if link else B.ORANGE)
        self.b_conn.set_label(text="disconnect" if link else "connect")
        if link is None:
            status = self.msg or ("not connected. every page shows the "
                                  "simulated motor.")
        else:
            console = {None: "console: asking",
                       1: "console 1: no parameters, no auto tune",
                       2: "console 2: parameters and auto tune"}.get(
                           link.fw, f"console {link.fw}")
            status = (f"connected to {link.port}.\n"
                      f"{link.rate:.0f} state lines a second, "
                      f"{link.latency_ms:.1f} ms to answer.\n"
                      f"{console}\n"
                      f"loop: {link.mode or 'finding out'}"
                      + (f"\nflash: {self._flash_words(link)}"
                         if link.store else "")
                      + (f"\n{link.notice}" if link.notice else ""))
        self.set(self.t_status, status)

        summary = link.summary if link else {}
        flags = summary.get("flags")
        for bit, lamp in self.flags:
            lamp.set(None if flags is None else (B.INK if flags & bit else B.RED))
        self.set(self.t_flags, "" if flags is None else
                 f"{bin(flags).count('1')} of 8")
        st = link.state if link else None
        for key, item in self.values:
            if key == "moe":
                text = "–" if not st else ("on" if st["moe"] else "off")
            elif key == "fault":
                text = "–" if not st else st["fault"]
            elif key == "watchdog":
                text = ("–" if key not in summary else
                        ("running" if summary[key] else "off"))
            else:
                text = str(summary.get(key, "–")).replace("-", "−") \
                    if isinstance(summary.get(key), int) else str(summary.get(key, "–"))
            bad = (key == "fault" and st and st["fault"] != "none") or \
                  (key == "moe" and st and st["moe"])
            self.set(item, text, B.RED if bad else B.INK)

        serial = link.log_serial if link else -1
        if serial != self._log_serial:
            self._log_serial = serial
            lines = list(link.log)[-self.LOG_LINES:] if link else []
            cv.itemconfigure(self.t_log, text="\n".join(s[:66] for s in lines))


# -------------------------------------------------------------------- app ---

class VehiclePage(vehicle.VehiclePage, Page):
    X1 = X1


class BenchPage(Page):
    """What the control loop is actually driving.

    The same firmware can be made to drive three different things, and
    which one it is driving is the single most important fact about a
    measurement. It was buried in two parameters on page four. Here it is
    a choice of three, each with the reason you would pick it.
    """

    key = "bench"
    title = "bench"
    subtitle = "what the control loop is driving: a model, or something on the wires"

    MODES = (
        ("sim", "this window", "host simulator",
         "No board at all. The motor, the inverter and the sensors are "
         "arithmetic in this window. Nothing has to be plugged in and "
         "nothing can be damaged. The place to learn what each control "
         "does before touching hardware."),
        ("model", "the board, its own model", "firmware + model motor",
         "The stm32 runs the real control loop at 20 kHz - the same Clarke, "
         "Park, two PI controllers and SVPWM that a motor would get - but "
         "the motor it believes it is turning is a model inside the same "
         "chip. Real code and real timing, no wires."),
        ("pins", "the board, through its pins", "firmware + hardware in the loop",
         "The firmware reads its Hall pins and switches its real outputs. "
         "What is on the other end is yours to choose: the HIL board "
         "running the motor model, or a motor with real sensors. This is "
         "the only mode where the loop leaves the chip."),
    )

    WIRES = (
        ("PA8", "phase a high", "HIL PA0"),
        ("PA9", "phase b high", "HIL PA6"),
        ("PA10", "phase c high", "HIL PB6"),
        ("PB13", "phase a low", "HIL PB7"),
        ("PB14", "phase b low", "HIL PB8"),
        ("PB15", "phase c low", "HIL PB9"),
        ("PB5", "hall a", "HIL PB12"),
        ("PB6", "hall b", "HIL PB13"),
        ("PB7", "hall c", "HIL PB14"),
        ("GND", "a common ground", "HIL GND"),
    )

    def setup(self):
        self.msg = ""
        self._asked = 0.0

    def build(self):
        cv = self.cv
        xa, xb = X0, 648
        self.cards = []
        for i, (key, _short, name, body) in enumerate(self.MODES):
            y = 122 + i * 142
            tag = f"mode{key}"
            bg = cv.create_rectangle(xa, y, xb, y + 126, fill=B.PANEL,
                                     outline="", tags=(tag,))
            bar = cv.create_rectangle(xa, y, xa + 3, y + 126, fill=B.ORANGE,
                                      width=0, tags=(tag,))
            lamp = B.Lamp(cv, xa + 28, y + 28, "", 14, B.INK)
            title = self.text(xa + 50, y + 18, name, 18, B.INK)
            self.text(xa + 50, y + 46, body, 12, B.INK2, width=xb - xa - 74)
            cv.tag_bind(tag, "<ButtonPress-1>", lambda e, k=key: self.pick(k))
            self.cards.append((key, bg, bar, lamp, title))

        y = 122 + 3 * 142
        self.text(xa, y, "hall edges", 12, B.INK3)
        self.text(xa, y + 20,
                  "The pins mode needs the six switching angles measured "
                  "once. The firmware refuses to steer by sensors it has "
                  "not calibrated. Spin the shaft - or let the HIL board "
                  "turn its model - and run this.", 12, B.INK2,
                  width=xb - xa - 180)
        B.RoundButton(cv, xb - 80, y + 44, 48, B.YELLOW,
                      lambda: self.command("hallcal"), label="calibrate",
                      label_size=11)
        self.t_msg = self.text(xa, y + 96, "", 13, B.ORANGE, width=xb - xa)

        xc = 688
        self.text(xc, 122, "WHAT THE BOARD SAYS", 12, B.INK3)
        cv.create_line(xc, 142, X1, 142, fill=B.LINE)
        self.rows = []
        for i, label in enumerate(("angle source", "hall signals from",
                                   "hall code now", "hall table")):
            y = 154 + i * 30
            self.text(xc, y, label, 12, B.INK2)
            self.rows.append(self.text(X1, y - 2, "–", 14, B.INK, "ne"))

        self.text(xc, 300, "THE TEN WIRES", 12, B.INK3)
        self.text(xc, 320,
                  "Only the pins mode uses them. Six carry the bridge out "
                  "to the bench board, three carry the sensors back, one is "
                  "the ground they are all measured against.", 12, B.INK2,
                  width=X1 - xc)
        cv.create_line(xc, 388, X1, 388, fill=B.LINE)
        for i, (pin, what, other) in enumerate(self.WIRES):
            y = 398 + i * 26
            self.text(xc, y, pin, 13, B.INK, num=True)
            self.text(xc + 58, y + 1, what, 11, B.INK3)
            self.text(X1, y, other, 13, B.INK2, "ne", num=True)

        self.text(xc, 652, "PHASE CURRENTS  \u2192  page 3", 12, B.INK3)
        cv.create_line(xc, 670, X1, 670, fill=B.LINE)
        self.i_lamps = []
        for i, (src, label) in enumerate(
                ((0, "the model in this chip"),
                 (1, "the pins PA0 and PA1"))):
            y = 690 + i * 28
            lamp = B.Lamp(cv, xc + 8, y, label, 12, B.INK2)
            tag = f"isrc{src}"
            cv.create_rectangle(xc, y - 10, X1, y + 10, fill="", outline="",
                                tags=(tag,))
            cv.tag_bind(tag, "<ButtonPress-1>",
                        lambda e, v=src: self.set_isrc(v))
            self.i_lamps.append((src, lamp))
        B.RoundButton(cv, xc + 16, 760, 24, B.DARK2,
                      lambda: self.command("izero"), label="learn the zero",
                      label_side="e", label_size=11)
        self.t_iraw = self.text(xc, 780, "", 11, B.INK3, width=X1 - xc)

    # ---- what mode is live ------------------------------------------------

    def mode(self):
        link = self.app.board
        if link is None:
            return "sim"
        veh = link.veh
        if "hsrc" not in link.params and "sens" not in veh:
            return None
        hsrc = link.params.get("hsrc")
        sens = veh.get("sens", link.params.get("sens"))
        if hsrc is None or sens is None:
            return None
        if hsrc == 1 and sens == 1:
            return "pins"
        if hsrc == 1:
            # The pins are being read, but the loop is still steering by the
            # model: `sens 1` was refused, almost always for want of a hall
            # table. Saying "model" here would hide the thing to fix.
            return "pins?"
        return "model"

    def pick(self, key):
        app = self.app
        link = app.board
        if key == "sim":
            if link is not None:
                app.disconnect()
            self.msg = "the board is disconnected; this window is the motor."
            return
        if link is None:
            port = app.pages["board"].sel
            if port is None:
                self.msg = ("no board to connect to. plug it in, then pick a "
                            "port on the board page.")
                return
            why = app.connect(port)
            if why:
                self.msg = why
                return
            link = app.board
        link.send("off")
        if key == "model":
            link.send("set sens 0")
            link.send("set hsrc 0")
            self.msg = ("the board is driving its own model again. "
                        "nothing on the pins is being believed.")
        else:
            link.send("set hsrc 1")
            link.send("set sens 1")
            self.msg = ("asked for the pins. the outputs are idle: arm "
                        "them, or press run on the drive page.")
        link.send("get")
        link.send("v")

    def command(self, text):
        link = self.app.board
        if link is None:
            self.msg = "connect a board first."
            return
        link.send(text)
        self.msg = f"sent {text}. watch the hall table line."

    def set_isrc(self, src):
        """Where the loop reads its two phase currents.

        Separate from the angle on purpose: either can come from inside
        the chip or off the pins, and the four combinations are four
        different experiments. Currents off the pins with no filter wired
        to them reads whatever the pins happen to float at, so the drive
        is put back to idle first.
        """
        link = self.app.board
        if link is None:
            self.msg = "connect a board first."
            return
        link.send("off")
        link.send(f"isrc {int(src)}")
        link.send("i")
        self.msg = ("the loop will read the ADC pins. run `learn the zero` "
                    "with the bench idle before arming."
                    if src else
                    "the loop is back on the model inside the chip.")

    # ---- drawing ----------------------------------------------------------

    CAL = {0: "not calibrated", 1: "calibrated", 2: "running",
           3: "too slow to measure", 4: "edges did not agree",
           9: "refused: the drive must be running"}

    def update(self):
        app = self.app
        link = app.board
        now = time.perf_counter()
        if link is not None and now - self._asked > 0.5:
            self._asked = now
            link.send("v")
            link.send("i")

        live = self.mode()
        half = live == "pins?"
        for key, bg, bar, lamp, title in self.cards:
            on = key == live or (half and key == "pins")
            self.cv.itemconfigure(bg, fill=B.PANEL if on else B.PAPER)
            self.cv.itemconfigure(bar, state="normal" if on else "hidden")
            self.cv.itemconfigure(bar, fill=B.YELLOW if (on and half)
                                  else B.ORANGE)
            lamp.set((B.YELLOW if half else B.GREEN) if on else None)
            self.cv.itemconfigure(title, fill=B.INK if on else B.INK2)

        if link is None:
            vals = ("this window", "nothing: there is no board",
                    "–", "–")
        else:
            veh = link.veh
            sens = veh.get("sens", link.params.get("sens"))
            hsrc = link.params.get("hsrc")
            hall = veh.get("hall")
            cal = veh.get("cal")
            vals = (
                ("–" if sens is None else
                 ("encoder, the model's own", "hall sensors",
                  "observer, no sensors")[min(int(sens), 2)]),
                ("–" if hsrc is None else
                 ("the firmware's model", "the pins PB5 PB6 PB7")[min(int(hsrc), 1)]),
                ("–" if hall is None else f"{int(hall):03b}"),
                ("–" if cal is None else self.CAL.get(int(cal), str(cal))),
            )
        for item, text in zip(self.rows, vals):
            self.set(item, text)

        info = getattr(link, "isrc", None) if link is not None else None
        for src, lamp in self.i_lamps:
            on = info is not None and info["src"] == src
            lamp.set(B.GREEN if on else None)
        if info is None:
            self.set(self.t_iraw, "")
        else:
            self.set(self.t_iraw,
                     f"pins reading {info['raw_a']} and {info['raw_b']}, "
                     f"zero at {info['zero_a']} and {info['zero_b']}, "
                     f"{info['gain']} Q15 per count")
        if live == "pins?":
            self.msg = ("the pins are being read, but the loop is still "
                        "steering by the model: the hall table has not been "
                        "calibrated. spin the shaft and press calibrate.")
        self.set(self.t_msg, self.msg,
                 B.RED if live == "pins?" else B.ORANGE)


class CurrentPage(Page):
    """The analog path, stage by stage, with its live numbers.

    Stage two is the part of this bench that is hardest to believe,
    because nothing about it is visible: two pins carry a PWM, two
    passives average it, and a number appears inside the control loop.
    When it goes wrong - a wire off, a zero never learned, a gain out by
    a factor - every stage after the break still produces a number, and
    the numbers all look like currents.

    So the chain is drawn: counts, volts, counts about zero, Q15, and
    what the loop is actually regulating. The stage where it stops making
    sense is the stage that is broken.
    """

    key = "current"
    title = "current"
    subtitle = "the path from two pins to the number the loop regulates"
    VDDA = 3.313          # measured on this part by the power-on self-test
    SAMPLE = 0.08         # seconds between `i` requests
    HISTORY = 400

    def setup(self):
        self.msg = ""
        self._asked = 0.0
        self.a = []
        self.b = []

    def build(self):
        cv = self.cv
        xa, xb = X0, 640

        self.text(xa, 120, "WHERE THE LOOP READS ITS CURRENTS", 12, B.INK3)
        self.cards = []
        for i, (src, name, body) in enumerate((
                (0, "the model in this chip",
                 "The firmware's own motor model, which no wire can reach. "
                 "Nothing on the bench can load it."),
                (1, "the two ADC pins",
                 "PA0 and PA1, through an RC filter from the bench board. "
                 "A load applied over there arrives here."))):
            y = 144 + i * 74
            bg = cv.create_rectangle(xa, y, xb, y + 64, fill=B.PAPER,
                                     outline="", tags=(f"cur{src}",))
            bar = cv.create_rectangle(xa, y, xa + 3, y + 64, fill=B.ORANGE,
                                      width=0, tags=(f"cur{src}",))
            lamp = B.Lamp(cv, xa + 24, y + 22, "", 14, B.INK)
            title = self.text(xa + 46, y + 12, name, 16, B.INK)
            self.text(xa + 46, y + 34, body, 12, B.INK2, width=xb - xa - 70)
            cv.tag_bind(f"cur{src}", "<ButtonPress-1>",
                        lambda e, v=src: self.pick(v))
            self.cards.append((src, bg, bar, lamp, title))

        self.text(xa, 306, "THE CHAIN, AS IT IS RIGHT NOW", 12, B.INK3)
        cv.create_line(xa, 324, X1, 324, fill=B.LINE)
        self.chain = []
        rows = (("raw", "what the pins convert to", "counts of 4096"),
                ("volt", "the voltage that is", "V"),
                ("off", "with the learned zero taken off", "counts"),
                ("q15", "scaled by the gain", "Q15"),
                ("loop", "and what the loop made of it", ""))
        for i, (key, label, unit) in enumerate(rows):
            y = 340 + i * 54
            self.text(xa, y + 14, label, 13, B.INK2)
            a = self.text(xa + 330, y, "–", 20, B.INK, "ne", num=True)
            b = self.text(xa + 460, y, "–", 20, B.INK, "ne", num=True)
            self.text(xa + 470, y + 6, unit, 11, B.INK3)
            self.chain.append((key, a, b))
        self.text(xa + 330, 326, "phase a", 11, B.INK3, "ne")
        self.text(xa + 460, 326, "phase b", 11, B.INK3, "ne")
        # The last row is not a pair of phases: it is what came out the
        # far end of the chain, on the two axes the controller works in.
        y = 340 + 4 * 54
        self.text(xa + 330, y + 26, "i q", 11, B.INK3, "ne")
        self.text(xa + 460, y + 26, "i d", 11, B.INK3, "ne")

        self.text(xa, 606, "WHAT THE PINS ARE DOING", 12, B.INK3)
        self.chart = B.Chart(cv, xa, 626, xb - xa, 132, unit="counts")
        self.t_swing = self.text(xa, 764, "", 12, B.INK3, width=xb - xa)

        xc = 680
        self.text(xc, 120, "CALIBRATION", 12, B.INK3)
        cv.create_line(xc, 138, X1, 138, fill=B.LINE)
        self.t_zero = self.text(xc, 150, "", 13, B.INK2, width=X1 - xc)
        B.RoundButton(cv, xc + 20, 212, 30, B.YELLOW,
                      lambda: self.command("izero"), label="learn the zero",
                      label_side="e", label_size=12)
        self.text(xc, 244,
                  "With nothing turning, both pins should read the same "
                  "voltage they will read at zero current. That reading is "
                  "subtracted from every sample afterwards, so it is worth "
                  "doing once with the drive idle and the bench at rest.",
                  11, B.INK3, width=X1 - xc)

        self.text(xc, 330, "GAIN", 12, B.INK3)
        self.t_gain = self.text(xc, 350, "–", 26, B.INK, num=True)
        self.text(xc + 80, 358, "Q15 per count", 11, B.INK3)
        B.RoundButton(cv, xc + 22, 404, 28, B.DARK2,
                      lambda: self.nudge(-1), glyph="−", glyph_size=18)
        B.RoundButton(cv, xc + 74, 404, 28, B.ORANGE,
                      lambda: self.nudge(1), glyph="+", glyph_size=18)
        self.text(xc, 432,
                  "Eight is right for the bench as it is built: a full-scale "
                  "Q15 current drives the pin the whole way from mid-rail to "
                  "a rail, which is 2048 counts.", 11, B.INK3, width=X1 - xc)

        self.text(xc, 510, "IS IT ARRIVING STRAIGHT", 12, B.INK3)
        cv.create_line(xc, 528, X1, 528, fill=B.LINE)
        self.l_move = B.Lamp(cv, xc + 8, 548, "the pins are moving", 13, B.INK2)
        self.l_dax = B.Lamp(cv, xc + 8, 578, "i d is near zero", 13, B.INK2)
        self.text(xc, 600,
                  "A current can arrive and still be useless. If it is "
                  "scaled wrongly or lands at the wrong angle, the d axis "
                  "is where it shows: the controller fights itself and i d "
                  "wanders. Both lamps green is the honest test that the "
                  "wires, the filters and the angle all agree.",
                  11, B.INK3, width=X1 - xc)
        self.t_msg = self.text(xc, 724, "", 12, B.ORANGE, width=X1 - xc)

    # ---- actions -----------------------------------------------------------

    def pick(self, src):
        link = self.app.board
        if link is None:
            self.msg = "connect a board first."
            return
        link.send("off")
        link.send(f"isrc {int(src)}")
        link.send("i")
        self.msg = ("the loop will read the pins, and the outputs are idle: "
                    "changing the source under a running loop is a step the "
                    "controller never asked for. learn the zero, then run."
                    if src else
                    "the loop is back on the model in this chip, outputs "
                    "idle.")

    def command(self, text):
        link = self.app.board
        if link is None:
            self.msg = "connect a board first."
            return
        link.send(text)
        self.msg = f"sent {text}."

    def nudge(self, step):
        link = self.app.board
        info = getattr(link, "isrc", None) if link is not None else None
        if info is None:
            self.msg = "no reading from the board yet."
            return
        link.send(f"iscl {max(1, min(256, info['gain'] + step))}")
        link.send("i")

    # ---- drawing -----------------------------------------------------------

    def update(self):
        app = self.app
        link = app.board
        now = time.perf_counter()
        if link is not None and now - self._asked > self.SAMPLE:
            self._asked = now
            link.send("i")

        info = getattr(link, "isrc", None) if link is not None else None
        for src, bg, bar, lamp, title in self.cards:
            on = info is not None and info["src"] == src
            self.cv.itemconfigure(bg, fill=B.PANEL if on else B.PAPER)
            self.cv.itemconfigure(bar, state="normal" if on else "hidden")
            lamp.set(B.GREEN if on else None)
            self.cv.itemconfigure(title, fill=B.INK if on else B.INK2)

        if info is None:
            for key, a, b in self.chain:
                self.set(a, "–")
                self.set(b, "–")
            self.set(self.t_zero, "no reading from the board yet.")
            self.set(self.t_gain, "–")
            self.set(self.t_msg, self.msg)
            return

        ra, rb = info["raw_a"], info["raw_b"]
        if not self.a or (ra, rb) != (self.a[-1], self.b[-1]):
            self.a.append(ra)
            self.b.append(rb)
            del self.a[:-self.HISTORY]
            del self.b[:-self.HISTORY]

        oa, ob = ra - info["zero_a"], rb - info["zero_b"]
        g = info["gain"]
        st = link.state or {}
        values = {
            "raw": (ra, rb),
            "volt": (ra / 4096.0 * self.VDDA, rb / 4096.0 * self.VDDA),
            "off": (oa, ob),
            "q15": (max(-32768, min(32767, oa * g)),
                    max(-32768, min(32767, ob * g))),
            "loop": (st.get("iq"), st.get("id")),
        }
        for key, a, b in self.chain:
            va, vb = values[key]
            dec = 3 if key == "volt" else 0
            self.set(a, "–" if va is None else B.num(va, dec))
            self.set(b, "–" if vb is None else B.num(vb, dec))
        self.cv.itemconfigure(self.chain[4][1], fill=B.ORANGE)

        self.set(self.t_zero,
                 f"zero at {info['zero_a']} and {info['zero_b']} counts, "
                 f"which is {info['zero_a'] / 4096.0 * self.VDDA:.3f} V and "
                 f"{info['zero_b'] / 4096.0 * self.VDDA:.3f} V.")
        self.set(self.t_gain, str(g))

        if len(self.a) > 4:
            swing = max(max(self.a) - min(self.a), max(self.b) - min(self.b))
            lo, hi, step = B.axis(min(min(self.a), min(self.b)),
                                  max(max(self.a), max(self.b)), 120.0)
            self.chart.note("")
            self.chart.draw(((self.b, B.YELLOW, 1.4), (self.a, B.WHITE, 1.6)),
                            lo, hi, step,
                            seconds=len(self.a) * self.SAMPLE)
            self.set(self.t_swing,
                     f"{swing} counts of swing over the last "
                     f"{len(self.a) * self.SAMPLE:.0f} seconds. A rotor that "
                     f"is turning should move them by hundreds; a few counts "
                     f"is the converter's own noise and nothing else.")
            self.l_move.set(B.GREEN if swing > 150 else
                            (B.RED if info["src"] == 1 else None))
        else:
            self.chart.note("waiting for samples")

        idd = st.get("id")
        if idd is None or info["src"] == 0:
            self.l_dax.set(None)
        else:
            self.l_dax.set(B.GREEN if abs(idd) < 400 else B.RED)
        self.set(self.t_msg, self.msg)


class MotorPage(Page):
    """The other board: the one that is being the motor.

    Two windows worked, and the way they had to share the two ports was
    the first thing anyone had to be told. One window holds both: the
    drive on its console, the bench on its own, and nothing between them
    but the wires they were always talking over.
    """

    key = "motor"
    title = "motor"
    subtitle = "the bench board standing in for the motor, and the load on it"
    PORT_ROWS = 3
    LADDER = (0, 1, 2, 3, 4, 6, 8, 11, 15, 20, 27, 36, 48, 64, 85, 110)
    HISTORY = 600

    def setup(self):
        self.ports = []
        self.sel = None
        self.msg = ""
        self.trace = []
        self._scan_at = 0.0

    def build(self):
        cv = self.cv
        xa, xb = X0, 640
        xc = 680

        self.text(xa, 120, "BENCH STATUS", 12, B.INK3)
        self.rows = []
        for i, chk in enumerate(self.app.checks):
            y = 142 + i * 46
            cv.create_rectangle(xa, y - 4, xb, y + 36, fill=B.PANEL, width=0)
            lamp = B.Lamp(cv, xa + 20, y + 16, "", 13, B.INK)
            title = self.text(xa + 40, y + 2, chk.title, 15, B.INK)
            detail = self.text(xa + 40, y + 20, "", 12, B.INK3, width=xb - xa - 60)
            self.rows.append((chk, lamp, title, detail))
        self.verdict = self.text(xa, 472, "", 13, B.INK2, width=xb - xa)

        self.text(xa, 516, "SHAFT SPEED, AS THE MODEL KNOWS IT", 12, B.INK3)
        self.chart = B.Chart(cv, xa, 536, xb - xa, 180, unit="units/ms")
        self.t_trace = self.text(xa, 724, "", 12, B.INK3, width=xb - xa)

        # ---- the right column: the board, the load, the model
        self.text(xc, 120, "BENCH BOARD", 12, B.INK3)
        cv.create_line(xc, 138, X1, 138, fill=B.LINE)
        self.port_rows = []
        for i in range(self.PORT_ROWS):
            y = 148 + i * 34
            tag = f"bport{i}"
            bg = cv.create_rectangle(xc, y, X1, y + 30, fill=B.PAPER,
                                     outline="", tags=(tag,))
            bar = cv.create_rectangle(xc, y, xc + 3, y + 30, fill=B.ORANGE,
                                      width=0, tags=(tag,))
            name = self.text(xc + 14, y + 4, "", 16, B.INK, num=True, tags=(tag,))
            cv.tag_bind(tag, "<ButtonPress-1>", lambda e, i=i: self.pick(i))
            self.port_rows.append((bg, bar, name))
        self.b_conn = B.RoundButton(cv, xc + 34, 282, 46, B.ORANGE,
                                    self.toggle, label="connect", label_size=12)
        B.RoundButton(cv, xc + 120, 282, 36, B.DARK2, self.rescan,
                      label="rescan", label_size=12)
        self.t_status = self.text(xc, 340, "", 12, B.INK3, width=X1 - xc)

        self.text(xc, 378, "LOAD ON THE SHAFT", 12, B.INK3)
        self.t_load = self.text(xc, 398, "–", 34, B.INK, num=True)
        self.t_load_unit = self.text(xc + 2, 442, "", 11, B.INK3,
                                     width=X1 - xc)
        B.RoundButton(cv, xc + 30, 492, 36, B.DARK2, lambda: self.step(-1),
                      glyph="−", glyph_size=20)
        B.RoundButton(cv, xc + 92, 492, 36, B.ORANGE, lambda: self.step(1),
                      glyph="+", glyph_size=20)
        B.RoundButton(cv, xc + 166, 492, 24, B.DARK2,
                      lambda: self.bench_send("set load 0"), label="none",
                      label_side="e", label_size=11)
        B.RoundButton(cv, xc + 30, 548, 24, B.DARK2, self.toggle_lock,
                      label="clamp the shaft", label_side="e", label_size=11)
        self.text(xc, 578,
                  "Drag opposes the rotor whichever way it turns. Raise it "
                  "and watch the drive answer with current on page 3; raise "
                  "it far enough and it runs out of torque.",
                  11, B.INK3, width=X1 - xc)

        self.text(xc, 652, "WHAT THE MODEL HOLDS", 12, B.INK3)
        cv.create_line(xc, 670, X1, 670, fill=B.LINE)
        self.model = []
        for i, label in enumerate(("true speed  units/ms", "true angle  units",
                                   "hall code", "phase duties  of 1800")):
            y = 680 + i * 26
            self.text(xc, y, label, 12, B.INK2)
            self.model.append(self.text(X1, y - 2, "–", 14, B.INK, "ne",
                                        num=True))

    # ---- the board

    def rescan(self):
        self.ports = [p for p, _k in benchlink.list_ports()
                      if p.startswith("COM")][:self.PORT_ROWS]
        link = self.app.bench
        if link is not None:
            self.sel = link.port
        elif self.sel not in self.ports:
            # the drive's port is taken; offer the other one first
            drive = self.app.board.port if self.app.board else None
            free = [p for p in self.ports if p != drive]
            self.sel = free[0] if free else (self.ports[0] if self.ports else None)

    def pick(self, i):
        if self.app.bench is None and i < len(self.ports):
            self.sel = self.ports[i]
            self.msg = ""

    def toggle(self):
        app = self.app
        if app.bench is not None:
            app.bench_disconnect()
            self.msg = ""
        elif self.sel is None:
            self.msg = "no port to connect to. plug the bench board in, then rescan."
        else:
            self.msg = app.bench_connect(self.sel)

    def bench_send(self, text):
        if self.app.bench is None:
            self.msg = "connect the bench board first."
            return
        self.app.bench.send(text)

    def step(self, direction):
        st = self.app.bench.state if self.app.bench else {}
        now = st.get("load", 0) or 0
        if direction > 0:
            nxt = next((v for v in self.LADDER if v > now), self.LADDER[-1])
        else:
            nxt = next((v for v in reversed(self.LADDER) if v < now), 0)
        self.bench_send(f"set load {nxt}")

    def toggle_lock(self):
        st = self.app.bench.state if self.app.bench else {}
        self.bench_send("set lock %d" % (0 if st.get("lock") else 1))

    # ---- drawing

    def update(self):
        app = self.app
        link = app.bench
        now = time.perf_counter()
        if now - self._scan_at > 2.0 and link is None:
            self._scan_at = now
            self.rescan()

        for i, (bg, bar, name) in enumerate(self.port_rows):
            port = self.ports[i] if i < len(self.ports) else ""
            on = port and port == self.sel
            self.cv.itemconfigure(bg, fill=B.PANEL if on else B.PAPER)
            self.cv.itemconfigure(bar, state="normal" if on else "hidden")
            self.set(name, port)
        self.b_conn.set_label(text="disconnect" if link else "connect")
        self.b_conn.set_color(B.DARK2 if link else B.ORANGE)
        self.set(self.t_status,
                 self.msg or (f"{link.port}, {link.version or 'no answer yet'}"
                              if link else
                              "the bench board has its own port, separate "
                              "from the drive's."))

        for chk, lamp, title, detail in self.rows:
            colour = (B.MUTE if chk.ok is None else
                      (B.GREEN if chk.ok else B.RED))
            lamp.set(colour)
            self.set(detail, chk.detail or chk.explain)
            self.set(title, chk.title, B.INK if chk.ok is not False else B.RED)

        bad = [c for c in app.checks if c.ok is False]
        if bad:
            self.set(self.verdict,
                     "Something is wrong: " + "; ".join(c.wrong for c in bad)
                     + ".", B.RED)
        elif all(c.ok for c in app.checks):
            self.set(self.verdict,
                     "Everything checks out. The drive is controlling a motor "
                     "that lives in the other microcontroller.", B.INK2)
        else:
            self.set(self.verdict,
                     "Waiting for both boards and for the drive to switch.",
                     B.ORANGE)

        st = link.state if link else {}
        load, lock = st.get("load"), st.get("lock")
        if load is None:
            self.set(self.t_load, "–")
            self.set(self.t_load_unit, "no bench board, or a firmware "
                                       "without the load setting")
        elif lock:
            self.set(self.t_load, "clamped", B.RED)
            self.set(self.t_load_unit, "the shaft is held still")
        else:
            self.set(self.t_load, str(load), B.ORANGE if load else B.INK)
            self.set(self.t_load_unit, "drag, 0 is a free shaft")

        duty = "  ".join(str(st.get(k, "–")) for k in ("da", "db", "dc"))
        for item, value in zip(self.model,
                               (st.get("spd"), st.get("ang"),
                                None if st.get("hall") is None
                                else f"{st['hall']:03b}", duty)):
            self.set(item, "–" if value is None else str(value))

        spd = st.get("spd")
        if spd is not None:
            self.trace.append(spd)
            del self.trace[:-self.HISTORY]
        if len(self.trace) > 3:
            lo, hi, step = B.axis(min(self.trace), max(self.trace), 200.0)
            self.chart.note("")
            self.chart.draw([(self.trace, B.ORANGE, 1.8, True)], lo, hi, step,
                            seconds=len(self.trace) / benchlink.POLL_HZ)
            self.set(self.t_trace,
                     "the model's own speed, which the drive cannot see: it "
                     "only knows what its Hall pins told it.")
        else:
            self.chart.note("connect the bench board")


class SyncPage(Page):
    """The same quantity, measured on both boards, side by side."""

    key = "sync"
    title = "sync"
    subtitle = "what each board thinks is happening, next to each other"

    ROWS = (("speed", "speed", "units/ms", 0.08),
            ("angle", "electrical angle", "units", None),
            ("current", "q-axis current", "Q15 counts", 0.35),
            ("volt", "phase voltage amplitude", "Q15 counts", 0.35))

    def build(self):
        cv = self.cv
        xa, xb = X0, 660
        self.text(xa, 120, "THE DRIVE", 12, B.INK3)
        self.text(xa + 260, 120, "THE BENCH", 12, B.INK3)
        self.text(xb - 10, 120, "APART BY", 12, B.INK3, "ne")
        self.cells = {}
        for i, (key, label, unit, _tol) in enumerate(self.ROWS):
            y = 146 + i * 76
            cv.create_rectangle(xa, y - 8, xb, y + 56, fill=B.PANEL, width=0)
            self.text(xa + 16, y + 30, f"{label}   {unit}", 12, B.INK3)
            self.cells[key] = (
                self.text(xa + 16, y, "–", 26, B.INK, num=True),
                self.text(xa + 276, y, "–", 26, B.INK, num=True),
                self.text(xb - 10, y + 4, "", 20, B.INK2, "ne", num=True))

        self.verdict = self.text(xa, 464, "", 14, B.INK, width=xb - xa)

        self.text(xa, 524, "WHAT ACTUALLY CROSSES THE WIRES", 12, B.INK3)
        self.text(xa, 544,
                  "The rotor angle, always. The phase currents too, once "
                  "page 3 is set to the pins. The drive works its speed out "
                  "from the Hall edges arriving on three of them; the bench "
                  "knows the true speed of the model that produced those "
                  "edges. Neither board can see the other's number, so when "
                  "the two agree the loop really is closed through the "
                  "copper - and if the currents are still coming from the "
                  "model inside the drive, the current row will not agree "
                  "and is not meant to.", 12, B.INK3, width=xb - xa)

        xc = 700
        self.text(xc, 120, "THE REVERSE CHECK", 12, B.INK3)
        cv.create_line(xc, 138, X1, 138, fill=B.LINE)
        self.reverse = self.text(xc, 150, "", 12, B.INK2, width=X1 - xc)
        self.text(xc, 300, "READ IT BACKWARDS", 12, B.INK3)
        self.text(xc, 320,
                  "The duty the bench measured on the drive's pins, turned "
                  "back into the voltage the drive says it applied. Two "
                  "chips, two measurements, one number - arrived at from "
                  "opposite ends.", 11, B.INK3, width=X1 - xc)

    def update(self):
        app = self.app
        link, bench = app.board, app.bench
        d = (link.state or {}) if link else {}
        b = bench.state if bench else {}

        def put(key, left, right, tol):
            l_item, r_item, diff = self.cells[key]
            if left is None or right is None:
                self.set(l_item, "–")
                self.set(r_item, "–")
                self.set(diff, "")
                return
            self.set(l_item, f"{left:,.0f}")
            self.set(r_item, f"{right:,.0f}")
            span = max(abs(left), abs(right), 1.0)
            err = abs(left - right) / span
            self.set(diff, f"{err * 100:.1f} %",
                     B.GREEN if (tol and err <= tol) else B.RED)

        put("speed", d.get("spd"), b.get("spd"), 0.08)

        d_ang, b_ang = d.get("ang"), b.get("ang")
        l_item, r_item, diff = self.cells["angle"]
        if d_ang is None or b_ang is None:
            self.set(l_item, "–")
            self.set(r_item, "–")
            self.set(diff, "")
        else:
            wrap = (d_ang - b_ang) % 65536
            wrap = wrap if wrap < 32768 else wrap - 65536
            self.set(l_item, f"{d_ang:,}")
            self.set(r_item, f"{b_ang:,}")
            self.set(diff, f"{wrap / 65536 * 360:+.0f}°",
                     B.GREEN if abs(wrap) < 6000 else B.MUTE)

        ia, ib, ang = b.get("ia"), b.get("ib"), b.get("ang")
        bench_q = None
        if None not in (ia, ib, ang):
            beta = (ia + 2.0 * ib) / 1.7320508075688772
            th = ang / 65536.0 * 2.0 * math.pi
            bench_q = -ia * math.sin(th) + beta * math.cos(th)
        put("current", d.get("iq"), bench_q, 0.35)

        vd, vq = (link.vd, link.vq) if link else (None, None)
        drive_v = (math.hypot(vd, vq) if None not in (vd, vq) else None)
        duties = [b.get(k) for k in ("da", "db", "dc")]
        bench_v = None
        if None not in duties:
            # the spread of the three duties is the line-to-line amplitude;
            # over root three it is the phase amplitude, in the same Q15
            # scale the drive reports its voltage vector in
            bench_v = (max(duties) - min(duties)) / 1800.0 * 32768.0 / 1.7320508
        put("volt", drive_v, bench_v, 0.35)

        chk = next(c for c in app.checks if c.key == "sync")
        if bench is None:
            self.set(self.verdict,
                     "Connect the bench board on page 4 to compare them.",
                     B.INK3)
        elif chk.ok is None:
            self.set(self.verdict, "Waiting for both boards to say something.",
                     B.INK3)
        elif chk.ok:
            self.set(self.verdict,
                     "The two agree. The drive is controlling a motor it can "
                     "only reach through wires.", B.GREEN)
        else:
            self.set(self.verdict,
                     "The two do not agree. If page 2 is not set to the "
                     "pins, the drive is steering by its own model and the "
                     "rotor over there is only being dragged along.",
                     B.ORANGE)

        if drive_v is not None and bench_v is not None:
            self.set(self.reverse,
                     f"The drive reports a voltage vector of {drive_v:,.0f} "
                     f"counts. The bench measured duties spread over "
                     f"{max(duties) - min(duties)} of 1800 on the drive's own "
                     f"pins, which is {bench_v:,.0f} counts of phase "
                     f"amplitude. Nothing told either board the other's "
                     f"answer.")
        else:
            self.set(self.reverse, "needs both boards, and the drive "
                                   "switching.")


class App(tk.Tk):
    NAV = (("drive", "drive"), ("bench", "bench"), ("current", "current"),
           ("motor", "motor"), ("sync", "sync"), ("display", "display"),
           ("tune", "auto tune"), ("params", "parameters"),
           ("scope", "scope"), ("board", "board"), ("vehicle", "vehicle"))
    LAMP = {"ready": B.MUTE, "run": B.GREEN, "stopping": B.YELLOW,
            "tuning": B.ORANGE, "fault": B.RED}

    def __init__(self):
        super().__init__()
        self.title("FOC studio")
        self.geometry(f"{W}x{H}+60+40")
        self.resizable(False, False)
        self.configure(bg=B.PAPER)
        try:
            # `default` is not a detail: the plain form sets the icon on
            # this window only (WM_SETICON), and the taskbar draws the
            # window class's icon, which Tk registers with its own feather.
            # This form sets the application's, class icon included, and
            # covers every toplevel opened later.
            self.iconbitmap(default=resource_path(ICON_FILE))
        except tk.TclError:
            try:
                self.iconbitmap(resource_path(ICON_FILE))
            except tk.TclError:
                pass

        self.drive = Drive()
        self.tune = AutoTune(self.drive)
        self.board = None
        self.btune = None               # auto tune for the board, if any
        self.board_targets = {"cbw": 400.0, "sbw": 20.0}
        # The bench board, when this window is holding it as well. It is
        # not a second drive: nothing in here joins the two, they are
        # joined by the wires between the boards.
        self.bench = None
        self.checks = benchlink.build_checks()
        # The drive is asked a hundred times a second and the bench
        # twenty; both numbers ride the loop's ripple. The comparison is
        # between what each board says on average, or the lamp reports
        # the sampling instant rather than the agreement.
        self._drive_spd_mean = None
        self._rejoin = None             # [port, give up at, try next at]: on CAN
        self._sim_params = SimParams(self)
        self._board_params = BoardParams(self)
        self.src = SimSource(self)
        self.setpoint = 0.0
        self._sim_setpoint = 0.0
        self._spd_due = False
        self._spd_sent = 0.0
        self.disp = dict(rpm=0.0, iq=0.0, id=0.0, iqref=0.0, v=0.0, p=0.0,
                         tq=0.0, f=0.0)
        self._font12 = tkfont.Font(root=self, font=B.font(12))

        self._build_rail()
        self.pages = {cls.key: cls(self) for cls in
                      (DrivePage, BenchPage, CurrentPage, MotorPage,
                       SyncPage, DisplayPage, TunePage, ParamPage, ScopePage,
                       BoardPage, VehiclePage)}
        self.current = None
        self.show("drive")

        self.bind("<Key>", self._key)
        self.protocol("WM_DELETE_WINDOW", self.quit_app)
        self._last = time.perf_counter()
        self._acc = 0.0
        self.after(20, self._tick)

    def measure(self, text):
        return self._font12.measure(text)

    def board_can_tune(self):
        return self.board is not None and (self.board.fw or 0) >= 2

    def board_has_vehicle(self):
        return self.board is not None and (self.board.fw or 0) >= 3

    def rejoining(self):
        """True while the board is away on CAN and expected back."""
        return self._rejoin is not None

    def active_tune(self):
        """The auto tune the tune page is showing: the board's, when the
        board can be tuned, otherwise the simulator's."""
        return self.btune if self.board_can_tune() else self.tune

    def param_source(self):
        return self._board_params if self.board_can_tune() else self._sim_params

    # ---- the dark rail on the left

    def _build_rail(self):
        cv = tk.Canvas(self, width=RAIL, height=H, bg=B.DARK,
                       highlightthickness=0, bd=0)
        cv.place(x=0, y=0)
        self.rail = cv
        cv.create_text(30, 28, anchor="nw", text="foc", fill=B.WHITE,
                       font=B.font(38, bold=True))
        cv.create_text(32, 74, anchor="nw", text="studio", fill=B.MUTE,
                       font=B.font(16))
        cv.create_line(30, 116, RAIL - 30, 116, fill=B.DARK3)
        self.nav = {}
        for i, (key, label) in enumerate(self.NAV):
            self.nav[key] = B.RoundButton(
                cv, 46, 138 + i * 42, 32, B.DARK3, lambda k=key: self.show(k),
                glyph=str(i + 1), glyph_size=12, label=label, label_side="e",
                label_size=13, label_color=B.MUTE)
        cv.create_image(RAIL / 2, 626, image=B.grille(13, 5, 12, 5))
        cv.create_line(30, 668, RAIL - 30, 668, fill=B.DARK3)
        self.lamp = B.Lamp(cv, 37, 696, "ready", d=11, text_color=B.WHITE,
                           off=B.DARK3)
        self.t_time = cv.create_text(30, 722, anchor="nw", fill=B.MUTE,
                                     font=B.font(11), text="")
        cv.create_text(30, 758, anchor="nw", fill=B.DARK3, font=B.font(11),
                       text=f"v{VERSION}")
        self._rail_state = None

    def _rail_update(self):
        link = self.board
        if link is not None:
            fault = link.fault
            state = "fault" if fault else ("run" if link.running else "ready")
            text = (f"fault · {fault}" if fault else
                    (link.mode or "connecting"))
            line2 = f"board  {link.port}  ·  {link.rate:.0f} Hz"
        else:
            d = self.drive
            state = d.state
            text = f"fault · {d.fault}" if d.fault else state
            line2 = f"simulator  ·  t {d.t:.1f} s"
        if (text, state) != self._rail_state:
            self._rail_state = (text, state)
            self.lamp.set(self.LAMP[state])
            self.rail.itemconfigure(self.lamp.text, text=text)
        self.rail.itemconfigure(self.t_time, text=line2)

    def show(self, key):
        page = self.pages[key]
        if self.current is page:
            return
        if self.current is not None:
            self.current.cv.place_forget()
        self.current = page
        page.cv.place(x=RAIL, y=0)
        page.cv.focus_set()
        for k, button in self.nav.items():
            on = k == key
            button.set_color(B.ORANGE if on else B.DARK3)
            button.set_label(color=B.WHITE if on else B.MUTE)

    # ---- the board

    def connect(self, port):
        """Open the board. Returns "" or the reason it could not be opened."""
        if self.board is not None:
            return ""
        if self.tune.active:
            return "auto tune is running on the simulator. let it finish first."
        try:
            link = boardlink.BoardLink(port)
        except OSError as err:
            return str(err) + "."
        self.board = link
        self.btune = boardtune.BoardTune(link, self.board_targets)
        self.src = BoardSource(link)
        self._sim_setpoint = self.setpoint
        self.setpoint = 0.0
        self._spd_due = False
        self._source_changed()
        return ""

    def disconnect(self, why=""):
        link = self.board
        if link is None:
            return
        if self.btune is not None and self.btune.active:
            self.btune.abort()          # puts the board's gains back first
        self.board = None
        self.btune = None
        self.src = SimSource(self)
        link.close()
        self.setpoint = self._sim_setpoint
        self._last = time.perf_counter()       # the simulator was paused
        self._acc = 0.0
        self.pages["board"].msg = why
        self._source_changed()

    def _source_changed(self):
        for key in ("rpm", "iq", "id", "iqref", "v", "p", "tq", "f"):
            self.disp[key] = 0.0
        for key in ("drive", "display", "scope"):
            self.pages[key].rebuild()

    def bench_connect(self, port):
        """Open the bench board's console. Its own port, never the drive's."""
        if self.board is not None and port == self.board.port:
            return "that is the drive's port."
        try:
            self.bench = benchlink.BenchLink(port)
        except OSError as exc:
            self.bench = None
            return str(exc)
        return ""

    def bench_disconnect(self):
        if self.bench is not None:
            self.bench.close()
            self.bench = None

    def quit_app(self):
        self.disconnect()
        self.destroy()

    # ---- commands

    def set_setpoint(self, rpm):
        vmax = self.src.vmax
        self.setpoint = float(max(-vmax, min(vmax, round(rpm / 10.0) * 10.0)))
        link = self.board
        if link is not None:
            # Not on every pixel of a drag: each `spd` restarts the
            # firmware's speed integrator. _tick sends at most 10 a second.
            if link.mode == "speed" and not self.btune.active:
                self._spd_due = True
            return
        d = self.drive
        if d.enabled and not d.stopping and not self.tune.active:
            d.set_target_rpm(self.setpoint)

    def cmd_run(self):
        link = self.board
        if link is not None:
            if self.btune.active:
                return
            link.run_speed(self.setpoint)
            self._spd_due = False
            self._spd_sent = time.perf_counter()
            return
        d = self.drive
        if self.tune.active or d.fault:
            return
        if not d.enabled or d.stopping:
            d.run()
        d.set_target_rpm(self.setpoint)

    def cmd_stop(self):
        if self.board is not None:
            self._spd_due = False
            if self.btune.active:
                self.btune.abort()
            else:
                self.board.stop()
        elif not self.tune.active:
            self.drive.stop()

    def cmd_reset(self):
        if self.board is not None:
            self.board.send("clear")
        else:
            self.drive.reset_fault()

    def tune_start(self):
        if self.board is None:
            self.tune.start()
        elif self.board_can_tune():
            self.btune.start()

    def set_param(self, key, value):
        d = self.drive
        q = BY_KEY[key]
        if self.tune.active:
            return False, "auto tune is running."
        if self.tune.pending:
            return False, "apply or discard the auto tune result first."
        if q.runlock and d.enabled:
            return False, "stop the drive to change this parameter."
        if q.runlock and abs(d.rpm) > 5.0:
            return False, "wait for the shaft to come to rest."
        before = d.p[key]
        d.set_param(key, value)
        if key == "max_speed" and d.p[key] != before:
            self._rescale()
        return True, ""

    def _rescale(self):
        if self.board is not None:
            self._sim_setpoint = max(-self.drive.p["max_speed"],
                                     min(self.drive.p["max_speed"],
                                         self._sim_setpoint))
            return
        self.set_setpoint(self.setpoint)
        self.pages["drive"].rebuild()
        self.pages["display"].rebuild()

    def _key(self, e):
        if isinstance(self.focus_get(), tk.Entry):
            return
        if e.char and e.char in "1234567":
            self.show(self.NAV[int(e.char) - 1][0])
        elif e.keysym == "space":
            running = (self.board.running if self.board is not None else
                       (self.drive.enabled and not self.drive.stopping))
            if running:
                self.cmd_stop()
            else:
                self.cmd_run()
        elif e.keysym in ("Right", "Up"):
            self.set_setpoint(self.setpoint + 50)
        elif e.keysym in ("Left", "Down"):
            self.set_setpoint(self.setpoint - 50)

    # ---- the frame loop

    def _bench_tick(self):
        if self.bench is not None:
            self.bench.poll()
            if self.bench.error:
                self.bench_disconnect()
        if self.board is not None and self.board.state:
            spd = self.board.state.get("spd")
            if isinstance(spd, int):
                self._drive_spd_mean = (
                    float(spd) if self._drive_spd_mean is None else
                    self._drive_spd_mean + 0.02 * (spd - self._drive_spd_mean))
        else:
            self._drive_spd_mean = None
        benchlink.run_checks(self.checks, self.bench, self._drive_spd_mean)

    def _tick(self):
        now = time.perf_counter()
        elapsed = min(now - self._last, 0.08)
        self._last = now
        self._bench_tick()
        a = 1.0 - math.exp(-elapsed / 0.12)
        disp = self.disp

        link = self.board
        if link is not None and link.error and link.away:
            # It was told to change over to CAN, so the port going is the
            # expected thing. Keep trying it until the board is back.
            port = link.port
            wait = float(link.params.get("canback", 10) or 600) + 12.0
            self.disconnect("the board is on can. waiting for it to come "
                            "back to usb ...")
            self._rejoin = [port, now + wait, now + 1.0]
            link = None
        elif link is not None and link.error:
            self.disconnect(f"lost the board on {link.port}: {link.error}.")
            link = None
        elif link is not None and link.silent_for > 3.0:
            self.disconnect(f"{link.port} stopped answering. is it still "
                            "the foc firmware, and still plugged in?")
            link = None

        if self._rejoin is not None and link is None and now >= self._rejoin[2]:
            port, until, _next = self._rejoin
            why = self.connect(port)
            if not why:
                self._rejoin = None
                self.pages["board"].msg = "back from can."
                link = self.board
            elif now > until:
                self._rejoin = None
                self.pages["board"].msg = (f"{port} did not come back from "
                                           f"can: {why}")
            else:
                self._rejoin[2] = now + 0.5

        if link is not None:
            # The simulator stands still while the board is the source.
            if self.btune.active:
                self.btune.advance()
            if (self._spd_due and link.mode == "speed"
                    and now - self._spd_sent > 0.1):
                link.run_speed(self.setpoint)
                self._spd_due = False
                self._spd_sent = now
            st = link.state
            if st:
                for key, value in (("rpm", link.rpm), ("iq", st["iq"]),
                                   ("id", st["id"]), ("iqref", link.iqref)):
                    disp[key] += (value - disp[key]) * a
        else:
            self._acc += elapsed
            n = int(self._acc / Drive.DT)
            self._acc -= n * Drive.DT
            if self.tune.active:
                self.tune.advance(n)
            else:
                self.drive.advance(n)
            d = self.drive
            for key, value in (("rpm", d.rpm), ("iq", d.iqm), ("id", d.idm),
                               ("v", d.vmag), ("p", d.power),
                               ("tq", d.torque_mnm), ("f", d.freq_hz)):
                disp[key] += (value - disp[key]) * a

        self._rail_update()
        self.current.update()
        self.after(20, self._tick)


# ------------------------------------------------------------ entry points ---

def run_selftest(path):
    rows = selfcheck()
    failed = sum(1 for _n, ok, _d in rows if not ok)
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"FOC studio {VERSION} self-check\n")
        for name, ok, detail in rows:
            f.write(("ok    " if ok else "FAIL  ") + f"{name:38s} {detail}\n")
        f.write(f"{len(rows) - failed} of {len(rows)} passed\n")
    return 1 if failed else 0


def run_shots(app, outdir, port=None):
    """Walk through every page, saving a picture of each. With a port, do
    the board pages as well. Sends spd and off to the board, nothing else."""
    from PIL import ImageGrab
    os.makedirs(outdir, exist_ok=True)
    app.attributes("-topmost", True)

    def snap(name):
        app.update()
        x, y = app.winfo_rootx(), app.winfo_rooty()
        ImageGrab.grab(bbox=(x, y, x + W, y + H)).save(
            os.path.join(outdir, name + ".png"))

    def script():
        yield 700
        snap("1-drive-idle")
        app.set_setpoint(1800)
        app.cmd_run()
        yield 1300
        app.drive.set_param("x_load", 120)
        yield 900
        snap("2-drive-running")
        app.show("display")
        yield 600
        snap("3-display")
        app.show("scope")
        yield 300
        app.set_setpoint(-1200)
        yield 1000
        snap("4-scope")
        app.pages["scope"].span = 0
        yield 300
        snap("5-scope-fast")
        app.pages["scope"].span = 2
        app.show("params")
        app.pages["params"].select("m_ls")
        yield 400
        snap("6-parameters")
        app.drive.set_param("x_load", 0)
        app.cmd_stop()
        while app.drive.enabled:
            yield 100
        app.show("tune")
        yield 300
        app.tune_start()
        yield 2500
        snap("7-auto-tune-running")
        while app.tune.active:
            yield 200
        yield 400
        snap("8-auto-tune-done")
        app.tune.apply()
        app.show("drive")
        app.set_setpoint(2400)
        app.cmd_run()
        yield 1600
        snap("9-drive-tuned")
        app.cmd_stop()
        app.show("board")
        yield 500
        snap("10-board-not-connected")

        if port:
            why = app.connect(port)
            if why:
                with open(os.path.join(outdir, "board-error.txt"), "w") as f:
                    f.write(why + "\n")
            else:
                yield 1200
                snap("11-board-connected")
                app.show("drive")
                app.set_setpoint(1500)
                app.cmd_run()
                yield 1500
                app.set_setpoint(3000)
                yield 1200
                snap("12-board-drive")
                app.show("display")
                yield 500
                snap("13-board-display")
                app.show("scope")
                yield 300
                app.set_setpoint(-2000)
                yield 1500
                snap("14-board-scope")
                app.cmd_stop()
                yield 800
                app.show("display")
                yield 300
                snap("15-board-display-idle")
                app.show("board")
                yield 500
                snap("16-board-after")
                if app.board_can_tune():
                    app.show("params")
                    yield 400
                    snap("17-board-parameters")
                    app.show("tune")
                    yield 300
                    app.tune_start()
                    yield 2000
                    snap("18-board-auto-tune-running")
                    while app.btune.active:
                        yield 200
                    yield 400
                    snap("19-board-auto-tune-done")
                    app.btune.apply()
                    app.show("drive")
                    app.set_setpoint(1500)
                    app.cmd_run()
                    yield 1500
                    app.set_setpoint(3000)
                    yield 1200
                    snap("20-board-drive-tuned")
                    app.show("params")
                    yield 400
                    snap("21-board-parameters-tuned")
                    app.cmd_stop()
                    yield 600
                    app.pages["params"]._reset_all()
                    yield 400
        app.quit_app()

    gen = script()

    def step():
        try:
            delay = next(gen)
        except StopIteration:
            return
        app.after(delay, step)

    app.after(100, step)


def run_vehicle_shots(app, outdir, port):
    """Pictures of the vehicle page. Sends throttle, brake, sensor and
    parameter commands; never arms the outputs."""
    from PIL import ImageGrab
    os.makedirs(outdir, exist_ok=True)
    app.attributes("-topmost", True)
    page = app.pages["vehicle"]

    def snap(name):
        app.update()
        x, y = app.winfo_rootx(), app.winfo_rooty()
        ImageGrab.grab(bbox=(x, y, x + W, y + H)).save(
            os.path.join(outdir, name + ".png"))

    def script():
        app.show("vehicle")
        yield 600
        snap("v1-not-connected")
        why = app.connect(port)
        if why:
            with open(os.path.join(outdir, "vehicle-error.txt"), "w") as f:
                f.write(why + "\n")
            app.quit_app()
            return
        yield 1500
        snap("v2-idle")
        page._oneline()
        page._drive_on()
        yield 500
        page._set_throttle(70, True)
        yield 2500
        snap("v3-throttle")
        page._brake()
        yield 250
        snap("v4-braking")
        yield 1500
        page._drive_off()
        yield 600
        page._hall_kind(1)
        yield 300
        page._calibrate()
        yield 1000
        snap("v5-hall-calibrating")
        while app.board.veh.get("ph") == 4:
            yield 200
        yield 800
        page._sensor(1)
        yield 400
        page._drive_on()
        yield 400
        page._set_throttle(40, True)
        yield 2500
        page._pin_check()
        yield 1500
        snap("v6-hall-3144")
        page._drive_off()
        yield 1200
        page._motor(1)
        yield 600
        page._sensor(2)
        yield 400
        page._weaken()
        yield 300
        page._drive_on()
        yield 400
        page._set_throttle(100, True)
        yield 1200
        snap("v7-hub-observer-starting")
        yield 5000
        snap("v8-hub-weakening")
        page._drive_off()
        yield 3000
        page._weaken()
        page._sensor(0)
        yield 400
        page._motor(0)
        page._hall_kind(0)
        page._oneline()
        yield 800
        app.show("params")
        yield 400
        app.pages["params"].pick_group(6)
        yield 500
        snap("v9-parameters")
        app.show("vehicle")
        if "--can" in sys.argv:
            app.board.set_param("canback", 4)
            yield 300
            page._can_on()
            yield 2000
            snap("v10-on-can")
            for _ in range(60):
                if app.board is not None:
                    break
                yield 300
            yield 1500
            snap("v11-back-from-can")
            if app.board is not None:
                app.board.set_param("canback", 10)
                yield 300
        app.quit_app()

    gen = script()

    def step():
        try:
            delay = next(gen)
        except StopIteration:
            return
        except Exception:
            import traceback
            with open(os.path.join(outdir, "vehicle-error.txt"), "w") as f:
                traceback.print_exc(file=f)
            app.quit_app()
            return
        app.after(delay, step)

    app.after(100, step)


def main(argv):
    if "--version" in argv:
        return 0
    if "--selftest" in argv:
        return run_selftest(argv[argv.index("--selftest") + 1])
    try:
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_ID)
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass
    app = App()
    if "--vehicle-shots" in argv:
        port = (argv[argv.index("--port") + 1] if "--port" in argv
                else boardlink.EMULATOR_PORT)
        run_vehicle_shots(app, argv[argv.index("--vehicle-shots") + 1], port)
    if "--shots" in argv:
        port = argv[argv.index("--port") + 1] if "--port" in argv else None
        run_shots(app, argv[argv.index("--shots") + 1], port)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

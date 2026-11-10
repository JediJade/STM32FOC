"""
A small drawing kit for tk.Canvas in the manner of 1960s Braun: warm paper
white, near-black, one orange, round push buttons, hairlines, lower case.

Tk draws shapes without anti-aliasing, so everything round or diagonal here
is rendered with Pillow at several times its size and scaled down: buttons,
slider caps, the dial face and needle, and every trace on the dark screens.
Text and axis-aligned hairlines are left to Tk, which does those well.
"""

import math
import tkinter as tk

from PIL import Image, ImageDraw, ImageFilter, ImageTk

PAPER = "#E9E6DE"
PANEL = "#F6F4EF"
INK = "#1C1C1A"
INK2 = "#55534D"
INK3 = "#99968D"
LINE = "#CFCBC0"
SLOT = "#BDB9AD"
DARK = "#1E1E1C"
DARK2 = "#2E2E2B"
DARK3 = "#454540"
GRID = "#34342F"
ORANGE = "#E8541E"
YELLOW = "#E9B124"
GREEN = "#4F8F5B"
RED = "#C93A2E"
WHITE = "#F4F2EC"
MUTE = "#8C897F"

UI = "Arial"
NUM = "Bahnschrift Light"


def font(size, bold=False, num=False):
    return (NUM if num else UI, -int(size), "bold" if bold else "normal")


def rgb(c):
    return tuple(int(c[i:i + 2], 16) for i in (1, 3, 5))


def shade(c, f):
    return tuple(max(0, min(255, int(v * f))) for v in rgb(c))


def nice_step(span, target=4):
    """A 1-2-5 step that divides span into roughly `target` parts."""
    if span <= 0:
        return 1.0
    raw = span / target
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


def nice_range(lo, hi, min_span):
    """Round a data range outwards to a tidy axis."""
    if hi - lo < min_span:
        mid = 0.5 * (hi + lo)
        lo = mid - 0.5 * min_span
        hi = mid + 0.5 * min_span
    step = nice_step(hi - lo)
    return math.floor(lo / step) * step, math.ceil(hi / step) * step


def num(value, decimals=0):
    """A number for display: real minus sign, and never a negative zero."""
    s = f"{value:.{decimals}f}"
    if s.startswith("-"):
        s = "0" if float(s) == 0 else "−" + s[1:]
        if s == "0" and decimals:
            s = f"{0:.{decimals}f}"
    return s


# ---------------------------------------------------------------- sprites ---

_cache = {}


def _sprite(key, build):
    if key not in _cache:
        _cache[key] = ImageTk.PhotoImage(build())
    return _cache[key]


def _disc_image(d, color, pressed, flat):
    s = 4
    pad = 7
    w = d + 2 * pad
    im = Image.new("RGBA", (w * s, w * s), (0, 0, 0, 0))
    if not flat:
        drop = 1 if pressed else 3
        sh = Image.new("RGBA", im.size, (0, 0, 0, 0))
        ImageDraw.Draw(sh).ellipse(
            [pad * s, (pad + drop) * s, (pad + d) * s, (pad + d + drop) * s],
            fill=(0, 0, 0, 70 if pressed else 105))
        im.alpha_composite(sh.filter(ImageFilter.GaussianBlur(2.0 * s)))
        grad = Image.linear_gradient("L").resize((d * s, d * s))
        top = shade(color, 0.96 if pressed else 1.13)
        bot = shade(color, 0.90 if pressed else 0.84)
        body = Image.composite(Image.new("RGB", (d * s, d * s), bot),
                               Image.new("RGB", (d * s, d * s), top), grad)
    else:
        body = Image.new("RGB", (d * s, d * s), rgb(color))
    mask = Image.new("L", (d * s, d * s), 0)
    ImageDraw.Draw(mask).ellipse([0, 0, d * s - 1, d * s - 1], fill=255)
    im.paste(body, (pad * s, pad * s), mask)
    return im.resize((w, w), Image.LANCZOS)


def disc(d, color, pressed=False, flat=False):
    return _sprite(("disc", d, color, pressed, flat),
                   lambda: _disc_image(d, color, pressed, flat))


def _rrect_image(w, h, r, fill, outline):
    s = 4
    im = Image.new("RGBA", (w * s, h * s), (0, 0, 0, 0))
    ImageDraw.Draw(im).rounded_rectangle(
        [0, 0, w * s - 1, h * s - 1], radius=r * s, fill=rgb(fill) + (255,),
        outline=(rgb(outline) + (255,)) if outline else None, width=s)
    return im.resize((w, h), Image.LANCZOS)


def rrect(w, h, r, fill, outline=None):
    return _sprite(("rrect", w, h, r, fill, outline),
                   lambda: _rrect_image(w, h, r, fill, outline))


def _cap_image():
    s = 4
    w, h, pad = 22, 40, 6
    im = Image.new("RGBA", ((w + 2 * pad) * s, (h + 2 * pad) * s), (0, 0, 0, 0))
    sh = Image.new("RGBA", im.size, (0, 0, 0, 0))
    ImageDraw.Draw(sh).rounded_rectangle(
        [pad * s, (pad + 3) * s, (pad + w) * s, (pad + h + 3) * s],
        radius=4 * s, fill=(0, 0, 0, 110))
    im.alpha_composite(sh.filter(ImageFilter.GaussianBlur(2.0 * s)))
    grad = Image.linear_gradient("L").resize((w * s, h * s))
    body = Image.composite(Image.new("RGB", (w * s, h * s), shade(INK, 0.85)),
                           Image.new("RGB", (w * s, h * s), shade(INK, 1.9)), grad)
    mask = Image.new("L", (w * s, h * s), 0)
    ImageDraw.Draw(mask).rounded_rectangle([0, 0, w * s - 1, h * s - 1],
                                           radius=4 * s, fill=255)
    im.paste(body, (pad * s, pad * s), mask)
    cx = (pad + w / 2) * s
    ImageDraw.Draw(im).rounded_rectangle(
        [cx - s, (pad + 8) * s, cx + s, (pad + h - 8) * s], radius=s,
        fill=rgb(ORANGE) + (255,))
    return im.resize((w + 2 * pad, h + 2 * pad), Image.LANCZOS)


def cap():
    return _sprite(("cap",), _cap_image)


def _grille_image(cols, rows, pitch, dot, color):
    s = 4
    w, h = cols * pitch, rows * pitch
    im = Image.new("RGBA", (w * s, h * s), (0, 0, 0, 0))
    dr = ImageDraw.Draw(im)
    fill = rgb(color) + (255,)
    for r in range(rows):
        for c in range(cols):
            x = (c * pitch + pitch / 2) * s
            y = (r * pitch + pitch / 2) * s
            dr.ellipse([x - dot * s / 2, y - dot * s / 2,
                        x + dot * s / 2, y + dot * s / 2], fill=fill)
    return im.resize((w, h), Image.LANCZOS)


def grille(cols, rows, pitch=12, dot=5, color="#0E0E0D"):
    return _sprite(("grille", cols, rows, pitch, dot, color),
                   lambda: _grille_image(cols, rows, pitch, dot, color))


# ---------------------------------------------------------------- widgets ---

class RoundButton:
    """A convex round push button, with an optional glyph on it and a label
    beside or below it."""

    def __init__(self, cv, x, y, d, color, command=None, glyph="",
                 glyph_color=WHITE, glyph_size=None, label="",
                 label_color=INK2, label_side="s", label_size=11):
        self.cv = cv
        self.x, self.y, self.d = x, y, d
        self.command = command
        self.color = None
        self.tag = f"rb{id(self)}"
        self.item = cv.create_image(x, y, tags=(self.tag,))
        self.glyph = None
        if glyph:
            self.glyph = cv.create_text(
                x, y, text=glyph, fill=glyph_color, tags=(self.tag,),
                font=font(glyph_size or int(d * 0.42)))
        self.label = None
        if label:
            if label_side == "s":
                self.label = cv.create_text(
                    x, y + d / 2 + 12, text=label, fill=label_color,
                    font=font(label_size), anchor="n", tags=(self.tag,))
            else:
                self.label = cv.create_text(
                    x + d / 2 + 14, y, text=label, fill=label_color,
                    font=font(label_size), anchor="w", tags=(self.tag,))
        self.set_color(color)
        cv.tag_bind(self.tag, "<ButtonPress-1>", self._down)
        cv.tag_bind(self.tag, "<ButtonRelease-1>", self._up)

    def set_color(self, color):
        if color != self.color:
            self.color = color
            self.cv.itemconfigure(self.item, image=disc(self.d, color))

    def set_label(self, color=None, text=None, bold=None):
        if self.label is None:
            return
        if color is not None:
            self.cv.itemconfigure(self.label, fill=color)
        if text is not None:
            self.cv.itemconfigure(self.label, text=text)

    def _down(self, _e):
        self.cv.itemconfigure(self.item, image=disc(self.d, self.color, True))
        if self.glyph:
            self.cv.coords(self.glyph, self.x, self.y + 1)

    def _up(self, _e):
        self.cv.itemconfigure(self.item, image=disc(self.d, self.color))
        if self.glyph:
            self.cv.coords(self.glyph, self.x, self.y)
        if self.command:
            self.command()


class Lamp:
    """A small flat indicator with its label to the right."""

    OFF = "#C6C2B6"

    def __init__(self, cv, x, y, text, d=11, text_color=INK2, off=None):
        self.cv = cv
        self.d = d
        self.off = off or self.OFF
        self.color = None
        self.item = cv.create_image(x, y)
        self.text = cv.create_text(x + d / 2 + 9, y, text=text, anchor="w",
                                   fill=text_color, font=font(12))
        self.set(None)

    def set(self, color):
        color = color or self.off
        if color != self.color:
            self.color = color
            self.cv.itemconfigure(self.item, image=disc(self.d, color, flat=True))


class Fader:
    """A horizontal slide control with a scale underneath."""

    def __init__(self, cv, x, y, length, lo, hi, value=0.0, command=None,
                 step=1.0, snap=None, majors=(), minors=(), label=str,
                 bg=PAPER):
        self.cv = cv
        self.x, self.y, self.length = x, y, length
        self.lo, self.hi = lo, hi
        self.step = step
        self.snap = snap
        self.command = command
        self.dragging = False
        self.tag = f"fd{id(self)}"
        cv.create_rectangle(x - 16, y - 26, x + length + 16, y + 26,
                            fill=bg, outline="", tags=(self.tag,))
        cv.create_image(x + length / 2, y, image=rrect(length + 14, 8, 4, SLOT),
                        tags=(self.tag,))
        for v in minors:
            px = self._px(v)
            cv.create_line(px, y + 18, px, y + 23, fill=INK3)
        for v in majors:
            px = self._px(v)
            cv.create_line(px, y + 18, px, y + 27, fill=INK2)
            cv.create_text(px, y + 31, text=label(v), anchor="n", fill=INK3,
                           font=font(11))
        self.value = value
        self.cap = cv.create_image(self._px(value), y, image=cap(),
                                   tags=(self.tag,))
        cv.tag_bind(self.tag, "<ButtonPress-1>", self._press)
        cv.tag_bind(self.tag, "<B1-Motion>", self._drag)
        cv.tag_bind(self.tag, "<ButtonRelease-1>", self._release)

    def _px(self, v):
        return self.x + (v - self.lo) / (self.hi - self.lo) * self.length

    def set(self, value, notify=False):
        value = max(self.lo, min(self.hi, value))
        value = round(value / self.step) * self.step
        if value != self.value:
            self.value = value
            self.cv.coords(self.cap, self._px(value), self.y)
            if notify and self.command:
                self.command(value)

    def _from_event(self, e):
        v = self.lo + (e.x - self.x) / self.length * (self.hi - self.lo)
        if self.snap is not None and abs(v) < self.snap:
            v = 0.0
        self.set(v, notify=True)

    def _press(self, e):
        self.dragging = True
        self._from_event(e)

    def _drag(self, e):
        self._from_event(e)

    def _release(self, _e):
        self.dragging = False

    def hit(self, x, y):
        return (self.x - 16 <= x <= self.x + self.length + 16 and
                self.y - 30 <= y <= self.y + 44)

    def nudge(self, steps):
        self.set(self.value + steps * self.step, notify=True)


class Screen:
    """A dark display window. plot() and xy() render anti-aliased traces."""

    SS = 2

    def __init__(self, cv, x, y, w, h, inset=10):
        self.cv = cv
        cv.create_image(x, y, anchor="nw", image=rrect(w, h, 10, DARK))
        self.ix, self.iy = x + inset, y + inset
        self.iw, self.ih = w - 2 * inset, h - 2 * inset
        self.photo = ImageTk.PhotoImage(Image.new("RGB", (self.iw, self.ih), rgb(DARK)))
        cv.create_image(self.ix, self.iy, anchor="nw", image=self.photo)
        self.t_hi = cv.create_text(self.ix + 4, self.iy + 1, anchor="nw",
                                   fill=MUTE, font=font(10), text="")
        self.t_lo = cv.create_text(self.ix + 4, self.iy + self.ih - 1,
                                   anchor="sw", fill=MUTE, font=font(10), text="")
        self.t_rt = cv.create_text(self.ix + self.iw - 4, self.iy + self.ih - 1,
                                   anchor="se", fill=MUTE, font=font(10), text="")
        self._labels = (None, None, None)

    def labels(self, hi="", lo="", right=""):
        if (hi, lo, right) != self._labels:
            self._labels = (hi, lo, right)
            self.cv.itemconfigure(self.t_hi, text=hi)
            self.cv.itemconfigure(self.t_lo, text=lo)
            self.cv.itemconfigure(self.t_rt, text=right)

    def _canvas(self):
        s = self.SS
        im = Image.new("RGB", (self.iw * s, self.ih * s), rgb(DARK))
        return im, ImageDraw.Draw(im), self.iw * s, self.ih * s

    def plot(self, series, ymin, ymax, xdiv=10, ydiv=4):
        """series: (values, colour, width). Values are spread evenly over
        the width, oldest at the left."""
        im, dr, w, h = self._canvas()
        grid = rgb(GRID)
        for i in range(1, xdiv):
            x = w * i // xdiv
            dr.line([(x, 0), (x, h)], fill=grid, width=1)
        for i in range(1, ydiv):
            y = h * i // ydiv
            dr.line([(0, y), (w, y)], fill=grid, width=1)
        span = ymax - ymin
        k = (h - 1) / span
        if ymin < 0.0 < ymax:
            y0 = ymax * k
            dr.line([(0, y0), (w, y0)], fill=rgb(DARK3), width=2)
        for values, color, width in series:
            n = len(values)
            if n < 2:
                continue
            sx = (w - 1) / (n - 1)
            dr.line([(i * sx, (ymax - v) * k) for i, v in enumerate(values)],
                    fill=rgb(color), width=int(width * self.SS), joint="curve")
        self.photo.paste(im.reduce(self.SS))

    def xy(self, xs, ys, radius, color, rings=2):
        """A Lissajous view, square, centred, with guide rings."""
        im, dr, w, h = self._canvas()
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 8
        grid = rgb(GRID)
        dr.line([(cx - r, cy), (cx + r, cy)], fill=grid, width=1)
        dr.line([(cx, cy - r), (cx, cy + r)], fill=grid, width=1)
        for i in range(1, rings + 1):
            q = r * i / rings
            dr.ellipse([cx - q, cy - q, cx + q, cy + q], outline=grid, width=2)
        if len(xs) > 1 and radius > 0:
            k = r / radius
            dr.line([(cx + x * k, cy - y * k) for x, y in zip(xs, ys)],
                    fill=rgb(color), width=3, joint="curve")
        self.photo.paste(im.reduce(self.SS))


class Dial:
    """A round analogue speed indicator: zero at the top, 135 degrees either
    way, an orange needle for the value and a black index for the setpoint."""

    SWEEP = 135.0

    def __init__(self, cv, cx, cy, r, vmax, bg=PAPER):
        self.cv = cv
        self.cx, self.cy, self.r = cx, cy, r
        self.vmax = vmax
        self.major = nice_step(vmax, 3)
        self.minor = self.major / 5.0
        self._face = ImageTk.PhotoImage(self._face_image())
        cv.create_image(cx, cy, image=self._face)
        v = -math.floor(vmax / self.major) * self.major
        while v <= vmax + 1e-9:
            a = math.radians(v / vmax * self.SWEEP)
            rr = r - 46
            cv.create_text(cx + rr * math.sin(a), cy - rr * math.cos(a),
                           text=num(v / 1000.0, 0 if self.major >= 1000 else 1),
                           fill=INK, font=font(17, num=True))
            v += self.major
        cv.create_text(cx, cy - r * 0.36, text="rpm × 1000", fill=INK3,
                       font=font(11))
        self.size = 2 * r
        self.layer = ImageTk.PhotoImage(Image.new("RGBA", (self.size, self.size)))
        cv.create_image(cx, cy, image=self.layer)
        self._last = None

    def _face_image(self):
        s = 3
        r = self.r
        pad = 12
        w = 2 * (r + pad)
        im = Image.new("RGBA", (w * s, w * s), (0, 0, 0, 0))
        sh = Image.new("RGBA", im.size, (0, 0, 0, 0))
        ImageDraw.Draw(sh).ellipse(
            [pad * s, (pad + 4) * s, (pad + 2 * r) * s, (pad + 2 * r + 4) * s],
            fill=(0, 0, 0, 70))
        im.alpha_composite(sh.filter(ImageFilter.GaussianBlur(3.5 * s)))
        dr = ImageDraw.Draw(im)
        dr.ellipse([pad * s, pad * s, (pad + 2 * r) * s, (pad + 2 * r) * s],
                   fill=rgb(PANEL) + (255,), outline=rgb(LINE) + (255,), width=s)
        c = (pad + r) * s
        n = int(round(self.vmax / self.minor))
        for i in range(-n, n + 1):
            v = i * self.minor
            a = math.radians(v / self.vmax * self.SWEEP)
            is_major = abs(v / self.major - round(v / self.major)) < 1e-6
            r1 = (r - 14) * s
            r2 = (r - (30 if is_major else 22)) * s
            dr.line([(c + r1 * math.sin(a), c - r1 * math.cos(a)),
                     (c + r2 * math.sin(a), c - r2 * math.cos(a))],
                    fill=rgb(INK if is_major else INK3) + (255,),
                    width=int((2.2 if is_major else 1.2) * s))
        return im.resize((w, w), Image.LANCZOS)

    def update(self, value, setpoint):
        lim = self.vmax * 1.06
        value = max(-lim, min(lim, value))
        setpoint = max(-self.vmax, min(self.vmax, setpoint))
        key = (round(value / self.vmax * 1200), round(setpoint / self.vmax * 1200))
        if key == self._last:
            return
        self._last = key
        s = 2
        r = self.r
        w = self.size * s
        c = r * s
        im = Image.new("RGBA", (w, w), (0, 0, 0, 0))
        dr = ImageDraw.Draw(im)

        a = math.radians(setpoint / self.vmax * self.SWEEP)
        sa, ca = math.sin(a), math.cos(a)
        tip = (r - 13) * s
        base = (r - 3) * s
        half = 5 * s
        dr.polygon([(c + tip * sa, c - tip * ca),
                    (c + base * sa - half * ca, c - base * ca - half * sa),
                    (c + base * sa + half * ca, c - base * ca + half * sa)],
                   fill=rgb(INK) + (255,))

        a = math.radians(value / self.vmax * self.SWEEP)
        sa, ca = math.sin(a), math.cos(a)
        tip = (r - 34) * s
        tail = -26 * s
        hw = 2.6 * s
        dr.polygon([(c + tip * sa - 0.5 * s * ca, c - tip * ca - 0.5 * s * sa),
                    (c + tip * sa + 0.5 * s * ca, c - tip * ca + 0.5 * s * sa),
                    (c + tail * sa + hw * ca, c - tail * ca + hw * sa),
                    (c + tail * sa - hw * ca, c - tail * ca - hw * sa)],
                   fill=rgb(ORANGE) + (255,))
        hub = 13 * s
        dr.ellipse([c - hub, c - hub, c + hub, c + hub], fill=rgb(INK) + (255,))
        dot = 3 * s
        dr.ellipse([c - dot, c - dot, c + dot, c + dot], fill=rgb(ORANGE) + (255,))
        self.layer.paste(im.reduce(s))

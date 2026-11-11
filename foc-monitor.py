#!/usr/bin/env python3
"""
FOC monitor: a live view of the board over its USB serial console.

    python foc-monitor.py            the window
    python foc-monitor.py --check    exercise every command, no window

Built as a single .exe by build-exe.cmd, it takes the same arguments:

    foc-monitor.exe                  the window
    foc-monitor.exe --check          exercise every command, no window

No dependencies beyond the standard library. pyserial is not installed on this
machine, so the port is driven through Win32 directly with ctypes - which also
lets the read timeouts be set properly, so the UI never blocks waiting for a
board that has nothing to say.

What it shows and does:

  * the live loop state at 10 Hz: electrical angle, speed, i_d, i_q, the
    current reference, MOE and any latched fault
  * a scrolling chart of i_d, i_q and speed
  * the self-test summary
  * commands: current, speed and position targets, align, arm, disarm, clear

The board's console is the only interface used, so anything this window can do
can also be typed into a terminal - and vice versa.
"""

import collections
import ctypes
import ctypes.wintypes as wt
import os
import re
import sys
import tkinter as tk
import tkinter.ttk as ttk
import winreg

# The UnityMbed logo, lifted from unitymbed-desktop.exe's own resources so the
# window and the .exe wear the same icon as the IDE.
ICON_FILE = "unitymbed.ico"
APP_ID = "unitymbed.stm32foc.monitor"
VERSION = "1.0"


def resource_path(name):
    """Data files sit beside the script, or inside the one-file .exe."""
    base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, name)

# ----------------------------------------------------------------- serial ---

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
MAXDWORD = 0xFFFFFFFF
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
ATTACH_PARENT_PROCESS = -1
STD_OUTPUT_HANDLE = -11


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wt.DWORD), ("BaudRate", wt.DWORD), ("fFlags", wt.DWORD),
        ("wReserved", wt.WORD), ("XonLim", wt.WORD), ("XoffLim", wt.WORD),
        ("ByteSize", wt.BYTE), ("Parity", wt.BYTE), ("StopBits", wt.BYTE),
        ("XonChar", ctypes.c_char), ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char), ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char), ("wReserved1", wt.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wt.DWORD),
        ("ReadTotalTimeoutMultiplier", wt.DWORD),
        ("ReadTotalTimeoutConstant", wt.DWORD),
        ("WriteTotalTimeoutMultiplier", wt.DWORD),
        ("WriteTotalTimeoutConstant", wt.DWORD),
    ]


k32 = ctypes.WinDLL("kernel32", use_last_error=True)


class Serial:
    """Just enough of a serial port: open, read what is there, write a line."""

    def __init__(self, port, baud=115200):
        self.h = k32.CreateFileW(f"\\\\.\\{port}",
                                 GENERIC_READ | GENERIC_WRITE,
                                 0, None, OPEN_EXISTING, 0, None)
        if self.h == INVALID_HANDLE_VALUE:
            raise OSError(f"cannot open {port}: win32 error "
                          f"{ctypes.get_last_error()}")

        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not k32.GetCommState(self.h, ctypes.byref(dcb)):
            self.close()
            raise OSError("GetCommState failed")

        dcb.BaudRate = baud
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        # fBinary | fDtrControl=DTR_CONTROL_ENABLE. Some CDC hosts only pass
        # data once DTR is asserted, and it costs nothing to assert it.
        dcb.fFlags = 0x0001 | (1 << 4)
        if not k32.SetCommState(self.h, ctypes.byref(dcb)):
            self.close()
            raise OSError("SetCommState failed")

        # Return immediately with whatever has arrived: the UI polls.
        to = COMMTIMEOUTS(MAXDWORD, 0, 0, 0, 500)
        k32.SetCommTimeouts(self.h, ctypes.byref(to))

    def read(self, n=4096):
        buf = ctypes.create_string_buffer(n)
        got = wt.DWORD(0)
        if not k32.ReadFile(self.h, buf, n, ctypes.byref(got), None):
            return b""
        return buf.raw[:got.value]

    def write_line(self, text):
        data = (text + "\r").encode("ascii", "ignore")
        got = wt.DWORD(0)
        k32.WriteFile(self.h, data, len(data), ctypes.byref(got), None)

    def close(self):
        if getattr(self, "h", None) not in (None, INVALID_HANDLE_VALUE):
            k32.CloseHandle(self.h)
            self.h = None


def list_ports():
    """Serial ports Windows knows about, newest-looking last."""
    out = []
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                             r"HARDWARE\DEVICEMAP\SERIALCOMM")
    except OSError:
        return out
    with key:
        i = 0
        while True:
            try:
                _, value, _ = winreg.EnumValue(key, i)
            except OSError:
                break
            out.append(value)
            i += 1
    return sorted(set(out), key=lambda p: (len(p), p))


# ------------------------------------------------------------------ parse ---

# ang 25849  spd 3280  id -1  iq 3797  iqref 3794  moe 0  fault none
FIELD = re.compile(r"\b(ang|spd|id|iq|iqref|moe)\s+(-?\d+)")
FAULT = re.compile(r"\bfault\s+(.+?)\s*$")


def parse_state(line):
    vals = {k: int(v) for k, v in FIELD.findall(line)}
    if not vals:
        return None
    m = FAULT.search(line)
    vals["fault"] = m.group(1) if m else "?"
    return vals


# -------------------------------------------------------------------- chart -

class Strip(tk.Canvas):
    """A small scrolling chart. Three traces, shared autoscaled axis."""

    TRACES = (("i_q", "#2e7d32"), ("i_d", "#c62828"), ("speed", "#1565c0"))

    def __init__(self, master, points=240, **kw):
        super().__init__(master, background="#fbfbfb",
                         highlightthickness=1, highlightbackground="#d0d0d0",
                         **kw)
        self.n = points
        self.data = {name: collections.deque([0] * points, maxlen=points)
                     for name, _ in self.TRACES}
        self.bind("<Configure>", lambda e: self.redraw())

    def push(self, iq, idq, spd):
        self.data["i_q"].append(iq)
        self.data["i_d"].append(idq)
        self.data["speed"].append(spd)
        self.redraw()

    def redraw(self):
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 10 or h < 10:
            return

        hi = max(max(d) for d in self.data.values())
        lo = min(min(d) for d in self.data.values())
        span = max(abs(hi), abs(lo), 100) * 1.15
        mid = h / 2

        self.create_line(0, mid, w, mid, fill="#e0e0e0")
        self.create_text(4, 4, anchor="nw", fill="#909090",
                         font=("Consolas", 8), text=f"+{span:.0f}")
        self.create_text(4, h - 4, anchor="sw", fill="#909090",
                         font=("Consolas", 8), text=f"-{span:.0f}")

        step = w / (self.n - 1)
        for name, colour in self.TRACES:
            pts = []
            for i, v in enumerate(self.data[name]):
                pts += [i * step, mid - (v / span) * (mid - 6)]
            self.create_line(*pts, fill=colour, width=1)

        x = w - 8
        for name, colour in reversed(self.TRACES):
            t = self.create_text(x, 6, anchor="ne", fill=colour,
                                 font=("Consolas", 8), text=name)
            x -= self.bbox(t)[2] - self.bbox(t)[0] + 10


# --------------------------------------------------------------------- app --

class App(tk.Tk):
    POLL_MS = 50

    def __init__(self):
        super().__init__()
        self.title("STM32 FOC monitor")
        self.geometry("880x620")
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
                pass          # running without the icon beside us; not fatal
        self.minsize(740, 520)

        self.ser = None
        self.rx = ""
        self.telemetry = False

        self._build()
        self.after(self.POLL_MS, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._quit)

        ports = list_ports()
        if ports:
            self.port.set(ports[-1])
        # With exactly one port there is nothing to choose, and the whole
        # point of this window is to be watching. Connect straight away.
        if len(ports) == 1:
            self.after(200, self._connect)

    # ---- layout

    def _build(self):
        pad = dict(padx=6, pady=4)

        bar = ttk.Frame(self)
        bar.pack(fill="x", **pad)

        ttk.Label(bar, text="port").pack(side="left")
        self.port = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port, width=10,
                                     values=list_ports(), state="readonly")
        self.port_box.pack(side="left", padx=(4, 8))
        ttk.Button(bar, text="rescan",
                   command=lambda: self.port_box.configure(values=list_ports())
                   ).pack(side="left")
        self.connect_btn = ttk.Button(bar, text="connect",
                                      command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=8)
        self.status = ttk.Label(bar, text="not connected", foreground="#808080")
        self.status.pack(side="left", padx=8)

        # live values
        grid = ttk.LabelFrame(self, text="loop state")
        grid.pack(fill="x", **pad)
        self.vals = {}
        for col, (key, label) in enumerate([
                ("ang", "angle"), ("spd", "speed"), ("id", "i_d"),
                ("iq", "i_q"), ("iqref", "i_q ref"), ("moe", "MOE"),
                ("fault", "fault")]):
            ttk.Label(grid, text=label, foreground="#707070").grid(
                row=0, column=col, padx=10, pady=(6, 0), sticky="w")
            v = ttk.Label(grid, text="-", font=("Consolas", 15))
            v.grid(row=1, column=col, padx=10, pady=(0, 6), sticky="w")
            self.vals[key] = v

        self.chart = Strip(self, height=200)
        self.chart.pack(fill="both", expand=True, **pad)

        # commands
        cmd = ttk.LabelFrame(self, text="commands")
        cmd.pack(fill="x", **pad)

        self.entry = {}
        for col, (key, label, default) in enumerate([
                ("iq", "iq", "2000"), ("spd", "spd", "3280"),
                ("pos", "pos", "16384")]):
            box = ttk.Frame(cmd)
            box.grid(row=0, column=col, padx=8, pady=6, sticky="w")
            e = ttk.Entry(box, width=8)
            e.insert(0, default)
            e.pack(side="left")
            self.entry[key] = e
            ttk.Button(box, text=label, width=6,
                       command=lambda k=key: self._send_target(k)
                       ).pack(side="left", padx=4)

        right = ttk.Frame(cmd)
        right.grid(row=0, column=3, padx=12, sticky="e")
        for text, cmd_text in (("off", "off"), ("align", "align"),
                               ("clear", "clear"), ("tests", "t")):
            ttk.Button(right, text=text, width=7,
                       command=lambda c=cmd_text: self.send(c)
                       ).pack(side="left", padx=3)

        arm = ttk.Frame(cmd)
        arm.grid(row=0, column=4, padx=(18, 8), sticky="e")
        self.arm_btn = tk.Button(arm, text="ARM (on)", width=11,
                                 bg="#b71c1c", fg="white",
                                 activebackground="#8e0000",
                                 command=lambda: self.send("on"))
        self.arm_btn.pack(side="left")

        self.telem_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(cmd, text="telemetry", variable=self.telem_var,
                        command=lambda: self.send("m")
                        ).grid(row=0, column=5, padx=8)

        # log
        logf = ttk.LabelFrame(self, text="console")
        logf.pack(fill="both", expand=True, **pad)
        self.log = tk.Text(logf, height=9, font=("Consolas", 9),
                           background="#1e1e1e", foreground="#d4d4d4",
                           insertbackground="#d4d4d4", wrap="none")
        self.log.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=sb.set)

    # ---- connection

    def _toggle_connect(self):
        if self.ser:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port.get().strip()
        if not port:
            self._say("pick a port first")
            return
        try:
            self.ser = Serial(port)
        except OSError as e:
            self._say(str(e))
            return

        self.connect_btn.configure(text="disconnect")
        self.status.configure(text=f"{port} open", foreground="#2e7d32")
        self._say(f"--- connected to {port} ---")

        # Ask for the test summary, then turn telemetry on. The board's `m` is
        # a toggle, so the reply is what tells us which way it went.
        self.send("t")
        self.send("m")

    def _disconnect(self):
        if self.ser:
            try:
                if self.telemetry:
                    self.send("m")
                self.ser.close()
            except OSError:
                pass
        self.ser = None
        self.telemetry = False
        self.telem_var.set(False)
        self.connect_btn.configure(text="connect")
        self.status.configure(text="not connected", foreground="#808080")
        self._say("--- disconnected ---")

    def _quit(self):
        self._disconnect()
        self.destroy()

    # ---- traffic

    def send(self, text):
        if not self.ser:
            self._say("not connected")
            return
        try:
            self.ser.write_line(text)
        except OSError as e:
            self._say(f"write failed: {e}")
            self._disconnect()
            return
        self._say(f"> {text}")

    def _send_target(self, key):
        raw = self.entry[key].get().strip()
        try:
            int(raw)
        except ValueError:
            self._say(f"{key}: '{raw}' is not a number")
            return
        self.send(f"{key} {raw}")

    def _poll(self):
        if self.ser:
            try:
                chunk = self.ser.read()
            except OSError as e:
                self._say(f"read failed: {e}")
                self._disconnect()
                chunk = b""
            if chunk:
                self.rx += chunk.decode("ascii", "replace")
                while "\n" in self.rx:
                    line, self.rx = self.rx.split("\n", 1)
                    self._line(line.strip("\r").rstrip())
        self.after(self.POLL_MS, self._poll)

    def _line(self, line):
        if not line:
            return

        state = parse_state(line)
        if state:
            for k, v in state.items():
                if k in self.vals:
                    self.vals[k].configure(text=str(v))
            moe = state.get("moe", 0)
            fault = state.get("fault", "none")
            self.vals["moe"].configure(
                foreground="#b71c1c" if moe else "#2e7d32")
            self.vals["fault"].configure(
                foreground="#b71c1c" if fault != "none" else "#2e7d32")
            self.chart.push(state.get("iq", 0), state.get("id", 0),
                            state.get("spd", 0))
            # Only log state lines that were asked for, not the telemetry
            # stream - otherwise the log is useless for anything else.
            if not self.telemetry:
                self._say(line)
            return

        if line.startswith("telemetry "):
            self.telemetry = line.endswith("on")
            self.telem_var.set(self.telemetry)

        self._say(line)

    def _say(self, text):
        self.log.insert("end", text + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 400:
            self.log.delete("1.0", "100.0")
        self.log.see("end")


# ---------------------------------------------------------------- check ----

def check():
    """
    Exercise every command the window can send, and verify the board answered.

    The buttons do nothing except call send() with these exact strings, so
    this covers the whole command path short of the Tk wiring itself - which
    is the part a person clicking once will find faster than any test.

    Run it with the window closed: the port is exclusive.
    """
    import time

    ports = list_ports()
    if not ports:
        print("no serial ports found")
        return 1

    port = ports[-1]
    print(f"port {port}")
    try:
        ser = Serial(port)
    except OSError as e:
        print(f"FAIL  {e}")
        print("      (is the monitor window still open? the port is exclusive)")
        return 1

    def talk(cmd, quiet=0.25, cap=3.0):
        """
        Send a command and read until the line goes quiet.

        Waiting a fixed time was not good enough: `?` prints ten lines through
        ten separate 50 ms waits, so its reply can take half a second, and a
        fixed 0.7 s window let the tail spill into the next exchange. Two of
        every three runs then failed on whichever check happened to be next.
        """
        ser.read()                      # drop anything pending
        ser.write_line(cmd)

        out = b""
        start = time.time()
        last = start
        while time.time() - start < cap:
            chunk = ser.read()
            if chunk:
                out += chunk
                last = time.time()
            elif (time.time() - last) > quiet:
                break
            time.sleep(0.02)
        return out.decode("ascii", "replace")

    # Start from a state this run can rely on, whatever the previous one left
    # behind. A check that only passes on a freshly flashed board is not much
    # of a check: run 2 failed once because the board was still in speed mode
    # with telemetry streaming, so replies and state lines bunched together.
    talk("off", 0.5)
    if "telemetry on" in talk("m", 0.5):
        talk("m", 0.5)
    time.sleep(0.3)
    ser.read()

    checks = []

    def expect(name, cmd, predicate, quiet=0.25):
        reply = talk(cmd, quiet)
        ok = predicate(reply)
        checks.append((name, ok))
        first = next((l for l in reply.splitlines() if l.strip()), "")
        print(f"  {'ok  ' if ok else 'FAIL'}  {name:<22} {first.strip()[:52]}")

    try:
        # a state line must parse, and carry every field the window shows
        def state_ok(r):
            for line in r.splitlines():
                st = parse_state(line)
                if st and all(k in st for k in
                              ("ang", "spd", "id", "iq", "iqref", "moe", "fault")):
                    return True
            return False

        expect("state parses", "s", state_ok)
        expect("test summary", "t", lambda r: "ok_flags" in r)
        expect("help", "?", lambda r: "commands:" in r)

        expect("current mode", "iq 2000", lambda r: "current mode" in r)
        expect("current reached",  "s",
               lambda r: any((parse_state(l) or {}).get("iq", 0) > 1500
                             for l in r.splitlines()))

        expect("idle", "off", lambda r: "idle" in r)

        expect("speed mode", "spd 3280", lambda r: "speed mode" in r)
        time.sleep(0.4)                 # the speed loop settles in ~31 ms
        expect("speed reached", "s",
               lambda r: any(abs((parse_state(l) or {}).get("spd", 0) - 3280) < 200
                             for l in r.splitlines()))

        expect("position mode", "pos 8192", lambda r: "position mode" in r)
        expect("idle again", "off", lambda r: "idle" in r)

        # arming: allowed from idle, refused once a mode is selected
        expect("arm from idle", "on", lambda r: "armed" in r)
        expect("disarm", "off", lambda r: "idle" in r)
        talk("spd 1000")
        expect("arm refused", "on", lambda r: "refused" in r)
        talk("off")

        expect("clear fault", "clear", lambda r: "cleared" in r)

        # telemetry toggles both ways
        expect("telemetry on", "m", lambda r: "telemetry on" in r)
        expect("telemetry off", "m", lambda r: "telemetry off" in r)

        # align goes last: it blocks the console for 300 ms, so commands and
        # replies bunch up behind it and a plain request/response exchange
        # mis-attributes them. Anything after it has to settle first.
        expect("align", "align", lambda r: "align" in r.lower(), quiet=0.8)
        talk("off")
    finally:
        try:
            ser.write_line("off")
            time.sleep(0.3)
        finally:
            ser.close()

    bad = [n for n, ok in checks if not ok]
    print()
    print(f"{len(checks) - len(bad)} of {len(checks)} checks passed")
    if bad:
        print("failed: " + ", ".join(bad))
    return 1 if bad else 0


def attach_console():
    """Give --check somewhere to print when built as a windowed .exe.

    PyInstaller's --windowed build has no console, so sys.stdout is None and
    print() raises.

    Order matters. If whoever launched us handed down a pipe or a file - which
    is what happens under `aieos run`, or any `exe --check > out.txt` - we must
    write *there*, or the output lands on a console nobody is reading and the
    captured log comes back empty. Only with no usable handle do we fall back
    to attaching a console.
    """
    if sys.stdout is not None:
        return

    import msvcrt

    kernel = ctypes.windll.kernel32
    kernel.GetStdHandle.restype = ctypes.c_void_p
    h = kernel.GetStdHandle(STD_OUTPUT_HANDLE)
    if h and h != INVALID_HANDLE_VALUE and kernel.GetFileType(ctypes.c_void_p(h)):
        fd = msvcrt.open_osfhandle(h, os.O_WRONLY)
        sys.stdout = os.fdopen(fd, "w", buffering=1)
    else:
        if not kernel.AttachConsole(ATTACH_PARENT_PROCESS):
            kernel.AllocConsole()
        sys.stdout = open("CONOUT$", "w", buffering=1)
    sys.stderr = sys.stdout


if __name__ == "__main__":
    # Answer the flags a tool might probe with before anything can open a
    # window. `aieos run` asks every command it captures for --version, and a
    # GUI that opens a window instead of answering hangs it forever.
    if "--version" in sys.argv or "-V" in sys.argv:
        attach_console()
        print(f"foc-monitor {VERSION}")
        sys.exit(0)
    if "--help" in sys.argv or "-h" in sys.argv:
        attach_console()
        print(__doc__.strip())
        sys.exit(0)
    if "--check" in sys.argv:
        attach_console()
        sys.exit(check())
    # Without this Windows groups us under python.exe and shows its icon in
    # the taskbar instead of ours.
    try:
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_ID)
    except Exception:
        pass
    App().mainloop()

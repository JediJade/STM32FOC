"""
The link to the STM32FOC firmware, over its USB CDC console.

The firmware (src/console.c) speaks lines of text. This module sends the
same commands a person would type and reads the same replies:

    s            one state line
                   ang 47205  spd 1000  id 1  iq 1075  iqref 1077  moe 0  fault none
    t            four lines of self-test summary
    spd <n>      speed mode, n in angle units per ms
    iq <n>       current mode       pos <n>   position mode
    off          idle, outputs off  on        arm the outputs (refused unless idle)
    clear        clear a latched fault        align     see below
    m            toggle the firmware's own 10 Hz telemetry

A background thread asks for a state line 100 times a second. Measured on
the board, a reply comes back in about 2 ms and the console keeps up with
nearly 500 requests a second, so 100 Hz is well inside what it can do - and
ten times finer than the 10 Hz telemetry stream, which is left switched off.

Units are the firmware's own, converted only where the conversion is exact:

    speed    angle units per ms. One electrical turn is 65536 units and the
             firmware assumes one pole pair, so rpm = units * 60000 / 65536.
    current  Q15 counts. There is no shunt gain and no zero-current offset in
             the firmware yet, so there is no honest conversion to amps and
             none is attempted.

There are two consoles in the field, and `ver` tells them apart:

    1   the console as first written. It has no `ver` and answers
        "? for help". Its gains are compile-time constants, so there is
        nothing to set and nothing to tune from here.
    2   adds, on top of everything above:
          ver              fw 2 cap 256
          get              p <name> <value> for each parameter, then get end
          set <name> <n>   p <name> <value>, the value as actually stored
          id <n>           d-axis current target
          vd <n>           voltage mode: v_d = n, no controller
          cap <ch> <div>   arm the recorder; it starts with the next target
          dump             the record: a header, lines of d <8 values>, cap end
          x                x vd N  vq N  loop N  outer N  cap N  n N

    3   adds the vehicle side - see ../README.md for each:
          v                throttle, brake, sensor, weakening, in one line
          th <n>  brk <n>  drv <n>     throttle and brake in place of the pins
          hallcal  hall    learn, and list, where the Hall edges are
          olf              the one-line dashboard frame last sent
          can  /  can on   CAN state  /  change over to CAN: the port goes
                           away, and comes back when the board returns to USB
        and twenty-four more parameters, and four more recorder channels.

With console 2 and later the loop mode is read from `x` instead of guessed,
and `spd` is live in every mode, so the two workarounds below apply to
console 1 only.

One thing the firmware reports cannot be taken at face value. `spd` is
measured inside the speed loop (outer.c), which only runs in speed and
position mode. In idle and in current mode the field keeps its last value:
after `off` the board went on reporting -2640 while the motor model had
already stopped. So outside those two modes the speed shown here is worked
out from how far `ang` moved between two state lines instead. At standstill
that is exactly zero; while moving it carries a few percent of timing noise.
`iqref` is stale in idle in the same way - it kept -2457 from the last speed
command while the loop was applying zero volts - so in idle it is shown as
zero, which is what the loop is actually asking for.

`align` is never sent from here. In this firmware it blocks for 300 ms in
delay_ms(), which does not refresh the 100 ms watchdog, so the board resets:
observed as the reply stopping after "aligning...", the port dropping, and
the next `t` reading "reset watchdog, ... fault watchdog reset".

Stdlib only. The serial port is driven through Win32 with ctypes, as
foc-monitor.py does, because pyserial is not installed on this machine.
"""

import collections
import ctypes
import ctypes.wintypes as wt
import queue
import re
import threading
import time
import winreg

import fwsim

UNITS_PER_RPM = 65536.0 / 60000.0
SPEED_MAX_RPM = 6000            # OUTER_SPEED_MAX is 6560 units/ms = 6006 rpm
POLL_HZ = 100.0
HISTORY_SECONDS = 20.0

# Listed as a port, last and labelled for what it is: fwsim.py standing in
# for a board, so that everything a board does can be tried without one.
EMULATOR_PORT = "emulator"
emulator_enabled = True

# name -> (default the firmware is built with, lowest, highest), for console
# 3. A board with console 2 has the first nine and refuses the rest.
PARAM_RANGE = {name: (default, lo, hi) for name, default, lo, hi in fwsim.PARAMS}

VEHICLE_STATES = ("off", "waiting for the throttle to be released", "running",
                  "braking", "throttle fault")
SENSOR_NAMES = ("encoder", "hall sensors", "observer")
SENSOR_PHASES = ("", "hall sensors", "forced angle, starting",
                 "following the back-emf", "calibrating")
CAL_RESULTS = {0: "not calibrated", 1: "calibrated",
               2: "failed: the two directions saw different states",
               3: "failed: not six states - are three sensors connected?",
               4: "failed: the states do not follow one another in a ring",
               9: "stopped by a command"}

# bits of selftest.ok_flags, in the order selftest.h defines them
FLAGS = (
    (0x01, "dead-time"),
    (0x02, "adc trigger"),
    (0x04, "fixed-point math"),
    (0x08, "current loop"),
    (0x10, "alignment"),
    (0x20, "speed and position"),
    (0x40, "break input"),
    (0x80, "current sensing"),
)

CHANNELS = ("rpm", "ref", "iq", "iqref", "id", "thr", "idfw")


# ----------------------------------------------------------------- serial ---

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
MAXDWORD = 0xFFFFFFFF
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value


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
k32.CreateFileW.restype = wt.HANDLE
k32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, ctypes.c_void_p,
                            wt.DWORD, wt.DWORD, wt.HANDLE]
k32.ReadFile.argtypes = [wt.HANDLE, ctypes.c_void_p, wt.DWORD,
                         ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
k32.WriteFile.argtypes = [wt.HANDLE, ctypes.c_void_p, wt.DWORD,
                          ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
k32.GetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
k32.SetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
k32.SetCommTimeouts.argtypes = [wt.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
k32.CloseHandle.argtypes = [wt.HANDLE]


class Serial:
    """Open, read what has arrived, write a line. Same settings as the
    port class in foc-monitor.py; unlike it, a failed read or write raises,
    because that is how an unplugged board shows itself."""

    def __init__(self, port, baud=115200):
        self.h = k32.CreateFileW(f"\\\\.\\{port}", GENERIC_READ | GENERIC_WRITE,
                                 0, None, OPEN_EXISTING, 0, None)
        if self.h is None or self.h == INVALID_HANDLE_VALUE:
            err = ctypes.get_last_error()
            self.h = None
            why = {2: "the port does not exist",
                   5: "another program has it open"}.get(err, f"win32 error {err}")
            raise OSError(f"cannot open {port}: {why}")

        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not k32.GetCommState(self.h, ctypes.byref(dcb)):
            self.close()
            raise OSError(f"{port} is not a serial port")
        dcb.BaudRate = baud
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        # fBinary | DTR_CONTROL_ENABLE: some CDC hosts pass data only once
        # DTR is asserted, and it costs nothing to assert it.
        dcb.fFlags = 0x0001 | (1 << 4)
        if not k32.SetCommState(self.h, ctypes.byref(dcb)):
            self.close()
            raise OSError(f"{port} refused the port settings")
        # reads return at once with whatever has arrived
        k32.SetCommTimeouts(self.h, ctypes.byref(
            COMMTIMEOUTS(MAXDWORD, 0, 0, 0, 500)))

    def read(self, n=4096):
        buf = ctypes.create_string_buffer(n)
        got = wt.DWORD(0)
        if not k32.ReadFile(self.h, buf, n, ctypes.byref(got), None):
            raise OSError(f"read failed, win32 error {ctypes.get_last_error()}")
        return buf.raw[:got.value]

    def write_line(self, text):
        data = (text + "\r").encode("ascii", "ignore")
        got = wt.DWORD(0)
        if (not k32.WriteFile(self.h, data, len(data), ctypes.byref(got), None)
                or got.value != len(data)):
            raise OSError(f"write failed, win32 error {ctypes.get_last_error()}")

    def close(self):
        if self.h is not None:
            k32.CloseHandle(self.h)
            self.h = None


def list_ports():
    """(port, kind) for every serial port Windows knows about."""
    out = {}
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                             r"HARDWARE\DEVICEMAP\SERIALCOMM")
    except OSError:
        return []
    with key:
        i = 0
        while True:
            try:
                device, port, _ = winreg.EnumValue(key, i)
            except OSError:
                break
            low = device.lower()
            if "usbser" in low:
                kind = "usb serial device"
            elif "bth" in low:
                kind = "bluetooth"
            elif "\\serial" in low:
                kind = "built-in port"
            else:
                kind = device.rsplit("\\", 1)[-1].lower()
            out[port] = kind
            i += 1
    ports = sorted(out.items(), key=lambda p: (len(p[0]), p[0]))
    if emulator_enabled:
        ports.append((EMULATOR_PORT, "firmware model, not a board"))
    return ports


# ------------------------------------------------------------------ parse ---

FIELD = re.compile(r"\b(ang|spd|id|iq|iqref|moe)\s+(-?\d+)")
FAULT = re.compile(r"\bfault\s+(.+?)\s*$")
STATE_KEYS = ("ang", "spd", "id", "iq", "iqref", "moe")


def parse_state(line):
    """A state line as a dict, or None if the line is something else."""
    vals = {k: int(v) for k, v in FIELD.findall(line)}
    if any(k not in vals for k in STATE_KEYS):
        return None
    m = FAULT.search(line)
    vals["fault"] = m.group(1) if m else "?"
    return vals


SUMMARY = (
    (re.compile(r"^ok_flags (0x[0-9a-f]+) of 0xff"),
     lambda m: {"flags": int(m.group(1), 16)}),
    (re.compile(r"^deadtime (-?\d+) cx100 ticks, adc (-?\d+) Hz"),
     lambda m: {"deadtime_x100": int(m.group(1)), "adc_hz": int(m.group(2))}),
    (re.compile(r"^reset (.+), wdg (\d+), fault (.+)$"),
     lambda m: {"reset": m.group(1), "watchdog": int(m.group(2))}),
    (re.compile(r"^align err (-?\d+), loop settle (-?\d+), ctrl (-?\d+) cyc"),
     lambda m: {"align_err": int(m.group(1)), "settle": int(m.group(2)),
                "ctrl_cycles": int(m.group(3))}),
)

EXTRA = re.compile(r"^x vd (-?\d+)\s+vq (-?\d+)\s+loop (\d+)\s+outer (\d+)"
                   r"\s+cap (\d+)\s+n (\d+)")
VEHICLE = re.compile(r"^v ((?:\w+ -?\d+ ?)+)$")
HALL = re.compile(r"^hall ok (\d) err (\d+) e((?: -?\d+){8})$")
CAN = re.compile(r"^can test (\d+) mode (\d+) id (\d+) br (\d+) rx (\d+) tx (\d+) drop (\d+)")
OLF = re.compile(r"^olf((?: [0-9a-f]{2}){12})$")
VERSION = re.compile(r"^fw (\d+) cap (\d+)")
PARAM = re.compile(r"^p (\w+) (-?\d+)$")
DUMP = re.compile(r"^cap (\w+) div (\d+) n (\d+) state (\d+)$")

MODE_REPLY = {
    "speed mode": "speed",
    "current mode": "current",
    "position mode": "position",
    "idle, outputs off": "idle",
}


# ------------------------------------------------------------------- link ---

class BoardLink:
    """One open connection. Create it to connect; close() to disconnect."""

    def __init__(self, port):
        self.port = port
        if port == EMULATOR_PORT:
            self.ser = fwsim.open_emulator()    # raises OSError while it is away
        else:
            self.ser = Serial(port)         # raises OSError with the reason
        self.fw = None                      # console version, once known
        self.params = {}                    # name -> value, console 2
        self.params_serial = 0              # bumps when `get` has finished
        self.vd = self.vq = 0               # applied voltage, console 2
        self.cap_state = None               # recorder state from `x`
        self.cap_data = []                  # the last record read out
        self.cap_div = 1
        self.cap_serial = 0                 # bumps when a dump has arrived
        self.x_every = 5                    # one `x` per this many `s`
        self.veh = {}                       # the last `v` line, console 3
        self.hall = None                    # (ok, errors, [8 edges]) from `hall`
        self.hall_serial = 0
        self.can = None                     # the last `can` line, as a dict
        self.ol_frame = None                # the last `olf` line, 12 bytes
        self.away = False                   # `can on` was sent: expect to lose the port
        self._dump = None
        self._polls = 0
        self.unanswered = 0                 # `s` requests that got no reply
        self.telemetry = None               # the firmware's own 10 Hz stream
        self._x_sent = 0                    # `x` requests written so far
        self._x_seen = 0                    # `x` replies read so far
        self._x_fresh = 0                   # replies up to here predate a `cap`
        self.error = None                   # set if the link dies
        self.mode = None                    # idle / speed / current / position / running
        self.started = False                # this session commanded motion
        self.target_rpm = 0.0
        self.state = None                   # the latest parsed state line
        self.summary = {}                   # parsed `t` reply
        self.notice = ""                    # last refusal or note from the board
        self.latency_ms = 0.0
        self.log = collections.deque(maxlen=200)
        self.log_serial = 0
        self._cmds = queue.Queue()
        self._lock = threading.Lock()
        self._t = []
        self._ch = {c: [] for c in CHANNELS}
        self._last_sample = time.perf_counter()
        self._prev = None                   # (time, ang) of the last state line
        self._est = 0.0                     # speed from the angle, units per ms
        self._seen = []                     # first few angle steps, to guess the mode
        self.units = 0.0                    # the speed in use, units per ms
        self.iqref = 0                      # the reference in use, counts
        self._alive = True
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="boardlink")
        self.send("t")
        self.send("m")      # a toggle: the reply says which way it went
        self.send("ver")    # console 1 answers "? for help"
        self._thread.start()

    # ---- commands

    def send(self, text):
        text = text.strip()
        if not text:
            return
        word = text.split()[0]
        if word in ("spd", "iq", "pos", "vd"):
            self.started = True         # so close() knows to stop it again
            if word == "spd":
                # whoever sent it - the slider, auto tune, the console box -
                # this is the reference the speed traces are drawn against
                try:
                    self.target_rpm = int(text.split()[1]) / UNITS_PER_RPM
                except (IndexError, ValueError):
                    pass
        elif word == "cap":
            self.cap_state = None       # stale until the next `x` says
        elif word == "drv" and text.split()[-1] != "0":
            self.started = True
        elif text == "can on":
            self.away = True
        self._cmds.put(text)

    def run_speed(self, rpm):
        rpm = max(-SPEED_MAX_RPM, min(SPEED_MAX_RPM, rpm))
        units = int(round(rpm * UNITS_PER_RPM))
        self.target_rpm = units / UNITS_PER_RPM
        self.started = True
        self.send(f"spd {units}")

    def stop(self):
        self.send("off")

    # ---- what the front end reads

    @property
    def connected(self):
        return self.error is None and self._alive

    @property
    def silent_for(self):
        return time.perf_counter() - self._last_sample

    @property
    def running(self):
        return self.mode not in (None, "idle")

    @property
    def rpm(self):
        return self.units / UNITS_PER_RPM

    @property
    def speed_measured(self):
        """True while the firmware's own `spd` is live, False while the
        speed shown is worked out from the angle."""
        return ((self.fw or 0) >= 2 or
                self.mode in ("speed", "position", "running"))

    def set_param(self, name, value):
        """Ask the board to store a parameter. The reply updates params."""
        _default, lo, hi = PARAM_RANGE[name]
        self.send(f"set {name} {int(max(lo, min(hi, round(value))))}")

    @property
    def fault(self):
        if not self.state or self.state["fault"] == "none":
            return None
        return self.state["fault"]

    @property
    def rate(self):
        """State lines received in the last second."""
        with self._lock:
            t = self._t
            if not t:
                return 0.0
            edge = t[-1] - 1.0
            n = 0
            for v in reversed(t):
                if v < edge:
                    break
                n += 1
        return float(n) if time.perf_counter() - self._last_sample < 1.0 else 0.0

    def window(self, channel, seconds, max_points=1000):
        """The last `seconds` of one channel, oldest first."""
        if channel not in self._ch:
            return []
        with self._lock:
            t = self._t
            if not t:
                return []
            edge = t[-1] - seconds
            n = 0
            for v in reversed(t):
                if v < edge:
                    break
                n += 1
            seg = self._ch[channel][-n:]
        step = max(1, len(seg) // max_points)
        return seg[::step] if step > 1 else seg

    # ---- the thread

    def _say(self, text):
        self.log.append(text)
        self.log_serial += 1

    def _track_angle(self, st, now):
        """Speed from two consecutive angles. The angle wraps every turn, so
        of all the speeds that fit the step, take the one nearest the last
        estimate - which follows a motor well past one turn per sample."""
        if self._prev is None:
            self._prev = (now, st["ang"])
            return None
        t0, a0 = self._prev
        self._prev = (now, st["ang"])
        dt_ms = (now - t0) * 1000.0
        if dt_ms <= 0.5:
            return None
        step = (st["ang"] - a0) & 0xFFFF
        if step == 0:
            self._est = 0.0
            return 0
        turns = round((self._est * dt_ms - step) / 65536.0)
        self._est = (step + turns * 65536.0) / dt_ms
        return step

    def _line(self, line, now):
        st = parse_state(line)
        if st:
            step = self._track_angle(st, now)
            if self.mode is None and step is not None and (self.fw or 0) < 2:
                # The state line does not say which mode the loop is in, and
                # `spd` may be stale. Watch the angle for a few lines: if it
                # is barely moving and no torque is being asked for, the loop
                # is idle, whatever `spd` claims. "Barely" and not "not":
                # the firmware's motor model never quite stops in idle - its
                # integer friction term rounds to nothing below about 80
                # units per ms, and it was seen creeping at 18 rpm.
                self._seen.append(step)
                if len(self._seen) >= 4:
                    if abs(self._est) < 120.0 and abs(st["iq"]) < 200:
                        self.mode = "idle"
                    elif abs(st["spd"] - self._est) <= max(60.0, 0.3 * abs(self._est)):
                        self.mode = "running"       # speed or position mode
                    else:
                        self.mode = "current"
            if self.speed_measured:
                self.units = float(st["spd"])
                self._est = float(st["spd"])        # keep the tracker seeded
            else:
                # light smoothing: the estimate inherits host timing jitter
                self.units = (0.0 if self._est == 0.0 else
                              self.units + 0.3 * (self._est - self.units))
            rpm = self.units / UNITS_PER_RPM
            self.iqref = 0 if self.mode == "idle" else st["iqref"]
            with self._lock:
                self._t.append(now)
                ch = self._ch
                ch["rpm"].append(rpm)
                ch["ref"].append(self.target_rpm if self.mode == "speed" else rpm)
                ch["iq"].append(st["iq"])
                ch["iqref"].append(self.iqref)
                ch["id"].append(st["id"])
                ch["thr"].append(self.veh.get("thr", 0))
                ch["idfw"].append(self.veh.get("idfw", 0))
                if len(self._t) > int(HISTORY_SECONDS * POLL_HZ * 1.5):
                    cut = len(self._t) - int(HISTORY_SECONDS * POLL_HZ * 1.2)
                    del self._t[:cut]
                    for c in ch.values():
                        del c[:cut]
            self.state = st
            self._last_sample = now
            return True

        m = EXTRA.match(line)
        if m:
            vd, vq, loop, outer, cap, _n = (int(v) for v in m.groups())
            self.vd, self.vq = vd, vq
            # An `x` asked for before a `cap` went out can still be on its
            # way back, and it describes the previous record. Believing it
            # read out a recorder that had not started yet.
            self._x_seen += 1
            if self._x_seen > self._x_fresh:
                self.cap_state = cap
            self.mode = ("speed" if outer == 1 else "position" if outer == 2
                         else ("idle", "align", "current", "voltage")[min(loop, 3)])
            return False
        m = VEHICLE.match(line)
        if m:
            parts = m.group(1).split()
            self.veh = {k: int(v) for k, v in zip(parts[0::2], parts[1::2])}
            return False
        if self._dump is not None:
            if line.startswith("d"):
                self._dump.extend(int(v) for v in line[1:].split())
                return False
            if line == "cap end":
                self.cap_data = self._dump
                self._dump = None
                self.cap_serial += 1
                return False
        m = DUMP.match(line)
        if m:
            self._dump = []
            self.cap_div = int(m.group(2))

        self._say(line)
        m = PARAM.match(line)
        if m:
            self.params[m.group(1)] = int(m.group(2))
            return False
        if line == "get end":
            self.params_serial += 1
            return False
        m = VERSION.match(line)
        if m:
            self.fw = int(m.group(1))
            self.send("get")
            if self.fw >= 3:
                self.send("hall")
                self.send("can")
            return False
        m = HALL.match(line)
        if m:
            self.hall = (int(m.group(1)), int(m.group(2)),
                         [int(v) for v in m.group(3).split()])
            self.hall_serial += 1
            return False
        m = CAN.match(line)
        if m:
            self.can = dict(zip(("test", "mode", "id", "br", "rx", "tx", "drop"),
                                (int(v) for v in m.groups())))
            return False
        m = OLF.match(line)
        if m:
            self.ol_frame = [int(v, 16) for v in m.group(1).split()]
            return False
        if line == "? for help" and self.fw is None:
            self.fw = 1
            return False
        if line in MODE_REPLY:
            if (self.fw or 0) < 2:      # console 2 says so itself, in `x`
                self.mode = MODE_REPLY[line]
        elif line == "telemetry on":
            self.telemetry = True
            self.send("m")              # we poll instead; switch it back off
        elif line == "telemetry off":
            self.telemetry = False
        elif (line.startswith("refused:") or line.startswith("armed")
              or line.startswith("? stop") or line.startswith("drive o")
              or line.startswith("hall calibration")):
            self.notice = line
        elif line == "fault cleared":
            self.notice = line
        else:
            for pattern, fields in SUMMARY:
                m = pattern.match(line)
                if m:
                    self.summary.update(fields(m))
                    break
        return False

    def _run(self):
        rx = ""
        period = 1.0 / POLL_HZ
        next_poll = time.perf_counter()
        asked = None
        try:
            while self._alive:
                while True:
                    try:
                        cmd = self._cmds.get_nowait()
                    except queue.Empty:
                        break
                    if cmd.startswith("cap "):
                        self.cap_state = None
                        self._x_fresh = self._x_sent
                    self.ser.write_line(cmd)
                    self._say("> " + cmd)

                now = time.perf_counter()
                if asked is None and now >= next_poll:
                    self.ser.write_line("s")
                    self._polls += 1
                    asked = now
                    next_poll = max(next_poll + period, now)
                elif asked is not None and now - asked > 0.1:
                    # No answer in fifty times the usual wait: ask again,
                    # and let silent_for tell the front end. Asking again
                    # is also what frees a reply the host is sitting on -
                    # a board with the older USB driver sends some state
                    # lines as one full 64-byte packet, which the host
                    # holds until the next packet arrives.
                    self.unanswered += 1
                    asked = None

                chunk = self.ser.read()
                if chunk:
                    rx += chunk.decode("ascii", "replace")
                    while "\n" in rx:
                        line, rx = rx.split("\n", 1)
                        line = line.strip("\r").rstrip()
                        if not line:
                            continue
                        now = time.perf_counter()
                        # A reply is stamped with when it was asked for:
                        # the asking is on a steady 10 ms beat, the arrival
                        # is not, and the angle-based speed divides by this.
                        if (self._line(line, asked if asked is not None else now)
                                and asked is not None):
                            lat = (now - asked) * 1000.0
                            self.latency_ms += (lat - self.latency_ms) * 0.05
                            asked = None
                            # `x` goes out only now, with the `s` before it answered:
                            # one request in flight at a time. Two sent back to back
                            # put a packet in each direction on the wire at once,
                            # and that is what hung the firmware's USB driver (a
                            # lost CTR_RX, since fixed there - but a board with the
                            # older driver should not be hung from here).
                            fw = self.fw or 0
                            if fw >= 2 and self._polls % self.x_every == 0:
                                self.ser.write_line("x")
                                self._x_sent += 1
                            elif fw >= 3 and self._polls % self.x_every == 2:
                                self.ser.write_line("v")
                else:
                    time.sleep(0.001)
        except OSError as err:
            self.error = str(err)

    def close(self):
        """Disconnect. If this session started the motor model moving, put
        the loop back to idle first; otherwise leave the board as it was."""
        if (self._alive and self.error is None and self.started
                and (self.running or self.veh.get("veh") in (1, 2, 3))):
            self.send("off")
            end = time.perf_counter() + 0.4
            while self.mode != "idle" and time.perf_counter() < end:
                time.sleep(0.01)
        self._alive = False
        self._thread.join(timeout=1.0)
        self.ser.close()

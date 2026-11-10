# Host software

Everything in this project that runs on the PC rather than on the board.
`DEVELOPMENT.md` is the firmware side; this is the other half.

| File | What it is |
| --- | --- |
| `foc-monitor.py` | the window, and `--check`, the headless command test |
| `read-selftest.py` | reads the self-test struct out of RAM over SWD |
| `build-exe.cmd` | packs the monitor into one `.exe` |
| `unitymbed.ico` | the UnityMbed logo, used by the window and the `.exe` |

Standard library only. No pyserial, no matplotlib, nothing to install - the
one exception is PyInstaller, and only if you want the `.exe`.

## Two ways to reach the board, and they do not collide

```
                 USB CDC, COM13                      SWD, ST-Link
  foc-monitor  ------------------>  STM32F103  <------------------  read-selftest
                 exclusive                            works while the
                 one owner at a time                  CPU keeps running
```

This matters more than it sounds. The serial port has exactly one owner: while
the window is open, nothing else can open COM13, and `--check` will refuse with
`GetCommState failed`. SWD is a separate path into the same chip and does not
care who holds the port - it reads RAM without halting the core.

That is how the button clicks were verified while the window held the port:
OpenOCD read `loop_mode`, `speed_ref`, `speed_meas`, `i_q` and `theta` from the
side, and the values showed the speed loop regulating exactly what the window
had sent.

## The monitor, layer by layer

### Serial, through Win32 directly

`pyserial` is not installed on this machine, and the port is simple enough that
`CreateFileW` + `SetCommState` is less work than adding a dependency. The
`Serial` class is about forty lines.

Two settings carry the whole design:

```python
dcb.fFlags = 0x0001 | (1 << 4)      # fBinary | fDtrControl = DTR_CONTROL_ENABLE
to = COMMTIMEOUTS(MAXDWORD, 0, 0, 0, 500)
```

  * **DTR asserted.** Some CDC hosts pass no data until DTR is raised. It costs
    nothing, so it is always on.
  * **`ReadIntervalTimeout = MAXDWORD`, totals zero.** This is the documented
    way to say *return immediately with whatever has arrived*. `read()` never
    blocks, so the Tk loop never freezes waiting for a board that has nothing
    to say - which is exactly what a board sitting in `idle` does.

Ports come from the registry, not from probing:

```
HKLM\HARDWARE\DEVICEMAP\SERIALCOMM
```

sorted by `(len(name), name)` so `COM13` sorts after `COM3`, and the newest
device tends to land last. With exactly one port the window connects to it
200 ms after start - there is nothing to choose, and the point of the window is
to already be watching.

### Lines, not bytes

Serial reads arrive in arbitrary chunks, and a state line can be split across
two of them. `_poll` runs every 50 ms, appends whatever arrived to `self.rx`
and only hands out whole lines:

```python
self.rx += chunk.decode("ascii", "replace")
while "\n" in self.rx:
    line, self.rx = self.rx.split("\n", 1)
    self._line(line.strip("\r").rstrip())
```

Telemetry streams at 10 Hz; polling at 20 Hz means a line is picked up within
one poll of arriving, with slack to spare.

### Parsing a state line

```
ang 25849  spd 3280  id -1  iq 3797  iqref 3794  moe 0  fault none
```

```python
FIELD = re.compile(r"\b(ang|spd|id|iq|iqref|moe)\s+(-?\d+)")
FAULT = re.compile(r"\bfault\s+(.+?)\s*$")
```

Field order is not assumed, missing fields are not fatal, and anything that
matches nothing returns `None` and is treated as ordinary console text. The
fault name is taken to the end of the line because it is a word, not a number.

### The chart

240 points, three traces, one shared axis. At 10 Hz that is 24 seconds of
history. The axis autoscales to the largest absolute value in the window,
times 1.15, with a floor of 100 so an idle board does not show amplified noise
as a full-scale waveform.

i_q green, i_d red, speed blue, redrawn on every sample and on `<Configure>`
so a resize is not left with a stale picture.

### Connect, and what the window asks for

On connect it sends `t` then `m`: print the self-test summary, then toggle
telemetry. The board's `m` is a *toggle*, not a switch, so the window never
assumes which way it went - it watches for the reply:

```python
if line.startswith("telemetry "):
    self.telemetry = line.endswith("on")
    self.telem_var.set(self.telemetry)
```

The checkbox follows the board, not the other way round.

While telemetry is streaming, state lines update the readouts and the chart but
are **not** written to the console log - at 10 Hz they would bury everything
else. A state line that arrives because someone pressed `s` is logged, because
then it is an answer to a question.

### The buttons do nothing clever

Every button calls `send()` with a fixed string. `iq`, `spd` and `pos` read
their entry box, check it parses as an integer, and send `"<cmd> <value>"`.
That is the whole command path - which is what makes `--check` meaningful: it
sends the same strings.

## The console protocol

One command per line, terminated with `\r`. Every reply is one or more lines.

| Command | Reply | Effect |
| --- | --- | --- |
| `?` or `help` | `commands:` + ten lines | help |
| `s` | one state line | print state once |
| `t` | `ok_flags 0xff of 0xff`, plus timings | self-test summary |
| `m` | `telemetry on` / `telemetry off` | toggle the 10 Hz stream |
| `iq <v>` | `current mode` | current loop, target clamped to ±16384 |
| `spd <v>` | `speed mode` | speed loop, clamped to ±`OUTER_SPEED_MAX` |
| `pos <v>` | `position mode` | position loop, target is *relative* to now |
| `align` | `aligning...` then the test summary | re-run rotor alignment, blocks ~300 ms |
| `off` | `idle, outputs off` | idle, disarm |
| `on` | `armed, outputs enabled` or `refused: <fault>` | arm - only from idle |
| `clear` | `fault cleared` | clear the latched fault |
| `hang` | `hanging: ...` | deliberate lockup, the watchdog resets the board |

Note `pos` is relative: the firmware adds the value to the current angle. The
window's default of 16384 is therefore "a quarter turn from wherever you are",
not "go to position 16384".

## `--check`: the same path, no window

```bash
python foc-monitor.py --check     # or dist\foc-monitor.exe --check
```

Seventeen checks: state parses with every field the window displays, the test
summary, help, each of the three modes entered *and reached*, idle, arm from
idle, disarm, arm refused once a mode is selected, clear, telemetry both ways,
and align.

Run it with the window closed. The port is exclusive, and the failure message
says so.

Two design decisions, both of them the result of flaky runs:

  * **Read until the line goes quiet, not for a fixed time.** `?` prints ten
    lines through ten separate 50 ms waits, so its reply can take half a
    second. A fixed 0.7 s window let the tail spill into the next exchange, and
    two of every three runs failed on whichever check happened to be next.
    `talk()` now reads until 250 ms pass with nothing arriving, capped at 3 s.
  * **Start from a known state.** A check that only passes on a freshly flashed
    board is not much of a check. The preamble sends `off`, and turns telemetry
    off if it was on, so a previous run leaving the board in speed mode with
    the stream running cannot corrupt the next one.

`align` goes last because it blocks the console for 300 ms, so anything behind
it bunches up and a plain request/response exchange mis-attributes the replies.

Result: 17 of 17, three runs in a row, and again from the built `.exe`.

## Building the `.exe`

```bat
build-exe.cmd
dist\foc-monitor.exe              the window
dist\foc-monitor.exe --check      the 17 checks
dist\foc-monitor.exe --version    foc-monitor 1.0, no window
```

One file, 9.9 MB, runs on a machine with no Python. Five details that are easy
to get wrong, every one of them found by something breaking:

  * **`--windowed` means no stdout.** `sys.stdout` is `None` and `--check`
    would die on its first `print()`. `attach_console()` fixes it, but the
    order inside it matters: if the launcher handed down a pipe or a file -
    which is what `aieos run`, or `exe --check > out.txt`, does - the output
    must go *there*. Attaching a console first instead produces a capture that
    is empty while the run looks successful. Only with no usable std handle
    does it attach or allocate a console and write to `CONOUT$`.
  * **`--version` and `--help` are answered before any window opens.** A GUI
    that opens a window when probed for its version hangs whatever probed it.
    `aieos run` probes every command it captures, and the first attempt to
    record a `--check` run sat there until it was killed, with
    `dist\foc-monitor.exe --version` showing a window nobody asked for.
  * **The icon is the real one.** All six sizes (16, 24, 32, 48, 64, 256) were
    lifted out of `unitymbed-desktop.exe`'s own `RT_GROUP_ICON` / `RT_ICON`
    resources. Extracting it back out of the built `.exe` gives a byte-identical
    file. `SetCurrentProcessExplicitAppUserModelID` stops Windows from grouping
    the window under python.exe and showing python's icon in the taskbar.
  * **`--add-data` resolves against the spec file,** which `--specpath` moves.
    The paths in `build-exe.cmd` are absolute (`%~dp0...`) for that reason.
  * **PyInstaller's scratch goes in `dist\work`.** `build\` belongs to the
    firmware; letting PyInstaller use its default would mix object files and
    bootloader scratch in one directory.

At runtime the icon is found through `resource_path()`, which prefers
`sys._MEIPASS` (the one-file extraction directory) and falls back to the
script's own directory, so the same code works run either way.

## `read-selftest.py`

```bash
python read-selftest.py --wait 5
```

Prints the self-test struct with field names. Nothing about the layout is
written down in it: the field list is parsed out of `inc/selftest.h` and the
address out of the ELF, on every run.

That is deliberate, and it is the scar from two separate silent failures:

  * a hand-maintained field list in the reader - adding one field to the struct
    shifts every index after it, the read still succeeds, and the output is
    neighbouring fields under the wrong names
  * a hardcoded address - `0x20000004`, then `0x2000002c`, then `0x20000034`,
    then `0x2000006c`, because adding one static variable anywhere moves `.bss`

Both failure modes produce numbers that look entirely plausible. Neither the
address nor the layout is allowed to be a constant here. The struct also
carries a magic word (`0xF0C00001`) that the reader checks before believing
anything else.

## Things that bite

| Symptom | Cause | Fix |
| --- | --- | --- |
| `GetCommState failed`, or `--check` refuses | something else holds the port | close the monitor window; the port has one owner |
| the window reconnects but the port still looks busy | a one-file `.exe` is two processes; killing the parent leaves the child | `taskkill /IM foc-monitor.exe /F /T` |
| no ports listed | board unplugged, or the USB cable is charge-only | check `SERIALCOMM` in the registry; a known-good binary that fails the same way means it is not the firmware |
| the window is open but hidden | `SetForegroundWindow` is not granted to a background launcher | it is running; alt-tab, or capture it with `PrintWindow` |
| `exe --check > out.txt` writes nothing | PowerShell does not redirect GUI-subsystem executables at all | run the `.py`, or capture through a real pipe (`subprocess`, `aieos run`), both of which work |
| COM number changed | re-enumeration after a re-flash | press `rescan` |

## What the monitor deliberately does not do

  * **No units.** Everything is in the firmware's Q15 counts. Converting i_q to
    amps needs a shunt gain and a zero-current offset measured against real
    sensors, and there is no driver and no motor yet. A plausible-looking amp
    figure derived from a guessed gain would be worse than a raw count.
  * **No logging to file.** 24 seconds of chart and 400 lines of console, then
    the oldest go. Long captures belong in a script, not in a window.
  * **No safety logic of its own.** `on` can be refused, and only the firmware
    decides that. The window shows the refusal; it never works around it.
  * **No firmware flashing.** That is `make flash`, deliberately a separate
    step from the thing that watches.

## What was actually verified, and how

| Claim | How |
| --- | --- |
| the serial layer reads and parses | run against the board before any GUI existed: 35 of 40 lines parsed, the other five were partial lines mid-chunk |
| every command works and the board answers | `--check`, 17 of 17, three runs in a row |
| the buttons reach the board | read `loop_mode`, `speed_ref`, `speed_meas`, `i_q`, `theta` over SWD while the window held COM13 |
| the `.exe` is the same program | `--check` passed 17 of 17 from the built `.exe` |
| the `.exe` carries the real logo | extracted the icon back out of it: byte-identical to `unitymbed.ico` |
| the shortcut works | launched from the Desktop `.lnk`, window came up, connected to COM13 on its own, chart moving |

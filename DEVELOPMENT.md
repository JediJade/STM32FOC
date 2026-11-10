# Developing on STM32FOC

How to work on this firmware: what the pieces are, what the rules are, and
what to do when something does not behave. `README.md` is the record of what
was measured; this is the guide to changing it.

ฉบับภาษาไทยอยู่ที่ [DEVELOPMENT.th.md](DEVELOPMENT.th.md).

The host-side tools - the monitor window, `--check`, the `.exe` build and the
SWD reader - have their own manual in [SOFTWARE.md](SOFTWARE.md)
([ภาษาไทย](SOFTWARE.th.md)).

Everything here was learned by doing it wrong first, so the rules are stated
as rules rather than as suggestions.

---

## 1. The three tools

Nothing about this project is developed by reading code alone. Every change
gets built, flashed and measured.

```bash
# build (UnityMbed's bundled make and toolchain, not whatever is on PATH)
make PROJECT=STM32FOC

# flash
openocd -f openocd.cfg -c "program build/STM32FOC.hex verify reset exit"

# read the self-test results
python read-selftest.py --wait 15

# watch it: windowed monitor, standard library only
python foc-monitor.py

# the same monitor as one .exe, for a machine with no Python
build-exe.cmd
dist\foc-monitor.exe

# talk to it
#   any terminal on the USB serial port, or from PowerShell:
#   $p = New-Object System.IO.Ports.SerialPort 'COM13',115200,'None',8,'One'
```

The toolchain lives at
`C:\Users\USER\AppData\Local\UnityMbed\tools\{make,arm-none-eabi,openocd-xpack}`.
`arm-none-eabi-nm`, `objdump` and `addr2line` are **not** in UnityMbed's
toolchain; use the ones under `C:\Program Files (x86)\ArmGNUToolchain...`.

The self-tests take about 12 seconds now (the latency sweep is slow). Give
`--wait` enough room, and **check `magic` before trusting any field** -
`read-selftest.py` does this for you and says so when the tests are still
running.

---

## 2. Layout

| file | what it owns |
| --- | --- |
| `src/startup.c` | vector table in `.isr_vector`, `Reset_Handler` |
| `src/main.c` | clock, LED, boot order, the test sequence, the console loop |
| `src/tim1.c` | TIM1: centre-aligned 3-phase PWM, dead-time, break input, TRGO |
| `src/foc.c` | Clarke, Park, inverse Park, SVPWM - all Q15 |
| `src/foc_sin_table.c` | generated 256-entry Q15 sine table |
| `src/pi.c` | one PI controller, Q30 integrator, clamped |
| `src/loop.c` | the 20 kHz current loop: idle / align / current modes |
| `src/outer.c` | the 1 kHz speed and position cascade |
| `src/vmotor.c` | the virtual motor: electrical state in the *real* rotor frame, mechanics, a sensor with a hidden offset |
| `src/selftest.c` | the eight self-tests **and the ADC interrupt** |
| `src/usb_cdc.c` | USB CDC-ACM device, on the registers |
| `src/console.c` | line parser, commands, telemetry |
| `src/safety.c` | the gate on MOE: arming rules, latched faults, watchdog |
| `cmsis/` | ST register headers only - no HAL, no middleware, no CMSIS `.c` |
| `read-selftest.py` | host-side reader; parses the struct out of the header |

**Known structural wart:** the ADC interrupt handler lives in `selftest.c`
because that is where it grew, and it is what drives the control loop. A
cleaner home is `loop.c` or its own `isr.c`. If you move it, keep the
statistics gating (section 4) intact.

---

## 3. What runs where, and how often

```
ADC1_2_IRQHandler            20 kHz, priority 1   <- the control loop
  foc_loop_step()              every period
  outer_tick()                 every period, decimates to 1 kHz itself
  statistics                   only while a test asks for them

USB_LP_CAN1_RX0_IRQHandler   as needed, priority 2
SysTick_Handler              1 kHz, lowest priority

main loop                    console, telemetry, watchdog refresh
```

One PWM period is 3600 cycles at 72 MHz. The control path costs about 1000 of
them - 28% - and `loop_cycles_min` equals `loop_cycles_max`, so it is
branch-free with respect to its data. If a change makes those two differ, the
loop has become data-dependent and the timing is no longer guaranteed.

---

## 4. Rules

### Interrupt priority: nothing may preempt the control loop

ADC is priority 1. USB is 2. SysTick is lowest. A console that can delay a
current loop is a console that will, eventually, at the worst moment.

Raising USB to 0 during unrelated debugging pushed the measured sample jitter
from 0 to 148 ticks - 2 us of uncertainty about when the current was sampled -
and the console worked perfectly the whole time. The only thing that noticed
was the jitter measurement.

### A snapshot struct must not contain live counters

`selftest` is read over SWD after the fact. Any field the interrupt keeps
writing is meaningless by the time it is read, and worse, it reads as a
plausible number.

This went wrong four times: `adc_isr_count` (made a 20 kHz chain look like
38 kHz), `adc_cnt_min/max` (reported the cascade test's latency as the ADC
test's jitter), `adc_period_*`, and the loop test's fields (overwritten by the
cascade). The pattern that works:

- a `..._track` flag, set when the owning test starts and cleared when it
  ends, gating every statistic that test owns;
- a frozen result field computed once at the end (`adc_rate_hz`, not
  `adc_isr_count / window`).

### Never hardcode the struct address

Adding one static variable moved `selftest` from `0x20000004` to `0x2000006c`.
A stale address still reads successfully and prints neighbouring variables
under the wrong names. `read-selftest.py` takes the address from the ELF and
the field order from `inc/selftest.h` on every run, and nothing about the
layout is written down in it. Keep it that way.

### A delay that must happen must not be an empty loop

`-Os` deletes `for (volatile uint32_t i = 0; i < 1000; i++) {}` and is
entitled to. It did, twice:

- ADC: `RSTCAL` issued before the analog block had settled;
- USB: `FRES` released a couple of cycles after `PDWN` cleared.

Use `delay_ms()`, which is backed by SysTick. If SysTick is not running yet,
restructure so that it is.

### `TIM1_PERIOD` is ARR, which is *half* the period

Centre-aligned mode counts up to ARR and back down, so one PWM period is
`2 * TIM1_PERIOD` ticks. Writing `TIM1_PERIOD / 2` for "half a period" sets a
bar at a quarter of it, which failed a loop that comfortably fitted.

### Only `ep_set_*_status()` may write `USB_EPnR`

`STAT_RX`, `STAT_TX`, `DTOG_*` are toggle-write: the hardware XORs what you
write, so setting a value means writing `current ^ wanted`. `CTR_RX` and
`CTR_TX` are rc_w0: every write must put 1 there or a completed transfer
vanishes. A plain `EPR(0) = EP_RX_VALID;` is wrong in two different ways.

The exception is configuring a fresh endpoint, where the toggle fields read 0
and a direct write lands correctly.

### MOE comes up zero and only `safety_arm()` turns it on

`tim1_init()` configures everything and leaves the main output disabled.
Arming requires idle loop, outer loops off and no latched fault. Faults latch
until a human types `clear`. Do not add a path that enables MOE directly -
that is what the gate is for.

### The outer loop must be slower than the inner loop

A proportional position loop driving a speed reference behaves as a first
order system with time constant `1/POS_KP` milliseconds. At 0.15 that is
6.7 ms, outside a speed loop that needs tens of milliseconds - unstable by
construction. The position step ended 118 degrees from target. At 0.01 it
settles 0.66 degrees away.

### Anything that blocks the main loop must refresh the watchdog

`emit()` and `delay_ms()` both do. The reasoning matters, because adding
refreshes carelessly turns a watchdog into decoration: both are *bounded*
waits, and the control loop runs in the ADC interrupt, so neither is the hung
main loop the watchdog exists to catch. A bare `while (1)` - what `hang`
does - still trips it.

Before this, `align` from the console reset the board every time (300 ms
against a 100 ms period) and then refused to arm with `refused: watchdog
reset`, on a board that had never hung.

After adding a long-running console command, run `python foc-monitor.py
--check`. It is what caught this.

### A debugger halts the core but not the watchdog

IWDG keeps counting while the core is stopped, so flashing - which halts for
far longer than the 100 ms period - caused a watchdog reset on resume and a
latched fault on a board that had not hung.
`DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP` in `safety_watchdog_start()` handles
it, and does nothing when no debugger is attached. If you add another
independent timer that must survive debugging, check whether it has a
`DBG_..._STOP` bit too.

### Do not trust VREFINT on this board

This part's internal reference measures 1.088 V against a specified 1.16 to
1.24 V. `FE_VDDA_MV` in `selftest.c` is the multimeter reading, 3313 mV, and
the calculation runs from the known supply to the reported reference - not the
other way round. Current scaling must use a measured VDDA.

---

## 5. Adding a self-test

The eight existing ones follow one shape. Copy it.

1. **Fields** in `selftest_results` (`inc/selftest.h`), as `uint32_t` or
   `int32_t` scalars. No arrays - `read-selftest.py` parses scalars.
2. **A pass flag**: `#define SELFTEST_OK_<NAME> 0x..`, next free bit.
3. **A run function** `void selftest_run_<name>(void)` in `src/selftest.c`
   that sets its own fields, then ORs its flag into `ok_flags` only if the
   measurement passes.
4. **Call it from `main()`** in an order that respects dependencies - the ADC
   tests need the ADC interrupt; alignment must precede the current loop; the
   break test goes last because it trips the fault path.
5. **Scope any statistics** the interrupt collects with a `track` flag.
6. Widen the mask in `count_bits(selftest.ok_flags & 0x..U)` and the names
   list in `read-selftest.py`.

**Write the pass criterion so it can fail.** The ADC test once reported a pass
because the edit that reordered its setup had deleted the line enabling its
own measurement: the spread came out `0 - 0 = 0`, comfortably under the bar.
Every other field looked right. A test that cannot fail is worse than no test.

**Put the bar on the thing being tested.** The front-end test judges the
converter, not the board's supply, because this part's reference is known to
be out of spec and a test that failed every boot for a known reason would
simply be ignored.

---

## 6. Adding a console command

In `console.c`, `dispatch()` checks **multi-character commands first**.
`spd 3280` begins with `s`, and checking the single-letter commands first
turned every speed command into a request to print the state - and replied
with a plausible-looking state line.

Output goes through `emit()`, which gives up after 50 ms if the host is not
draining. Do not add anything that can block the main loop indefinitely: the
watchdog is refreshed from there.

Formatting is `put_str` / `put_int` / `put_hex` into a local buffer. There is
no `stdio` and none is wanted.

---

## 7. Bringing up real hardware

In this order. Each step is checkable before the next one can hurt anything.

1. **Measure the supply.** A multimeter on 3V3. If it is not within a few
   percent of 3.3 V, stop and fix that first; everything downstream is scaled
   by it. Update `FE_VDDA_MV`.
2. **Wire the overcurrent comparator to PB12 (BKIN)**, active low. `BDTR.BKE`
   is already set and the pin is pulled up, so a fault drags it low and the
   timer kills all six outputs with no software in the path. Test it by
   shorting PB12 to ground and checking `s` reports `fault break input`.
3. **Re-measure the dead-time for the FETs you actually have.** 1.000 us suits
   slow parts with a cheap driver. Fit the two jumpers (PA8 → PA6,
   PB13 → PA7), set `TIM1_DEADTIME_TICKS`, and read `dt_mean_x100` back. Too
   little dead-time is a dead short through both FETs.
4. **Current sense.** Replace `vmotor_phase_currents()` in the loop with the
   two injected ADC results, ADC1 and ADC2 in simultaneous mode - already
   configured and verified. Calibrate the zero-current offset first: average a
   few hundred samples with the outputs disabled, store it, subtract it from
   every reading. A wrong offset puts a standing error on `i_d` that looks
   exactly like a decoupling failure.
5. **Scale.** Convert LSB to amps using the shunt value, the amplifier gain
   and the measured VDDA. This is the only unknown left in the measurement
   path.
6. **Alignment with the real sensor.** Replace `vmotor_sensor_angle()` with
   the encoder. Run `align` and check the offset is repeatable across power
   cycles - if it is not, the procedure is being disturbed by friction or
   cogging and needs a longer settle or a higher align voltage.
7. **First power-up:** `clear`, `align`, `on`, then `iq` with a small value -
   a few hundred of 32767 - watching `m` telemetry. Only then increase.
8. **Re-tune.** `LOOP_KP/KI`, the decoupling constants, `SPEED_KP/KI` and
   `POS_KP` were all tuned against `vmotor.c`'s made-up plant. The structure
   transfers; the numbers do not. The decoupling constants in particular must
   become functions of measured speed (`omega*L`, `omega*lambda`) rather than
   compile-time values.

Keep `vmotor.c`. Running the loop against it is how you tell a firmware
regression from a hardware problem.

---

## 8. When something does not behave

| symptom | check first |
| --- | --- |
| no USB device at all | **flash a known-good binary.** If that fails the same way, it is not your code. An unplugged cable cost several rounds of hypotheses here. |
| every field reads zero | `magic`. The tests take ~12 s; you are reading mid-flight. |
| plausible numbers, wrong meaning | the struct moved, or the reader's field list is stale. Use `read-selftest.py`. |
| the loop does not respond to commands | is anything driving it? `foc_loop_step()` and `outer_tick()` must run unconditionally in the ISR; `LOOP_MODE_IDLE` and `OUTER_MODE_OFF` are how "do nothing" is expressed. |
| a number you cannot explain | sweep a parameter and look at the slope. The unexplained part of the ADC latency turned out to be a constant 86 ticks once seven sample times were measured; two rounds of guessing had got nowhere. |
| firmware appears hung | halt and read PC over SWD, then `addr2line`. One "hang" was a per-attempt timeout multiplied by an attempt count: 100k spins times 20k attempts, about seven minutes. |

And the one that applies to all of them: **when two measurements disagree,
find a third.** VREFINT said 3.69 V, the ST-Link said 3.23 V, a multimeter
said 3.313 V. The instrument I trusted first was the one that was wrong.

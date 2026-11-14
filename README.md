# STM32F103 FOC groundwork - bare metal

**This build does not drive a motor.** It sets up the inverter timing exactly
as a FOC loop needs it, measures itself, and then parks with the outputs off.

The reason it exists in this shape: the two mistakes that destroy hardware in
a FOC project are a dead-time shorter than you believe, and a current sample
taken at the wrong point in the PWM period. Both are measurable on a bare
Blue Pill with no gate driver and no motor, so they get measured first.

Same bare metal style as the USB HID project: no HAL, no middleware, CMSIS
register headers only.

New here? [QUICKSTART.th.md](QUICKSTART.th.md) is twenty minutes from
nothing to a motor turning on the screen, in Thai, and needs no board to be
worth reading.

The motor can also be a second chip: [STM32FOC-HIL](https://github.com/JediJade/STM32FOC-HIL)
is a hardware-in-the-loop bench built from another Blue Pill, running this
firmware's own `vmotor.c` unchanged.

The other three documents: [DEVELOPMENT.md](DEVELOPMENT.md)
([ภาษาไทย](DEVELOPMENT.th.md)) is how to change this firmware, and
[SOFTWARE.md](SOFTWARE.md) ([ภาษาไทย](SOFTWARE.th.md)) is the host-side half -
the monitor window, `--check`, the `.exe` build and the SWD reader.

| file | role |
| --- | --- |
| `src/startup.c` | vector table in `.isr_vector`, `Reset_Handler` |
| `src/main.c` | 72 MHz clock, LED, runs the self-tests, parks safely |
| `src/tim1.c` | three-phase centre-aligned PWM, hardware dead-time, break input |
| `src/foc.c` | Clarke, Park, inverse Park, SVPWM - all Q15 |
| `src/foc_sin_table.c` | generated 256-entry Q15 sine table |
| `src/pi.c` | fixed-point PI with anti-windup |
| `src/loop.c` | the control loop: idle, align and closed-current modes |
| `src/vmotor.c` | virtual motor: electrical state in the real rotor frame, mechanics, and a sensor with an offset the firmware must discover |
| `src/outer.c` | speed and position loops, cascaded at 1 kHz |
| `src/usb_cdc.c` | USB CDC-ACM device on the registers: no HAL, no middleware |
| `src/console.c` | line-based console on that port: commands and telemetry |
| `src/drive.c` | v0.3: throttle, brake, field weakening - the layer that makes it a vehicle drive |
| `src/sensor.c` | v0.3: the rotor angle from Hall sensors or from an observer, and Hall calibration |
| `src/oneline.c` | v0.3: the one-line (SIF) dashboard signal |
| `src/canbus.c` | v0.3: CAN on the registers, its protocol, and its power-on loopback test |
| `src/safety.c` | the gate on MOE: arming rules, latched faults, watchdog |
| `read-selftest.py` | host-side reader: parses the field order out of the header and the address out of the ELF |
| `src/selftest.c` | the eight measurements |
| `inc/`, `cmsis/` | headers; `cmsis/` is register definitions only |

```sh
make PROJECT=STM32FOC     # -> build/STM32FOC.elf / .bin / .hex   (2.7 KB flash)
make addr                 # address of the self-test result struct
make flash
```

## Pin map

| pin | function |
| --- | --- |
| PA8 / PB13 | TIM1 CH1 / CH1N - phase A high / low |
| PA9 / PB14 | TIM1 CH2 / CH2N - phase B high / low |
| PA10 / PB15 | TIM1 CH3 / CH3N - phase C high / low |
| PB12 | TIM1 BKIN, break input, **active low, pulled up** |
| PA6 / PA7 | TIM3 CH1 / CH2, dead-time measurement inputs |
| PA0 | ADC1 channel 0, stands in for a current shunt amplifier |
| PC13 | LED, blinks the number of passed self-tests |

PWM is 20 kHz centre-aligned (`ARR = 1800` at 72 MHz) and dead-time is asked
for as `BDTR.DTG = 72`, which should be 72 x 13.889 ns = **1.000 us**.

CH4 is never enabled as an output. It exists only as the ADC trigger, and its
pin would be PA11 - USB D-.

## The eight self-tests

**1. Dead-time, measured at the pins.** Needs two jumpers: **PA8 -> PA6** and
**PB13 -> PA7**. TIM3 runs free at the full 72 MHz and timestamps the edges of
TIM1's own outputs, so one tick is 13.889 ns. At every switching edge the low
side turns off first and the high side turns on one dead-time later, so
`t(PA6 rising) - t(PA7 falling)` *is* the dead-time. Expect a mean of ~72
ticks. With no jumpers fitted the test reports `dt_samples = 0` instead of a
plausible-looking wrong number.

**2. Where the ADC actually samples.** TIM1 fires TRGO when the counter
reaches `CCR4`, which starts an injected conversion with no software in the
path. The interrupt then records `TIM1->CNT`, so the reported latency covers
sample-and-hold plus conversion plus interrupt entry - about 20 ADC clocks at
12 MHz, so ~1.67 us, ~120 ticks. The useful property is that it is
*constant*; a value that jitters means something else is taking the trigger.

**3. The fixed-point math, no hardware.** Over a full electrical revolution it
checks that `sin^2 + cos^2` stays at 1 (32767 in Q15), that inverse Park
followed by Park returns the original `(d, q)` - which catches the sign error
that makes a motor fight itself - and that the SVPWM duties stay inside
`[0, period]` while their spread swings by exactly `2/sqrt(3) = 1.1547`. That
ratio is a fixed property of a balanced three-phase set (`max - min` is
`sqrt(3)*V` at the sector boundaries and `2*V` between them), so it makes a
much sharper test than "roughly constant" would.

**4. The closed current loop.** Runs the real loop in the real interrupt at
the real rate, with `vmotor.c` standing in for the motor: a first-order RL
model in the rotor frame, with cross-coupling and back-EMF, reporting *phase*
currents so that Clarke and Park are genuinely exercised. It checks that the
signs through the whole chain agree, that the integrator removes the back-EMF
that proportional gain alone cannot, that the d axis stays near zero against
the cross-coupling, and that one control period fits inside one PWM period.

**5. Rotor alignment.** Everything else assumes the loop knows where the
rotor is. It does not: a position sensor has its own zero, unrelated to the
rotor's magnetic zero, and commanding torque at the wrong angle gives less
torque and more heat - at 90 degrees out, maximum current and no torque at
all. The procedure is open-loop on purpose, because the angle cannot be
trusted until it has run: apply a fixed field at electrical angle zero, let
the rotor pull itself into line, then read the sensor. Whatever it reads is
the offset. `vmotor.c` hides the real offset from the firmware and starts the
rotor at an arbitrary angle, so this is a real search.

**6. The speed and position cascade.** Position P -> speed reference, speed
PI -> current reference, current PI -> voltage, with the outer pair running at
1 kHz, every 20th current period. Two phases: a speed step, then a position
step of a quarter turn. The position loop has no integrator on purpose, so if
it settles on target it is because the speed loop underneath it is working.

**7. The break input.** Everything else here is software protecting
hardware. This is the one thing that protects hardware from software: BKIN
going active clears `MOE` inside the timer, asynchronously, with no
interrupt, no handler and no code in the path, so all six outputs go inactive
whatever the CPU is doing - even if it is stuck commanding full duty. It had
been configured since the first version of `tim1.c` and never once
triggered, and a protection path that has never been triggered is a guess.

Both paths are checked, because they prove different things: `EGR.BG`, the
timer's own software break generator, proves break -> `MOE` cleared ->
outputs off; pulling PB12 low proves the pin, its configuration and the
`BDTR.BKP` polarity, which is what an overcurrent comparator will use. The
firmware pulls its own fault pin low by switching that pin's pull-up to a
pull-down - the same trick used to press the demo buttons over SWD.

It also checks that `MOE` *stays* off while the break is held. `AOE` is
clear, so re-enabling must take software: a fault cannot clear itself and
restart the inverter.

**8. The measurement front end.** Two phase currents have to be sampled at
the *same* instant: the Clarke transform treats them as one vector at one
point in time, and readings a microsecond apart on a 20 kHz waveform are not
that. ADC1 and ADC2 in injected simultaneous mode do it in hardware - one
trigger, both results latched - and this test proves that path runs, then
measures the zero-current offset and the noise on each channel. The offset is
what a real build subtracts from every reading; getting it wrong puts a
standing error on `i_d` that looks exactly like a decoupling problem.

It then checks the converter against something that needs no external wiring
at all: the chip's internal 1.2 V reference on channel 17.

## The console

The board comes up as a USB serial port - `USB Serial Device (COMn)`, VID
0x0483 PID 0x5752 - and the self-test results, the live loop state and the
targets are all reachable from it. Any terminal at any baud rate: there is no
UART behind it, so the line coding is accepted and ignored.

```
?          help
s          state: angle, speed, i_d, i_q, i_q reference, MOE, fault
t          self-test summary, reset cause, watchdog state
m          toggle 10 Hz telemetry
iq <n>     current mode, q-axis target in Q15
spd <n>    speed mode, reference in angle units per ms
pos <n>    position mode, target = current angle + n
align      re-run the rotor alignment
off        idle, outputs disabled
on         arm: enable the PWM outputs, refused unless idle and fault-free
clear      clear a latched fault
hang       stop refreshing the watchdog, on purpose
```

Console version 2 (2026-11-10) adds the commands a host needs to set the
gains and to measure the plant, without a rebuild:

```
ver             fw 2 cap 256 - the original console answers "? for help"
x               applied v_d and v_q, loop mode, outer mode, recorder state
get             every settable parameter, as "p <name> <value>", then "get end"
set <p> <n>     set one; the reply is the value as stored, after limiting
id <n>          d-axis current target in Q15 (no torque: the rotor stays put)
vd <n>          voltage mode: v_d = n, v_q = 0, no controller, at most half scale
cap <ch> <div>  arm the recorder on id, iq, vd, vq, spd or ang; 256 samples,
                one every <div> control periods, starting with the next target
dump            the record: a header, lines of "d" and eight values, "cap end"
```

| parameter | what it is | built-in | range |
| --- | --- | --- | --- |
| `ckp` | current Kp, Q15 | 6000 | 0 - 262144 |
| `cki` | current Ki, Q15 per 50 us period | 900 | 0 - 32767 |
| `cdec` | feed-forward decoupling | 1 | 0 - 1 |
| `ccross` | cross-coupling feed-forward, Q15 | 20580 | 0 - 32767 |
| `cemf` | back-EMF feed-forward, Q15 | 6554 | 0 - 32767 |
| `skp` | speed Kp, Q15 | 32768 | 0 - 1048576 |
| `ski` | speed Ki, Q15 per 1 ms update | 6144 | 0 - 131072 |
| `simax` | current the speed loop may ask for | 16384 | 256 - 29491 |
| `pkp` | position Kp, Q15 | 328 | 0 - 32768 |

They live in RAM until `save` writes them to flash - console version 4,
below. With the built-in values the loops compute exactly what they did when
these were `#define`s.
`studio/` is the host that uses all of this - a parameters page and an auto
tune that measures the plant through `vd`, `id` and `cap` and then sets the
four gains.

Three things changed underneath, each found on the board while adding the
above:

- **`spd` is live in every mode.** Speed was measured only while a speed or
  position loop was closed, so in idle and in current mode the state line
  kept its last value: 2640 for a rotor that had stopped.
- **A completed USB transfer could be lost.** Clearing `CTR_TX` wrote
  `CTR_RX` back as it had been read, so an OUT packet finishing in between
  was cleared unseen, and the endpoint stayed at NAK for good. Two requests
  sent back to back did it in three seconds (EP1R read 0x2021, `rx_len` 0).
  After the fix: 106,704 overlapping requests, none unanswered.
- **A reply of exactly 64 bytes arrived one request late.** A full-size
  packet does not end a bulk transfer, so the host held it until the next
  one. The state line is 64 bytes for some values. Packets are now at most
  63 bytes: 615 such replies, all within 3 ms.

Not changed: `align` still ends in a watchdog reset (it blocks for 300 ms in
`delay_ms()`, which does not refresh the 100 ms watchdog), and the current
loop self-test's settling count still depends on where the control interrupt
lands among its set-up statements (83 or 85 periods from the same
arithmetic, shown by adding nothing but a delay).

Measured over the port, with the plant still virtual:

```
--> spd 3280
  ang 50681  spd 3280  id  0  iq 3795  iqref 3794   speed held exactly
  ang 58777  spd 3280  id  4  iq 3793  iqref 3794
--> spd 0
  ang 13218  spd    0  id  2  iq  -38  iqref  -38   braked and holding
--> pos 16384
  ang 29322  spd    0  id  0  iq   20  iqref   22   a quarter turn, already
  ang 29455  spd    0  id  0  iq   69  iqref   70   settling on target
```

The USB stack is the bare metal one from the HID project with the class
swapped, which needed one thing HID never did: an EP0 **OUT** data stage.
`SET_LINE_CODING` sends seven bytes to the device, and a stack that stalls
every request with an OUT payload - as the HID one did, because it never
needed to receive on EP0 - will not finish enumerating as CDC.

## Version 0.3: the vehicle side (console version 3)

Seven things a drive needs before it can sit in a vehicle. All of them are in
the firmware and all of them have a page in `studio/`. What they were tested
against is said plainly, because it matters: **the motor is still the model
inside the firmware, and nothing was connected to the new pins.** The
controller code runs on the real chip, at the real rate; the throttle, the
brake lever, the Hall sensors, a dashboard and a CAN bus were not on the
bench.

| | what | where |
| --- | --- | --- |
| throttle | analog input, calibrated, ramped, with a broken-wire fault | PA2 (0 to 3.3 V), or `th <n>` |
| brake | cuts the throttle and regenerates, fading out near standstill | PB11 to ground, or `brk <0/1>` |
| field weakening | negative d-axis current when the voltage runs out | `fw`, `fwstart`, `vlim` |
| sensor selector | encoder, Hall sensors, or no sensor (observer) | `sens` 0 / 1 / 2 |
| Hall sensors 3144 / 3143 | three open-collector inputs, edges learned by calibration | PB5, PB6, PB7 (pulled up) |
| one-line dashboard | 12-byte SIF frame every 170 ms | PB10, push-pull |
| CAN | protocol below; USB and CAN change over in software | PB8 receive, PB9 transmit |

```
v               th N thr N brk N veh N vflt N sens N ph N hall N cal N idfw N aerr N
th <n>          throttle reading in place of the pin, 0..4095 ADC counts
brk <0|1>       brake in place of the pin
drv <0|1>       throttle mode on / off. It starts only once the throttle is released
hallcal         learn where the six Hall edges are (about 1.7 s)
hall            hall ok N err N e <eight edge angles, -1 = state does not occur>
olf             the one-line frame last sent, twelve bytes in hex
can             can test N mode N id N br N rx N tx N drop N
can on          change over to CAN: the USB console closes
cap ol|hall|aerr|idfw <div>     four more recorder channels
```

Twenty-four more parameters (`get` lists 33). The first nine are as they were,
except that `cdec` now has a third setting and two limits are wider:

| parameter | what it is | built-in | range |
| --- | --- | --- | --- |
| `cdec` | feed-forward: 0 off, 1 fixed (as before), 2 scaled by measured speed | 1 | 0 - 2 |
| `ccross`, `cemf` | the two feed-forward terms, as they are at 6560 units/ms | 20580, 6554 | to 65535, 131071 |
| `vlim` | voltage either axis may use, Q15 | 29491 | 4096 - 29491 |
| `fw` | most field-weakening current, Q15; 0 = off | 0 | 0 - 16384 |
| `fwstart` | weakening starts at this fraction of `vlim`, Q15 | 29491 | 16384 - 32767 |
| `sens` | 0 encoder, 1 Hall, 2 observer. Only while idle | 0 | 0 - 2 |
| `hsrc` | Hall signals: 0 the model's, 1 the pins | 0 | 0 - 1 |
| `hwid` | how much of a turn a model Hall sensor stays on | 32768 | 22000 - 43000 |
| `sla` | observer: control period over L/R, Q15 | 1638 | 128 - 32767 |
| `slmin` | observer takes over above this speed, units/ms | 600 | 100 - 3000 |
| `slcur` | current while the rotor is on the forced angle | 4000 | 0 - 16384 |
| `slramp` | how fast the forced angle speeds up, units/ms per s | 2000 | 100 - 20000 |
| `insrc` | throttle and brake: 0 console, 1 pins | 0 | 0 - 1 |
| `thmin`, `thmax` | throttle reading released / fully open, ADC counts | 900, 3300 | 0 - 4095 |
| `thmode` | throttle is 0 torque, 1 speed | 0 | 0 - 1 |
| `thramp` | throttle may rise this much per ms, of 32767 | 66 | 1 - 32767 |
| `drvspd` | top speed under throttle, units/ms | 6560 | 100 - 16000 |
| `regen` | braking current, Q15 | 4096 | 0 - 16384 |
| `mot` | the model's motor: 0 gimbal, 1 hub. Only while idle | 0 | 0 - 1 |
| `ol`, `olt` | one-line output on / off, and its time unit in 50 us periods | 0, 10 | 0 - 1, 5 - 40 |
| `canid`, `canbr` | node number; bit rate 125, 250, 500 or 1000 kbit/s | 1, 500 | |
| `canper` | status frames every this many ms | 20 | 5 - 1000 |
| `canback` | on CAN, back to USB after this many s without a frame; 0 never | 10 | 0 - 600 |

### What each one does, and what the board showed

`studio/checks/v3_features.py COM13 --can` runs all of this through the
console and prints one line per check. On the board, 2026-11-10: 33 of 33.

**Throttle and brake** (`src/drive.c`). The reading is scaled between
`thmin` and `thmax` with the bottom thirty-second ignored, rises at
`thramp` and falls at once. A reading under half of `thmin`, or more than
half way from `thmax` to full scale, for 50 ms, is a wire and not a
position: the drive stops and latches `vflt` until `clear`. `drv 1` does
not start until it has seen the throttle released. The brake always wins:
throttle to zero, then `regen` of braking current, fading out below 300
units/ms so that it cannot drive the rotor backwards. On the board: full
throttle held at 6784 units/ms against a limit of 6560 (the torque is taken
away over the next 256), braking current -3809 counts 12 ms after the
brake, standstill without reversing.

**Field weakening.** Once the loop uses more than `fwstart` of `vlim`,
an integrator raises negative d-axis current, up to `fw`. The model's
first motor is the wrong motor to show it on - its winding is nearly all
resistance at any speed it reaches, so the current costs more voltage than
it frees. So the model has a second motor (`mot 1`: L/R 5 ms, five times
the back-EMF, a heavier rotor with rolling resistance). On that one, on
the board: top speed 5280 units/ms without, 10220 with (i_d -8192). For
this the feed-forward had to follow the speed (`cdec 2`): it was two
constants, right at one speed.

**The speed a Hall sensor gives, and the bias in it.** A sector's speed
is its calibrated width over the time it took. The time is honest to one
period; the width is a measurement with an error of its own, and that error
is fixed per sector, so each sector reports the same speed a few percent
wrong in its own direction, turn after turn. The loop then chases a
staircase that is not there. Recorded on the board at 1000 units/ms, the
reported speed stepped at every edge over a 20 percent band while the same
loop on the encoder held 4 percent.

Six sectors make a turn, and over a whole turn the widths have to add up to
65536 however wrong each one is - so the speed over the last six sectors is
free of the bias, and each sector's own estimate is compared against it. A
per-sector correction creeps towards what would make the two agree (Q12, a
sixteenth of the gap per edge, clamped to a factor of two), and it is
learned only while the turn as a whole is steady, or real acceleration gets
written into the table and read back out as a wobble. The delay stays one
sector, which is what the loop needs, and the corrections are forgotten
whenever the table changes, the rotor reverses or it stops.

`studio/checks/hall_speed.py COM13` is the measurement, encoder and Hall
one after the other on the same motor with the same gains. On the board,
2026-11-11, ripple as a fraction of the speed held:

| asked, units/ms | before | after | encoder |
| --- | --- | --- | --- |
| 600 | 16 % | 5.4 % | 1.9 % |
| 1000 | 15 % | 2.8 % | 1.2 % |
| 2000 | 15 % | 2.8 % | 0.6 % |
| 4000 | 15 % | 3.5 % | 0.5 % |

At 300 it is still 65 %, and no arithmetic here can change that: one speed
update per 180 ms is what the sensor gives.

**Sensor selector, Hall sensors, observer** (`src/sensor.c`). The 3144
and 3143 are unipolar switches, not latches: each is on for less than half
a turn, so the six states are not 60 degrees each. The edges are therefore
measured, not assumed: `hallcal` turns a field slowly one way and back,
the rotor follows, and the field angle at each edge is written down, the
two directions averaged. With model sensors on for 148 degrees it found
sectors of 92 and 28 degrees, each edge within 0.11 degree of where the
model has it. Between edges the angle is carried at the speed the last two
sectors took. With the speed loop at 3000 units/ms the angle in use stayed
within -0.3 to +2.3 degrees of the model's rotor. The observer takes the
back-EMF from the applied voltage and the measured current and follows it
with a phase-locked loop; below `slmin` the rotor is dragged at a forced
angle with `slcur`. On the board it took over 378 ms after `spd 3000` and
then stayed within 0.9 degree; on the second motor, 0.7 degree.

**One line** (`src/oneline.c`). Idle low; sync 31 T low and 1 T high; a
zero is 2 T low, 1 T high and a one the other way round; twelve bytes,
most significant bit first; the last is the exclusive-or of the rest.
`cap ol 1` records the pin itself as run lengths, so the waveform can be
checked with nothing connected: read back from PB10, 194 level changes,
T exactly 500 us, sync -310 / +10 periods, frame `08 61 00 40 00 1b 00 00
89 00 00 bb`, checksum good. Displays do not agree on what the bytes
mean; the layout is the common one and is all in `oneline_build()`.

**CAN** (`src/canbus.c`). On this chip the CAN controller and the USB
device share memory and cannot both be on. `can on` shuts USB down, holds
D+ low so the host sees an unplug, and brings CAN up on PB8 / PB9. It goes
back on a command frame with mode 0x55, or by itself after `canback`
seconds without a frame - so a board with nothing on its CAN pins comes
back. On the board: the port was away for 3.5 s with `canback 3`, came
back with its parameters intact, and the reset cause had not changed. So
that the controller is not code that has only been compiled, it runs in
loopback before USB starts at power-on: a frame out and back unchanged,
then a parameter request to itself answered with the right value (`can`
reads `test 7`). That test first read 0: left on its reset mapping the
controller listens on PA11, which is USB's D- line, held low by the host,
and it never acknowledged initialisation.

```
0x100+n  in    command     [mode, flags, value lo, value hi]
                 mode 0 off, 1 torque current, 2 speed, 3 throttle 0..32767
                 (flags bit0 = brake), 0x55 back to USB
0x200+n  in    parameter   [op, index, 0, 0, value int32]   op 1 read, 2 write
0x300+n  out   status 1    speed, iq, id (int16), state, fault
0x400+n  out   status 2    vd, vq (int16), angle (uint16), throttle/128, flags
0x500+n  out   reply       [op | 0x80, index, 0, 0, value]
0x700+n  out   heartbeat   [console version, vehicle state], once a second
```

A drive told to turn that hears no command for half a second stops. The
outputs cannot be armed over CAN. A transceiver is needed to reach a bus,
and none was fitted: on the bench every status frame was dropped unsent.

### What it cost, and two things fixed on the way

- **The loop got faster, not slower.** The first build with all of this
  took 1897 cycles a period against a bar of 1800 and failed its own
  timing test. Park and inverse Park each looked up the same sine and
  cosine; so did the model. Looked up once and shared, with the two
  transforms inlined, the period is 1441 cycles (1731 before this version).
  Self-test result unchanged: `0xbe`, the two that need jumpers.
- **Without a host the board reset ten times a second.** The main loop's
  LED blink waited 150 ms and more inside `delay_ms()` with a 100 ms
  watchdog running. Unnoticed, because with a PC attached that branch is
  never taken - until coming back from CAN took it. Those waits now
  refresh the watchdog.
- Flash 23.9 KB of 64, RAM 1.8 KB of 20.

### What is not there

- No real throttle, brake, Hall sensor, dashboard or transceiver has been
  connected. `hsrc 1` and `insrc 1` read the pins; nothing has driven them.
- With Hall sensors the speed is as old as the last sector, and on the
  model's 3144-like sectors the wide one lasts 180 ms at 300 units/ms - so
  below about 600 units/ms the loop is told the speed five times a second
  and rides on it. Torque mode, which is what a throttle is, does not have
  this problem. Above that it is steady: see the per-sector correction
  below.
- Below `slmin` the observer's rotor is on a forced angle with no damping
  of its own. It starts and it turns; it is not a position hold.
  At the weakened top speed of the second motor its angle was 4.4 degrees
  behind the model's rotor.
- No bus voltage is measured, so the dashboard's battery byte is zero and
  regeneration has nothing watching where the energy goes.
- `align` still ends in a watchdog reset.

## Version 0.3: the settings the board keeps (console version 4)

Everything above that can be set was lost at reset. A board that had been
tuned, had its throttle ends measured and its Hall edges learned came back
from a power cycle as a board that had had none of it done. On a bench that
is a nuisance; in a vehicle it is the feature missing, because nobody
calibrates a throttle at every switch-on.

```
save       write the parameters and the Hall table to flash
load       put back what is written there
sto        sto state <w> boot <w> console N p N hall N
sto clear  erase the page
```

`state` is what the page holds now and `boot` is what happened to it at
start-up - each one word: `ok`, `empty`, `damaged`, `version`, `busy`,
`error`. `save` and `load` answer `save ok` or `save refused: <why>`.

One 1 KB page, the last in the device (`0x0800FC00`), holds one record: the
33 parameters as the console numbers them, the Hall table, and a checksum.
`linker.ld` ends the code region one page short of it, so a build that grew
into the page would fail to link rather than erase a piece of itself on the
first save. `main()` loads the record before the watchdog starts, so the
console is already answering as the board that was put away.

Four things worth stating.

**It is refused while anything is being driven.** Erasing a page stalls
every instruction fetch from flash for tens of milliseconds, which means
the control interrupt does not run for that long. With the outputs live
that is a motor commanded by whatever the duty registers happen to hold, so
`store_save()` asks `drive_idle()` and `safety_armed()` first and the
watchdog is fed from inside the wait loop.

**The record carries the console version, and a different one is refused.**
The parameters are stored by number, so a record written by a firmware with
a different parameter list would set the wrong things. `sto` reports
`version` and the board stays on its built-in values rather than being
quietly wrong.

**The Hall table goes back before the parameters.** `sens 1` is refused
while no table has been learned - rightly, because an angle from an
unlearned table is a wrong angle. Restoring the parameters first therefore
threw the sensor selection away without a word: the board came up on the
encoder having been saved on its Hall sensors. The check caught it, the
order fixed it, and the order is now stated in `store_load()` with the
reason.

**A sum, not a CRC.** What it has to catch is a record whose write was cut
off - a power cut between the erase and the last halfword. An erased page
reads `0xFFFF` everywhere, so a truncated record differs in whole halfwords,
which a rotating sum catches as surely as a CRC in a tenth of the code.

`studio/checks/store_flash.py COM13 --hall` is the evidence, and it does not
stop at `save` answering `ok`: it marks six parameters with values no
default has, saves, makes the board reset through the watchdog (`hang` - the
only reset this console can ask for, and the one that cuts the firmware off
without a chance to write anything on the way down), waits for the port to
come back and reads them out again. On the board, 2026-11-11: 23 of 23,
including the six learned Hall edges identical before and after the reset
and the board coming back up on its Hall sensors. The emulator answers the
same 23, with a file in place of the page.

What is still true: the page is erased and rewritten whole on every `save`,
which is about 10,000 saves on this part and not a thing anybody does in a
loop; and `openocd program` leaves the page alone, so a reflash of the
firmware keeps the settings.

## Version 0.4: the loop leaves the chip

The v0.3 section above says, and was right to say, that the motor is still
the model inside the firmware and nothing is connected to the new pins. This
is the version that retires that sentence.

A second STM32F103 - [STM32FOC-HIL](https://github.com/JediJade/STM32FOC-HIL) -
stands in for the motor. It runs `vmotor.c` from this repository, copied
without a change, measures this board's three phase duties with hardware
timers, and sends the rotor angle back as Hall signals. Ten wires, no power
stage, no gate driver, no motor and no battery anywhere in it.

Two halves cross, and they are independent:

| | crosses on | turned on by | what it proves |
| --- | --- | --- | --- |
| the rotor angle | PB5, PB6, PB7 | `set hsrc 1`, `set sens 1` | the Hall path: edges, calibration, the speed estimate |
| the phase currents | PA0, PA1 | `isrc 1` | the current path: the ADC front end, its zero, its scaling |

**The angle.** `hallcal` measured the six switching angles from a signal
produced in another chip, and the speed loop then held 1500 units/ms on an
angle that crosses on a wire. The two boards' speeds agree to 0.2 % - neither
can see the other's number, so when they agree the loop really is closed
through the copper.

**The currents.** `isrc 1` takes i_a and i_b from PA0 and PA1 instead of from
the model. Nothing new had to be set up for it: the power-on self-tests
already leave ADC1 and ADC2 in injected simultaneous mode on a TIM1 trigger,
converting both pins from one trigger - that path was built so a real front
end could be dropped into it. `izero` learns what the two pins read at zero
current, averaged over 40 ms, because that offset is subtracted from every
reading afterwards and a noisy estimate of it becomes a standing error on
i_d that looks exactly like a decoupling problem.

That is the half that makes a load mean something. With a brake applied to
the rotor inside the other chip, at a held speed of 1200 units/ms:

```
  load   drive spd   drive iq   drive id
     0        1190       1331         42
     8        1169       2316        -12
    27        1181       4151         80
    48        1201       6226          2
    85        1416      10240       -360
```

`iq` is this board's own reading off its own pins, and it rises nearly five
times while the speed holds. With only the angle crossing, that column did
not move at all. `id` stays within a few tens of zero until the drive runs
out at a drag of 85, which is the part that says the current is arriving at
the right angle and not merely arriving.

**`cap iab`** records both phase currents into the one buffer, alternately:
128 pairs, each pair measured in the same interrupt. A current vector drawn
from two separate records is not a vector. The studio's scope page draws the
three phase currents and the current vector from the board because of it.

**Two pages in `studio/`.** *bench* is what the control loop is driving, as
a choice of three with the reason you would pick each, and it draws the
difference between asking for the pins and getting them - the firmware
refuses to steer by sensors it has not calibrated, and that state is amber
rather than a quiet fall back to the model. *current* is the analog path
stage by stage with live numbers: counts, volts, counts about the learned
zero, Q15, and what the loop made of it. When that path breaks, every stage
after the break still produces a number and all of them look like currents.

The console commands are `isrc`, `iscl` and `izero`, and they are commands
rather than numbered parameters on purpose: the flash store keys its record
on how many parameters the console has, so adding two would throw away every
board's saved settings and Hall table to announce a feature those boards may
not even be wired for.

## The gate on MOE

`on` used to enable the outputs unconditionally. That is fine against a
virtual motor and unacceptable against real FETs, so three rules are enforced
in `safety.c`:

**Arming requires a known-safe state.** Idle loop, outer loops off, no latched
fault. Arming out of speed mode with a reference already set would start
driving the moment the command landed.

**Faults latch.** The break input clears MOE in hardware and, with `AOE`
clear, it stays off; this layer records why and refuses to arm until a human
types `clear`. A fault that clears itself is a fault that gets ignored.

**A hung main loop must not leave the bridge switching.** The independent
watchdog resets the chip, which returns MOE to its reset value of zero. The
reset cause survives the reset, so the console says what happened instead of
coming up as though nothing did.

Measured over the console:

```
--> on                      from idle
armed, outputs enabled
--> spd 2000
--> on                      with a speed reference set
refused: not idle

--> hang                    stops refreshing the watchdog
    (board resets by itself, re-runs the self-tests, re-enumerates)
--> t
reset watchdog, wdg 1, fault watchdog reset
--> on
refused: watchdog reset     the fault survived the reset
--> clear
fault cleared
--> on
armed, outputs enabled
```

The watchdog starts only after the self-tests. Those spend seconds inside
`delay_ms()`, and sprinkling refreshes through them to keep a watchdog happy
would mean the tests were also testing the watchdog, badly. Its period is
~100 ms at LSI's nominal 40 kHz, and LSI is untrimmed - roughly 66 to 133 ms
across the specified range - against a console loop that refreshes every few
milliseconds.

## Reading the results over SWD

No UART, no USB, no scope. The results sit in one struct in RAM; OpenOCD reads
it while the CPU keeps running:

A windowed monitor is included as well, for watching the loop rather than
poking at it:

```bash
python foc-monitor.py
```

It lists the serial ports, shows the loop state at 10 Hz with a scrolling
chart of `i_d`, `i_q` and speed, and carries the same commands the console
takes. Standard library only - no pyserial on this machine, so the port is
driven through Win32 with ctypes.

The monitor also has a headless mode that exercises every command and checks
the board answered - the buttons do nothing but send these strings, so it
covers the command path short of the Tk wiring:

```bash
python foc-monitor.py --check     # 17 checks, repeatable
```

### The monitor as an .exe

`build-exe.cmd` packs the monitor into one file that runs on a machine with no
Python installed - 9.9 MB, `dist\foc-monitor.exe`, same arguments:

```bat
build-exe.cmd
dist\foc-monitor.exe              the window
dist\foc-monitor.exe --check      the 17 checks
```

Three details that are easy to get wrong:

  * it is built `--windowed`, so there is no console and `sys.stdout` is
    `None`. `--check` would die on its first `print()`. `attach_console()`
    attaches to the console that launched it, or makes one.
  * the icon is the UnityMbed logo, taken out of `unitymbed-desktop.exe`'s own
    resources - all six sizes, 16 to 256, byte for byte. It is both the file's
    icon and the window's, and `SetCurrentProcessExplicitAppUserModelID` keeps
    Windows from showing python's icon in the taskbar instead.
  * `--add-data` resolves relative to the spec file, which `--specpath` moves,
    so the paths in `build-exe.cmd` are absolute. PyInstaller's scratch goes in
    `dist\work`, not `build\` - that one is the firmware's.

Verified: the icon extracted back out of the built .exe is identical to the
source, the window comes up titled and connects to COM13 on its own, and
`--check` passes 17 of 17 from the .exe against the board.

```bash
make addr      # the struct moves whenever .bss changes - never hardcode it
openocd -f openocd.cfg -c "init; set v [read_memory <addr> 32 40]; echo \$v; exit"
```

Easiest way, which applies both rules below by construction:

```bash
python read-selftest.py --wait 5
```

Two rules for reading it, both learned the hard way:

- **Take the address from `make addr` every time.** Adding one static variable
  moved this struct from `0x20000004` to `0x2000002c`, and a stale address
  reads neighbouring variables as plausible-looking nonsense.
- **Check `magic` first.** `main()` writes it last, after every test has
  finished. The self-tests take about three seconds, and reading `ok_flags`
  before `magic` is set catches the firmware mid-test: it reported 0x7 instead
  of 0xf once for exactly that reason.
- **Do not keep the field order in the reader by hand.** Adding one field
  shifts every index after it; the read still succeeds and prints neighbouring
  values under the wrong names. That produced several wrong conclusions in
  this project before `read-selftest.py` started parsing the struct out of
  `inc/selftest.h` on every run.

The 19 words, in order:

```
 0  magic                 0xF0C00001 once the tests have finished
 1  dt_samples            valid capture pairs (0 = no jumpers)
 2  dt_min                ticks
 3  dt_max                ticks
 4  dt_mean_x100          mean * 100, so 7200 means 72.00 ticks = 1.000 us
 5  dt_expected           what BDTR.DTG was set to
 6  dt_sr_snapshot        TIM3->SR accumulated: bit1 = PA6 ever captured,
                          bit2 = PA7 ever captured. Tells a missing jumper
                          apart from a dead channel.
 7  adc_isr_count         injected conversions completed in 100 ms
 8  adc_trigger_ccr       requested trigger count (CCR4)
 9  adc_cnt_at_isr        TIM1->CNT inside the ISR
10  adc_dir_at_isr        0 = counting up, 1 = counting down
11  adc_latency_ticks     |cnt_at_isr - trigger|
12  math_park_max_err     worst round-trip error, LSB of 16384
13  math_sincos_min       min of sin^2 + cos^2 (ideal 32767)
14  math_sincos_max
15  math_duty_min         duty extremes over one electrical turn
16  math_duty_max
17  math_span_min         max duty - min duty, min over the turn
18  math_span_max         ratio to span_min should be 2/sqrt(3) = 1.1547
19  ok_flags              bit0 dead-time, bit1 ADC, bit2 math
```

The LED blinks the number of set bits in `ok_flags`, so the board reports
itself with nothing attached.

## Measured on hardware

All six tests pass, `ok_flags` 0x3f, with the two jumpers fitted
(PA8 -> PA6 and PB13 -> PA7):

```
dead-time    200 samples, mean 73.00 ticks = 1014 ns   asked for 1000 ns
adc          19990 Hz = one conversion per PWM period
             sample instant jitter 0 ticks - the same count every period
             interval 3598 .. 3602 ticks against a 3600 tick period
             latency 204 ticks = 2.83 us, of which 86 are fixed overhead
math         Park round trip 5 LSB of 16384; SVPWM span ratio 1.1541
             against the theoretical 2/sqrt(3) = 1.1547
align        offset found 12475 against a true 12345
             error +130 angle units = +0.714 electrical degrees
loop         i_q reached 8193 of 8192, settled in 85 periods
             i_d band -4 .. +16, mean 0
             control path 992 cycles = 13.78 us
cascade A    speed step 3280 -> final 3280, peak 4720, settled 31 ms
cascade B    position step of a quarter turn -> final error +120 units
             = +0.66 degrees, settled 389 ms
outer loop   142 cycles = 1.97 us per 1 kHz update
```

14 ns of jitter on the sample instant is the counter's own resolution, so the
trigger chain is as deterministic as this timer can measure.

```
break        pin idle high 1          the pull-up holds BKIN inactive
             MOE before 1             the outputs really were enabled
             MOE after EGR.BG 0       the software break generator works
             MOE after pulling PB12 0 and so does the real pin
             stayed off 1             software cannot re-enable while held
             recovered 1              and it does come back once released

front end    pairs 1000               ADC1+ADC2 simultaneous conversions
             offset A 2093, B 2084    zero-current reading, LSB of 4095
             noise A 328, B 2219      floating pins, reported not judged
             vrefint 1346 LSB = 1088 mV   against a specified 1160..1240
```

The two latency figures, 214 cycles for the break generator and 47 for the
pin, are upper bounds set by the polling loop that watches for `MOE` to drop,
not measurements of the hardware. The response is asynchronous inside the
timer; by the time either loop could look, it had already happened. What the
test establishes is that it happens at all, which nothing had checked before.

### Retracted: there was never an ADC jitter regression

An earlier version of this file recorded the sample jitter growing from 97 ns
to 1389 ns once the interrupt got heavier, and called it unexplained. It was
not real. `adc_cnt_min/max` were live fields that every later and much
heavier test kept writing, so the figure was the entry latency of the cascade
test, not of the ADC test. Scoping the statistics to the window that measures
them, and reading `TIM1->CNT` as the genuinely first statement in the
handler, gives 1 tick.

Two rounds of hypotheses were spent chasing that phantom - the arming
transient, then variable work ahead of the counter read - before the
measurement itself turned out to be the thing at fault.

### Where the 50 us goes

The loop runs in the ADC interrupt at 20 kHz, so one period is 3600 cycles at
72 MHz. The virtual motor is timed separately, because reading a model of a
motor is work that real hardware does in silicon and copper:

```
measured step        1753 cycles   24.35 us   49% of the period
  of which plant      745 cycles   10.35 us
  real control path  1008 cycles   14.00 us   28%
whole ISR            2008 cycles   27.89 us   56%
```

**1008 cycles, 28% of the period**, is the number that matters: Clarke, Park,
two PI controllers with feed-forward decoupling, inverse Park, SVPWM and the
duty write. The earlier figures in this file of 1290 and 1381 cycles included
plant work that had not been separated out yet.

`loop_cycles_min` equals `loop_cycles_max` exactly, so the control path is
branch-free with respect to its data: the same cost every period, which is
what a 20 kHz deadline needs.

### What feed-forward decoupling is worth

Both measured in the same run, same gains, same plant, decoupling switched off
and then on:

| | PI only | + feed-forward |
| --- | --- | --- |
| settling | 125 periods, 6.25 ms | **84 periods, 4.20 ms** |
| peak \|i_d\| | 1999, 24.4% of target | **14, 0.2%** |
| i_d band in steady state | -97 .. +1636 | **-4 .. +4** |
| i_q final error | -0.024% | +0.037% |
| control path | 1290 cycles | 1381 cycles |

91 extra cycles - 1.3 us - buys a third off the settling time and takes the
d-axis excursion from a quarter of the commanded current down to the Q15
quantisation floor.

It also settles an open question from before this was implemented. The d axis
had been showing sparse positive spikes up to +1636, and the guess recorded
here was a numerical artifact in the phase-current round trip at particular
angles. That was wrong. They were the cross-coupling disturbance
`omega*L*i_q`, which the d-axis PI could only chase after the fact; feeding it
forward removes them completely. Nothing in the transform chain was at fault.

### Resolved: where the ADC latency goes

The reported latency mixes a fixed overhead with the conversion time, and
about half of it was unaccounted for. Guessing at the remainder failed twice,
so it was swept instead: seven sample times from 1.5 to 71.5 ADC cycles,
latency measured at each.

```
sample   total cycles   latency    implied fixed overhead
  1.5        14.0         170           86 ticks
  7.5        20.0         206           86
 13.5        26.0         242           86
 28.5        41.0         332           86
 41.5        54.0         408           84
 55.5        68.0         494           86
 71.5        84.0         590           86
```

**Slope: 5.998 timer ticks per ADC cycle**, against 6.000 expected from
72 MHz / 12 MHz. The model is right, so the intercept is the answer:
**86 ticks = 1.190 us of fixed overhead**, identical at 14 and at 84 ADC
cycles. A six-fold change in conversion time moved it by two ticks, which is
what makes it genuinely fixed rather than something that scales with
something else: trigger synchronisation to the ADC clock, interrupt entry,
and the handful of instructions ahead of the counter read.

### Resolved: a console that could delay the control loop

With that model in hand the leftover jitter became obvious. While chasing the
enumeration failure the USB interrupt had been raised to priority 0 - above
the ADC's 1 - and the higher priority stayed behind. USB is busiest during
enumeration, which is exactly when the ADC timing test measures, so a USB
handler preempting the control interrupt was pushing the measured sample
jitter to 148 ticks: 2 us of uncertainty about when the current was sampled.

Putting USB back below the ADC:

```
adc_cnt_min 204, adc_cnt_max 204  ->  jitter 0 ticks
```

Zero. The sample lands on the same counter value every single period. **A
console must never be able to delay a current loop**, and the jitter
measurement is what caught the inversion - the console itself worked
perfectly either way.

### Resolved: this part's internal reference is out of spec

Three instruments were compared, because two of them disagreed by 15%:

| | reading | implies |
| --- | --- | --- |
| VREFINT, taken as 1.20 V nominal | 1346 LSB | VDDA = 3.69 V |
| ST-Link VTREF sense | - | 3.23 V |
| **multimeter on 3V3** | - | **3.313 V** |

The two that agree win. So the supply is 3.313 V and this part's internal
reference is `1346 * 3.313 / 4095 = `**1.088 V**, against a specified 1.16 to
1.24 V - about 7% below the minimum, stable and repeatable across runs.

The ADC setup was verified rather than assumed before blaming the part:
`SMPR1` reads back `0xe00000`, so channel 17 gets 239.5 cycles - 21 us at the
12 MHz ADC clock against the 17.1 us the reference requires - and `SQR3` is
17. Averaging sixteen conversions and discarding the first four moved the
result by 6 LSB, so it is not a settling artifact either.

The device ID registers do not settle whether this is a genuine ST die: the
UID is structured (`0x066cff56 0x54528651 0x87012415`) and `0x20036410` is a
real revision. So the honest claim is the narrow one - *this part's VREFINT is
out of spec* - not a verdict on where the die came from.

**What it changes in the firmware.** `FE_VDDA_MV` in `selftest.c` is the
measured 3313 mV, and the calculation runs in that direction: the supply is
known, the reference is the thing being reported. Current readings have to be
scaled by a measured VDDA. Scaling them by a VREFINT-derived supply would put
a 12% error on every current in the system, and no amount of controller tuning
fixes that, because the controller would be faithfully regulating the wrong
number.

The wider lesson is cheaper than it looks: factory-calibrated values on a
cheap board are worth one cross-check before anything depends on them.

### The console could outrun the watchdog

Two paths in the main loop blocked for longer than the 100 ms watchdog period,
and both reset the board:

- `emit()` waits up to 50 ms for the USB endpoint when the host is not
  draining, and `?` prints ten lines - half a second of waiting;
- `align` from the console calls a self-test routine that sits in
  `delay_ms(300)`.

Both now refresh the watchdog while they wait, which is correct rather than a
cheat: the control loop runs in the ADC interrupt and is unaffected, and
`delay_ms()` is bounded by its own argument, so refreshing inside it cannot
mask code that has stopped returning to the main loop. The `hang` command,
which is a bare infinite loop, still trips the watchdog as it should.

The symptom was ugly to read: `refused: watchdog reset` from a board that had
never hung, because running `align` had reset it and latched the fault. The
monitor's `--check` mode found both.

### Debugging and the watchdog

IWDG has its own clock domain and keeps counting while the core is halted, so
every flash-and-reset cycle - which stops the CPU for far longer than the
100 ms period - produced a watchdog reset on resume. The board then came up
with a latched watchdog fault and refused to arm, purely because it had been
programmed.

`DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP` in `safety_watchdog_start()` freezes
the watchdog whenever a debugger has the core stopped. It does nothing when
no debugger is attached, so it costs nothing in the field. It turned up while
testing the monitor: the first thing the new window reported was
`fault watchdog reset`, on a board that had not hung.

### Known next steps

- The arming rules and the watchdog are verified by the console procedure
  above rather than by a boot-time self-test: a watchdog that resets the chip
  cannot be checked by code running on that chip before the reset.
- Current scaling now has a measured constant to work from, so the shunt
  gain is the only unknown left in the measurement path - and that one needs
  the shunt.
- Everything testable without a motor is now done. What is left - current
  sense scaling, the zero-current offset calibration, the fault recovery
  policy with a real driver - can be written but not verified until a
  three-phase driver and a motor exist, and writing unverifiable firmware is
  what this project has been avoiding.
- The speed step overshoots by 44% (peak 4720 against a commanded 3280).
  Acceptable for a first cut, and the obvious thing to trade against the
  31 ms settling when the gains are tuned properly.
- The decoupling constants here are the plant model's rearranged. On real
  hardware they scale with measured speed, so they become `omega*L` and
  `omega*lambda` computed per period rather than compile-time values.

### SWSTART is not a start bit

Writing `ADC_CR2_SWSTART` does nothing unless the regular group's trigger is
also enabled and selected as software: `EXTSEL = 111` and `EXTTRIG = 1`. With
`EXTTRIG` clear the write is simply ignored, the conversion never happens, and
`DR` holds whatever was there before - which read back as 67 LSB and an
implied supply of 73 volts. The register dump said the channel and the sample
time were right, because they were; the trigger was the part that was not.

### Three bugs the console turned up

**A known-good binary failing the same way is the end of the investigation,
not the start of another hypothesis.** The CDC device would not enumerate.
Several rounds went into the new code - the USB startup delay written as an
empty loop, the interrupt priority, and a reading of `RCC_CFGR` that came
back as zero and looked like the clock configuration had been lost. That last
one was a misread: `0x40021008` is `RCC_CIR`; `CFGR` is at `+0x04`, and the
tick rate measured against the host clock said the core was at 72 MHz all
along. Flashing the *known-good HID firmware* then showed it failing exactly
the same way, which it had not done an hour earlier. The board's own USB
cable was not plugged in. That test should have been the first thing tried,
not the last.

**"Armed" flags that only the tests set.** `foc_loop_step()` ran only while a
self-test armed it, and `outer_tick()` only while the cascade test armed it.
Once the tests finished, nothing drove either: the console could select speed
mode, set a target, and absolutely nothing moved - with every register and
every variable reading exactly as intended. Both now run on every conversion;
`LOOP_MODE_IDLE` and `OUTER_MODE_OFF` are how "do nothing" is expressed,
which is the loop's business and not the interrupt's.

**Command dispatch order.** `spd 3280` begins with `s`, so checking the
single-letter commands first turned every speed command into a request to
print the state - and printed a plausible-looking state line in reply.

### A test that could not fail

Rewriting the ADC initialisation order quietly deleted the block that
initialises the timing statistics, including the flag that enables them. The
measurement then never ran, the spread came out as `0 - 0 = 0`, and the test
reported a pass. Every other field looked right and `ok_flags` read 0x3f.

This is the worst failure in the project so far, because a false pass is
indistinguishable from a real one until something else contradicts it - here,
a reader that printed every field by name and showed six of them sitting at
zero. A test that cannot fail is worse than no test.

### Two real bugs the cascade work turned up

**ADC initialisation order.** Enabling the injected external trigger before
calibrating the ADC lets a trigger arrive in the middle of calibration, and
using an empty `for` loop for the stabilisation delay gives `-Os` something it
is entitled to delete - which it did. The pair of them wedged the ADC in a
state where `JSTRT` was set, `JEOC` never arrived, no interrupt ever reached
the handler, and every register read back exactly as intended. Four tests
failed at once with zeros in every field. The fix is to power up, wait with
the SysTick-based delay that cannot be optimised away, calibrate, and arm the
trigger last.

**Cascade bandwidth, in the wrong order.** The position loop was given a gain
of 0.15, which is a 6.7 ms time constant, sitting outside a speed loop that
needed tens of milliseconds to settle. An outer loop that demands changes
faster than the inner loop can follow is unstable by construction: the
position step ended 118 degrees from where it was sent. At 0.01 - a 100 ms
time constant, several times slower than the speed loop - it settles 0.66
degrees from target.

### How the trigger got there, and the three wrong guesses on the way

The first version used `TRGO = OC4REF` with `CCR4` picking the instant. It
produced 34-39 kHz on a 20 kHz PWM, and the sample instant wandered over
474 ticks (6.6 us). Three hypotheses were tested and discarded before the
real causes showed up, which is the reason this project measures instead of
trusting its own comments:

1. *The JEOC flag is not cleared before the handler returns, so it re-enters.*
   Adding a read-back after the clear made the count go **up**, not down.
   Wrong.
2. *The centre-aligned counter crosses `CCR4` twice per period.* Switching to
   `TRGO = update event` with `RCR = 1` changed nothing at first - the rate
   stayed at 38 kHz. Looks wrong, but see below.
3. *`millis()` is running slow, so the window is not really 100 ms.* Measured
   `tick_ms` against the host clock over 3 s: 1.004x real time. Wrong.

What was actually going on, in two parts:

- `RCR = 1` with `TRGO = update` **did** fix the trigger. Reading `TIM3` as a
  stopwatch inside the ISR showed the interval was already exactly 3600 ticks.
  The rate *looked* unchanged only because of the second problem.
- `adc_isr_count` is a live counter that keeps incrementing after the test
  window closes. Reading the struct over SWD a moment later therefore showed
  a larger number and a fabricated 38 kHz. The fix is `adc_rate_hz`, frozen
  once when the window closes. A snapshot struct must not contain live
  counters.

The residual 3.29 us of latency is larger than sample-and-hold plus
conversion plus interrupt entry accounts for (about 1.84 us at a 12 MHz ADC
clock with 7.5 cycles of sampling). The missing ~1.5 us is not explained yet;
it is constant, so it does not threaten a current loop, but it is not
understood either.

## Before you connect a real gate driver

1. **Re-measure the dead-time** for the driver you actually have. 1 us suits
   slow FETs with a cheap driver; fast parts want less, and a half-bridge with
   a long gate-charge tail wants more. Too little is a dead short across the
   DC bus through both FETs.
2. **Wire an overcurrent comparator to PB12 (BKIN).** `BDTR.BKE` is already
   enabled and the pin is pulled up, so a fault pulling it low kills all six
   outputs in hardware, asynchronously, with no software in the path. That is
   the only thing standing between a software bug and smoke.
3. **Check that `MOE` is clear at reset.** `tim1_init()` configures everything
   but leaves the main output disabled on purpose; only
   `tim1_outputs_enable()` lets anything reach the pins.
4. **Add the rotor alignment step.** Nothing here knows where the rotor's
   magnetic zero is relative to the encoder zero. Commanding torque at the
   wrong angle gives maximum current and no torque.
5. Start with a gimbal motor. 10-15 ohms of winding resistance limits the
   current by itself; a real BLDC under 1 ohm does not forgive a first
   mistake.

## What is deliberately not here yet

No current measurement (the hardware does not exist yet), so no current loop,
no PI controllers, no speed or position loop, and no sensorless estimator.
`foc.c` has the coordinate transforms and SVPWM those loops need, verified in
fixed point, and `tim1.c` has the timing they run on, verified at the pins.

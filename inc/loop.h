/*
 * The current control loop: one call per PWM period.
 *
 * This is the function that will eventually run in the ADC interrupt with
 * real shunt readings and a real encoder. Right now the currents and the
 * angle both come from vmotor.c, which lets the whole chain - sensor, Clarke,
 * Park, two PI controllers, inverse Park, SVPWM and the duty write - be
 * exercised and timed at the real 20 kHz rate before any power electronics
 * exist.
 *
 * The angle the loop uses is the sensor reading minus a stored offset. That
 * offset is not known at reset: align mode finds it.
 */
#ifndef LOOP_H
#define LOOP_H

#include <stdint.h>
#include "foc.h"

typedef enum {
  LOOP_MODE_IDLE = 0,   /* zero volts on all three phases */
  LOOP_MODE_ALIGN,      /* a fixed field at electrical angle zero */
  LOOP_MODE_CURRENT     /* closed current loop on i_d and i_q */
} loop_mode;

/* The voltage used while aligning: enough to hold the rotor against friction,
   far below full scale so a stalled alignment cannot cook anything. */
#define LOOP_ALIGN_V      6000

void     foc_loop_init(void);
void     foc_loop_set_mode(loop_mode mode);

/* Target q-axis current in Q15; d axis is always driven to zero. */
void     foc_loop_set_target(int16_t iq_target);

/*
 * Feed-forward decoupling.
 *
 * The two axes are not independent: the plant pushes omega*L*i_q into the d
 * axis and omega*L*i_d plus the back-EMF omega*lambda into the q axis. A PI
 * controller can only remove those after they have already shown up as
 * error. Feeding the predictable part forward lets the controllers deal with
 * what is left, which is the unpredictable part.
 *
 * Switchable at runtime so the self-test can measure both and compare in one
 * run, rather than trusting a before-and-after across two flashes.
 */
void     foc_loop_set_decouple(int on);

/* The sensor-to-rotor offset that align mode discovers. */
void     foc_loop_set_offset(uint16_t offset);
uint16_t foc_loop_offset(void);

/* One control period. Returns the (d, q) currents it measured. */
foc_dq   foc_loop_step(void);

uint16_t foc_loop_angle(void);

/* The last (d, q) the loop measured, for telemetry. */
foc_dq   foc_loop_last_dq(void);
loop_mode foc_loop_mode(void);

/*
 * Cycles the last foc_loop_step() spent inside vmotor: reading the sensor,
 * reading the phase currents and advancing the plant. All of that is work
 * real hardware does in silicon and copper, so subtracting it from the
 * measured step cost is what gives the real cost of the control path.
 */
uint32_t foc_loop_plant_cycles(void);

#endif /* LOOP_H */

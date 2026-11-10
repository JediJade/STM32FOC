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
  LOOP_MODE_CURRENT,    /* closed current loop on i_d and i_q */
  LOOP_MODE_VOLTAGE     /* a fixed (v_d, v_q), no controller: identification */
} loop_mode;

/* The voltage used while aligning: enough to hold the rotor against friction,
   far below full scale so a stalled alignment cannot cook anything. */
#define LOOP_ALIGN_V      6000

void     foc_loop_init(void);
void     foc_loop_set_mode(loop_mode mode);

/* Target q-axis current in Q15. */
void     foc_loop_set_target(int16_t iq_target);

/*
 * Target d-axis current in Q15, zero unless something asks otherwise.
 *
 * Current on the d axis makes no torque, which is what makes it useful: a
 * known d current can be held while the rotor stays where it is, and the
 * voltage it takes is the winding's resistance with nothing else mixed in.
 */
void     foc_loop_set_target_d(int16_t id_target);

/*
 * The voltage LOOP_MODE_VOLTAGE applies, in the rotor frame, Q15.
 *
 * No controller is involved, on purpose. A step of voltage on the d axis and
 * a record of the current that follows is the winding's time constant - and
 * a time constant measured through a PI controller is the controller's.
 */
void     foc_loop_set_voltage(int16_t vd, int16_t vq);

/*
 * The current controller's gains, Q15. Both axes share one pair.
 *
 * They were compile-time constants while the only plant was the model in
 * vmotor.c. A real motor has its own resistance and inductance, and the
 * gains that suit it have to be found on it and set without a rebuild.
 * Changing them does not touch the integrators.
 */
void     foc_loop_set_gains(int32_t kp, int32_t ki);
void     foc_loop_get_gains(int32_t *kp, int32_t *ki);

/* The two feed-forward constants, Q15: cross-coupling and back-EMF. */
void     foc_loop_set_feedforward(int32_t cross, int32_t back_emf);
void     foc_loop_get_feedforward(int32_t *cross, int32_t *back_emf);

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
int      foc_loop_decouple(void);

/* The sensor-to-rotor offset that align mode discovers. */
void     foc_loop_set_offset(uint16_t offset);
uint16_t foc_loop_offset(void);

/* One control period. Returns the (d, q) currents it measured. */
foc_dq   foc_loop_step(void);

uint16_t foc_loop_angle(void);

/* The last (d, q) the loop measured, for telemetry. */
foc_dq   foc_loop_last_dq(void);

/* The last (v_d, v_q) the loop applied: the controllers' output plus the
   feed-forward, after limiting. Zero in idle and during alignment. */
foc_dq   foc_loop_last_vdq(void);
loop_mode foc_loop_mode(void);

/*
 * Cycles the last foc_loop_step() spent inside vmotor: reading the sensor,
 * reading the phase currents and advancing the plant. All of that is work
 * real hardware does in silicon and copper, so subtracting it from the
 * measured step cost is what gives the real cost of the control path.
 */
uint32_t foc_loop_plant_cycles(void);

#endif /* LOOP_H */

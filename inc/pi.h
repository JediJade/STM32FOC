/*
 * Fixed-point PI controller, one per axis.
 *
 * Gains and signals are Q15. The integrator is kept in Q30 (Q15 scaled by
 * another 2^15) because it has to accumulate errors far smaller than one Q15
 * LSB per sample - an integrator truncated to Q15 simply stops moving once
 * the error gets small, and the loop parks with a permanent offset.
 *
 * The integrator is clamped to the output limit, which is the cheap form of
 * anti-windup: without it, a saturated axis keeps integrating while the
 * output cannot follow, and the controller then takes just as long to come
 * back out of saturation as it spent going in.
 *
 * Gains can be changed while the loop runs. With the gains this firmware was
 * first written with, kp * error and ki * error fit in 32 bits whatever the
 * error. Larger gains do not: a speed Kp of 10 is 327680 in Q15, and times an
 * error of 13000 that is twice what an int32 holds - and an overflow there
 * does not saturate, it wraps, which is full output in the wrong direction.
 *
 * So pi_configure() works out, once, how large an error can be before the
 * output is pinned at its limit anyway, and pi_step() clips the error there
 * before multiplying. The result is exactly what unlimited arithmetic would
 * give, in 32 bits. Gains small enough not to need it take the original
 * path, untouched, so the loop's timing with the built-in gains is what it
 * always was: a 64-bit multiply here cost a hundred cycles per control
 * period, which was enough to fail the loop's own timing test.
 */
#ifndef PI_H
#define PI_H

#include <stdint.h>

typedef struct {
  int32_t kp;       /* Q15 */
  int32_t ki;       /* Q15, already multiplied by the sample time */
  int32_t kd;       /* Q15, output per unit of measurement change */
  int32_t integ;    /* Q30 accumulator */
  int32_t d_prev;   /* the measurement one sample ago */
  int32_t d_filt;   /* the derivative term, filtered, Q15 of the output */
  uint8_t primed;   /* d_prev holds a real sample, not a zero */
  int32_t out_max;  /* Q15 output limit, symmetric */
  int32_t wide;     /* nonzero: the gains need the error clipped first */
  int32_t ep_max;   /* beyond this error the proportional term saturates */
  int32_t ei_max;   /* beyond this error the integrator saturates in one step */
} pi_state;

/* Set gains and limit and clear the integrator. */
void    pi_init(pi_state *s, int32_t kp, int32_t ki, int32_t out_max);

/* Set gains and limit, keep the integrator (clipped to the new limit).
   Not atomic: call it with the control interrupt held off. */
void    pi_configure(pi_state *s, int32_t kp, int32_t ki, int32_t out_max);

/*
 * The derivative gain, in Q15 of output per Q15 of measurement change
 * per sample. Zero - the default - is the plain PI this started as.
 *
 * It acts on the measurement, not on the error: the setpoint here moves
 * in steps and the derivative of a step is an impulse, which would put
 * the output on its limit for one period every time a target changed.
 * For a setpoint that is holding still the two are the same thing.
 */
void    pi_set_kd(pi_state *s, int32_t kd);
int32_t pi_kd(const pi_state *s);

void    pi_reset(pi_state *s);

/* error, and the measurement the error was formed from. */
int16_t pi_step(pi_state *s, int32_t error, int32_t meas);

#endif /* PI_H */

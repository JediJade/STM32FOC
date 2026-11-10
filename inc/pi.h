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
  int32_t integ;    /* Q30 accumulator */
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

void    pi_reset(pi_state *s);
int16_t pi_step(pi_state *s, int32_t error);

#endif /* PI_H */

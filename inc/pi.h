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
 */
#ifndef PI_H
#define PI_H

#include <stdint.h>

typedef struct {
  int32_t kp;       /* Q15 */
  int32_t ki;       /* Q15, already multiplied by the sample time */
  int32_t integ;    /* Q30 accumulator */
  int32_t out_max;  /* Q15 output limit, symmetric */
} pi_state;

void    pi_init(pi_state *s, int32_t kp, int32_t ki, int32_t out_max);
void    pi_reset(pi_state *s);
int16_t pi_step(pi_state *s, int32_t error);

#endif /* PI_H */

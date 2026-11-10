#include "pi.h"

/* Gains up to these never overflow 32 bits, for any error a 16-bit signal
   can produce (|error| <= 65535) and any integrator within its limit:
   32768 * 65535 and 16384 * 65535 + (29491 << 15) are both under 2^31. */
#define PI_NARROW_KP_MAX   32768
#define PI_NARROW_KI_MAX   16384

void pi_configure(pi_state *s, int32_t kp, int32_t ki, int32_t out_max)
{
  int32_t imax = out_max << 15;

  s->kp      = kp;
  s->ki      = ki;
  s->out_max = out_max;

  if (s->integ > imax) {
    s->integ = imax;
  } else if (s->integ < -imax) {
    s->integ = -imax;
  }

  /*
   * The proportional term only matters up to 2 * out_max: the integrator
   * contributes at most out_max the other way, so a proportional term past
   * 2 * out_max pins the output whatever the integrator holds. The few extra
   * counts cover the rounding of the division and of the >> 15.
   *
   *   kp * ep_max <= (2 * out_max + 3 + (kp >> 15)) << 15  <  2^31
   *
   * for every out_max up to 29491 and kp up to 32.0.
   */
  if (kp > 0) {
    s->ep_max = ((2 * out_max + 3 + (kp >> 15)) << 15) / kp;
  } else {
    s->ep_max = 0x7FFFFFFF;
  }

  /*
   * The integrator moves by ki * error and is held within +-imax, so a step
   * of 2 * imax saturates it from anywhere.
   *
   *   ki * ei_max <= 2 * imax + ki  <  2^31
   */
  if (ki > 0) {
    s->ei_max = ((2 * imax) / ki) + 1;
  } else {
    s->ei_max = 0x7FFFFFFF;
  }

  s->wide = ((kp > PI_NARROW_KP_MAX) || (ki > PI_NARROW_KI_MAX)) ? 1 : 0;
}

void pi_init(pi_state *s, int32_t kp, int32_t ki, int32_t out_max)
{
  s->integ = 0;
  pi_configure(s, kp, ki, out_max);
}

void pi_reset(pi_state *s)
{
  s->integ = 0;
}

/* The same controller for gains too large for the plain products. */
static int16_t pi_step_wide(pi_state *s, int32_t error)
{
  int32_t imax = s->out_max << 15;
  int32_t e;
  int32_t step;
  int32_t out;

  /* Integrator: clip the error, then add without ever forming a sum that
     could pass 2^31 - compare against the room left instead. */
  e = error;
  if (e > s->ei_max) {
    e = s->ei_max;
  } else if (e < -s->ei_max) {
    e = -s->ei_max;
  }
  step = s->ki * e;

  if (step > (imax - s->integ)) {
    s->integ = imax;
  } else if (step < (-imax - s->integ)) {
    s->integ = -imax;
  } else {
    s->integ += step;
  }

  /* Proportional: clip the error where the output is pinned regardless. */
  e = error;
  if (e > s->ep_max) {
    e = s->ep_max;
  } else if (e < -s->ep_max) {
    e = -s->ep_max;
  }
  out = ((s->kp * e) >> 15) + (s->integ >> 15);

  if (out > s->out_max) {
    out = s->out_max;
  } else if (out < -s->out_max) {
    out = -s->out_max;
  }

  return (int16_t)out;
}

int16_t pi_step(pi_state *s, int32_t error)
{
  int32_t imax;
  int32_t p;
  int32_t out;

  if (s->wide != 0) {
    return pi_step_wide(s, error);
  }

  /* From here down is the controller exactly as it was first written. */
  imax = s->out_max << 15;

  /* Integrate in Q30 so sub-LSB errors still move the accumulator */
  s->integ += (s->ki * error);

  if (s->integ > imax) {
    s->integ = imax;
  } else if (s->integ < -imax) {
    s->integ = -imax;
  }

  p   = (s->kp * error) >> 15;
  out = p + (s->integ >> 15);

  if (out > s->out_max) {
    out = s->out_max;
  } else if (out < -s->out_max) {
    out = -s->out_max;
  }

  return (int16_t)out;
}

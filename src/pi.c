#include "pi.h"

void pi_init(pi_state *s, int32_t kp, int32_t ki, int32_t out_max)
{
  s->kp      = kp;
  s->ki      = ki;
  s->out_max = out_max;
  s->integ   = 0;
}

void pi_reset(pi_state *s)
{
  s->integ = 0;
}

int16_t pi_step(pi_state *s, int32_t error)
{
  int32_t imax = s->out_max << 15;
  int32_t p;
  int32_t out;

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

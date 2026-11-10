#include "foc.h"

/* Q15 constants */
#define Q15_INV_SQRT3   18918   /* 1/sqrt(3) = 0.577350 */
#define Q15_SQRT3_DIV2  28378   /* sqrt(3)/2 = 0.866025 */

#define ANGLE_QUARTER   16384u  /* 90 degrees in 16-bit angle units */

/* Saturate a 32-bit intermediate back into Q15 instead of wrapping.
   Wrapping here is the classic fixed-point bug: a current spike folds to the
   opposite sign and the PI loop slams the duty to the other rail. */
static inline int16_t q15_sat(int32_t v)
{
  if (v > 32767) {
    return 32767;
  }
  if (v < -32768) {
    return -32768;
  }
  return (int16_t)v;
}

int16_t foc_sin(uint16_t angle)
{
  uint32_t idx  = (uint32_t)(angle >> FOC_SIN_TABLE_SHIFT);
  int32_t  frac = (int32_t)(angle & 0xFFu);
  int32_t  s0   = foc_sin_table[idx];
  int32_t  s1   = foc_sin_table[(idx + 1u) & (FOC_SIN_TABLE_LEN - 1u)];

  /* linear interpolation between table entries */
  return (int16_t)(s0 + (((s1 - s0) * frac) >> FOC_SIN_TABLE_SHIFT));
}

int16_t foc_cos(uint16_t angle)
{
  return foc_sin((uint16_t)(angle + ANGLE_QUARTER));
}

foc_ab foc_clarke(int16_t a, int16_t b)
{
  foc_ab out;

  /* alpha = a
     beta  = (a + 2b) / sqrt(3)
     Both terms are widened to 32-bit first: a + 2b alone can overflow int16. */
  int32_t sum = (int32_t)a + (2 * (int32_t)b);

  out.alpha = a;
  out.beta  = q15_sat((sum * Q15_INV_SQRT3) >> 15);

  return out;
}

foc_dq foc_park(foc_ab in, uint16_t angle)
{
  foc_dq out;
  int16_t s = foc_sin(angle);
  int16_t c = foc_cos(angle);

  /*  d =  alpha*cos + beta*sin
      q = -alpha*sin + beta*cos   */
  out.d = q15_sat((int32_t)q15_mul(in.alpha, c) + (int32_t)q15_mul(in.beta, s));
  out.q = q15_sat(-(int32_t)q15_mul(in.alpha, s) + (int32_t)q15_mul(in.beta, c));

  return out;
}

foc_ab foc_inv_park(foc_dq in, uint16_t angle)
{
  foc_ab out;
  int16_t s = foc_sin(angle);
  int16_t c = foc_cos(angle);

  /*  alpha = d*cos - q*sin
      beta  = d*sin + q*cos   */
  out.alpha = q15_sat((int32_t)q15_mul(in.d, c) - (int32_t)q15_mul(in.q, s));
  out.beta  = q15_sat((int32_t)q15_mul(in.d, s) + (int32_t)q15_mul(in.q, c));

  return out;
}

foc_abc foc_svpwm(foc_ab v, uint16_t period)
{
  foc_abc out;
  int32_t va, vb, vc;
  int32_t vmax, vmin, vcom;
  int32_t half = (int32_t)(period >> 1);

  /* inverse Clarke: two axes back to three phase voltages
       va = alpha
       vb = -alpha/2 + sqrt(3)/2 * beta
       vc = -alpha/2 - sqrt(3)/2 * beta  */
  va = (int32_t)v.alpha;
  vb = -((int32_t)v.alpha >> 1) + (int32_t)q15_mul(v.beta, Q15_SQRT3_DIV2);
  vc = -((int32_t)v.alpha >> 1) - (int32_t)q15_mul(v.beta, Q15_SQRT3_DIV2);

  /* The space vector part: shift all three by the same amount, which the
     motor cannot see (only line-to-line differences matter) but which
     recentres the group and buys ~15% more voltage before clipping. */
  vmax = (va > vb) ? va : vb;
  vmax = (vmax > vc) ? vmax : vc;
  vmin = (va < vb) ? va : vb;
  vmin = (vmin < vc) ? vmin : vc;
  vcom = (vmax + vmin) / 2;

  va -= vcom;
  vb -= vcom;
  vc -= vcom;

  /* Q15 voltage -> timer compare value, centred on 50% duty */
  va = half + ((va * half) >> 15);
  vb = half + ((vb * half) >> 15);
  vc = half + ((vc * half) >> 15);

  /* Clip: a compare value above the period would mean 100% duty forever,
     which with low-side current shunts also means no measurement window. */
  if (va < 0)              { va = 0; }
  if (vb < 0)              { vb = 0; }
  if (vc < 0)              { vc = 0; }
  if (va > (int32_t)period) { va = (int32_t)period; }
  if (vb > (int32_t)period) { vb = (int32_t)period; }
  if (vc > (int32_t)period) { vc = (int32_t)period; }

  out.a = (int16_t)va;
  out.b = (int16_t)vb;
  out.c = (int16_t)vc;

  return out;
}

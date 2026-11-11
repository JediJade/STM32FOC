/*
 * Fixed-point FOC math: Clarke, Park, inverse Park and space vector PWM,
 * all in Q15 because the Cortex-M3 has no FPU.
 *
 * Q15 means a signed 16-bit integer holding a fraction in [-1, 1):
 * 32767 is +0.99997, -32768 is -1.0. Multiplying two Q15 values gives Q30,
 * so the product is shifted right by 15 to get back to Q15 - that single
 * shift is the whole trick, and q15_mul() is where it lives.
 *
 * Angles are uint16 electrical angle: 0..65535 maps to 0..2*pi. Wrapping is
 * just integer overflow, so there is no modulo in the control loop.
 */
#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#define FOC_SIN_TABLE_LEN   256
#define FOC_SIN_TABLE_SHIFT 8      /* 16-bit angle >> 8 = table index */

extern const int16_t foc_sin_table[FOC_SIN_TABLE_LEN];

/* Two-axis quantities. Same struct for currents and voltages. */
typedef struct { int16_t alpha, beta; } foc_ab;   /* stator frame  */
typedef struct { int16_t d, q; }        foc_dq;   /* rotor frame   */
typedef struct { int16_t a, b, c; }     foc_abc;  /* three phases  */

/* sin/cos of a 16-bit electrical angle, Q15, linearly interpolated */
int16_t foc_sin(uint16_t angle);
int16_t foc_cos(uint16_t angle);

/* Q15 multiply: (a * b) >> 15, done in 32-bit so nothing overflows */
static inline int16_t q15_mul(int16_t a, int16_t b)
{
  return (int16_t)(((int32_t)a * (int32_t)b) >> 15);
}

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

/*
 * The sine and cosine of one angle, looked up once.
 *
 * A control period turns currents into the rotor frame and voltages back out
 * of it at the same angle. Park and inverse Park each looked the pair up for
 * themselves, which is the same two table reads done twice - and the motor
 * model did the same again. Looked up once and handed to both, the result is
 * bit for bit what it was, and the period is some hundred cycles shorter,
 * which is what paid for the sensor selector and field weakening.
 */
typedef struct { int16_t s, c; } foc_sc;

static inline foc_sc foc_sincos(uint16_t angle)
{
  foc_sc out;

  out.s = foc_sin(angle);
  out.c = foc_cos(angle);
  return out;
}

static inline foc_dq foc_park_sc(foc_ab in, foc_sc k)
{
  foc_dq out;

  /*  d =  alpha*cos + beta*sin
      q = -alpha*sin + beta*cos   */
  out.d = q15_sat((int32_t)q15_mul(in.alpha, k.c) + (int32_t)q15_mul(in.beta, k.s));
  out.q = q15_sat(-(int32_t)q15_mul(in.alpha, k.s) + (int32_t)q15_mul(in.beta, k.c));
  return out;
}

static inline foc_ab foc_inv_park_sc(foc_dq in, foc_sc k)
{
  foc_ab out;

  /*  alpha = d*cos - q*sin
      beta  = d*sin + q*cos   */
  out.alpha = q15_sat((int32_t)q15_mul(in.d, k.c) - (int32_t)q15_mul(in.q, k.s));
  out.beta  = q15_sat((int32_t)q15_mul(in.d, k.s) + (int32_t)q15_mul(in.q, k.c));
  return out;
}

/* 3 phase -> 2 axis. Assumes a + b + c == 0, so only a and b are needed. */
foc_ab foc_clarke(int16_t a, int16_t b);

/* stator frame -> rotor frame (the transform that turns AC into DC) */
foc_dq foc_park(foc_ab in, uint16_t angle);

/* rotor frame -> stator frame */
foc_ab foc_inv_park(foc_dq in, uint16_t angle);

/*
 * Space vector PWM: two-axis voltage -> three duty cycles in [0, period].
 *
 * Plain sinusoidal PWM wastes headroom because it centres each phase on
 * period/2 independently. SVPWM instead shifts all three phases by the same
 * amount - half of (max + min) - which keeps the line-to-line voltages
 * identical while pulling the whole group away from the rails. That buys
 * 2/sqrt(3) = 1.1547, so about 15% more usable voltage from the same DC bus.
 */
foc_abc foc_svpwm(foc_ab v, uint16_t period);

#endif /* FOC_H */

#include "vmotor.h"
#include "foc.h"

/*
 * Plant constants, representative of a small gimbal motor.
 *
 * Angles are the same 16-bit electrical units the rest of the firmware uses,
 * and speed is kept as angle units per sample shifted left by 8, so that
 * accelerations far below one unit per sample still accumulate.
 *
 *   electrical time constant L/R = 1 ms against a 50 us sample -> a = 0.05
 *   cross-coupling c = omega * dt, which in these units is omega * 2*pi/65536
 *   back-EMF = ke * omega
 *   acceleration = k * i_q, friction proportional to omega
 */
#define VM_A          1638      /* Q15, dt / (L/R) = 0.05 */
#define VM_KE           20      /* back-EMF per unit of speed */
#define VM_ACCEL       336      /* i_q (Q15) -> speed change */
#define VM_FRICTION     30      /* Q15 fraction of speed lost per sample */

#define SPEED_SHIFT      8      /* speed is angle-units-per-sample << 8 */

static int32_t  i_d;            /* Q15, in the REAL rotor frame */
static int32_t  i_q;
static int32_t  omega_fx;       /* angle units per sample, << SPEED_SHIFT */
static uint16_t theta;          /* real electrical angle */
static vm_mech  mech;

static int32_t clamp_q15(int32_t v)
{
  if (v > 32767) {
    return 32767;
  }
  if (v < -32768) {
    return -32768;
  }
  return v;
}

void vmotor_reset(void)
{
  i_d      = 0;
  i_q      = 0;
  omega_fx = 0;
  /* Start somewhere arbitrary: the alignment procedure has to work from an
     unknown rotor position, which is the whole point of it. */
  theta    = 20000u;
  mech     = VM_MECH_FREE;
}

void vmotor_set_mech(vm_mech mode, int32_t omega_step)
{
  mech = mode;

  if (mode == VM_MECH_LOCKED) {
    omega_fx = 0;
  } else if (mode == VM_MECH_FORCED) {
    omega_fx = omega_step << SPEED_SHIFT;
  }
}

void vmotor_step(int16_t v_alpha, int16_t v_beta)
{
  foc_ab  v_ab;
  foc_dq  v_dq;
  int32_t omega_step;
  int32_t c;
  int32_t d_new;
  int32_t q_new;

  /* Stator frame -> the rotor's REAL frame. The controller's estimate of the
     angle plays no part here; this is the motor's own view. */
  v_ab.alpha = v_alpha;
  v_ab.beta  = v_beta;
  v_dq = foc_park(v_ab, theta);

  omega_step = omega_fx >> SPEED_SHIFT;

  /* c = omega * 2*pi/65536 in Q15 is omega_step * 3.1416 */
  c = (omega_step * 3217) >> 10;

  d_new = i_d
        + ((VM_A * ((int32_t)v_dq.d - i_d)) >> 15)
        + ((c * i_q) >> 15);

  q_new = i_q
        + ((VM_A * ((int32_t)v_dq.q - i_q)) >> 15)
        - ((c * i_d) >> 15)
        - ((VM_A * (VM_KE * omega_step)) >> 15);

  i_d = clamp_q15(d_new);
  i_q = clamp_q15(q_new);

  /* Mechanics: torque is whatever current landed on the real q axis */
  if (mech == VM_MECH_FREE) {
    omega_fx += ((VM_ACCEL * i_q) >> 15);
    omega_fx -= ((VM_FRICTION * omega_fx) >> 15);
  }

  if (mech != VM_MECH_LOCKED) {
    theta = (uint16_t)((int32_t)theta + (omega_fx >> SPEED_SHIFT));
  }
}

void vmotor_phase_currents(int16_t *ia, int16_t *ib)
{
  foc_dq  dq;
  foc_ab  ab;
  int32_t a;
  int32_t b;

  dq.d = (int16_t)i_d;
  dq.q = (int16_t)i_q;

  /* The real rotor frame back out to the stator, then to phase currents:
       ia = alpha
       ib = -alpha/2 + sqrt(3)/2 * beta   */
  ab = foc_inv_park(dq, theta);

  a = ab.alpha;
  b = -((int32_t)ab.alpha >> 1) + (int32_t)q15_mul(ab.beta, 28378);

  *ia = (int16_t)clamp_q15(a);
  *ib = (int16_t)clamp_q15(b);
}

uint16_t vmotor_sensor_angle(void)
{
  return (uint16_t)(theta + VMOTOR_ENCODER_OFFSET);
}

uint16_t vmotor_true_angle(void)
{
  return theta;
}

int32_t vmotor_true_speed(void)
{
  return omega_fx;
}

foc_dq vmotor_true_dq(void)
{
  foc_dq out;

  out.d = (int16_t)i_d;
  out.q = (int16_t)i_q;
  return out;
}

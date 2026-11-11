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

/*
 * The constants in use. They start as the gimbal motor above, and stay that
 * way through the power-on self-tests, which were written against it.
 *
 * The second set is shaped like a hub motor: five times the electrical time
 * constant, five times the back-EMF, a heavier rotor, and rolling resistance
 * - a constant drag - in place of most of the viscous friction. It exists
 * because some things cannot be shown on the first: with L/R at 1 ms the
 * winding is nearly all resistance at any speed the model reaches, and
 * weakening the field of a motor like that buys almost nothing.
 */
static int32_t  vm_a     = VM_A;
static int32_t  vm_ke    = VM_KE;
static int32_t  vm_accel = VM_ACCEL;
static int32_t  vm_fric  = VM_FRICTION;
static int32_t  vm_load  = 0;   /* speed lost per sample to a constant drag */

static int32_t  i_d;            /* Q15, in the REAL rotor frame */
static int32_t  i_q;
static int32_t  omega_fx;       /* angle units per sample, << SPEED_SHIFT */
static uint16_t theta;          /* real electrical angle */
static foc_sc   theta_sc;       /* its sine and cosine, kept in step with it */
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
  theta_sc = foc_sincos(theta);
  mech     = VM_MECH_FREE;
}

void vmotor_set_preset(int preset)
{
  if (preset == 1) {
    vm_a     = 328;             /* L/R = 5 ms */
    vm_ke    = 100;
    vm_accel = 40;
    vm_fric  = 1;
    vm_load  = 1;
  } else {
    vm_a     = VM_A;
    vm_ke    = VM_KE;
    vm_accel = VM_ACCEL;
    vm_fric  = VM_FRICTION;
    vm_load  = 0;
  }
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
  v_dq = foc_park_sc(v_ab, theta_sc);

  omega_step = omega_fx >> SPEED_SHIFT;

  /* c = omega * 2*pi/65536 in Q15 is omega_step * 3.1416 */
  c = (omega_step * 3217) >> 10;

  d_new = i_d
        + ((vm_a * ((int32_t)v_dq.d - i_d)) >> 15)
        + ((c * i_q) >> 15);

  q_new = i_q
        + ((vm_a * ((int32_t)v_dq.q - i_q)) >> 15)
        - ((c * i_d) >> 15)
        - ((vm_a * (vm_ke * omega_step)) >> 15);

  i_d = clamp_q15(d_new);
  i_q = clamp_q15(q_new);

  /* Mechanics: torque is whatever current landed on the real q axis */
  if (mech == VM_MECH_FREE) {
    omega_fx += ((vm_accel * i_q) >> 15);
    omega_fx -= ((vm_fric * omega_fx) >> 15);

    if (vm_load != 0) {
      if (omega_fx > vm_load) {
        omega_fx -= vm_load;
      } else if (omega_fx < -vm_load) {
        omega_fx += vm_load;
      } else {
        omega_fx = 0;           /* not enough torque to break away */
      }
    }
  }

  if (mech != VM_MECH_LOCKED) {
    theta = (uint16_t)((int32_t)theta + (omega_fx >> SPEED_SHIFT));
    theta_sc = foc_sincos(theta);
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
  ab = foc_inv_park_sc(dq, theta_sc);

  a = ab.alpha;
  b = -((int32_t)ab.alpha >> 1) + (int32_t)q15_mul(ab.beta, 28378);

  *ia = (int16_t)clamp_q15(a);
  *ib = (int16_t)clamp_q15(b);
}

uint16_t vmotor_sensor_angle(void)
{
  return (uint16_t)(theta + VMOTOR_ENCODER_OFFSET);
}

uint8_t vmotor_hall(uint16_t width)
{
  uint8_t  code = 0u;
  uint16_t at   = (uint16_t)(theta + VMOTOR_HALL_PHASE);

  if (at < width)                         { code |= 1u; }
  if ((uint16_t)(at - 21845u) < width)    { code |= 2u; }
  if ((uint16_t)(at - 43690u) < width)    { code |= 4u; }
  return code;
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

#include "stm32f103xb.h"
#include "loop.h"
#include "foc.h"
#include "pi.h"
#include "tim1.h"
#include "vmotor.h"

/*
 * Gains. Kp is deliberately modest and Ki carries the steady state: the
 * plant has a 1 ms electrical time constant against a 50 us sample time, so
 * there is plenty of phase margin, and the back-EMF is a constant
 * disturbance that only the integrator can remove.
 *
 * out_max is 0.9 of full scale rather than 1.0, so SVPWM never asks for a
 * duty of exactly 0 or 100%. At 100% a low-side shunt has no window in which
 * to measure anything, which on real hardware means the loop goes blind at
 * precisely the moment it is working hardest.
 */
#define LOOP_KP        6000    /* Q15, 0.18 */
#define LOOP_KI         900    /* Q15, 0.027 per sample */
#define LOOP_OUT_MAX  29491    /* Q15, 0.90 */

/*
 * Decoupling constants.
 *
 * These are the plant's own terms rearranged: holding i_d at zero needs
 * v_d = -(c/a)*i_q, and holding i_q at the target needs
 * v_q = i_q + (c/a)*i_d + back_emf, where a and c are vmotor.c's decay and
 * cross-coupling gains at the speed the current loop is characterised at.
 * On real hardware they are not constants - they come from omega, L, R and
 * the flux constant, so both terms scale with measured speed.
 */
#define LOOP_DECOUPLE_CA   20580   /* Q15, c/a = 1029/1638 = 0.628 */
#define LOOP_BACK_EMF      6554    /* Q15, omega*lambda = 0.20     */

/*
 * The values in use. They start as the constants above, so a board that is
 * never told otherwise behaves exactly as it did when they were #defines -
 * including through the power-on self-tests, which run before any console.
 */
static int32_t  gain_kp  = LOOP_KP;
static int32_t  gain_ki  = LOOP_KI;
static int32_t  ff_cross = LOOP_DECOUPLE_CA;
static int32_t  ff_emf   = LOOP_BACK_EMF;

static pi_state pi_d;
static pi_state pi_q;
static uint16_t angle;
static uint16_t offset;
static int16_t  target_q;
static int16_t  target_d;
static foc_dq   v_cmd;
static int      decouple;
static loop_mode mode;
static uint32_t plant_cycles;
static foc_dq   last_dq;
static foc_dq   last_vdq;

void foc_loop_init(void)
{
  pi_init(&pi_d, gain_kp, gain_ki, LOOP_OUT_MAX);
  pi_init(&pi_q, gain_kp, gain_ki, LOOP_OUT_MAX);
  angle    = 0u;
  target_q = 0;
  target_d = 0;
  v_cmd.d  = 0;
  v_cmd.q  = 0;
  mode     = LOOP_MODE_IDLE;
  vmotor_reset();
}

void foc_loop_set_mode(loop_mode m)
{
  mode = m;
}

void foc_loop_set_target(int16_t iq_target)
{
  target_q = iq_target;
}

void foc_loop_set_target_d(int16_t id_target)
{
  target_d = id_target;
}

void foc_loop_set_voltage(int16_t vd, int16_t vq)
{
  v_cmd.d = vd;
  v_cmd.q = vq;
}

void foc_loop_set_gains(int32_t kp, int32_t ki)
{
  gain_kp = kp;
  gain_ki = ki;
  pi_configure(&pi_d, kp, ki, LOOP_OUT_MAX);
  pi_configure(&pi_q, kp, ki, LOOP_OUT_MAX);
}

void foc_loop_get_gains(int32_t *kp, int32_t *ki)
{
  *kp = gain_kp;
  *ki = gain_ki;
}

void foc_loop_set_feedforward(int32_t cross, int32_t back_emf)
{
  ff_cross = cross;
  ff_emf   = back_emf;
}

void foc_loop_get_feedforward(int32_t *cross, int32_t *back_emf)
{
  *cross    = ff_cross;
  *back_emf = ff_emf;
}

void foc_loop_set_decouple(int on)
{
  decouple = on;
}

int foc_loop_decouple(void)
{
  return decouple;
}

void foc_loop_set_offset(uint16_t off)
{
  offset = off;
}

uint16_t foc_loop_offset(void)
{
  return offset;
}

uint16_t foc_loop_angle(void)
{
  return angle;
}

uint32_t foc_loop_plant_cycles(void)
{
  return plant_cycles;
}

foc_dq foc_loop_last_dq(void)
{
  return last_dq;
}

foc_dq foc_loop_last_vdq(void)
{
  return last_vdq;
}

loop_mode foc_loop_mode(void)
{
  return mode;
}

foc_dq foc_loop_step(void)
{
  int16_t ia;
  int16_t ib;
  foc_ab  i_ab;
  foc_dq  i_dq;
  foc_ab  v_ab;
  foc_abc duty;

  /* 1. where the rotor is, as far as this firmware knows: the sensor reading
        corrected by the offset that alignment found. Before alignment the
        offset is zero and this angle is simply wrong, which is exactly the
        situation the alignment procedure exists to fix. */
  uint32_t plant_c0 = DWT->CYCCNT;

  angle = (uint16_t)(vmotor_sensor_angle() - offset);

  /* 2. measure: two phase currents. On real hardware this is where the two
        injected ADC results would be read instead. */
  vmotor_phase_currents(&ia, &ib);

  plant_cycles = DWT->CYCCNT - plant_c0;

  /* 3. three phase -> two axis -> rotor frame */
  i_ab = foc_clarke(ia, ib);
  i_dq = foc_park(i_ab, angle);

  switch (mode) {
  case LOOP_MODE_ALIGN:
    /* A fixed field at electrical angle zero in the stator frame. The rotor
       pulls itself into line with it, which puts the real rotor angle at
       zero and leaves the sensor reading equal to the offset. No feedback is
       involved: this is open-loop on purpose, because the angle cannot be
       trusted until it is done. */
    v_ab.alpha = LOOP_ALIGN_V;
    v_ab.beta  = 0;
    last_vdq.d = 0;
    last_vdq.q = 0;
    break;

  case LOOP_MODE_CURRENT: {
    int32_t vd = pi_step(&pi_d, (int32_t)target_d - (int32_t)i_dq.d);
    int32_t vq = pi_step(&pi_q, (int32_t)target_q - (int32_t)i_dq.q);
    foc_dq  v_dq;

    if (decouple != 0) {
      /* Hand the controllers the part of the plant that is predictable */
      vd -= (int32_t)q15_mul((int16_t)i_dq.q, (int16_t)ff_cross);
      vq += (int32_t)q15_mul((int16_t)i_dq.d, (int16_t)ff_cross);
      vq += ff_emf;
    }

    if (vd >  LOOP_OUT_MAX) { vd =  LOOP_OUT_MAX; }
    if (vd < -LOOP_OUT_MAX) { vd = -LOOP_OUT_MAX; }
    if (vq >  LOOP_OUT_MAX) { vq =  LOOP_OUT_MAX; }
    if (vq < -LOOP_OUT_MAX) { vq = -LOOP_OUT_MAX; }

    v_dq.d = (int16_t)vd;
    v_dq.q = (int16_t)vq;

    last_vdq = v_dq;
    v_ab = foc_inv_park(v_dq, angle);
    break;
  }

  case LOOP_MODE_VOLTAGE:
    /* The commanded voltage, straight through. The console has already
       limited it; nothing here second-guesses the number. */
    last_vdq = v_cmd;
    v_ab = foc_inv_park(v_cmd, angle);
    break;

  case LOOP_MODE_IDLE:
  default:
    v_ab.alpha = 0;
    v_ab.beta  = 0;
    last_vdq.d = 0;
    last_vdq.q = 0;
    break;
  }

  /* 4. two axis voltage -> three duty cycles */
  duty = foc_svpwm(v_ab, TIM1_PERIOD);
  tim1_set_duty(duty);

  /* 5. advance the plant with the voltages just applied. On real hardware
        this step is the motor itself and costs nothing. */
  plant_c0 = DWT->CYCCNT;
  vmotor_step(v_ab.alpha, v_ab.beta);
  plant_cycles += (DWT->CYCCNT - plant_c0);

  last_dq = i_dq;
  return i_dq;
}

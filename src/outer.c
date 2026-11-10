#include "stm32f103xb.h"
#include "outer.h"
#include "pi.h"
#include "loop.h"
#include "foc.h"

/*
 * Gains.
 *
 * The mechanical pole is slow: friction takes the plant's speed down with a
 * time constant of about 55 ms, against a 1 ms update. So the speed loop can
 * afford real integral gain, and the position loop above it is proportional
 * only - an integrator there would wind up every time the speed loop hits its
 * current limit, and position error integrates into a runaway rather than a
 * correction.
 */
#define SPEED_KP      32768   /* Q15, 1.00 */
#define SPEED_KI       6144   /* Q15, 0.1875 per 1 ms tick */
#define SPEED_IQ_MAX  16384   /* Q15, hold the current demand to half scale */

/*
 * Position gain, and the reason it is small.
 *
 * A proportional position loop driving a speed reference behaves as a first
 * order system with time constant 1/POS_KP in milliseconds. At 0.15 that is
 * 6.7 ms, against a speed loop that needs tens of milliseconds to settle -
 * the outer loop demanding changes faster than the inner one can follow,
 * which is unstable by construction, not by bad luck. Measured: the position
 * step ended 118 degrees away from where it was sent.
 *
 * 0.01 gives a 100 ms time constant, several times slower than the measured
 * speed-loop settling, which is the separation a cascade needs.
 */
#define POS_KP          328   /* Q15, 0.01 -> about a 100 ms time constant */

static pi_state  pi_speed;
static outer_mode mode;
static uint16_t  prev_angle;
static int       have_prev;
static uint8_t   divider;

static int32_t   speed_meas;
static int32_t   speed_ref;
static int32_t   iq_ref;
static uint16_t  position_ref;
static uint32_t  cycles;

void outer_init(void)
{
  pi_init(&pi_speed, SPEED_KP, SPEED_KI, SPEED_IQ_MAX);
  mode         = OUTER_MODE_OFF;
  have_prev    = 0;
  divider      = 0u;
  speed_meas   = 0;
  speed_ref    = 0;
  iq_ref       = 0;
  position_ref = 0u;
  cycles       = 0u;
}

void outer_set_mode(outer_mode m)
{
  mode = m;
  pi_reset(&pi_speed);
}

void outer_set_speed_ref(int32_t units_per_ms)
{
  speed_ref = units_per_ms;
}

void outer_set_position_ref(uint16_t angle)
{
  position_ref = angle;
}

void outer_tick(uint16_t measured_angle)
{
  uint32_t c0;

  if (mode == OUTER_MODE_OFF) {
    return;
  }

  divider++;
  if (divider < OUTER_DIVIDER) {
    return;
  }
  divider = 0u;

  c0 = DWT->CYCCNT;

  /* Speed by differentiating the angle. The int16 cast is what makes the
     wrap-around work: 65535 -> 1 is +2, not -65534. */
  if (have_prev != 0) {
    speed_meas = (int32_t)(int16_t)(measured_angle - prev_angle);
  }
  prev_angle = measured_angle;
  have_prev  = 1;

  /* Position loop, proportional only, output limited to a sane speed */
  if (mode == OUTER_MODE_POSITION) {
    int32_t perr = (int32_t)(int16_t)(position_ref - measured_angle);
    int32_t sref = (POS_KP * perr) >> 15;

    if (sref >  OUTER_SPEED_MAX) { sref =  OUTER_SPEED_MAX; }
    if (sref < -OUTER_SPEED_MAX) { sref = -OUTER_SPEED_MAX; }
    speed_ref = sref;
  }

  /* Speed loop -> current reference for the loop running at 20 kHz */
  iq_ref = (int32_t)pi_step(&pi_speed, speed_ref - speed_meas);
  foc_loop_set_target((int16_t)iq_ref);

  cycles = DWT->CYCCNT - c0;
}

outer_mode outer_mode_get(void) { return mode; }
int32_t  outer_speed(void)     { return speed_meas; }
int32_t  outer_speed_ref(void) { return speed_ref; }
int32_t  outer_iq_ref(void)    { return iq_ref; }
uint32_t outer_cycles(void)    { return cycles; }

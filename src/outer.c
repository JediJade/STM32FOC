#include "stm32f103xb.h"
#include "outer.h"
#include "pi.h"
#include "loop.h"
#include "foc.h"
#include "sensor.h"

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

/* The values in use. They start as the constants above, so nothing changes
   for a board that is never told otherwise, self-tests included. */
static int32_t   gain_kp     = SPEED_KP;
static int32_t   gain_ki     = SPEED_KI;
static int32_t   gain_iq_max = SPEED_IQ_MAX;
static int32_t   gain_pos_kp = POS_KP;

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
static uint8_t   use_sensor;    /* speed comes from sensor.c, not the angle */

void outer_init(void)
{
  pi_init(&pi_speed, gain_kp, gain_ki, gain_iq_max);
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

  divider++;
  if (divider < OUTER_DIVIDER) {
    return;
  }
  divider = 0u;

  c0 = DWT->CYCCNT;

  /*
   * Speed by differentiating the angle. The int16 cast is what makes the
   * wrap-around work: 65535 -> 1 is +2, not -65534.
   *
   * Measured every millisecond, whatever the mode. It used to sit below the
   * mode check, so it ran only while a speed or position loop was closed -
   * and in idle and in current mode outer_speed() went on returning the last
   * value it had. The console reported 2640 for a rotor that had stopped,
   * and anything that wanted to watch a rotor accelerate under a known
   * current had no speed to watch. It also left prev_angle stale across an
   * idle spell, so the first sample of the next speed command was the angle
   * travelled since the previous one, wrapped: one tick of nonsense fed
   * straight into the speed controller.
   */
  if (have_prev != 0) {
    speed_meas = (int32_t)(int16_t)(measured_angle - prev_angle);
  }
  prev_angle = measured_angle;
  have_prev  = 1;

  /* An angle put together from Hall edges, or from an observer, is not
     smooth enough to differentiate over one millisecond. What produced it
     has a better idea of the speed. */
  if (use_sensor != 0u) {
    speed_meas = sensor_speed();
  }

  if (mode == OUTER_MODE_OFF) {
    return;
  }

  /* Position loop, proportional only, output limited to a sane speed */
  if (mode == OUTER_MODE_POSITION) {
    int32_t perr = (int32_t)(int16_t)(position_ref - measured_angle);
    int32_t sref = (gain_pos_kp * perr) >> 15;

    if (sref >  OUTER_SPEED_MAX) { sref =  OUTER_SPEED_MAX; }
    if (sref < -OUTER_SPEED_MAX) { sref = -OUTER_SPEED_MAX; }
    speed_ref = sref;
  }

  /* Speed loop -> current reference for the loop running at 20 kHz */
  iq_ref = (int32_t)pi_step(&pi_speed, speed_ref - speed_meas);
  foc_loop_set_target((int16_t)iq_ref);

  cycles = DWT->CYCCNT - c0;
}

void outer_set_gains(int32_t speed_kp, int32_t speed_ki, int32_t iq_max)
{
  gain_kp     = speed_kp;
  gain_ki     = speed_ki;
  gain_iq_max = iq_max;
  pi_configure(&pi_speed, speed_kp, speed_ki, iq_max);
}

void outer_get_gains(int32_t *speed_kp, int32_t *speed_ki, int32_t *iq_max)
{
  *speed_kp = gain_kp;
  *speed_ki = gain_ki;
  *iq_max   = gain_iq_max;
}

void outer_set_pos_kp(int32_t kp)
{
  gain_pos_kp = kp;
}

int32_t outer_pos_kp(void)
{
  return gain_pos_kp;
}

void outer_use_sensor_speed(int on)
{
  use_sensor = (uint8_t)((on != 0) ? 1u : 0u);
}

uint8_t outer_phase(void)
{
  return divider;
}

void outer_hold(void)
{
  pi_reset(&pi_speed);
}

outer_mode outer_mode_get(void) { return mode; }
int32_t  outer_speed(void)     { return speed_meas; }
int32_t  outer_speed_ref(void) { return speed_ref; }
int32_t  outer_iq_ref(void)    { return iq_ref; }
uint32_t outer_cycles(void)    { return cycles; }

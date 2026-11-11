#include "stm32f103xb.h"
#include "drive.h"
#include "loop.h"
#include "outer.h"
#include "vmotor.h"
#include "sensor.h"
#include "capture.h"

#define BRAKE_FADE      300     /* regen fades out below this speed, units per ms */
#define THROTTLE_BAD_MS 50u     /* out of range this long is a fault, not a glitch */

int32_t drive_p[DP_COUNT] = {
  [DP_VLIM]    = 29491,
  [DP_FW]      = 0,
  [DP_FWSTART] = 29491,
  [DP_SENS]    = 0,
  [DP_HSRC]    = 0,
  [DP_HWID]    = 32768,
  [DP_SLA]     = 1638,
  [DP_SLMIN]   = 600,
  [DP_SLCUR]   = 4000,
  [DP_SLRAMP]  = 2000,
  [DP_INSRC]   = 0,
  [DP_THMIN]   = 900,
  [DP_THMAX]   = 3300,
  [DP_THMODE]  = 0,
  [DP_THRAMP]  = 66,
  [DP_DRVSPD]  = 6560,
  [DP_REGEN]   = 4096,
  [DP_MOT]     = 0,
  [DP_OL]      = 0,
  [DP_OLT]     = 10,
  [DP_CANID]   = 1,
  [DP_CANBR]   = 500,
  [DP_CANPER]  = 20,
  [DP_CANBACK] = 10,
};

static volatile uint8_t veh;
static volatile uint8_t vflt;
static volatile int32_t th_sim = 900;
static volatile uint8_t brk_sim;
static int32_t  th_raw = 900;
static uint8_t  brake;
static int32_t  th_f;
static uint32_t th_bad;
static uint8_t  limit;
static int32_t  idfw_fx;
static int32_t  th_adc;

/* ---------------------------------------------------------- the pins ---- */

void drive_init(void)
{
  /* PB5, PB6, PB7: the Hall sensors. 3144 and 3143 outputs are open
     collector, so they need the pull-up. PB11: the brake switch, to ground. */
  GPIOB->CRL  = (GPIOB->CRL & ~0xFFF00000u) | 0x88800000u;
  GPIOB->BSRR = (7u << 5) | (1u << 11);

  /* PB10: the one-line output, held low by a pull-down until it is enabled. */
  GPIOB->CRH  = (GPIOB->CRH & ~0x0000FF00u) | 0x00008800u;
  GPIOB->BRR  = (1u << 10);

  veh     = VEH_OFF;
  vflt    = 0u;
  th_sim  = drive_p[DP_THMIN];
  brk_sim = 0u;
}

/*
 * The throttle is a regular conversion on ADC1, started by software once a
 * millisecond and read the next. The injected conversions the loop runs on
 * interrupt it and it resumes, so neither disturbs the other. None of this
 * is set up until the pins are asked for: a board that never is has an ADC
 * configured exactly as the self-tests left it.
 */
static void throttle_adc_setup(void)
{
  GPIOA->CRL  &= ~(0xFu << 8);                      /* PA2 analog */
  ADC1->SMPR2  = (ADC1->SMPR2 & ~ADC_SMPR2_SMP2) | (6u << ADC_SMPR2_SMP2_Pos);
  ADC1->SQR1   = 0u;
  ADC1->SQR3   = 2u;
  ADC1->CR2   |= ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;  /* trigger: SWSTART */
}

/* -------------------------------------------------------- commands ---- */

static void go_idle(void)
{
  outer_set_mode(OUTER_MODE_OFF);
  foc_loop_set_mode(LOOP_MODE_IDLE);
  foc_loop_set_target(0);
  foc_loop_set_target_d(0);
  foc_loop_set_voltage(0, 0);
}

static void leave_vehicle(void)
{
  if (veh != VEH_FAULT) {
    veh = VEH_OFF;
  }
  th_f = 0;
  sensor_cal_abort();
}

int drive_idle(void)
{
  return ((foc_loop_mode() == LOOP_MODE_IDLE) &&
          (outer_mode_get() == OUTER_MODE_OFF) &&
          ((veh == VEH_OFF) || (veh == VEH_FAULT)) &&
          (sensor_cal_running() == 0)) ? 1 : 0;
}

void drive_off(void)
{
  leave_vehicle();
  go_idle();
}

void drive_torque(int32_t iq)
{
  if (iq >  16384) { iq =  16384; }
  if (iq < -16384) { iq = -16384; }
  leave_vehicle();
  outer_set_mode(OUTER_MODE_OFF);
  vmotor_set_mech(VM_MECH_FREE, 0);
  foc_loop_set_mode(LOOP_MODE_CURRENT);
  foc_loop_set_target((int16_t)iq);
  capture_trigger();
}

void drive_speed(int32_t v)
{
  if (v >  OUTER_SPEED_MAX) { v =  OUTER_SPEED_MAX; }
  if (v < -OUTER_SPEED_MAX) { v = -OUTER_SPEED_MAX; }
  leave_vehicle();
  vmotor_set_mech(VM_MECH_FREE, 0);
  foc_loop_set_target_d(0);
  foc_loop_set_mode(LOOP_MODE_CURRENT);
  outer_set_mode(OUTER_MODE_SPEED);
  outer_set_speed_ref(v);
  capture_trigger();
}

void drive_position(int32_t delta)
{
  leave_vehicle();
  vmotor_set_mech(VM_MECH_FREE, 0);
  foc_loop_set_target_d(0);
  foc_loop_set_mode(LOOP_MODE_CURRENT);
  outer_set_mode(OUTER_MODE_POSITION);
  outer_set_position_ref((uint16_t)(foc_loop_angle() + (uint16_t)delta));
}

void drive_voltage(int32_t vd)
{
  /* Half scale at most. This is an open-loop voltage: on a real winding
     the current it makes is v / R with nothing watching, so the ceiling
     is here, in the one place the number comes in. */
  if (vd >  16384) { vd =  16384; }
  if (vd < -16384) { vd = -16384; }
  leave_vehicle();
  outer_set_mode(OUTER_MODE_OFF);
  vmotor_set_mech(VM_MECH_FREE, 0);
  foc_loop_set_voltage((int16_t)vd, 0);
  foc_loop_set_mode(LOOP_MODE_VOLTAGE);
  capture_trigger();
}

int drive_vehicle_on(void)
{
  if (sensor_cal_running() != 0) {
    return 1;
  }
  if (veh == VEH_FAULT) {
    return 2;
  }
  go_idle();
  vmotor_set_mech(VM_MECH_FREE, 0);
  th_f   = 0;
  th_bad = 0u;
  veh    = VEH_WAIT;
  return 0;
}

void drive_clear(void)
{
  vflt   = 0u;
  th_bad = 0u;
  sensor_clear_errors();
  if (veh == VEH_FAULT) {
    veh = VEH_OFF;
  }
}

void drive_sim_throttle(int32_t counts)
{
  if (counts < 0)    { counts = 0; }
  if (counts > 4095) { counts = 4095; }
  th_sim = counts;
}

void drive_sim_brake(int on)
{
  brk_sim = (uint8_t)((on != 0) ? 1u : 0u);
}

/* ------------------------------------------------------ parameters ---- */

int32_t drive_param_set(drive_param id, int32_t v)
{
  switch (id) {
  case DP_VLIM:
    foc_loop_set_vlim(v);
    break;

  case DP_SENS:
    /* Changing where the angle comes from under a running loop would hand
       it a different angle from one period to the next. */
    if ((drive_idle() == 0) || ((v == SENS_HALL) && (sensor_hall_valid() == 0))) {
      return drive_p[id];
    }
    drive_p[id] = v;
    sensor_select((int)v);
    return v;

  case DP_MOT:
    if (drive_idle() == 0) {
      return drive_p[id];
    }
    vmotor_set_preset((int)v);
    break;

  case DP_CANBR:
    v = (v < 188) ? 125 : ((v < 375) ? 250 : ((v < 750) ? 500 : 1000));
    break;

  case DP_THMIN:
    if (v > drive_p[DP_THMAX]) { v = drive_p[DP_THMAX]; }
    break;

  case DP_THMAX:
    if (v < drive_p[DP_THMIN]) { v = drive_p[DP_THMIN]; }
    break;

  case DP_INSRC:
    if (v != 0) {
      throttle_adc_setup();
    }
    break;

  case DP_OL:
    /* output push-pull 2 MHz, or back to an input with the pull-down */
    GPIOB->BRR = (1u << 10);
    GPIOB->CRH = (GPIOB->CRH & ~0x00000F00u) | ((v != 0) ? 0x00000200u : 0x00000800u);
    break;

  default:
    break;
  }

  drive_p[id] = v;
  sensor_derive();
  return v;
}

/* ------------------------------------------- once a millisecond ---- */

static void enter_run(void)
{
  veh  = VEH_RUN;
  th_f = 0;
  foc_loop_set_target_d(0);
  foc_loop_set_target(0);
  foc_loop_set_mode(LOOP_MODE_CURRENT);
  if (drive_p[DP_THMODE] != 0) {
    outer_set_speed_ref(0);
    outer_set_mode(OUTER_MODE_SPEED);
  } else {
    outer_set_mode(OUTER_MODE_OFF);
  }
}

static void vehicle_tick(void)
{
  int32_t raw;
  int32_t span;
  int32_t dead;
  int32_t den;
  int32_t num;
  int32_t pos = 0;
  int32_t speed;

  if (drive_p[DP_INSRC] != 0) {
    if ((ADC1->SR & ADC_SR_EOC) != 0u) {
      th_adc = (int32_t)(ADC1->DR & 0xFFFu);
    }
    ADC1->CR2 |= ADC_CR2_SWSTART;
    raw   = th_adc;
    brake = (uint8_t)(((GPIOB->IDR & (1u << 11)) == 0u) ? 1u : 0u);
  } else {
    raw   = th_sim;
    brake = brk_sim;
  }
  th_raw = raw;

  if ((veh == VEH_OFF) || (veh == VEH_FAULT)) {
    return;
  }

  /* The two ends, with the bottom thirty-second of the span ignored so a
     throttle that does not quite return to its stop still reads zero. */
  span = drive_p[DP_THMAX] - drive_p[DP_THMIN];
  dead = span >> 5;
  den  = span - dead;
  num  = raw - drive_p[DP_THMIN] - dead;
  if ((den >= 16) && (num > 0)) {
    pos = (num * 32767) / den;
    if (pos > 32767) { pos = 32767; }
  }

  /*
   * A Hall throttle idles well above zero volts and tops out well below the
   * supply. A reading outside that is not a throttle position, it is a wire:
   * open, or shorted to one rail. Half way from each end to its rail is the
   * line, and it has to stay over it for a while to count.
   */
  if ((raw < (drive_p[DP_THMIN] >> 1)) ||
      (raw > (drive_p[DP_THMAX] + ((4095 - drive_p[DP_THMAX]) >> 1)))) {
    th_bad++;
    if (th_bad >= THROTTLE_BAD_MS) {
      veh  = VEH_FAULT;
      vflt = 1u;
      th_f = 0;
      go_idle();
      return;
    }
  } else {
    th_bad = 0u;
  }

  limit = 0u;
  speed = outer_speed();

  switch (veh) {
  case VEH_WAIT:
    /* Not until the throttle has been seen closed. Switching on with it
       open must not be the same as opening it. */
    if (pos == 0) {
      enter_run();
    }
    break;

  case VEH_RUN:
    if (brake != 0u) {
      veh  = VEH_BRAKE;
      th_f = 0;
      outer_set_mode(OUTER_MODE_OFF);
      foc_loop_set_target(0);
    } else {
      /* Up at a limited rate, down at once. */
      if (pos > th_f) {
        th_f += drive_p[DP_THRAMP];
        if (th_f > pos) { th_f = pos; }
      } else {
        th_f = pos;
      }

      if (drive_p[DP_THMODE] != 0) {
        outer_set_speed_ref((th_f * drive_p[DP_DRVSPD]) >> 15);
      } else {
        int32_t iq;
        int32_t over = speed - drive_p[DP_DRVSPD];

        outer_get_gains(&span, &dead, &iq);         /* iq = simax */
        iq = (th_f * iq) >> 15;
        if (over > 0) {
          /* Past the top speed the torque is taken away over the next 256
             units, not all at once. */
          limit = 1u;
          iq = (over < 256) ? ((iq * (256 - over)) >> 8) : 0;
        }
        foc_loop_set_target((int16_t)iq);
      }
    }
    break;

  case VEH_BRAKE:
    if (brake == 0u) {
      enter_run();
    } else {
      int32_t a  = (speed < 0) ? -speed : speed;
      int32_t iq;

      if (a > BRAKE_FADE) { a = BRAKE_FADE; }
      iq = (drive_p[DP_REGEN] * a) / BRAKE_FADE;
      foc_loop_set_target((int16_t)((speed > 0) ? -iq : iq));
    }
    break;

  default:
    break;
  }
}

static uint32_t isqrt32(uint32_t v)
{
  uint32_t r = 0u;
  uint32_t b = 1u << 30;

  while (b > v) {
    b >>= 2;
  }
  while (b != 0u) {
    if (v >= (r + b)) {
      v -= r + b;
      r  = (r >> 1) + b;
    } else {
      r >>= 1;
    }
    b >>= 2;
  }
  return r;
}

/*
 * Field weakening.
 *
 * Above its base speed a motor's back-EMF is the whole of the voltage there
 * is, and the current loop has nothing left to push current with. Negative
 * d-axis current makes a voltage of its own, omega * L * i_d, that opposes
 * the back-EMF. So: watch how much of the voltage limit the loop is using,
 * and when it goes past `fwstart`, raise that current until it comes back.
 * An integrator on the difference, kept between zero and `fw`.
 *
 * It costs current that makes no torque, and on a winding that is mostly
 * resistance at the speed in question it costs more voltage than it frees.
 * The model's first motor is like that. Its second is not.
 */
static void weaken_tick(void)
{
  if ((drive_p[DP_FW] > 0) && (foc_loop_mode() == LOOP_MODE_CURRENT)) {
    foc_dq   v   = foc_loop_last_vdq();
    int32_t  mag = (int32_t)isqrt32((uint32_t)((int32_t)v.d * v.d) + (uint32_t)((int32_t)v.q * v.q));
    int32_t  thr = (drive_p[DP_FWSTART] * drive_p[DP_VLIM]) >> 15;
    int32_t  lo  = -(drive_p[DP_FW] << 8);

    idfw_fx += (thr - mag) * 32;
    if (idfw_fx > 0)  { idfw_fx = 0; }
    if (idfw_fx < lo) { idfw_fx = lo; }
  } else {
    idfw_fx = 0;
  }
  foc_loop_set_fw_d((int16_t)(idfw_fx >> 8));
}

void drive_tick(void)
{
  /* outer_tick() has just run. Its divider is back at zero on the period in
     which it did its 1 kHz work, and this follows in the same one. */
  if (outer_phase() != 0u) {
    return;
  }

  vehicle_tick();
  sensor_tick();
  weaken_tick();
  foc_loop_ff_speed(foc_loop_decouple() == 2, outer_speed());
}

/* -------------------------------------------------------- reporting ---- */

int32_t drive_throttle_raw_sim(void) { return th_sim; }
int32_t drive_throttle_raw(void) { return th_raw; }
int32_t drive_throttle(void)     { return th_f; }
int     drive_brake(void)        { return (int)brake; }
int     drive_state(void)        { return (int)veh; }
int     drive_fault(void)        { return (int)vflt; }
int     drive_limit(void)        { return (int)limit; }
int16_t drive_fw_d(void)         { return (int16_t)(idfw_fx >> 8); }

int drive_regen(void)
{
  return ((veh == VEH_BRAKE) && (drive_p[DP_REGEN] > 0)) ? 1 : 0;
}

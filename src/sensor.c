#include "stm32f103xb.h"
#include "sensor.h"
#include "loop.h"
#include "outer.h"
#include "vmotor.h"
#include "drive.h"

/* Angles and speeds are kept with eight more bits than the loop uses, so a
   speed of a fraction of a unit per period still moves the angle. */
#define FX              8

#define HALL_TIMEOUT    4000u               /* periods without an edge: stopped */

#define CAL_W           (8 << FX)           /* the field turns 8 units a period */
#define CAL_TURN        (98304u << FX)      /* one and a half turns each way */
#define CAL_RAMP        64                  /* change of field speed per ms */

#define PLL_KP          4
#define PLL_KI          10
#define PLL_W_MAX       (1000 << FX)

static uint8_t   kind;

/* ---- Hall */
static uint8_t   h_valid;
static uint16_t  h_edge[8];     /* where each state begins, going forward */
static uint16_t  h_wid[8];      /* how wide it is; 0 = this state never occurs */
static uint8_t   h_next[8];     /* the state after it, going forward */
static uint8_t   h_code;
static uint32_t  h_cnt;         /* periods since the last edge */
static uint32_t  h_cnt1;        /* periods the sector before took; 0 = not known */
static uint32_t  h_wid1;        /* and how wide that one was */
static int8_t    h_dir;
static uint32_t  h_ang_fx;
static int32_t   h_spd_fx;      /* what the angle is carried forward at */
static int32_t   h_rep_fx;      /* what the speed loop is told */
static uint32_t  h_trav;        /* how far the angle has been carried since the edge */
static uint32_t  h_lim;         /* and how far it may be: to the next edge */
static uint32_t  h_err;

/* ---- calibration */
static volatile uint8_t cal;
static uint8_t   cal_result;
static uint32_t  cal_ms;
static int32_t   cal_w_fx;
static uint32_t  cal_trav;
static uint32_t  phi_fx;        /* the forced angle: calibration, and open-loop start */
static uint16_t  f_edge[8];
static uint16_t  r_edge[8];
static uint8_t   f_next[8];
static uint8_t   f_mask;
static uint8_t   r_mask;

/* ---- observer */
static uint8_t   sl_closed;
static int32_t   wol_fx;        /* the forced speed */
static uint32_t  pll_th_fx;
static int32_t   pll_w_fx;
static int8_t    pll_dir = 1;
static uint32_t  sl_lock;
static int32_t   pia, pib, pva, pvb;
static int32_t   inva;          /* 1 / a, with eight fraction bits */
static int32_t   knorm;         /* back-EMF -> speed, Q15 */

uint8_t sensor_hall_now(void)
{
  if (drive_p[DP_HSRC] != 0) {
    return (uint8_t)((GPIOB->IDR >> 5) & 7u);       /* PB5, PB6, PB7 */
  }
  return vmotor_hall((uint16_t)drive_p[DP_HWID]);
}

void sensor_derive(void)
{
  int32_t cross;
  int32_t emf;
  int32_t a = drive_p[DP_SLA];

  if (a < 128) { a = 128; }
  inva = (32768 << FX) / a;

  /* cemf is the back-EMF at LOOP_FF_SPEED, which is 328 units a period.
     Dividing by it turns a back-EMF into the speed that made it. */
  foc_loop_get_feedforward(&cross, &emf);
  if (emf < 64) { emf = 64; }
  knorm = (328 << 15) / emf;
  if (knorm > 32767) { knorm = 32767; }
}

uint16_t sensor_angle(void)
{
  if (cal != 0u) {
    return (uint16_t)(phi_fx >> FX);
  }
  if (kind == SENS_HALL) {
    return (uint16_t)(h_ang_fx >> FX);
  }
  if (sl_closed != 0u) {
    /* The estimate is a period old by the time it is used. */
    return (uint16_t)((pll_th_fx + (uint32_t)pll_w_fx) >> FX);
  }
  return (uint16_t)(phi_fx >> FX);
}

int32_t sensor_speed(void)
{
  if (kind == SENS_HALL) {
    return (h_rep_fx * 20) >> FX;
  }
  if (sl_closed != 0u) {
    return (pll_w_fx * 20) >> FX;
  }
  return (wol_fx * 20) >> FX;
}

/* Start from the middle of whatever sector the rotor is in. */
static void hall_seed(void)
{
  uint8_t code = sensor_hall_now();

  h_code   = code;
  h_cnt    = 0u;
  h_cnt1   = 0u;
  h_dir    = 0;
  h_spd_fx = 0;
  h_rep_fx = 0;
  h_trav   = 0u;
  h_lim    = 0u;
  h_ang_fx = (uint32_t)(uint16_t)(h_edge[code] + (h_wid[code] >> 1)) << FX;
}

static void hall_step(void)
{
  uint8_t code = sensor_hall_now();

  if (code != h_code) {
    uint8_t old = h_code;

    if (h_wid[code] == 0u) {
      /* A state that calibration never saw: a wire off, or noise. Hold the
         angle and count it. */
      h_err++;
      h_spd_fx = 0;
      h_rep_fx = 0;
      h_dir    = 0;
      h_trav   = 0u;
      h_lim    = 0u;
      h_cnt1   = 0u;
    } else {
      int8_t   d = 0;
      uint16_t boundary = 0u;

      if (h_wid[old] != 0u) {
        if (h_next[old] == code) {
          d = 1;
          boundary = h_edge[code];
        } else if (h_next[code] == old) {
          d = -1;
          boundary = h_edge[old];
        } else {
          h_err++;                      /* skipped a state */
        }
      }

      if ((d != 0) && (d == h_dir) && (h_cnt > 0u) && (h_cnt < HALL_TIMEOUT)) {
        /*
         * The sector just left was crossed in h_cnt periods - and the one
         * before it in h_cnt1. Two together, because a time is only known
         * to the period, and a unipolar switch makes every other sector
         * narrow: 28 degrees at speed is some thirty periods, three percent
         * a count, and that error is then carried across the 92 degrees
         * that follow. Over both it is under one percent.
         */
        uint32_t wid = h_wid[old];
        uint32_t cnt = h_cnt;

        /*
         * The speed loop gets the last sector alone. It can live with a few
         * percent of ripple, and what it cannot live with is delay: a speed
         * that is two sectors old is, at low speed, ten milliseconds old,
         * and the loop rings on it.
         */
        h_rep_fx = d * (int32_t)((wid << FX) / cnt);

        if (h_cnt1 != 0u) {
          wid += h_wid1;
          cnt += h_cnt1;
        }
        h_spd_fx = d * (int32_t)((wid << FX) / cnt);

        /* The edge fell somewhere inside the last period: half a period
           past it, on average. */
        h_ang_fx = ((uint32_t)boundary << FX) + (uint32_t)(h_spd_fx >> 1);
        h_cnt1 = h_cnt;
        h_wid1 = h_wid[old];
      } else {
        /* First edge, or the first after a reversal: the speed is not known
           yet, and the middle of the sector is at most half of it wrong. */
        h_spd_fx = 0;
        h_rep_fx = 0;
        h_ang_fx = (uint32_t)(uint16_t)(h_edge[code] + (h_wid[code] >> 1)) << FX;
        h_cnt1 = 0u;
      }
      h_trav = 0u;
      h_lim  = (uint32_t)h_wid[code] << FX;
      h_dir  = d;
    }
    h_code = code;
    h_cnt  = 0u;
    return;
  }

  if (h_cnt < 60000u) {
    h_cnt++;
  }
  if (h_spd_fx != 0) {
    if (h_cnt > HALL_TIMEOUT) {
      /* Standing still. The angle may have been carried to the far end of
         the sector while the rotor stopped at the near end - a whole sector
         out, which with wide sectors is no torque at all, and a rotor that
         then never moves again. The middle is at most half a sector out. */
      h_spd_fx = 0;
      h_rep_fx = 0;
      h_ang_fx = (uint32_t)(uint16_t)(h_edge[h_code] + (h_wid[h_code] >> 1)) << FX;
    } else {
      uint32_t step = (uint32_t)((h_spd_fx < 0) ? -h_spd_fx : h_spd_fx);

      /* Overdue: at this speed the next edge should have come by now, so
         the rotor is slowing down. */
      if (h_cnt > (h_lim / step)) {
        h_spd_fx = h_dir * (int32_t)(h_lim / h_cnt);
        h_rep_fx = h_spd_fx;
        step = (uint32_t)((h_spd_fx < 0) ? -h_spd_fx : h_spd_fx);
      }
      /* Never past the next edge: the sensors have not said so. */
      if ((h_trav + step) <= h_lim) {
        h_ang_fx += (uint32_t)h_spd_fx;
        h_trav   += step;
      }
    }
  }
}

static void cal_step(void)
{
  uint8_t code;

  phi_fx   += (uint32_t)cal_w_fx;
  cal_trav += (uint32_t)((cal_w_fx < 0) ? -cal_w_fx : cal_w_fx);

  code = sensor_hall_now();
  if (code != h_code) {
    uint16_t phi = (uint16_t)(phi_fx >> FX);

    if ((cal == 2u) && (cal_w_fx > 0)) {
      /* forward: this is where the new state begins */
      f_edge[code]   = phi;
      f_next[h_code] = code;
      f_mask |= (uint8_t)(1u << code);
    } else if ((cal == 4u) && (cal_w_fx < 0)) {
      /* backward: the same edge is where the old state begins */
      r_edge[h_code] = phi;
      r_mask |= (uint8_t)(1u << h_code);
    }
    h_code = code;
  }
}

static void observer_step(foc_ab i, foc_ab v)
{
  int32_t  da = (int32_t)i.alpha - pia;
  int32_t  db = (int32_t)i.beta  - pib;
  int32_t  ea;
  int32_t  eb;
  int32_t  err;
  int32_t  errn;
  uint16_t th;

  if (da >  16383) { da =  16383; }
  if (da < -16383) { da = -16383; }
  if (db >  16383) { db =  16383; }
  if (db < -16383) { db = -16383; }

  /* What the last period's voltage should have done to the current, and did
     not: that is the back-EMF. */
  ea = pva - pia - ((da * inva) >> FX);
  eb = pvb - pib - ((db * inva) >> FX);
  pia = i.alpha;
  pib = i.beta;
  pva = v.alpha;
  pvb = v.beta;

  if (ea >  32767) { ea =  32767; }
  if (ea < -32767) { ea = -32767; }
  if (eb >  32767) { eb =  32767; }
  if (eb < -32767) { eb = -32767; }

  /* The back-EMF is E * (-sin, cos) of the rotor angle. Against the
     estimate's own (cos, sin) that leaves E * sin(real - estimate). */
  th  = (uint16_t)(pll_th_fx >> FX);
  err = -((ea * (int32_t)foc_cos(th) + eb * (int32_t)foc_sin(th)) >> 15);

  /* E changes sign with the direction of rotation, and with it the side
     the loop would settle on. */
  if (pll_dir < 0) {
    err = -err;
  }
  if (err >  32767) { err =  32767; }
  if (err < -32768) { err = -32768; }

  errn = (err * knorm) >> 11;           /* in speed units, times 16 */

  pll_w_fx += (PLL_KI * errn) >> 4;
  if (pll_w_fx >  PLL_W_MAX) { pll_w_fx =  PLL_W_MAX; }
  if (pll_w_fx < -PLL_W_MAX) { pll_w_fx = -PLL_W_MAX; }
  pll_th_fx += (uint32_t)(pll_w_fx + (PLL_KP * errn * 16));

  if (sl_closed == 0u) {
    phi_fx += (uint32_t)wol_fx;
  } else if (pll_w_fx > (5 << FX)) {
    pll_dir = 1;
  } else if (pll_w_fx < -(5 << FX)) {
    pll_dir = -1;
  }
}

void sensor_update(foc_ab i, foc_ab v)
{
  if (cal != 0u) {
    cal_step();
  } else if (kind == SENS_HALL) {
    hall_step();
  } else {
    observer_step(i, v);
  }
}

/* After the source changes, or calibration ends: start the estimators from
   scratch and tell the loops whose angle and speed to use. */
static void apply(void)
{
  uint32_t now = (uint32_t)foc_loop_angle() << FX;

  if (kind == SENS_HALL) {
    hall_seed();
  }
  sl_closed = 0u;
  wol_fx    = 0;
  phi_fx    = now;
  pll_th_fx = now;
  pll_w_fx  = 0;
  sl_lock   = 0u;
  pia = 0;
  pib = 0;
  pva = 0;
  pvb = 0;
  foc_loop_set_q_boost(0);
  foc_loop_set_ext_angle(kind != SENS_ENCODER);
  outer_use_sensor_speed(kind != SENS_ENCODER);
}

void sensor_select(int k)
{
  kind = (uint8_t)k;
  apply();
}

int sensor_kind_get(void)
{
  return (int)kind;
}

void sensor_init(void)
{
  kind = SENS_ENCODER;
  cal  = 0u;
  sensor_derive();
}

/* ------------------------------------------------------- calibration ---- */

int sensor_cal_start(void)
{
  if (drive_idle() == 0) {
    return 1;
  }

  __disable_irq();
  cal_ms     = 0u;
  cal_w_fx   = 0;
  cal_trav   = 0u;
  cal_result = 0u;
  phi_fx     = 0u;
  f_mask     = 0u;
  r_mask     = 0u;
  h_code     = sensor_hall_now();

  /* The same field alignment uses, on the d axis of an angle this module
     turns: the rotor lines up with it and follows. */
  vmotor_set_mech(VM_MECH_FREE, 0);
  foc_loop_set_voltage(LOOP_ALIGN_V, 0);
  outer_use_sensor_speed(0);
  foc_loop_set_ext_angle(1);
  cal = 1u;
  foc_loop_set_mode(LOOP_MODE_VOLTAGE);
  __enable_irq();
  return 0;
}

static void cal_end(uint8_t result)
{
  cal_result = result;
  cal = 0u;
  foc_loop_set_mode(LOOP_MODE_IDLE);
  foc_loop_set_voltage(0, 0);
  apply();
}

void sensor_cal_abort(void)
{
  if (cal != 0u) {
    cal_end(SENSOR_CAL_STOPPED);
  }
}

static void cal_finish(void)
{
  uint8_t mask = f_mask;
  uint8_t result = SENSOR_CAL_OK;
  uint8_t bits = 0u;
  uint8_t c;

  for (c = 0u; c < 8u; c++) {
    if ((mask & (1u << c)) != 0u) {
      bits++;
    }
  }

  if (mask != r_mask) {
    result = SENSOR_CAL_MISMATCH;
  } else if ((bits != 6u) || ((mask & 0x81u) != 0u)) {
    result = SENSOR_CAL_NOT_SIX;
  } else {
    uint8_t first = 1u;
    uint8_t seen = 0u;
    uint8_t k;

    while ((mask & (1u << first)) == 0u) {
      first++;
    }
    c = first;
    for (k = 0u; k < 6u; k++) {
      seen |= (uint8_t)(1u << c);
      c = f_next[c] & 7u;
    }
    if ((c != first) || (seen != mask)) {
      result = SENSOR_CAL_NO_CYCLE;
    }
  }

  if (result == SENSOR_CAL_OK) {
    for (c = 0u; c < 8u; c++) {
      if ((mask & (1u << c)) != 0u) {
        /* Half way between where the field was at this edge going forward
           and going back. The rotor trails the field by the same angle both
           ways, in opposite directions. */
        int16_t diff = (int16_t)(r_edge[c] - f_edge[c]);

        h_edge[c] = (uint16_t)(f_edge[c] + (uint16_t)(diff >> 1));
        h_next[c] = f_next[c];
      } else {
        h_next[c] = 0u;
        h_wid[c]  = 0u;
      }
    }
    for (c = 0u; c < 8u; c++) {
      if ((mask & (1u << c)) != 0u) {
        h_wid[c] = (uint16_t)(h_edge[h_next[c]] - h_edge[c]);
      }
    }
    h_valid = 1u;
    h_err   = 0u;
  }
  cal_end(result);
}

static void cal_tick(void)
{
  int32_t target = 0;

  cal_ms++;
  switch (cal) {
  case 1u:                                /* let the rotor settle on the field */
    if (cal_ms >= 200u) {
      cal = 2u;
      cal_trav = 0u;
    }
    break;
  case 2u:                                /* forward */
    target = CAL_W;
    if (cal_trav >= CAL_TURN) {
      cal = 3u;
      cal_ms = 0u;
    }
    break;
  case 3u:
    if (cal_ms >= 150u) {
      cal = 4u;
      cal_trav = 0u;
    }
    break;
  case 4u:                                /* and back */
    target = -CAL_W;
    if (cal_trav >= CAL_TURN) {
      cal = 5u;
      cal_ms = 0u;
    }
    break;
  default:
    if (cal_ms >= 100u) {
      cal_finish();
      return;
    }
    break;
  }

  if (cal_w_fx < target) {
    cal_w_fx += CAL_RAMP;
    if (cal_w_fx > target) { cal_w_fx = target; }
  } else if (cal_w_fx > target) {
    cal_w_fx -= CAL_RAMP;
    if (cal_w_fx < target) { cal_w_fx = target; }
  }
}

int sensor_cal_running(void)
{
  return (cal != 0u) ? 1 : 0;
}

int sensor_cal_result(void)
{
  return (int)cal_result;
}

/* ---------------------------------------------------------- observer ---- */

/*
 * Once a millisecond: who supplies the angle, the forced one or the
 * observer, and what the forced one should be doing.
 */
static void observer_tick(void)
{
  int32_t hand;
  int32_t iq_cmd;

  if (foc_loop_mode() != LOOP_MODE_CURRENT) {
    sl_closed = 0u;
    wol_fx    = 0;
    sl_lock   = 0u;
    foc_loop_set_q_boost(0);
    return;
  }

  hand   = (drive_p[DP_SLMIN] << FX) / 20;
  iq_cmd = foc_loop_target();

  if (sl_closed == 0u) {
    int32_t tgt;
    int32_t dirn;
    int32_t inc;

    if (outer_mode_get() != OUTER_MODE_OFF) {
      /* A speed is being asked for: go there, as far as the forced angle
         is allowed to go. */
      tgt = (outer_speed_ref() * (1 << FX)) / 20;
      if (tgt >  hand) { tgt =  hand; }
      if (tgt < -hand) { tgt = -hand; }
      dirn = (tgt > 0) - (tgt < 0);
      if (dirn == 0) {
        dirn = (wol_fx > 0) - (wol_fx < 0);
      }
      /* The speed that loop sees is the one being forced here. Letting it
         integrate that would hand over a wound-up controller. */
      outer_hold();
    } else {
      /* A torque is being asked for: turn the way it pushes. */
      tgt  = (iq_cmd > 200) ? hand : ((iq_cmd < -200) ? -hand : 0);
      dirn = (tgt > 0) - (tgt < 0);
    }

    inc = (drive_p[DP_SLRAMP] * 839) >> 16;
    if (inc < 1) { inc = 1; }
    if (wol_fx < tgt) {
      wol_fx += inc;
      if (wol_fx > tgt) { wol_fx = tgt; }
    } else if (wol_fx > tgt) {
      wol_fx -= inc;
      if (wol_fx < tgt) { wol_fx = tgt; }
    }

    if (dirn != 0) {
      /* Whatever was asked for, the current is the starting current, along
         the forced angle, the way the rotor is to turn. */
      foc_loop_set_q_boost((int16_t)((dirn * drive_p[DP_SLCUR]) - iq_cmd));
      pll_dir = (int8_t)dirn;
    } else {
      foc_loop_set_q_boost(0);
    }

    /* Hand over once the forced speed is up and the observer has been
       following a rotor that turns the same way, fast enough. */
    if ((tgt != 0) && (wol_fx == tgt) && ((tgt >= hand) || (tgt <= -hand)) &&
        ((pll_w_fx * dirn) > (hand - (hand >> 2)))) {
      sl_lock++;
    } else {
      sl_lock = 0u;
    }
    if (sl_lock >= 20u) {
      sl_closed = 1u;
      sl_lock   = 0u;
      foc_loop_set_q_boost(0);
    }
  } else {
    int32_t w = (pll_w_fx < 0) ? -pll_w_fx : pll_w_fx;

    if (w < (hand >> 1)) {
      /* Too slow to trust: back to the forced angle, from where it is. */
      sl_closed = 0u;
      phi_fx    = (uint32_t)foc_loop_angle() << FX;
      wol_fx    = pll_w_fx;
      sl_lock   = 0u;
    }
  }
}

void sensor_tick(void)
{
  if (cal != 0u) {
    cal_tick();
  } else if (kind == SENS_OBSERVER) {
    observer_tick();
  }
}

/* -------------------------------------------------------- reporting ---- */

int sensor_hall_valid(void)
{
  return (int)h_valid;
}

int32_t sensor_hall_edge(int code)
{
  if ((h_valid == 0u) || (h_wid[code & 7] == 0u)) {
    return -1;
  }
  return (int32_t)h_edge[code & 7];
}

/*
 * The learned table, for store.c.
 *
 * Only the edges and the ring are handed over. The widths are not: they are
 * the differences between neighbouring edges, so storing them as well would
 * be storing the same six numbers twice and leaving room for a record whose
 * widths and edges disagree.
 */
int sensor_hall_table_get(uint16_t *edge, uint8_t *next)
{
  int c;

  for (c = 0; c < 8; c++) {
    edge[c] = h_edge[c];
    next[c] = (h_wid[c] != 0u) ? h_next[c] : 0u;
  }
  return (int)h_valid;
}

/*
 * Put one back. A state whose `next` is zero never occurs - code 0 and code
 * 7 are all-off and all-on, which no working set of three sensors produces,
 * so zero cannot be a real successor and needs no separate flag.
 *
 * Returns 0 if it was taken. The caller holds the control interrupt off.
 */
int sensor_hall_table_set(const uint16_t *edge, const uint8_t *next)
{
  int      c;
  int      seen = 0;
  uint16_t ring = 0u;

  for (c = 1; c < 7; c++) {
    if (next[c] != 0u) {
      if ((next[c] > 6u) || (next[c] == (uint8_t)c)) {
        return -1;
      }
      ring |= (uint16_t)(1u << next[c]);
      seen++;
    }
  }
  /* Six states, each the successor of exactly one other: the ring the
     calibration found. Anything else is not a table this code can use. */
  if ((seen != 6) || (ring != 0x7Eu)) {
    return -1;
  }

  for (c = 0; c < 8; c++) {
    h_edge[c] = edge[c];
    h_next[c] = next[c];
    h_wid[c]  = 0u;
  }
  for (c = 0; c < 8; c++) {
    if (h_next[c] != 0u) {
      h_wid[c] = (uint16_t)(h_edge[h_next[c]] - h_edge[c]);
    }
  }
  h_valid = 1u;
  h_err   = 0u;
  if (kind == SENS_HALL) {
    hall_seed();              /* the angle in flight was built on the old table */
  }
  return 0;
}

uint32_t sensor_hall_errors(void)
{
  return h_err;
}

void sensor_clear_errors(void)
{
  h_err = 0u;
}

int sensor_phase(void)
{
  if (cal != 0u) {
    return SENSOR_PH_CAL;
  }
  if (kind == SENS_HALL) {
    return SENSOR_PH_HALL;
  }
  if (kind == SENS_OBSERVER) {
    return (sl_closed != 0u) ? SENSOR_PH_CLOSED : SENSOR_PH_OPEN;
  }
  return SENSOR_PH_NONE;
}

int16_t sensor_angle_error(void)
{
  /* The model has moved on by one period since the loop used its angle. */
  return (int16_t)(foc_loop_angle() - vmotor_true_angle()
                   + (uint16_t)(vmotor_true_speed() >> 8));
}

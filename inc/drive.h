/*
 * The layer between "a loop that takes a target" and "a thing with a
 * throttle on it".
 *
 * Everything below this was built to be commanded from a console: a speed,
 * a current, a position, typed in. A vehicle is commanded by a throttle and
 * a brake lever, and it has to behave when the throttle wire falls off. That
 * is what lives here, with the two other things that only make sense at
 * this level:
 *
 *   throttle     an analog input (PA2), or a number sent from the console in
 *                its place. Calibrated between two ends, with a dead band,
 *                a ramp on the way up, and a fault if the reading leaves the
 *                range a working throttle can produce. The drive will not
 *                start until it has seen the throttle released.
 *   brake        a switch to ground (PB11), or the console. It always wins:
 *                the throttle is cut and the motor regenerates, fading out
 *                near standstill so that it never drives backwards.
 *   weakening    when the loop is about to run out of voltage, negative
 *                d-axis current is added so the motor can go on speeding up.
 *
 * It runs once a millisecond in the control interrupt, after the speed loop,
 * and it commands the loops through the same calls the console uses.
 *
 * The parameters this layer, sensor.c, oneline.c and canbus.c use are all
 * here, in one array, so the console and the CAN side reach them by number.
 */
#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

typedef enum {
  DP_VLIM = 0,    /* voltage each axis may use, Q15                        */
  DP_FW,          /* most field-weakening current, Q15; 0 = off            */
  DP_FWSTART,     /* weakening starts at this fraction of vlim, Q15        */
  DP_SENS,        /* 0 encoder, 1 Hall, 2 observer                         */
  DP_HSRC,        /* Hall source: 0 the model's sensors, 1 pins PB5..PB7   */
  DP_HWID,        /* how much of a turn a model Hall sensor stays on       */
  DP_SLA,         /* observer: dt / (L/R), Q15                             */
  DP_SLMIN,       /* observer takes over above this speed, units per ms    */
  DP_SLCUR,       /* current while the rotor is on the forced angle, Q15   */
  DP_SLRAMP,      /* how fast the forced angle speeds up, units/ms per s   */
  DP_INSRC,       /* throttle and brake: 0 the console, 1 the pins         */
  DP_THMIN,       /* throttle reading when released, ADC counts            */
  DP_THMAX,       /* throttle reading when fully open                      */
  DP_THMODE,      /* 0 throttle is torque, 1 throttle is speed             */
  DP_THRAMP,      /* throttle may rise this much per ms, of 32767          */
  DP_DRVSPD,      /* top speed under throttle, units per ms                */
  DP_REGEN,       /* braking current, Q15; 0 = the brake only cuts         */
  DP_MOT,         /* the model's motor: 0 gimbal, 1 hub                    */
  DP_OL,          /* one-line output on PB10: 0 off, 1 on                  */
  DP_OLT,         /* its time unit, in 50 us periods                       */
  DP_CANID,       /* node number                                           */
  DP_CANBR,       /* bit rate, kbit/s: 125, 250, 500, 1000                 */
  DP_CANPER,      /* status frames every this many ms                      */
  DP_CANBACK,     /* back to USB after this many s without a frame; 0 never */
  DP_COUNT
} drive_param;

extern int32_t drive_p[DP_COUNT];

typedef enum {
  VEH_OFF = 0,
  VEH_WAIT,       /* on, waiting to see the throttle released */
  VEH_RUN,
  VEH_BRAKE,
  VEH_FAULT       /* latched: throttle out of range */
} drive_state_t;

void    drive_init(void);

/* Every control period, after outer_tick(); it follows that loop's 1 kHz. */
void    drive_tick(void);

/*
 * Store a parameter (already clipped to its range) and do what follows from
 * it. Returns what was stored, which is the old value if it was refused:
 * the sensor and the motor can only be changed while the drive is idle.
 * Call with the control interrupt held off.
 */
int32_t drive_param_set(drive_param id, int32_t v);

/* 1 when nothing is being driven and no calibration is running. */
int     drive_idle(void);

/* The console's commands, so that CAN can give the same ones. Each leaves
   throttle mode. Call with the control interrupt held off. */
void    drive_off(void);
void    drive_torque(int32_t iq);
void    drive_speed(int32_t units_per_ms);
void    drive_position(int32_t delta);
void    drive_voltage(int32_t vd);

/* Throttle mode. Returns 0, or 1 = a calibration is running, 2 = a throttle
   fault is latched. */
int     drive_vehicle_on(void);
void    drive_clear(void);

/* What the console sends in place of the pins. */
void    drive_sim_throttle(int32_t counts);
void    drive_sim_brake(int on);

int32_t drive_throttle_raw_sim(void);   /* what drive_sim_throttle() stored */
int32_t drive_throttle_raw(void);   /* ADC counts in use */
int32_t drive_throttle(void);       /* after calibration and ramp, 0..32767 */
int     drive_brake(void);
int     drive_state(void);
int     drive_fault(void);          /* 1 = throttle out of range */
int     drive_limit(void);          /* 1 = the speed limit is cutting torque */
int     drive_regen(void);          /* 1 = braking current is being asked for */
int16_t drive_fw_d(void);           /* field-weakening current now */

#endif /* DRIVE_H */

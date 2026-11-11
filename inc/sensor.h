/*
 * Where the rotor is, when the encoder is not the one saying.
 *
 * The loop was built on an encoder: an angle every period, good to a
 * fraction of a degree. Most motors that go into a vehicle do not have one.
 * They have three Hall switches, which say which sixth of a turn the rotor
 * is in, or nothing at all. This module turns either of those into the
 * angle the loop needs, and `sens` selects which:
 *
 *   0  encoder     as built. Nothing here runs.
 *   1  Hall        six edges per turn and the time between them.
 *   2  observer    no sensor: the rotor's own back-EMF.
 *
 * HALL. An edge says exactly where the rotor is at one instant; between
 * edges the angle is carried forward at the speed the last sector took.
 * Where the edges are is not assumed. A latch gives six equal sectors; a
 * unipolar switch such as the 3144 or 3143 does not, because it is on for
 * less than half a turn, and nobody mounts three sensors to a fraction of a
 * degree either. So the edges are measured once (sensor_cal_start): a field
 * is turned slowly one way and then the other, the rotor follows it, and
 * the angle of the field at each edge is written down. Averaging the two
 * directions cancels the angle the rotor trails the field by.
 *
 * OBSERVER. The voltage applied, less what the winding's resistance and
 * inductance account for, is the back-EMF, and the back-EMF points a
 * quarter turn ahead of the rotor. A phase-locked loop follows it. At a
 * standstill there is no back-EMF to follow, so below `slmin` the rotor is
 * dragged round at a forced angle with a fixed current instead - the same
 * idea as alignment, moving - and the observer takes over once the rotor is
 * turning fast enough for it to be right.
 *
 * Nothing here is told the truth. The model's real angle is read in one
 * place, sensor_angle_error(), which exists so the result can be checked.
 */
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "foc.h"

typedef enum {
  SENS_ENCODER = 0,
  SENS_HALL,
  SENS_OBSERVER
} sensor_kind;

/* What `v` reports as ph */
typedef enum {
  SENSOR_PH_NONE = 0,     /* encoder */
  SENSOR_PH_HALL,
  SENSOR_PH_OPEN,         /* observer selected, rotor on the forced angle */
  SENSOR_PH_CLOSED,       /* observer supplying the angle */
  SENSOR_PH_CAL           /* Hall calibration running */
} sensor_phase_t;

/* Calibration results. 0 = never run. */
#define SENSOR_CAL_OK          1
#define SENSOR_CAL_MISMATCH    2    /* the two directions saw different states */
#define SENSOR_CAL_NOT_SIX     3    /* not six states, or all-off / all-on seen */
#define SENSOR_CAL_NO_CYCLE    4    /* the states do not follow each other in one ring */
#define SENSOR_CAL_STOPPED     9    /* interrupted by a command */

void     sensor_init(void);

/* Select the source. The caller has checked the drive is idle and holds the
   control interrupt off. */
void     sensor_select(int kind);
int      sensor_kind_get(void);

/* From the control interrupt, when the loop has been told to use them. */
uint16_t sensor_angle(void);
void     sensor_update(foc_ab i, foc_ab v);

/* Angle units per ms, from whatever is supplying the angle. */
int32_t  sensor_speed(void);

/* Once a millisecond, from drive_tick(). */
void     sensor_tick(void);

/* Recompute what depends on parameters (sla, cemf). */
void     sensor_derive(void);

/* Hall calibration. Start returns 0 if it started. */
int      sensor_cal_start(void);
void     sensor_cal_abort(void);
int      sensor_cal_running(void);
int      sensor_cal_result(void);

int      sensor_hall_valid(void);

/* The learned table, for store.c: the edges and the ring, eight entries
   each. Get returns 1 if the table is valid; set returns 0 if it was taken,
   and wants the control interrupt held off. */
int      sensor_hall_table_get(uint16_t *edge, uint8_t *next);
int      sensor_hall_table_set(const uint16_t *edge, const uint8_t *next);
int32_t  sensor_hall_edge(int code);      /* -1 for a state that never occurs */
uint32_t sensor_hall_errors(void);
void     sensor_clear_errors(void);
uint8_t  sensor_hall_now(void);           /* the three bits as they read now */

int      sensor_phase(void);

/* The angle the loop used, minus where the model's rotor really was at that
   instant. For checking only. */
int16_t  sensor_angle_error(void);

#endif /* SENSOR_H */

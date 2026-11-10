/*
 * The loops outside the current loop, run at 1 kHz - every 20th current
 * period.
 *
 * Three nested loops, each slower than the one inside it:
 *
 *   position  P    ->  speed reference
 *   speed     PI   ->  q-axis current reference
 *   current   PI   ->  voltage            (loop.c, 20 kHz)
 *
 * The rates matter as much as the gains. An outer loop has to be slow enough
 * that the loop inside it has already settled, or the two fight each other;
 * 20:1 is the usual ratio and it is why the current loop's 85-period
 * settling, measured in the loop test, is what makes a 1 kHz speed loop
 * reasonable.
 *
 * Speed is measured by differentiating the position sensor, in angle units
 * per 1 ms tick. One pole pair is assumed, so electrical and mechanical angle
 * are the same thing; with more poles the position loop would need the
 * mechanical angle and a division.
 */
#ifndef OUTER_H
#define OUTER_H

#include <stdint.h>

typedef enum {
  OUTER_MODE_OFF = 0,   /* the current loop is commanded directly */
  OUTER_MODE_SPEED,     /* speed loop closed, position open */
  OUTER_MODE_POSITION   /* both closed */
} outer_mode;

#define OUTER_DIVIDER     20        /* 20 kHz / 20 = 1 kHz */
#define OUTER_SPEED_MAX   6560      /* angle units per ms, about 100 Hz electrical */

void    outer_init(void);
void    outer_set_mode(outer_mode mode);
void    outer_set_speed_ref(int32_t units_per_ms);
void    outer_set_position_ref(uint16_t angle);

/* Call once per current period; it decimates internally. */
void    outer_tick(uint16_t measured_angle);

outer_mode outer_mode_get(void);
int32_t outer_speed(void);          /* last measured speed */

/*
 * The speed and position gains, Q15, and the current the speed loop may ask
 * for. Compile-time constants until a real motor made them something to be
 * found rather than chosen. Changing them does not touch the integrator.
 */
void    outer_set_gains(int32_t speed_kp, int32_t speed_ki, int32_t iq_max);
void    outer_get_gains(int32_t *speed_kp, int32_t *speed_ki, int32_t *iq_max);
void    outer_set_pos_kp(int32_t kp);
int32_t outer_pos_kp(void);
int32_t outer_speed_ref(void);
int32_t outer_iq_ref(void);
uint32_t outer_cycles(void);        /* cost of the last 1 kHz update */

#endif /* OUTER_H */

/*
 * A one-shot recorder inside the control interrupt.
 *
 * The console answers a state request in about 2 ms, so a host can watch the
 * loop at a few hundred hertz at best. The current loop's time constant is
 * 1 ms. Anything that needs to see a current rise - which is how a time
 * constant gets measured - has to be recorded here, at the loop's own rate,
 * and read out afterwards.
 *
 * One channel, 256 samples, optionally thinned: every period is 12.8 ms of
 * signal at 20 kHz, every 20th is a quarter of a second at 1 kHz.
 *
 * Arming does not start it. It starts on capture_trigger(), which the console
 * calls in the same breath as it applies a new target, so sample 0 is the
 * last period before the step and the record begins where the step does.
 */
#ifndef CAPTURE_H
#define CAPTURE_H

#include <stdint.h>

#define CAPTURE_LEN   256u

typedef enum {
  CAPTURE_ID = 0,       /* measured d-axis current, Q15            */
  CAPTURE_IQ,           /* measured q-axis current, Q15            */
  CAPTURE_VD,           /* applied d-axis voltage, Q15             */
  CAPTURE_VQ,           /* applied q-axis voltage, Q15             */
  CAPTURE_SPEED,        /* speed, angle units per ms               */
  CAPTURE_ANGLE,        /* electrical angle the loop used          */
  CAPTURE_CHANNELS
} capture_channel;

typedef enum {
  CAPTURE_IDLE = 0,
  CAPTURE_ARMED,        /* waiting for capture_trigger()           */
  CAPTURE_RUNNING,
  CAPTURE_DONE
} capture_state;

/* divider: 1 = every control period, 20 = every speed-loop period. */
void          capture_arm(capture_channel channel, uint16_t divider);
void          capture_trigger(void);

/* Called from the control interrupt, once per period, after the loops. */
void          capture_tick(void);

capture_state capture_status(void);
uint16_t      capture_count(void);
capture_channel capture_which(void);
uint16_t      capture_divider(void);
int16_t       capture_sample(uint16_t index);

#endif /* CAPTURE_H */

#include "capture.h"
#include "loop.h"
#include "outer.h"
#include "sensor.h"
#include "drive.h"
#include "stm32f103xb.h"

static int16_t  buf[CAPTURE_LEN];

/* Written from the console in the main loop, read in the interrupt. Each is
   a single word, so a read is never torn; what matters is the order they are
   written in, and `state` always goes last. */
static volatile capture_state   state;
static volatile capture_channel channel;
static volatile uint16_t        divider;
static volatile uint16_t        count;
static volatile uint16_t        phase;
static uint16_t                 ol_run;
static uint8_t                  ol_level;

void capture_arm(capture_channel ch, uint16_t div)
{
  state = CAPTURE_IDLE;                 /* stop a record still in progress */

  if (ch >= CAPTURE_CHANNELS) {
    ch = CAPTURE_ID;
  }
  if (div == 0u) {
    div = 1u;
  }

  channel = ch;
  divider = div;
  count   = 0u;
  phase   = 0u;
  state   = CAPTURE_ARMED;
}

void capture_ol_frame(void)
{
  if (channel != CAPTURE_OL) {
    return;
  }
  if (state == CAPTURE_RUNNING) {
    state = CAPTURE_DONE;
  } else if (state == CAPTURE_ARMED) {
    count    = 0u;
    ol_run   = 0u;
    ol_level = 0u;
    state    = CAPTURE_RUNNING;
  }
}

void capture_trigger(void)
{
  /* The one-line record starts with a frame, not with a target. */
  if ((state == CAPTURE_ARMED) && (channel != CAPTURE_OL)) {
    count = 0u;
    phase = 0u;
    state = CAPTURE_RUNNING;
  }
}

void capture_tick(void)
{
  int16_t v;

  if (state != CAPTURE_RUNNING) {
    return;
  }

  if (channel == CAPTURE_OL) {
    uint8_t level = (uint8_t)((GPIOB->IDR >> 10) & 1u);

    if (level != ol_level) {
      if (count < CAPTURE_LEN) {
        buf[count] = (int16_t)((ol_level != 0u) ? (int32_t)ol_run : -(int32_t)ol_run);
        count++;
      }
      ol_run   = 0u;
      ol_level = level;
    }
    ol_run++;
    return;
  }

  if (phase != 0u) {
    phase--;
    return;
  }
  phase = (uint16_t)(divider - 1u);

  switch (channel) {
  case CAPTURE_ID:    v = foc_loop_last_dq().d;          break;
  case CAPTURE_IQ:    v = foc_loop_last_dq().q;          break;
  case CAPTURE_VD:    v = foc_loop_last_vdq().d;         break;
  case CAPTURE_VQ:    v = foc_loop_last_vdq().q;         break;
  case CAPTURE_SPEED: {
    int32_t s = outer_speed();
    if (s >  32767) { s =  32767; }
    if (s < -32768) { s = -32768; }
    v = (int16_t)s;
    break;
  }
  case CAPTURE_ANGLE: v = (int16_t)foc_loop_angle();     break;
  case CAPTURE_HALL:  v = (int16_t)sensor_hall_now();    break;
  case CAPTURE_AERR:  v = sensor_angle_error();          break;
  case CAPTURE_IDFW:  v = drive_fw_d();                  break;
  default:            v = 0;                             break;
  }

  buf[count] = v;
  count++;
  if (count >= CAPTURE_LEN) {
    state = CAPTURE_DONE;
  }
}

capture_state capture_status(void)
{
  return state;
}

uint16_t capture_count(void)
{
  return count;
}

capture_channel capture_which(void)
{
  return channel;
}

uint16_t capture_divider(void)
{
  return divider;
}

int16_t capture_sample(uint16_t index)
{
  return (index < CAPTURE_LEN) ? buf[index] : 0;
}

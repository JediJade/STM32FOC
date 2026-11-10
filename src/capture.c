#include "capture.h"
#include "loop.h"
#include "outer.h"

static int16_t  buf[CAPTURE_LEN];

/* Written from the console in the main loop, read in the interrupt. Each is
   a single word, so a read is never torn; what matters is the order they are
   written in, and `state` always goes last. */
static volatile capture_state   state;
static volatile capture_channel channel;
static volatile uint16_t        divider;
static volatile uint16_t        count;
static volatile uint16_t        phase;

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

void capture_trigger(void)
{
  if (state == CAPTURE_ARMED) {
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

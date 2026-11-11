#include "stm32f103xb.h"
#include "oneline.h"
#include "drive.h"
#include "loop.h"
#include "outer.h"
#include "sensor.h"
#include "safety.h"
#include "capture.h"

#define OL_PIN          10u     /* PB10 */
#define OL_SYNC_LOW     31u
#define OL_SYNC_HIGH    1u
#define OL_GAP          20u

/* Positions in a frame: the gap, the two halves of the sync, then a low and
   a high for each of the 96 bits. */
#define POS_GAP         0u
#define POS_SYNC_LOW    1u
#define POS_SYNC_HIGH   2u
#define POS_BITS        3u
#define POS_END         (POS_BITS + (2u * 8u * ONELINE_BYTES))

static uint8_t  frame[ONELINE_BYTES];
static uint16_t pos = POS_END;
static uint16_t left;

const uint8_t *oneline_frame(void)
{
  return frame;
}

/*
 * The twelve bytes, from the state as it is when the frame starts.
 *
 * Speed goes out the way these displays expect it: as the number of Hall
 * transitions in half a second, six per electrical turn. There is no
 * battery figure because nothing here measures the supply.
 */
static void oneline_build(void)
{
  int32_t  speed = outer_speed();
  int32_t  a     = (speed < 0) ? -speed : speed;
  int32_t  iq    = foc_loop_last_dq().q;
  uint32_t halls = ((uint32_t)a * 3000u) >> 16;
  uint8_t  x = 0u;
  uint8_t  i;

  if (iq < 0) { iq = -iq; }
  iq >>= 7;
  if (iq > 255) { iq = 255; }

  frame[0]  = 0x08u;                                  /* device code */
  frame[1]  = 0x61u;                                  /* sequence code */
  frame[2]  = (uint8_t)((((sensor_hall_errors() != 0u) &&
                          (sensor_kind_get() == SENS_HALL)) ? 0x40u : 0u)
                      | ((drive_fault() != 0) ? 0x20u : 0u)
                      | ((safety_last_fault() != SAFETY_FAULT_NONE) ? 0x10u : 0u));
  frame[3]  = (uint8_t)(((a > 20) ? 0x40u : 0u)
                      | ((drive_brake() != 0) ? 0x20u : 0u)
                      | ((drive_regen() != 0) ? 0x08u : 0u));
  frame[4]  = (uint8_t)(((speed < -20) ? 0x04u : 0u)
                      | ((drive_limit() != 0) ? 0x01u : 0u));
  frame[5]  = (uint8_t)iq;
  frame[6]  = 0u;
  frame[7]  = (uint8_t)(halls >> 8);
  frame[8]  = (uint8_t)halls;
  frame[9]  = 0u;
  frame[10] = (uint8_t)((drive_throttle() * 100 + 16384) >> 15);

  for (i = 0u; i < (ONELINE_BYTES - 1u); i++) {
    x ^= frame[i];
  }
  frame[ONELINE_BYTES - 1u] = x;
}

void oneline_tick(void)
{
  if (drive_p[DP_OL] == 0) {
    pos  = POS_END;
    left = 0u;
    return;
  }

  if (left == 0u) {
    uint16_t t = (uint16_t)drive_p[DP_OLT];
    uint16_t level;
    uint16_t len;

    if (pos >= POS_END) {
      oneline_build();
      pos = POS_GAP;
    }

    if (pos == POS_GAP) {
      level = 0u;
      len   = (uint16_t)(OL_GAP * t);
    } else if (pos == POS_SYNC_LOW) {
      /* The frame proper starts here: the recorder, if it is waiting for
         one, starts with it. */
      capture_ol_frame();
      level = 0u;
      len   = (uint16_t)(OL_SYNC_LOW * t);
    } else if (pos == POS_SYNC_HIGH) {
      level = 1u;
      len   = (uint16_t)(OL_SYNC_HIGH * t);
    } else {
      uint16_t k   = (uint16_t)(pos - POS_BITS);
      uint16_t bit = (uint16_t)(k >> 1);
      uint16_t one = (uint16_t)((frame[bit >> 3] >> (7u - (bit & 7u))) & 1u);

      level = (uint16_t)(k & 1u);
      /* a one is short low, long high; a zero is the other way round */
      len   = (uint16_t)(((level != 0u) == (one != 0u)) ? (2u * t) : t);
    }

    GPIOB->BSRR = (level != 0u) ? (1u << OL_PIN) : (1u << (OL_PIN + 16u));
    left = len;
    pos++;
  }
  left--;
}

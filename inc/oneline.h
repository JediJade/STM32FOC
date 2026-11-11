/*
 * The one-line dashboard signal ("yi xian tong", also sold as SIF): one wire
 * from the drive to the display, one direction, no clock.
 *
 *   idle      low
 *   sync      31 T low, 1 T high
 *   bit 0     2 T low, 1 T high
 *   bit 1     1 T low, 2 T high
 *   frame     12 bytes, most significant bit first, then low for 20 T
 *
 * A receiver needs no agreed speed: every bit is one low and one high, and
 * which of the two is longer is the bit. T is `olt` control periods, ten as
 * built, which is 0.5 ms and a frame every 170 ms.
 *
 * The pin is PB10, push-pull at 3.3 V, and it is driven from the control
 * interrupt, so its edges are as steady as the PWM's. A display that wants
 * an open collector needs a transistor after it.
 *
 * What the twelve bytes mean differs from one make of display to the next.
 * The layout here is the common one, and it is all in oneline_build().
 */
#ifndef ONELINE_H
#define ONELINE_H

#include <stdint.h>

#define ONELINE_BYTES   12u

/* Every control period. Does nothing while `ol` is 0. */
void oneline_tick(void);

/* The frame last sent, for the console. */
const uint8_t *oneline_frame(void);

#endif /* ONELINE_H */

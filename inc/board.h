/*
 * The two things the rest of the firmware needs from the board layer in
 * main.c: a millisecond tick and a blocking delay, both off SysTick.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

uint32_t millis(void);
void     delay_ms(uint32_t ms);

#endif /* BOARD_H */

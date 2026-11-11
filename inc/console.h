/*
 * A line-based console on the USB CDC port.
 *
 * Exists so the loop can be commanded and watched with a motor attached,
 * without a debugger: reading RAM over SWD is fine for self-tests that run
 * once at boot, and useless for watching a current loop while it turns.
 *
 * Everything here runs in the main loop, never in the control interrupt.
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

void console_init(void);
void console_poll(void);

/* What `ver` reports. */
int  console_version(void);

/*
 * The settable parameters by number, in the order `get` lists them, for the
 * CAN side. Setting clips to the parameter's range and does what the
 * console's `set` does.
 */
int     console_param_count(void);
int32_t console_param_get(int index);
void    console_param_set(int index, int32_t value);

/* 1, once, after `can on`: the main loop then changes over. */
int  console_can_requested(void);

#endif /* CONSOLE_H */

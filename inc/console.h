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

void console_init(void);
void console_poll(void);

#endif /* CONSOLE_H */

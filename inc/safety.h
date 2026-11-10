/*
 * The gate between "this firmware runs" and "this firmware drives a
 * three-phase bridge".
 *
 * Up to now MOE could be turned on by typing `on`, with no conditions
 * attached. That is fine against a virtual motor and unacceptable against
 * real FETs: enabling the outputs while a target is already set would start
 * driving the instant the command landed.
 *
 * Three rules, all enforced here:
 *
 *  1. Arming requires a known-safe state. Targets zeroed, loop idle, no
 *     latched fault. A refusal says which condition failed.
 *  2. A fault latches. The timer's break input clears MOE in hardware, and
 *     with AOE clear it stays off; this layer records why and refuses to arm
 *     again until a human clears it. A fault that clears itself is a fault
 *     that will be ignored.
 *  3. A hung main loop must not leave the bridge switching. The independent
 *     watchdog resets the chip, which puts MOE back to its reset value of 0.
 *     The reset cause survives, so the console can report what happened
 *     rather than coming up as if nothing did.
 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

typedef enum {
  SAFETY_FAULT_NONE = 0,
  SAFETY_FAULT_BREAK,          /* the timer's break input tripped */
  SAFETY_FAULT_WATCHDOG,       /* the previous run was reset by the watchdog */
  SAFETY_FAULT_NOT_IDLE,       /* refused to arm: a target was still set */
} safety_fault;

/* Reads and clears the reset cause; call before anything else enables IWDG. */
void safety_init(void);

/* Starts the independent watchdog. Called after the self-tests, which spend
   seconds in delay loops that would otherwise have to refresh it. */
void safety_watchdog_start(void);
void safety_watchdog_refresh(void);
int  safety_watchdog_running(void);

/* Call often: latches a fault if the break input has tripped. */
void safety_poll(void);

/* 0 on success, or the reason it was refused. */
safety_fault safety_arm(void);
void         safety_disarm(void);
int          safety_armed(void);

safety_fault safety_last_fault(void);
void         safety_clear_fault(void);

/* Human-readable, for the console. */
const char  *safety_fault_name(safety_fault f);

/* What reset the chip last time: a string, for the banner. */
const char  *safety_reset_cause(void);

#endif /* SAFETY_H */

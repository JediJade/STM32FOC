/*
 * The settings, kept across a power cycle.
 *
 * Everything the console can `set` has lived in RAM: a board that was tuned,
 * had its throttle ends measured and its Hall edges learned came back from a
 * reset as if none of it had happened. On a bench that is a nuisance. On a
 * vehicle it is the whole feature missing - nobody calibrates a throttle
 * every time they switch on.
 *
 * So: one 1 KB flash page, the last one in the device, holds one record -
 * the parameter array as the console numbers it, plus the Hall table, plus
 * a checksum. `save` writes it, `load` puts it back, and the firmware loads
 * it once at boot before the watchdog starts.
 *
 * Three things are deliberate.
 *
 * ONE PAGE, ONE RECORD, ERASED EVERY TIME. Wear levelling across several
 * pages would buy 100,000 saves instead of 10,000, and nothing here saves
 * in a loop: a save is a human typing `save`. Simplicity is worth more than
 * the endurance.
 *
 * THE RECORD CARRIES THE CONSOLE VERSION. The parameters are stored by
 * number, not by name, because the names cost 4 bytes each to no purpose
 * while one firmware writes and the same firmware reads. The moment the
 * parameter list changes, those numbers mean something else - so a record
 * written by a different console version is refused rather than applied to
 * the wrong parameters. It is reported, not silently ignored.
 *
 * SAVING STOPS THE WORLD FOR 40 ms. Erasing a page stalls every instruction
 * fetch from flash, which means the control interrupt does not run while it
 * happens, and the watchdog has to be fed from inside the wait loop. That is
 * acceptable with the outputs off and refused otherwise - which is why
 * store_save() asks safety and drive whether they are idle first.
 */
#ifndef STORE_H
#define STORE_H

#include <stdint.h>

/* What store_save() / store_load() / store_state() report. */
typedef enum {
  STORE_OK = 0,
  STORE_EMPTY,        /* the page has never been written */
  STORE_BAD,          /* magic or checksum wrong: a half-written record */
  STORE_VERSION,      /* written by another console version; not applied */
  STORE_BUSY,         /* the drive is running or the outputs are armed */
  STORE_FAILED        /* the flash controller reported an error */
} store_result;

/* Read the page and apply what is in it. */
store_result store_load(void);

/* The same, at start-up: it latches its result so that the console can say
   later what the board came up on. Called once, from main(). */
store_result store_boot(void);
store_result store_boot_result(void);

/* Collect the live settings and write them. Refused unless idle. */
store_result store_save(void);

/* Erase the page, so the next boot comes up on the built-in defaults. */
store_result store_erase(void);

/* What is in the page, without applying it. */
store_result store_state(void);

/* The console version that wrote what is in the page; 0 if there is none. */
int store_saved_console(void);

/* How many parameters the stored record holds; 0 if there is none. */
int store_saved_count(void);

/* 1 if the record holds a learned Hall table. */
int store_saved_hall(void);

/* Human-readable, for a refusal the console prints. */
const char *store_result_name(store_result r);

/* The same thing in one word, for a reply a host parses: every line the
   console reports is name-value pairs, and "another console version" would
   be four values. */
const char *store_result_word(store_result r);

#endif /* STORE_H */

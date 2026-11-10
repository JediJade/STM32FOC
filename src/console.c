#include "console.h"
#include "usb_cdc.h"
#include "board.h"
#include "loop.h"
#include "outer.h"
#include "vmotor.h"
#include "tim1.h"
#include "selftest.h"
#include "safety.h"

/* ------------------------------------------------------- formatting ----- */

/* No stdio with -nostdlib, and none is wanted: a console that can block for
   an unknown time is a console that can disturb the control loop. */

static char *put_str(char *p, const char *s)
{
  while (*s != '\0') {
    *p++ = *s++;
  }
  return p;
}

static char *put_int(char *p, int32_t v)
{
  char tmp[12];
  int  n = 0;
  uint32_t u;

  if (v < 0) {
    *p++ = '-';
    u = (uint32_t)(-v);
  } else {
    u = (uint32_t)v;
  }

  do {
    tmp[n++] = (char)('0' + (u % 10U));
    u /= 10U;
  } while (u != 0U);

  while (n > 0) {
    *p++ = tmp[--n];
  }
  return p;
}

static char *put_hex(char *p, uint32_t v)
{
  const char *digits = "0123456789abcdef";
  int i;

  *p++ = '0';
  *p++ = 'x';
  for (i = 28; i >= 0; i -= 4) {
    uint32_t d = (v >> i) & 0xFU;
    if ((d != 0U) || (i == 0) || (p[-1] != 'x')) {
      *p++ = digits[d];
    }
  }
  return p;
}

/*
 * Blocking only on the USB endpoint, with a bound: if the host is not
 * draining, give up rather than stall the main loop forever.
 *
 * The watchdog is refreshed while waiting, and that is correct rather than a
 * cheat: the control loop runs in the ADC interrupt and is unaffected by this
 * wait, so a console waiting on a slow host is not the hung main loop the
 * watchdog exists to catch.
 *
 * Without it, a multi-line reply to a host that had stopped reading was
 * enough to trip it: `?` prints ten lines, ten waits of 50 ms, half a second
 * against a 100 ms watchdog. The board reset, latched a watchdog fault, and
 * then refused to arm - and the only visible symptom was `refused: watchdog
 * reset` on a board that had not hung.
 */
static void emit(const char *buf, uint16_t len)
{
  uint32_t start = millis();

  while (len > 0U) {
    uint16_t n;

    if (usb_cdc_write_ready() == 0) {
      safety_watchdog_refresh();
      if ((millis() - start) > 50U) {
        return;
      }
      continue;
    }

    n = usb_cdc_write((const uint8_t *)buf, len);
    buf += n;
    len = (uint16_t)(len - n);
  }
}

static void line(const char *s)
{
  char buf[80];
  char *p = put_str(buf, s);

  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

/* ----------------------------------------------------------- state ------ */

static char    cmd[64];
static uint8_t cmd_len;
static uint8_t telemetry;
static uint32_t next_telem;

static int32_t parse_int(const char *s)
{
  int32_t v = 0;
  int     neg = 0;

  while ((*s == ' ') || (*s == '\t')) {
    s++;
  }
  if (*s == '-') {
    neg = 1;
    s++;
  }
  while ((*s >= '0') && (*s <= '9')) {
    v = (v * 10) + (*s - '0');
    s++;
  }
  return neg ? -v : v;
}

static int starts_with(const char *s, const char *prefix)
{
  while (*prefix != '\0') {
    if (*s++ != *prefix++) {
      return 0;
    }
  }
  return 1;
}

static void print_help(void)
{
  line("commands:");
  line("  s          state: angle, speed, id, iq, mode");
  line("  t          self-test summary");
  line("  m          toggle 10 Hz telemetry");
  line("  iq <n>     current mode, q-axis target (Q15, 0..16384)");
  line("  spd <n>    speed mode, reference in angle units per ms");
  line("  pos <n>    position mode, target = now + n angle units");
  line("  align      re-run the rotor alignment");
  line("  off        idle, outputs disabled");
  line("  on         arm: enable the PWM outputs (refused unless idle)");
  line("  clear      clear a latched fault");
  line("  hang       stop refreshing the watchdog, on purpose");
}

static void print_state(void)
{
  char buf[80];
  char *p;
  foc_dq dq = foc_loop_last_dq();

  p = put_str(buf, "ang ");
  p = put_int(p, (int32_t)foc_loop_angle());
  p = put_str(p, "  spd ");
  p = put_int(p, outer_speed());
  p = put_str(p, "  id ");
  p = put_int(p, dq.d);
  p = put_str(p, "  iq ");
  p = put_int(p, dq.q);
  p = put_str(p, "  iqref ");
  p = put_int(p, outer_iq_ref());
  p = put_str(p, "  moe ");
  p = put_int(p, tim1_outputs_enabled());
  p = put_str(p, "  fault ");
  p = put_str(p, safety_fault_name(safety_last_fault()));
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void print_tests(void)
{
  char buf[80];
  char *p;

  p = put_str(buf, "ok_flags ");
  p = put_hex(p, selftest.ok_flags);
  p = put_str(p, " of 0xff");
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));

  p = put_str(buf, "deadtime ");
  p = put_int(p, (int32_t)selftest.dt_mean_x100);
  p = put_str(p, " cx100 ticks, adc ");
  p = put_int(p, (int32_t)selftest.adc_rate_hz);
  p = put_str(p, " Hz");
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));

  p = put_str(buf, "reset ");
  p = put_str(p, safety_reset_cause());
  p = put_str(p, ", wdg ");
  p = put_int(p, safety_watchdog_running());
  p = put_str(p, ", fault ");
  p = put_str(p, safety_fault_name(safety_last_fault()));
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));

  p = put_str(buf, "align err ");
  p = put_int(p, selftest.align_error);
  p = put_str(p, ", loop settle ");
  p = put_int(p, (int32_t)selftest.loop_settle_iters);
  p = put_str(p, ", ctrl ");
  p = put_int(p, (int32_t)(selftest.loop_cycles_max - selftest.loop_plant_cycles_max));
  p = put_str(p, " cyc");
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void dispatch(void)
{
  if (cmd_len == 0U) {
    return;
  }

  /* Multi-character commands first: "spd 3280" starts with 's', and
     checking the single-letter commands first turned every speed command
     into a request to print the state. */
  if ((cmd[0] == '?') || starts_with(cmd, "help")) {
    print_help();
  } else if (starts_with(cmd, "iq ")) {
    int32_t v = parse_int(&cmd[3]);

    if (v > 16384) { v = 16384; }
    if (v < -16384) { v = -16384; }
    outer_set_mode(OUTER_MODE_OFF);
    vmotor_set_mech(VM_MECH_FREE, 0);
    foc_loop_set_mode(LOOP_MODE_CURRENT);
    foc_loop_set_target((int16_t)v);
    line("current mode");
  } else if (starts_with(cmd, "spd ")) {
    int32_t v = parse_int(&cmd[4]);

    if (v >  OUTER_SPEED_MAX) { v =  OUTER_SPEED_MAX; }
    if (v < -OUTER_SPEED_MAX) { v = -OUTER_SPEED_MAX; }
    vmotor_set_mech(VM_MECH_FREE, 0);
    foc_loop_set_mode(LOOP_MODE_CURRENT);
    outer_set_mode(OUTER_MODE_SPEED);
    outer_set_speed_ref(v);
    line("speed mode");
  } else if (starts_with(cmd, "pos ")) {
    int32_t v = parse_int(&cmd[4]);

    vmotor_set_mech(VM_MECH_FREE, 0);
    foc_loop_set_mode(LOOP_MODE_CURRENT);
    outer_set_mode(OUTER_MODE_POSITION);
    outer_set_position_ref((uint16_t)(foc_loop_angle() + (uint16_t)v));
    line("position mode");
  } else if (starts_with(cmd, "align")) {
    line("aligning...");
    selftest_run_align();
    print_tests();
  } else if (starts_with(cmd, "off")) {
    outer_set_mode(OUTER_MODE_OFF);
    foc_loop_set_mode(LOOP_MODE_IDLE);
    foc_loop_set_target(0);
    safety_disarm();
    line("idle, outputs off");
  } else if (starts_with(cmd, "on")) {
    safety_fault f = safety_arm();

    if (f == SAFETY_FAULT_NONE) {
      line("armed, outputs enabled");
    } else {
      char buf[72];
      char *p = put_str(buf, "refused: ");
      p = put_str(p, safety_fault_name(f));
      *p++ = '\r';
      *p++ = '\n';
      emit(buf, (uint16_t)(p - buf));
    }
  } else if (starts_with(cmd, "clear")) {
    safety_clear_fault();
    line("fault cleared");
  } else if (starts_with(cmd, "hang")) {
    line("hanging: the watchdog should reset the board");
    for (;;) {
      /* deliberately not refreshing */
    }
  } else if (cmd[0] == 's') {
    print_state();
  } else if (cmd[0] == 't') {
    print_tests();
  } else if (cmd[0] == 'm') {
    telemetry = (uint8_t)((telemetry != 0U) ? 0U : 1U);
    line((telemetry != 0U) ? "telemetry on" : "telemetry off");
  } else {
    line("? for help");
  }
}

void console_init(void)
{
  cmd_len    = 0U;
  telemetry  = 0U;
  next_telem = 0U;
}

void console_poll(void)
{
  uint8_t c;

  if (usb_cdc_configured() == 0) {
    return;
  }

  while (usb_cdc_read_byte(&c) != 0) {
    if ((c == '\r') || (c == '\n')) {
      cmd[cmd_len] = '\0';
      dispatch();
      cmd_len = 0U;
    } else if ((c == 8U) || (c == 127U)) {       /* backspace */
      if (cmd_len > 0U) {
        cmd_len--;
      }
    } else if (cmd_len < (sizeof(cmd) - 1U)) {
      cmd[cmd_len++] = (char)c;
    }
  }

  safety_poll();

  if ((telemetry != 0U) && (millis() >= next_telem)) {
    next_telem = millis() + 100U;
    print_state();
  }
}

#include "console.h"
#include "usb_cdc.h"
#include "board.h"
#include "loop.h"
#include "outer.h"
#include "vmotor.h"
#include "tim1.h"
#include "selftest.h"
#include "safety.h"
#include "capture.h"
#include "drive.h"
#include "sensor.h"
#include "oneline.h"
#include "canbus.h"
#include "store.h"
#include "stm32f103xb.h"

/*
 * What a host can ask this firmware for. 1 was the console as first written
 * (it has no `ver`, and answers "? for help"); 2 added settable gains, the
 * d-axis target, voltage mode, the recorder and the `x` line; 3 added the
 * throttle and brake, field weakening, the sensor selector, Hall
 * calibration, the one-line output, CAN and the `v` line; 4 added `save`,
 * `load` and `sto`, which keep the parameters and the Hall table in flash.
 *
 * A host may rely on a higher number having everything a lower one had.
 * `save` writes the parameters by number, so the number is also what tells
 * a stored record apart from one a different parameter list wrote.
 */
#define CONSOLE_VERSION   4

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
static uint8_t can_request;

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
  line("  ver        console version and recorder size");
  line("  x          applied voltage, loop modes, recorder state");
  line("  get        list the settable parameters");
  line("  set <p> <n>  set one of them");
  line("  id <n>     d-axis current target (Q15), used in current mode");
  line("  vd <n>     voltage mode: v_d = n, v_q = 0, no controller");
  line("  cap <ch> <div>  arm the recorder: id iq vd vq spd ang");
  line("             also ol (one-line pin), hall, aerr, idfw, iab");
  line("  trig       start an armed recorder now");
  line("  dump       print what the recorder holds");
  line("  v          throttle, brake, sensor, weakening");
  line("  th <n>     throttle reading in place of the pin, 0..4095");
  line("  brk <0|1>  brake in place of the pin");
  line("  drv <0|1>  throttle mode on / off");
  line("  hallcal    learn where the Hall edges are");
  line("  hall       the edges it learned");
  line("  olf        the one-line frame last sent");
  line("  can        CAN state; `can on` changes over to it");
  line("  save       keep the parameters and the Hall table in flash");
  line("  load       put back what was kept");
  line("  sto        what the flash page holds; `sto clear` erases it");
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

/* ------------------------------------------------------ parameters ------ */

/*
 * Everything that was a tuning #define, by a short name.
 *
 * The limits are not opinions about what tunes well. They are what the
 * arithmetic downstream can take: q15_mul() takes an int16, the speed
 * loop's output is a current reference that has to stay inside the current
 * loop's own limit, and a gain past these ranges is a typing mistake.
 */
typedef enum {
  P_CKP = 0, P_CKI, P_CDEC, P_CCROSS, P_CEMF,
  P_SKP, P_SKI, P_SIMAX, P_PKP,
  P_DRIVE,                              /* drive.h's, in its order, from here */
  P_COUNT = P_DRIVE + DP_COUNT
} param_id;

typedef struct {
  const char *name;
  int32_t     lo;
  int32_t     hi;
} param_def;

static const param_def params[P_COUNT] = {
  { "ckp",     0,  262144 },    /* current Kp, Q15: up to 8.0            */
  { "cki",     0,   32767 },    /* current Ki, Q15 per 50 us period      */
  { "cdec",    0,       2 },    /* feed-forward: off, fixed, with speed  */
  { "ccross",  0,   65535 },    /* cross-coupling feed-forward, Q15      */
  { "cemf",    0,  131071 },    /* back-EMF feed-forward, Q15            */
  { "skp",     0, 1048576 },    /* speed Kp, Q15: up to 32.0             */
  { "ski",     0,  131072 },    /* speed Ki, Q15 per 1 ms tick           */
  { "simax", 256,   29491 },    /* current the speed loop may ask for    */
  { "pkp",     0,   32768 },    /* position Kp, Q15                      */

  /* drive.h explains each. The order is DP_VLIM onwards. */
  { "vlim",    4096,  29491 },
  { "fw",         0,  16384 },
  { "fwstart", 16384, 32767 },
  { "sens",       0,      2 },
  { "hsrc",       0,      1 },
  { "hwid",   22000,  43000 },
  { "sla",      128,  32767 },
  { "slmin",    100,   3000 },
  { "slcur",      0,  16384 },
  { "slramp",   100,  20000 },
  { "insrc",      0,      1 },
  { "thmin",      0,   4095 },
  { "thmax",      0,   4095 },
  { "thmode",     0,      1 },
  { "thramp",     1,  32767 },
  { "drvspd",   100,  16000 },
  { "regen",      0,  16384 },
  { "mot",        0,      1 },
  { "ol",         0,      1 },
  { "olt",        5,     40 },
  { "canid",      1,    127 },
  { "canbr",    125,   1000 },
  { "canper",     5,   1000 },
  { "canback",    0,    600 },
};

static int32_t param_get(param_id id)
{
  int32_t a = 0;
  int32_t b = 0;
  int32_t c = 0;

  if (id >= P_DRIVE) {
    return drive_p[id - P_DRIVE];
  }

  switch (id) {
  case P_CKP:    foc_loop_get_gains(&a, &b);        return a;
  case P_CKI:    foc_loop_get_gains(&a, &b);        return b;
  case P_CDEC:   return foc_loop_decouple();
  case P_CCROSS: foc_loop_get_feedforward(&a, &b);  return a;
  case P_CEMF:   foc_loop_get_feedforward(&a, &b);  return b;
  case P_SKP:    outer_get_gains(&a, &b, &c);       return a;
  case P_SKI:    outer_get_gains(&a, &b, &c);       return b;
  case P_SIMAX:  outer_get_gains(&a, &b, &c);       return c;
  case P_PKP:    return outer_pos_kp();
  default:       return 0;
  }
}

static void param_set(param_id id, int32_t v)
{
  int32_t a = 0;
  int32_t b = 0;
  int32_t c = 0;

  if (v < params[id].lo) { v = params[id].lo; }
  if (v > params[id].hi) { v = params[id].hi; }

  /* A gain is several words in a controller's state. The control interrupt
     must not run between them and see half of the old with half of the new. */
  __disable_irq();
  if (id >= P_DRIVE) {
    (void)drive_param_set((drive_param)(id - P_DRIVE), v);
    __enable_irq();
    return;
  }
  switch (id) {
  case P_CKP:    foc_loop_get_gains(&a, &b);        foc_loop_set_gains(v, b);        break;
  case P_CKI:    foc_loop_get_gains(&a, &b);        foc_loop_set_gains(a, v);        break;
  case P_CDEC:   foc_loop_set_decouple((int)v);                                      break;
  case P_CCROSS: foc_loop_get_feedforward(&a, &b);  foc_loop_set_feedforward(v, b);  break;
  case P_CEMF:   foc_loop_get_feedforward(&a, &b);  foc_loop_set_feedforward(a, v);  break;
  case P_SKP:    outer_get_gains(&a, &b, &c);       outer_set_gains(v, b, c);        break;
  case P_SKI:    outer_get_gains(&a, &b, &c);       outer_set_gains(a, v, c);        break;
  case P_SIMAX:  outer_get_gains(&a, &b, &c);       outer_set_gains(a, b, v);        break;
  case P_PKP:    outer_set_pos_kp(v);                                                break;
  default:                                                                           break;
  }
  sensor_derive();              /* the observer scales by cemf */
  __enable_irq();
}

int console_version(void)
{
  return CONSOLE_VERSION;
}

int console_param_count(void)
{
  return (int)P_COUNT;
}

int32_t console_param_get(int index)
{
  return ((index >= 0) && (index < (int)P_COUNT)) ? param_get((param_id)index) : 0;
}

void console_param_set(int index, int32_t value)
{
  if ((index >= 0) && (index < (int)P_COUNT)) {
    param_set((param_id)index, value);
  }
}

int console_can_requested(void)
{
  if (can_request != 0U) {
    can_request = 0U;
    return 1;
  }
  return 0;
}

static void print_param(param_id id)
{
  char buf[40];
  char *p = put_str(buf, "p ");

  p = put_str(p, params[id].name);
  *p++ = ' ';
  p = put_int(p, param_get(id));
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

/* If s begins with word followed by a space or the end, the text after it. */
static const char *after_word(const char *s, const char *word)
{
  while (*word != '\0') {
    if (*s != *word) {
      return 0;
    }
    s++;
    word++;
  }
  if ((*s != ' ') && (*s != '\0')) {
    return 0;
  }
  while (*s == ' ') {
    s++;
  }
  return s;
}

static void cmd_set(const char *args)
{
  int i;

  for (i = 0; i < (int)P_COUNT; i++) {
    const char *value = after_word(args, params[i].name);

    if ((value != 0) && (*value != '\0')) {
      param_set((param_id)i, parse_int(value));
      print_param((param_id)i);
      return;
    }
  }
  line("? unknown parameter, or no value");
}

/* -------------------------------------------------------- recorder ------ */

static const char *const cap_names[CAPTURE_CHANNELS] = {
  "id", "iq", "vd", "vq", "spd", "ang", "ol", "hall", "aerr", "idfw",
  "iab"
};

static void cmd_cap(const char *args)
{
  int i;

  for (i = 0; i < (int)CAPTURE_CHANNELS; i++) {
    const char *rest = after_word(args, cap_names[i]);

    if (rest != 0) {
      int32_t div = (*rest != '\0') ? parse_int(rest) : 1;

      if (div < 1)    { div = 1; }
      if (div > 2000) { div = 2000; }
      capture_arm((capture_channel)i, (uint16_t)div);
      /* `cap iab` is for watching, not for catching a step: no new
         target is coming to start it, so it starts itself. */
      if ((capture_channel)i == CAPTURE_IAB) {
        capture_trigger();
      }
      line("cap armed");
      return;
    }
  }
  line("? cap <id|iq|vd|vq|spd|ang|ol|hall|aerr|idfw|iab> <divider>");
}

static void cmd_dump(void)
{
  char buf[80];
  char *p;
  uint16_t n = capture_count();
  uint16_t i = 0U;

  p = put_str(buf, "cap ");
  p = put_str(p, cap_names[capture_which()]);
  p = put_str(p, " div ");
  p = put_int(p, (int32_t)capture_divider());
  p = put_str(p, " n ");
  p = put_int(p, (int32_t)n);
  p = put_str(p, " state ");
  p = put_int(p, (int32_t)capture_status());
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));

  /* Eight to a line: a line is at most 2 + 8 * 7 characters, well inside
     the buffer, and thirty-two lines is a reply a host can frame. */
  while (i < n) {
    uint16_t k;

    p = put_str(buf, "d");
    for (k = 0U; (k < 8U) && (i < n); k++, i++) {
      *p++ = ' ';
      p = put_int(p, (int32_t)capture_sample(i));
    }
    *p++ = '\r';
    *p++ = '\n';
    emit(buf, (uint16_t)(p - buf));

    /* emit() refreshes the watchdog only while it is kept waiting. A host
       that drains fast keeps it from ever waiting, so refresh here too. */
    safety_watchdog_refresh();
  }
  line("cap end");
}

static void print_extra(void)
{
  char buf[80];
  char *p;
  foc_dq v = foc_loop_last_vdq();

  p = put_str(buf, "x vd ");
  p = put_int(p, v.d);
  p = put_str(p, "  vq ");
  p = put_int(p, v.q);
  p = put_str(p, "  loop ");
  p = put_int(p, (int32_t)foc_loop_mode());
  p = put_str(p, "  outer ");
  p = put_int(p, (int32_t)outer_mode_get());
  p = put_str(p, "  cap ");
  p = put_int(p, (int32_t)capture_status());
  p = put_str(p, "  n ");
  p = put_int(p, (int32_t)capture_count());
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

/* Throttle, brake, sensor and weakening in one line, for a host to poll. */
static void print_vehicle(void)
{
  char buf[120];
  char *p;

  p = put_str(buf, "v th ");
  p = put_int(p, drive_throttle_raw());
  p = put_str(p, " thr ");
  p = put_int(p, drive_throttle());
  p = put_str(p, " brk ");
  p = put_int(p, drive_brake());
  p = put_str(p, " veh ");
  p = put_int(p, drive_state());
  p = put_str(p, " vflt ");
  p = put_int(p, drive_fault());
  p = put_str(p, " sens ");
  p = put_int(p, sensor_kind_get());
  p = put_str(p, " ph ");
  p = put_int(p, sensor_phase());
  p = put_str(p, " hall ");
  p = put_int(p, (int32_t)sensor_hall_now());
  p = put_str(p, " cal ");
  p = put_int(p, sensor_cal_result());
  p = put_str(p, " idfw ");
  p = put_int(p, drive_fw_d());
  p = put_str(p, " aerr ");
  p = put_int(p, sensor_angle_error());
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void print_hall(void)
{
  char buf[120];
  char *p;
  int  c;

  p = put_str(buf, "hall ok ");
  p = put_int(p, sensor_hall_valid());
  p = put_str(p, " err ");
  p = put_int(p, (int32_t)sensor_hall_errors());
  p = put_str(p, " e");
  for (c = 0; c < 8; c++) {
    *p++ = ' ';
    p = put_int(p, sensor_hall_edge(c));
  }
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void print_oneline(void)
{
  static const char digits[] = "0123456789abcdef";
  const uint8_t *f = oneline_frame();
  char buf[64];
  char *p = put_str(buf, "olf");
  uint32_t i;

  for (i = 0U; i < ONELINE_BYTES; i++) {
    *p++ = ' ';
    *p++ = digits[f[i] >> 4];
    *p++ = digits[f[i] & 15U];
  }
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void print_can(void)
{
  char buf[96];
  char *p;

  p = put_str(buf, "can test ");
  p = put_int(p, (int32_t)can_test_result());
  p = put_str(p, " mode ");
  p = put_int(p, can_active());
  p = put_str(p, " id ");
  p = put_int(p, drive_p[DP_CANID]);
  p = put_str(p, " br ");
  p = put_int(p, drive_p[DP_CANBR]);
  p = put_str(p, " rx ");
  p = put_int(p, (int32_t)can_rx_count());
  p = put_str(p, " tx ");
  p = put_int(p, (int32_t)can_tx_count());
  p = put_str(p, " drop ");
  p = put_int(p, (int32_t)can_drop_count());
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void reply_int(const char *word, int32_t v)
{
  char buf[40];
  char *p = put_str(buf, word);

  *p++ = ' ';
  p = put_int(p, v);
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

/*
 * One line for what the flash page holds. `boot` is what happened to it at
 * start-up, which is the question being asked whenever a board comes up
 * behaving unlike the one that was put away: nothing stored, something
 * stored and applied, or something stored that this firmware refused.
 */
static void print_store(void)
{
  char buf[88];
  char *p = put_str(buf, "sto state ");

  p = put_str(p, store_result_word(store_state()));
  p = put_str(p, " boot ");
  p = put_str(p, store_result_word(store_boot_result()));
  p = put_str(p, " console ");
  p = put_int(p, (int32_t)store_saved_console());
  p = put_str(p, " p ");
  p = put_int(p, (int32_t)store_saved_count());
  p = put_str(p, " hall ");
  p = put_int(p, (int32_t)store_saved_hall());
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void reply_store(const char *what, store_result r)
{
  char buf[72];
  char *p = put_str(buf, what);

  p = put_str(p, (r == STORE_OK) ? " ok" : " refused: ");
  if (r != STORE_OK) {
    p = put_str(p, store_result_name(r));
  }
  *p++ = '\r';
  *p++ = '\n';
  emit(buf, (uint16_t)(p - buf));
}

static void dispatch(void)
{
  const char *arg;

  if (cmd_len == 0U) {
    return;
  }

  /* Multi-character commands first: "spd 3280" starts with 's', and
     checking the single-letter commands first turned every speed command
     into a request to print the state. */
  if ((cmd[0] == '?') || starts_with(cmd, "help")) {
    print_help();
  } else if (starts_with(cmd, "iq ")) {
    __disable_irq();
    drive_torque(parse_int(&cmd[3]));
    __enable_irq();
    line("current mode");
  } else if (starts_with(cmd, "spd ")) {
    __disable_irq();
    drive_speed(parse_int(&cmd[4]));
    __enable_irq();
    line("speed mode");
  } else if (starts_with(cmd, "pos ")) {
    __disable_irq();
    drive_position(parse_int(&cmd[4]));
    __enable_irq();
    line("position mode");
  } else if (starts_with(cmd, "align")) {
    line("aligning...");
    selftest_run_align();
    print_tests();
  } else if (starts_with(cmd, "off")) {
    __disable_irq();
    drive_off();
    __enable_irq();
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
    __disable_irq();
    drive_clear();
    __enable_irq();
    line("fault cleared");
  } else if (after_word(cmd, "hallcal") != 0) {
    line((sensor_cal_start() == 0) ? "hall calibration started"
                                   : "? stop the drive first");
  } else if (after_word(cmd, "hall") != 0) {
    print_hall();
  } else if (after_word(cmd, "save") != 0) {
    reply_store("save", store_save());
  } else if (after_word(cmd, "load") != 0) {
    reply_store("load", store_load());
  } else if ((arg = after_word(cmd, "sto")) != 0) {
    if (starts_with(arg, "clear")) {
      reply_store("sto clear", store_erase());
    } else {
      print_store();
    }
  } else if (starts_with(cmd, "hang")) {
    line("hanging: the watchdog should reset the board");
    for (;;) {
      /* deliberately not refreshing */
    }
  } else if ((arg = after_word(cmd, "set")) != 0) {
    cmd_set(arg);
  } else if (after_word(cmd, "get") != 0) {
    int i;

    for (i = 0; i < (int)P_COUNT; i++) {
      print_param((param_id)i);
    }
    line("get end");
  } else if (after_word(cmd, "ver") != 0) {
    char buf[48];
    char *p = put_str(buf, "fw ");

    p = put_int(p, CONSOLE_VERSION);
    p = put_str(p, " cap ");
    p = put_int(p, (int32_t)CAPTURE_LEN);
    *p++ = '\r';
    *p++ = '\n';
    emit(buf, (uint16_t)(p - buf));
  } else if ((arg = after_word(cmd, "id")) != 0) {
    int32_t v = parse_int(arg);

    if (v > 16384) { v = 16384; }
    if (v < -16384) { v = -16384; }
    __disable_irq();
    foc_loop_set_target_d((int16_t)v);
    capture_trigger();
    __enable_irq();
    line("id target");
  } else if ((arg = after_word(cmd, "vd")) != 0) {
    __disable_irq();
    drive_voltage(parse_int(arg));
    __enable_irq();
    line("voltage mode");
  } else if ((arg = after_word(cmd, "cap")) != 0) {
    cmd_cap(arg);
  } else if (after_word(cmd, "trig") != 0) {
    capture_trigger();
    line("trig");
  } else if (after_word(cmd, "dump") != 0) {
    cmd_dump();
  } else if ((arg = after_word(cmd, "th")) != 0) {
    drive_sim_throttle(parse_int(arg));
    reply_int("th", drive_throttle_raw_sim());
  } else if ((arg = after_word(cmd, "brk")) != 0) {
    drive_sim_brake(parse_int(arg) != 0);
    reply_int("brk", (parse_int(arg) != 0) ? 1 : 0);
  } else if ((arg = after_word(cmd, "drv")) != 0) {
    if (parse_int(arg) != 0) {
      int r;

      __disable_irq();
      r = drive_vehicle_on();
      __enable_irq();
      line((r == 0) ? "drive on: release the throttle to start"
         : (r == 1) ? "? stop the drive first"
                    : "refused: throttle fault, clear it first");
    } else {
      __disable_irq();
      drive_off();
      __enable_irq();
      line("drive off");
    }
  } else if (after_word(cmd, "olf") != 0) {
    print_oneline();
  } else if ((arg = after_word(cmd, "can")) != 0) {
    if (starts_with(arg, "on")) {
      char buf[72];
      char *p = put_str(buf, "can on: usb closes, back after ");

      p = put_int(p, drive_p[DP_CANBACK]);
      p = put_str(p, " s without a frame");
      *p++ = '\r';
      *p++ = '\n';
      emit(buf, (uint16_t)(p - buf));
      __disable_irq();
      drive_off();
      __enable_irq();
      can_request = 1U;
    } else {
      print_can();
    }
  } else if (after_word(cmd, "v") != 0) {
    print_vehicle();
  } else if (cmd[0] == 'x') {
    print_extra();
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
  can_request = 0U;
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

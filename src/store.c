#include "stm32f103xb.h"
#include "store.h"
#include "console.h"
#include "sensor.h"
#include "safety.h"
#include "drive.h"

/*
 * The last page of the 64 KB device. linker.ld stops the image one page
 * short of it, so the record can never be inside the code: if the firmware
 * grows past 63 KB the link fails rather than the first save erasing a
 * piece of itself.
 */
#define STORE_ADDR        0x0800FC00UL
#define STORE_PAGE        1024U

#define STORE_MAGIC       0x53544F31UL     /* "STO1", as bytes in flash */
#define STORE_FORMAT      1U               /* this record layout */
#define STORE_MAX_PARAM   48               /* room for the list to grow */

/*
 * Halfword-aligned by construction: every field is 2 or 4 bytes wide and
 * the array lengths are even, because the flash controller programs 16 bits
 * at a time and an odd tail would need a read-modify-write of a byte that
 * cannot be rewritten without erasing the page again.
 */
typedef struct {
  uint32_t magic;
  uint16_t format;              /* STORE_FORMAT */
  uint16_t console;             /* console version that wrote it */
  uint16_t n_param;
  uint16_t hall;                /* 1 = hall_edge / hall_next are a learned table */
  int32_t  param[STORE_MAX_PARAM];
  uint16_t hall_edge[8];
  uint8_t  hall_next[8];
  uint16_t spare;               /* keeps the checksum word 4-byte aligned */
  uint32_t sum;                 /* of every halfword before it */
} store_rec;

/* One buffer, reused by save and by load: 240 bytes is a lot of stack for a
   board with 20 KB of RAM, and neither path is re-entrant. */
static store_rec rec;

/* What the page did at start-up, kept for the console. STORE_EMPTY until
   store_boot() has run, which is also what an empty page reports. */
static store_result boot_r = STORE_EMPTY;

static const store_rec *in_flash(void)
{
  return (const store_rec *)STORE_ADDR;
}

/*
 * A plain halfword sum, not a CRC.
 *
 * What it has to catch is a record whose write was cut off half way - a
 * power cut, or a reset between the erase and the last halfword. An erased
 * page reads 0xFFFF everywhere, so a truncated record differs from a good
 * one in whole halfwords at a time, which a sum catches as surely as a CRC
 * and in a tenth of the code.
 */
static uint32_t rec_sum(const store_rec *r)
{
  const uint16_t *h = (const uint16_t *)r;
  uint32_t n = (sizeof(store_rec) - sizeof(uint32_t)) / sizeof(uint16_t);
  uint32_t s = 0x1234U;
  uint32_t i;

  for (i = 0U; i < n; i++) {
    s = (s << 1) + (s >> 31) + h[i];      /* rotate, so order matters */
  }
  return s;
}

/* ------------------------------------------------------------ flash ----- */

static int flash_wait(void)
{
  while ((FLASH->SR & FLASH_SR_BSY) != 0U) {
    safety_watchdog_refresh();            /* an erase outlasts the watchdog */
  }
  if ((FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) != 0U) {
    FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
    return -1;
  }
  FLASH->SR = FLASH_SR_EOP;
  return 0;
}

static void flash_unlock(void)
{
  if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
}

static void flash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;
}

static int page_erase(void)
{
  int r;

  if (flash_wait() != 0) {
    return -1;
  }
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR  = STORE_ADDR;
  FLASH->CR |= FLASH_CR_STRT;
  r = flash_wait();
  FLASH->CR &= ~FLASH_CR_PER;
  return r;
}

static int page_program(const store_rec *src)
{
  const uint16_t *h = (const uint16_t *)src;
  uint32_t n = sizeof(store_rec) / sizeof(uint16_t);
  uint32_t i;
  int r = 0;

  FLASH->CR |= FLASH_CR_PG;
  for (i = 0U; (i < n) && (r == 0); i++) {
    *(volatile uint16_t *)(STORE_ADDR + (i * 2U)) = h[i];
    r = flash_wait();
  }
  FLASH->CR &= ~FLASH_CR_PG;
  return r;
}

/* ------------------------------------------------------------ record ---- */

static store_result examine(const store_rec *r)
{
  if (r->magic != STORE_MAGIC) {
    /* An erased page is all ones; anything else there is somebody else's. */
    return STORE_EMPTY;
  }
  if (r->sum != rec_sum(r)) {
    return STORE_BAD;
  }
  if ((r->format != STORE_FORMAT) || (r->n_param > (uint16_t)STORE_MAX_PARAM)) {
    return STORE_BAD;
  }
  if ((r->console != (uint16_t)console_version()) ||
      (r->n_param != (uint16_t)console_param_count())) {
    /* The parameters are stored by number. A different console numbers them
       differently, so applying them would set the wrong things. */
    return STORE_VERSION;
  }
  return STORE_OK;
}

store_result store_state(void)
{
  return examine(in_flash());
}

int store_saved_console(void)
{
  const store_rec *r = in_flash();

  return (r->magic == STORE_MAGIC) ? (int)r->console : 0;
}

int store_saved_count(void)
{
  const store_rec *r = in_flash();

  return (r->magic == STORE_MAGIC) ? (int)r->n_param : 0;
}

int store_saved_hall(void)
{
  const store_rec *r = in_flash();

  return ((r->magic == STORE_MAGIC) && (r->hall != 0U)) ? 1 : 0;
}

store_result store_load(void)
{
  const store_rec *r = in_flash();
  store_result     st = examine(r);
  int              i;
  int              n;

  if (st != STORE_OK) {
    return st;
  }

  /*
   * The Hall table first, and that order is not arbitrary.
   *
   * `sens 1` means "take the angle from the Hall sensors", and drive.c
   * refuses it while no table has been learned - rightly, because an angle
   * from an unlearned table is a wrong angle. Restoring the parameters
   * before the table therefore threw the sensor selection away silently:
   * the board came back from a reset on the encoder, having been put away
   * on its Hall sensors. The table is what `sens` is checked against, so
   * the table goes in first.
   */
  if (r->hall != 0U) {
    uint16_t edge[8];
    uint8_t  next[8];

    for (i = 0; i < 8; i++) {
      edge[i] = r->hall_edge[i];
      next[i] = r->hall_next[i];
    }
    __disable_irq();
    (void)sensor_hall_table_set(edge, next);
    __enable_irq();
  }

  /*
   * Through the console's own setter, not into the arrays behind it: that
   * is what clips each value to its range, moves a gain into the controller
   * it belongs to, and re-derives what depends on it. A record from a flash
   * page is input like any other - it may have been written by a board that
   * was then flashed with different limits.
   */
  n = console_param_count();
  if ((int)r->n_param < n) {
    n = (int)r->n_param;
  }
  for (i = 0; i < n; i++) {
    console_param_set(i, r->param[i]);
  }
  return STORE_OK;
}

store_result store_boot(void)
{
  boot_r = store_load();
  return boot_r;
}

store_result store_boot_result(void)
{
  return boot_r;
}

store_result store_save(void)
{
  uint16_t edge[8];
  uint8_t  next[8];
  int      i;
  int      n;
  int      r;

  /*
   * Not while anything is being driven. An erase stalls the instruction bus
   * for tens of milliseconds, so the control interrupt stops running for
   * that long - with the outputs live that is a motor commanded by whatever
   * the duty registers happened to hold.
   */
  if ((drive_idle() == 0) || (safety_armed() != 0)) {
    return STORE_BUSY;
  }

  for (i = 0; i < STORE_MAX_PARAM; i++) {
    rec.param[i] = 0;
  }
  n = console_param_count();
  if (n > STORE_MAX_PARAM) {
    n = STORE_MAX_PARAM;
  }
  for (i = 0; i < n; i++) {
    rec.param[i] = console_param_get(i);
  }

  rec.magic   = STORE_MAGIC;
  rec.format  = STORE_FORMAT;
  rec.console = (uint16_t)console_version();
  rec.n_param = (uint16_t)n;
  rec.spare   = 0U;

  __disable_irq();
  rec.hall = (sensor_hall_table_get(edge, next) != 0) ? 1U : 0U;
  __enable_irq();
  for (i = 0; i < 8; i++) {
    rec.hall_edge[i] = (rec.hall != 0U) ? edge[i] : 0U;
    rec.hall_next[i] = (rec.hall != 0U) ? next[i] : 0U;
  }

  rec.sum = rec_sum(&rec);

  /*
   * Interrupts off for the erase and the programming. They would not run
   * during the stall anyway; what this stops is an interrupt landing between
   * two halfwords and taking a path that reads the page it is in the middle
   * of being written.
   */
  __disable_irq();
  flash_unlock();
  r = page_erase();
  if (r == 0) {
    r = page_program(&rec);
  }
  flash_lock();
  __enable_irq();

  if (r != 0) {
    return STORE_FAILED;
  }

  /* Read it back through the same check the next boot will use. */
  return (examine(in_flash()) == STORE_OK) ? STORE_OK : STORE_FAILED;
}

store_result store_erase(void)
{
  int r;

  if ((drive_idle() == 0) || (safety_armed() != 0)) {
    return STORE_BUSY;
  }

  __disable_irq();
  flash_unlock();
  r = page_erase();
  flash_lock();
  __enable_irq();

  return (r == 0) ? STORE_OK : STORE_FAILED;
}

const char *store_result_word(store_result r)
{
  switch (r) {
  case STORE_OK:      return "ok";
  case STORE_EMPTY:   return "empty";
  case STORE_BAD:     return "damaged";
  case STORE_VERSION: return "version";
  case STORE_BUSY:    return "busy";
  case STORE_FAILED:  return "error";
  default:            return "?";
  }
}

const char *store_result_name(store_result r)
{
  switch (r) {
  case STORE_OK:      return "ok";
  case STORE_EMPTY:   return "empty";
  case STORE_BAD:     return "damaged";
  case STORE_VERSION: return "another console version";
  case STORE_BUSY:    return "the drive is not idle";
  case STORE_FAILED:  return "flash error";
  default:            return "?";
  }
}

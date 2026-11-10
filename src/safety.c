#include "stm32f103xb.h"
#include "safety.h"
#include "tim1.h"
#include "loop.h"
#include "outer.h"

/*
 * Watchdog period.
 *
 * LSI is nominally 40 kHz and is not trimmed, so anything derived from it is
 * approximate - between about 30 and 60 kHz across parts and temperature.
 * With the /32 prescaler and a reload of 124 that is roughly 100 ms at the
 * nominal rate, and somewhere between 66 and 133 ms in the worst cases. The
 * console loop refreshes every few milliseconds, so the margin is large in
 * the direction that matters.
 */
#define IWDG_PRESCALER_DIV32   3U
#define IWDG_RELOAD            124U

static safety_fault latched;
static int          armed;
static int          wdg_running;
static const char  *reset_cause = "unknown";

void safety_init(void)
{
  uint32_t csr = RCC->CSR;

  if ((csr & RCC_CSR_IWDGRSTF) != 0U) {
    reset_cause = "watchdog";
    /* The previous run stopped refreshing the watchdog. Whatever it was
       doing, it was not running the control loop, so come up latched. */
    latched = SAFETY_FAULT_WATCHDOG;
  } else if ((csr & RCC_CSR_SFTRSTF) != 0U) {
    reset_cause = "software";
  } else if ((csr & RCC_CSR_PORRSTF) != 0U) {
    reset_cause = "power-on";
  } else if ((csr & RCC_CSR_PINRSTF) != 0U) {
    reset_cause = "reset pin";
  }

  RCC->CSR |= RCC_CSR_RMVF;        /* clear the flags for next time */

  armed       = 0;
  wdg_running = 0;
}

void safety_watchdog_start(void)
{
  /* IWDG runs from LSI, which has to be on and ready first */
  RCC->CSR |= RCC_CSR_LSION;
  while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U) {
  }

  IWDG->KR  = 0x5555U;             /* unlock PR and RLR */
  IWDG->PR  = IWDG_PRESCALER_DIV32;
  IWDG->RLR = IWDG_RELOAD;
  IWDG->KR  = 0xAAAAU;             /* reload */
  IWDG->KR  = 0xCCCCU;             /* start; cannot be stopped except by reset */

  wdg_running = 1;
}

void safety_watchdog_refresh(void)
{
  if (wdg_running != 0) {
    IWDG->KR = 0xAAAAU;
  }
}

int safety_watchdog_running(void)
{
  return wdg_running;
}

void safety_poll(void)
{
  /*
   * BIF is set by the timer when the break input trips. The hardware has
   * already cleared MOE by the time this runs - that is the point of it -
   * so there is nothing to switch off here, only something to record.
   */
  if ((TIM1->SR & TIM_SR_BIF) != 0U) {
    TIM1->SR &= ~TIM_SR_BIF;

    if (latched == SAFETY_FAULT_NONE) {
      latched = SAFETY_FAULT_BREAK;
    }
    armed = 0;
  }
}

safety_fault safety_arm(void)
{
  if (latched != SAFETY_FAULT_NONE) {
    return latched;
  }

  /* Arming from anything but idle would start driving the moment the command
     landed, which is exactly the surprise this gate exists to prevent. */
  if ((foc_loop_mode() != LOOP_MODE_IDLE) || (outer_mode_get() != OUTER_MODE_OFF)) {
    return SAFETY_FAULT_NOT_IDLE;
  }

  tim1_outputs_enable();
  armed = 1;
  return SAFETY_FAULT_NONE;
}

void safety_disarm(void)
{
  tim1_outputs_disable();
  armed = 0;
}

int safety_armed(void)
{
  return (armed != 0) && (tim1_outputs_enabled() != 0);
}

safety_fault safety_last_fault(void)
{
  return latched;
}

void safety_clear_fault(void)
{
  TIM1->SR &= ~TIM_SR_BIF;
  latched = SAFETY_FAULT_NONE;
}

const char *safety_fault_name(safety_fault f)
{
  switch (f) {
  case SAFETY_FAULT_NONE:     return "none";
  case SAFETY_FAULT_BREAK:    return "break input";
  case SAFETY_FAULT_WATCHDOG: return "watchdog reset";
  case SAFETY_FAULT_NOT_IDLE: return "not idle";
  default:                    return "?";
  }
}

const char *safety_reset_cause(void)
{
  return reset_cause;
}

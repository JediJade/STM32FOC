/*
 * FOC groundwork for STM32F103, bare metal.
 *
 * This build does NOT drive a motor. It sets up the inverter timing exactly
 * as a FOC loop needs it, then measures itself and parks with the outputs
 * off. The order matters: the two things that destroy hardware - too little
 * dead-time and a badly timed current sample - are both measured here, with
 * nothing connected, before any gate driver exists.
 *
 * Pins:
 *   PA8  / PB13   TIM1 CH1 / CH1N   phase A
 *   PA9  / PB14   TIM1 CH2 / CH2N   phase B
 *   PA10 / PB15   TIM1 CH3 / CH3N   phase C
 *   PB12          TIM1 BKIN, break input, active low, pulled up
 *   PA6  / PA7    TIM3 CH1 / CH2, dead-time measurement inputs
 *   PA0           ADC1 channel 0, stands in for a current shunt amplifier
 *   PC13          on-board LED, blinks the number of passed self-tests
 *
 * For the dead-time measurement, fit two jumpers: PA8 -> PA6, PB13 -> PA7.
 */

#include "stm32f103xb.h"
#include "board.h"
#include "tim1.h"
#include "foc.h"
#include "selftest.h"
#include "loop.h"
#include "usb_cdc.h"
#include "console.h"
#include "safety.h"

#define LED_PIN   13U     /* PC13, active low */

static volatile uint32_t tick_ms;

void SysTick_Handler(void)
{
  tick_ms++;
}

uint32_t millis(void)
{
  return tick_ms;
}

void delay_ms(uint32_t ms)
{
  uint32_t start = tick_ms;

  while ((tick_ms - start) < ms) {
  }
}

/* HSE 8 MHz -> PLL x9 -> 72 MHz, APB1 36 MHz, APB2 72 MHz */
static void clock_init(void)
{
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

  RCC->CR |= RCC_CR_HSEON;
  while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
  }

  RCC->CFGR = RCC_CFGR_PLLSRC
            | RCC_CFGR_PLLMULL9
            | RCC_CFGR_HPRE_DIV1
            | RCC_CFGR_PPRE1_DIV2
            | RCC_CFGR_PPRE2_DIV1;

  RCC->CR |= RCC_CR_PLLON;
  while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
  }

  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
  }
}

static void led_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  GPIOC->CRH = (GPIOC->CRH & ~(0xFUL << (4U * (LED_PIN - 8U))))
             | (0x2UL << (4U * (LED_PIN - 8U)));     /* output push-pull 2 MHz */
  GPIOC->BSRR = (1UL << LED_PIN);                    /* off */
}

static void led(int on)
{
  GPIOC->BSRR = on ? (1UL << (LED_PIN + 16U)) : (1UL << LED_PIN);
}

static uint32_t count_bits(uint32_t v)
{
  uint32_t n = 0U;

  while (v != 0U) {
    n += (v & 1U);
    v >>= 1;
  }
  return n;
}

/*
 * Pull D+ low briefly so the host notices a disconnect and re-enumerates.
 * Without it, a board just reflashed over SWD often keeps the host's old
 * session and never comes back as a new device.
 */
static void usb_force_reenumerate(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  GPIOA->CRH &= ~(0xFUL << (4U * (12U - 8U)));
  GPIOA->CRH |=  (0x2UL << (4U * (12U - 8U)));   /* output push-pull 2 MHz */
  GPIOA->BSRR = (1UL << (12U + 16U));            /* drive PA12 low */
  delay_ms(100U);

  GPIOA->CRH &= ~(0xFUL << (4U * (12U - 8U)));
  GPIOA->CRH |=  (0x4UL << (4U * (12U - 8U)));   /* back to input floating */
  delay_ms(10U);
}

int main(void)
{
  clock_init();
  SysTick_Config(72000000U / 1000U);
  led_init();

  /* Before anything else overwrites the flags: why did we just restart? */
  safety_init();

  /*
   * USB comes up first, before the self-tests.
   *
   * The board's D+ pull-up presents a device to the host from the moment it
   * is powered, so the host starts enumerating within milliseconds. Bringing
   * the peripheral up only after the tests - three and a half seconds later -
   * meant the host asked, got no answer at all, and had already given up by
   * the time the device started listening: nothing appeared in the device
   * list, not even a failed device.
   */
  usb_force_reenumerate();
  usb_cdc_init();

  selftest.ok_flags = 0U;

  /* Timer first, outputs still dark: tim1_init() leaves MOE clear */
  tim1_init();

  /* Test 3 needs no pins at all, so run it before anything can switch */
  selftest_run_math();

  /* From here the pins really do switch. Nothing is connected, so this is
     safe - but it is also the last point at which that is true, which is why
     the firmware turns them back off at the end. */
  tim1_outputs_enable();

  selftest_run_deadtime();
  selftest_run_adc_timing();

  /* These two need the ADC interrupt that the previous test set up, so they
     have to come after it. Outputs are still switching - the duty writes
     these tests make are real, they just drive nothing.

     Alignment runs first because the current loop needs a trustworthy angle,
     which is the same order a real startup has to use. */
  selftest_run_align();
  selftest_run_loop();
  selftest_run_cascade();

  selftest_run_frontend();
  selftest_run_latency_sweep();

  /* Last, because it deliberately trips the fault path and leaves the
     outputs off. */
  selftest_run_break();

  tim1_outputs_disable();

  selftest.magic = SELFTEST_MAGIC;

  /*
   * With the tests done, come up as a USB serial console so the results and
   * the live loop state can be read without a debugger, and targets can be
   * commanded with a motor attached.
   */
  uint32_t passed = count_bits(selftest.ok_flags & 0xFFU);
  uint32_t banner_sent = 0U;

  console_init();

  /*
   * The watchdog starts only now. The self-tests spend seconds inside
   * delay_ms(), and sprinkling refreshes through them to keep a watchdog
   * happy would mean the tests were also testing the watchdog, badly.
   */
  safety_watchdog_start();

  while (1) {
    /* LED: solid once the host has opened the port, otherwise blinking the
       number of passed self-tests. */
    if (usb_cdc_configured() != 0) {
      led(1);

      if (banner_sent == 0U) {
        banner_sent = 1U;
        console_poll();            /* drain anything already queued */
      }

      console_poll();
      safety_watchdog_refresh();
    } else {
      safety_watchdog_refresh();
      for (uint32_t i = 0U; i < passed; i++) {
        led(1);
        delay_ms(150U);
        led(0);
        delay_ms(250U);
      }
      delay_ms(1000U);
    }

    safety_watchdog_refresh();
    delay_ms(2U);
  }
}
////

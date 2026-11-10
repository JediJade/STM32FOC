#include "stm32f103xb.h"
#include "tim1.h"

/* GPIO configuration nibbles (CRL/CRH, 4 bits per pin) */
#define GPIO_CNF_AF_PP_50MHZ  0xBu   /* MODE = 11 (50 MHz), CNF = 10 (alternate function push-pull) */
#define GPIO_CNF_IN_PULL      0x8u   /* MODE = 00, CNF = 10 (input with pull-up/pull-down) */

static void gpio_set_nibble(volatile uint32_t *reg, uint32_t pin_in_reg, uint32_t nibble)
{
  uint32_t shift = 4u * pin_in_reg;

  *reg = (*reg & ~(0xFu << shift)) | (nibble << shift);
}

void tim1_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_TIM1EN;

  /* PA8, PA9, PA10 -> CH1, CH2, CH3 */
  gpio_set_nibble(&GPIOA->CRH, 0u, GPIO_CNF_AF_PP_50MHZ);   /* PA8  */
  gpio_set_nibble(&GPIOA->CRH, 1u, GPIO_CNF_AF_PP_50MHZ);   /* PA9  */
  gpio_set_nibble(&GPIOA->CRH, 2u, GPIO_CNF_AF_PP_50MHZ);   /* PA10 */

  /* PB13, PB14, PB15 -> CH1N, CH2N, CH3N */
  gpio_set_nibble(&GPIOB->CRH, 5u, GPIO_CNF_AF_PP_50MHZ);   /* PB13 */
  gpio_set_nibble(&GPIOB->CRH, 6u, GPIO_CNF_AF_PP_50MHZ);   /* PB14 */
  gpio_set_nibble(&GPIOB->CRH, 7u, GPIO_CNF_AF_PP_50MHZ);   /* PB15 */

  /* PB12 -> BKIN, pulled up so that an unconnected pin is NOT a fault */
  gpio_set_nibble(&GPIOB->CRH, 4u, GPIO_CNF_IN_PULL);
  GPIOB->BSRR = (1u << 12);

  /* Counter: centre-aligned mode 1, no prescaler, ARR preloaded.
     Centre-aligned is what puts the low-side-on window in a predictable
     place, which is the whole reason the ADC trigger can be exact. */
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0;
  TIM1->PSC = 0u;
  TIM1->ARR = TIM1_PERIOD;

#if TIM1_TRGO_UPDATE
  /* One update event per PWM period instead of two: see tim1.h */
  TIM1->RCR = 1u;
  TIM1->CR2 = TIM_CR2_MMS_1;                  /* TRGO = update event */
#else
  TIM1->RCR = 0u;
  TIM1->CR2 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1 | TIM_CR2_MMS_2;  /* TRGO = OC4REF */
#endif

  /* CH1..CH3: PWM mode 1 with compare preload.
     Preload matters: without it a mid-period write to CCRx takes effect
     immediately and can produce one malformed pulse. */
  TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
              | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE
              | (6u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

  /* Start at 50% on all three phases: zero volts across the motor, which is
     the only safe thing to come up with. */
  TIM1->CCR1 = TIM1_PERIOD / 2u;
  TIM1->CCR2 = TIM1_PERIOD / 2u;
  TIM1->CCR3 = TIM1_PERIOD / 2u;
  TIM1->CCR4 = TIM1_PERIOD - 10u;          /* provisional ADC trigger point */

  /* Enable the three complementary pairs. CC4E stays off on purpose: CH4 is
     the ADC trigger only, and its pin is USB D-. */
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE
             | TIM_CCER_CC2E | TIM_CCER_CC2NE
             | TIM_CCER_CC3E | TIM_CCER_CC3NE;

  /* Dead-time plus break input. MOE is deliberately left 0: nothing reaches
     the pins until tim1_outputs_enable() is called. */
  TIM1->BDTR = (TIM1_DEADTIME_TICKS & TIM_BDTR_DTG) | TIM_BDTR_BKE;

  /* Load the preloaded registers, then run */
  TIM1->EGR  = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void tim1_outputs_enable(void)
{
  TIM1->BDTR |= TIM_BDTR_MOE;
}

void tim1_outputs_disable(void)
{
  TIM1->BDTR &= ~TIM_BDTR_MOE;
}

int tim1_outputs_enabled(void)
{
  return ((TIM1->BDTR & TIM_BDTR_MOE) != 0u) ? 1 : 0;
}

void tim1_set_duty(foc_abc duty)
{
  TIM1->CCR1 = (uint16_t)duty.a;
  TIM1->CCR2 = (uint16_t)duty.b;
  TIM1->CCR3 = (uint16_t)duty.c;
}

void tim1_set_adc_trigger(uint16_t count)
{
  TIM1->CCR4 = count;
}

uint16_t tim1_counter(void)
{
  return (uint16_t)TIM1->CNT;
}

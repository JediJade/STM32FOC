/*
 * Reset entry point and vector table, in C - no assembly startup file and no
 * CMSIS startup_stm32f103xb.s. The table lives in .isr_vector, which the
 * linker script places first in flash and marks KEEP.
 *
 * Carried over unchanged from the bare metal USB HID project except for the
 * interrupt slots this build actually uses: SysTick, ADC1_2 and USB_LP.
 */

#include "stm32f103xb.h"

/* Provided by the linker script */
extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata;
extern uint32_t _sbss, _ebss;

int main(void);

void Reset_Handler(void);
void Default_Handler(void);

/* Handlers implemented elsewhere */
void SysTick_Handler(void);                  /* main.c */
void ADC1_2_IRQHandler(void);                /* selftest.c */
void USB_LP_CAN1_RX0_IRQHandler(void);       /* usb_cdc.c  */

/* Everything else falls back to Default_Handler */
#define ALIAS __attribute__((weak, alias("Default_Handler")))

void NMI_Handler(void)        ALIAS;
void HardFault_Handler(void)  ALIAS;
void MemManage_Handler(void)  ALIAS;
void BusFault_Handler(void)   ALIAS;
void UsageFault_Handler(void) ALIAS;
void SVC_Handler(void)        ALIAS;
void DebugMon_Handler(void)   ALIAS;
void PendSV_Handler(void)     ALIAS;

void (* const g_pfnVectors[])(void) __attribute__((section(".isr_vector"), used)) =
{
  (void (*)(void))(&_estack),  /*  0 initial stack pointer */
  Reset_Handler,               /*  1 */
  NMI_Handler,                 /*  2 */
  HardFault_Handler,           /*  3 */
  MemManage_Handler,           /*  4 */
  BusFault_Handler,            /*  5 */
  UsageFault_Handler,          /*  6 */
  0, 0, 0, 0,                  /*  7-10 reserved */
  SVC_Handler,                 /* 11 */
  DebugMon_Handler,            /* 12 */
  0,                           /* 13 reserved */
  PendSV_Handler,              /* 14 */
  SysTick_Handler,             /* 15 */

  Default_Handler,             /* IRQ  0 WWDG */
  Default_Handler,             /* IRQ  1 PVD */
  Default_Handler,             /* IRQ  2 TAMPER */
  Default_Handler,             /* IRQ  3 RTC */
  Default_Handler,             /* IRQ  4 FLASH */
  Default_Handler,             /* IRQ  5 RCC */
  Default_Handler,             /* IRQ  6 EXTI0 */
  Default_Handler,             /* IRQ  7 EXTI1 */
  Default_Handler,             /* IRQ  8 EXTI2 */
  Default_Handler,             /* IRQ  9 EXTI3 */
  Default_Handler,             /* IRQ 10 EXTI4 */
  Default_Handler,             /* IRQ 11 DMA1_Channel1 */
  Default_Handler,             /* IRQ 12 DMA1_Channel2 */
  Default_Handler,             /* IRQ 13 DMA1_Channel3 */
  Default_Handler,             /* IRQ 14 DMA1_Channel4 */
  Default_Handler,             /* IRQ 15 DMA1_Channel5 */
  Default_Handler,             /* IRQ 16 DMA1_Channel6 */
  Default_Handler,             /* IRQ 17 DMA1_Channel7 */
  ADC1_2_IRQHandler,           /* IRQ 18 ADC1_2  <- injected conversion done */
  Default_Handler,             /* IRQ 19 USB_HP_CAN1_TX */
  USB_LP_CAN1_RX0_IRQHandler,  /* IRQ 20 USB_LP_CAN1_RX0  <- the CDC console */
  Default_Handler,             /* IRQ 21 CAN1_RX1 */
  Default_Handler,             /* IRQ 22 CAN1_SCE */
  Default_Handler,             /* IRQ 23 EXTI9_5 */
  Default_Handler,             /* IRQ 24 TIM1_BRK */
  Default_Handler,             /* IRQ 25 TIM1_UP */
  Default_Handler,             /* IRQ 26 TIM1_TRG_COM */
  Default_Handler,             /* IRQ 27 TIM1_CC */
  Default_Handler,             /* IRQ 28 TIM2 */
  Default_Handler,             /* IRQ 29 TIM3 */
  Default_Handler,             /* IRQ 30 TIM4 */
  Default_Handler,             /* IRQ 31 I2C1_EV */
  Default_Handler,             /* IRQ 32 I2C1_ER */
  Default_Handler,             /* IRQ 33 I2C2_EV */
  Default_Handler,             /* IRQ 34 I2C2_ER */
  Default_Handler,             /* IRQ 35 SPI1 */
  Default_Handler,             /* IRQ 36 SPI2 */
  Default_Handler,             /* IRQ 37 USART1 */
  Default_Handler,             /* IRQ 38 USART2 */
  Default_Handler,             /* IRQ 39 USART3 */
  Default_Handler,             /* IRQ 40 EXTI15_10 */
  Default_Handler,             /* IRQ 41 RTC_Alarm */
  Default_Handler,             /* IRQ 42 USBWakeUp */
};

void Reset_Handler(void)
{
  uint32_t *src = &_sidata;
  uint32_t *dst = &_sdata;

  /* Copy initialised data from flash to RAM (no memcpy: -nostdlib) */
  while (dst < &_edata) {
    *dst++ = *src++;
  }

  /* Zero .bss */
  for (dst = &_sbss; dst < &_ebss; dst++) {
    *dst = 0U;
  }

  /* Vector table is at the start of flash */
  SCB->VTOR = FLASH_BASE;

  main();

  while (1) {
  }
}

void Default_Handler(void)
{
  while (1) {
  }
}

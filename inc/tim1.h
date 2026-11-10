/*
 * TIM1 as a three-phase inverter driver: centre-aligned PWM, complementary
 * outputs with hardware dead-time, break input, and CH4 used purely as the
 * ADC trigger position.
 *
 * Pins (TIM1 with no AFIO remap):
 *   PA8  CH1    PB13 CH1N      phase A high / low
 *   PA9  CH2    PB14 CH2N      phase B high / low
 *   PA10 CH3    PB15 CH3N      phase C high / low
 *   PB12 BKIN   break input, active low, internal pull-up
 *
 * CH4 (which would be PA11) is never enabled as an output: PA11 is USB D-,
 * and CH4 is only needed internally, as the trigger that starts the ADC.
 */
#ifndef TIM1_H
#define TIM1_H

#include <stdint.h>
#include "foc.h"

/* 72 MHz / (2 * 1800) = 20 kHz, the usual compromise: above audible range,
   still slow enough that switching losses and dead-time distortion stay small */
#define TIM1_PERIOD          1800u
#define TIM1_PWM_HZ          20000u

/* Dead-time: with CKD = 0 one tick is 1/72 MHz = 13.889 ns, and DTG values
   below 128 count ticks directly, so 72 ticks = 1.000 us. selftest.c measures
   what actually comes out of the pins rather than trusting this comment. */
#define TIM1_DEADTIME_TICKS  72u

/*
 * Where the ADC trigger comes from.
 *
 * 1 = update event with RCR = 1. A centre-aligned counter raises its update
 *     event at both ends of the ramp, so on its own that is two triggers per
 *     PWM period - measured here as 34-39 kHz on a 20 kHz PWM before this
 *     switch existed. The repetition counter is the part that fixes it: with
 *     RCR = 1 the event is raised every second over/underflow, giving exactly
 *     one trigger per PWM period, at the bottom of the ramp where all three
 *     low-side switches are on and the phase currents can be measured.
 *
 * 0 = OC4REF, so CCR4 chooses the instant freely. More flexible, and what you
 *     want once the sample point has to be moved off the bottom, but it is
 *     the configuration that produced two triggers per period.
 *
 * selftest.c measures the rate and the jitter either way; flip this and
 * re-run to see the difference rather than taking the comment's word for it.
 */
#define TIM1_TRGO_UPDATE     1

void     tim1_init(void);
void     tim1_outputs_enable(void);
void     tim1_outputs_disable(void);
int      tim1_outputs_enabled(void);

/* Compare values in [0, TIM1_PERIOD]; foc_svpwm() produces them. */
void     tim1_set_duty(foc_abc duty);

/* Counter value at which OC4REF fires TRGO and so starts the ADC. */
void     tim1_set_adc_trigger(uint16_t count);

uint16_t tim1_counter(void);

#endif /* TIM1_H */

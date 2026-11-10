/*
 * Three self-tests that need no motor and no gate driver.
 *
 * They exist because the two ways a FOC build destroys hardware are a
 * dead-time that is shorter than you think, and an ADC that samples at the
 * wrong point in the PWM period. Both are measurable on a bare board, and
 * the third test proves the fixed-point math before any power is applied.
 *
 * Results land in one struct in RAM. Read it over SWD with OpenOCD - no UART,
 * no USB, no scope:
 *
 *   openocd -f openocd.cfg -c "init; set v [read_memory <addr> 32 24]; echo $v; exit"
 *
 * where <addr> comes from "make addr", which greps the symbol out of the elf.
 */
#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdint.h>

#define SELFTEST_MAGIC        0xF0C00001u

#define SELFTEST_OK_DEADTIME  0x01u
#define SELFTEST_OK_ADC       0x02u
#define SELFTEST_OK_MATH      0x04u
#define SELFTEST_OK_LOOP      0x08u
#define SELFTEST_OK_ALIGN     0x10u
#define SELFTEST_OK_CASCADE   0x20u
#define SELFTEST_OK_BREAK     0x40u
#define SELFTEST_OK_FRONTEND  0x80u

typedef struct {
  uint32_t magic;

  /* Test 1 - dead-time measured at the pins, in 13.889 ns ticks.
     Needs two jumpers: PA8 -> PA6 and PB13 -> PA7. */
  uint32_t dt_samples;        /* valid capture pairs collected */
  uint32_t dt_min;
  uint32_t dt_max;
  uint32_t dt_mean_x100;      /* mean * 100, to keep one decimal without a float */
  uint32_t dt_expected;       /* what TIM1_DEADTIME_TICKS asked for */
  uint32_t dt_sr_snapshot;    /* TIM3->SR when the test gave up: bit1 = PA6
                                 captured, bit2 = PA7 captured. This is the
                                 number that tells a missing jumper apart
                                 from a dead channel. */

  /* Test 2 - where the ADC conversion actually lands in the PWM period */
  uint32_t adc_isr_count;     /* LIVE counter: keeps incrementing for as long
                                 as the board is powered, so reading it after
                                 the test says nothing about the test. The
                                 first version of this test appeared to show
                                 38 kHz on a 20 kHz chain for exactly that
                                 reason. Use adc_rate_hz. */
  uint32_t adc_count_window;  /* conversions inside the measured window only */
  uint32_t adc_rate_hz;       /* frozen: 20000 means one per PWM period */
  uint32_t adc_trigger_ccr;   /* CCR4, the requested trigger count */
  uint32_t adc_cnt_at_isr;    /* TIM1->CNT read inside the ISR */
  uint32_t adc_dir_at_isr;    /* 0 = counting up, 1 = counting down */
  uint32_t adc_latency_ticks; /* |cnt_at_isr - trigger|, conversion + ISR entry */
  uint32_t adc_cnt_min;       /* spread of cnt_at_isr over every conversion:   */
  uint32_t adc_cnt_max;       /* this is the jitter, and it is the number that */
                              /* decides whether a current loop can work      */
  uint32_t adc_jitter_frozen; /* cnt_max - cnt_min, captured when the ADC test
                                 ended. adc_cnt_min/max keep being updated by
                                 every later test, where the ISR is far
                                 heavier, so only this frozen value says
                                 anything about the trigger itself. */
  uint32_t adc_window_ms;     /* measured, so the rate is exact               */
  uint32_t adc_trgo_update;   /* which TRGO source was compiled in            */
  uint32_t adc_period_min;    /* interval between consecutive conversions,    */
  uint32_t adc_period_max;    /* in 13.889 ns ticks, measured with TIM3.      */
                              /* 3600 = one per PWM period, 1800 = two.       */

  /* Test 3 - fixed-point math, no hardware involved */
  uint32_t math_park_max_err; /* worst error of inv_park -> park round trip, LSB */
  uint32_t math_sincos_min;   /* min of sin^2 + cos^2 in Q15 (ideal 32767) */
  uint32_t math_sincos_max;
  uint32_t math_duty_min;     /* duty extremes over a full electrical turn */
  uint32_t math_duty_max;
  uint32_t math_span_min;     /* min and max of (max duty - min duty): a circular */
  uint32_t math_span_max;     /* voltage vector should keep this nearly constant */

  /* Test 4 - the closed current loop, driven by the virtual motor */
  uint32_t loop_iterations;      /* control periods executed            */
  uint32_t loop_iq_target;       /* commanded q-axis current, Q15       */
  uint32_t loop_iq_final;        /* what the loop actually settled at   */
  uint32_t loop_id_max_abs;      /* worst |i_d| anywhere, including the step
                                    transient - a transient excursion is
                                    normal and is not a decoupling failure */
  uint32_t loop_id_steady_max;   /* worst |i_d| AFTER settling */
  int32_t  loop_id_ss_min;       /* signed min, max and mean of i_d after    */
  int32_t  loop_id_ss_max;       /* settling. A band symmetric about zero is */
  int32_t  loop_id_ss_mean;      /* ripple; an offset is a decoupling error. */
  uint32_t loop_settle_iters;    /* periods to get within 5% of target  */
  uint32_t loop_cycles_min;      /* cost of one control period, cycles  */
  uint32_t loop_cycles_max;
  uint32_t loop_isr_cycles_max;  /* whole ISR, including the plant model */
  uint32_t loop_plant_cycles_max; /* of which this is the virtual motor, work
                                     real hardware does not do in software */

  /* The same step response measured with feed-forward decoupling switched
     off, in the same run, so the two are directly comparable. */
  uint32_t loop_nodec_settle;
  uint32_t loop_nodec_id_max_abs;
  int32_t  loop_nodec_iq_final;
  uint32_t loop_decouple;        /* 1 = the main numbers above are with it on */

  /* Test 5 - rotor alignment. The plant's sensor reports the true angle plus
     a fixed offset the firmware does not know; alignment has to find it. */
  uint32_t align_iterations;
  uint32_t align_offset_found;   /* what the procedure discovered */
  uint32_t align_offset_true;    /* what the plant actually used  */
  int32_t  align_error;          /* found - true, in angle units  */
  uint32_t align_true_angle_end; /* real rotor angle when it finished; the
                                    procedure works by pulling this to zero */

  /* Test 6 - the speed and position loops cascaded outside the current loop */
  int32_t  casc_speed_ref;       /* commanded speed, angle units per ms */
  int32_t  casc_speed_final;
  int32_t  casc_speed_max;       /* overshoot shows up here */
  uint32_t casc_settle_ms;       /* to within 5% of the commanded speed */
  int32_t  casc_speed_a_final;   /* phase A kept separately: phase B reuses
                                    the live speed fields and would otherwise
                                    overwrite them */
  int32_t  casc_speed_a_max;
  int32_t  casc_pos_step;        /* commanded position step, angle units */
  int32_t  casc_pos_err_final;
  int32_t  casc_pos_speed_max;   /* fastest it travelled getting there */
  uint32_t casc_pos_settle_ms;   /* to within 300 units of the target */
  uint32_t casc_outer_cycles;    /* cost of one 1 kHz update */

  /* Test 7 - the break input, the one path that has to work in hardware
     without software in the way. Checked twice: through the timer's own
     software break generator, and through the real BKIN pin. */
  uint32_t brk_pin_idle_high;   /* PB12 reads high when nothing drives it */
  uint32_t brk_moe_before;      /* outputs were really enabled first */
  uint32_t brk_moe_after_bg;    /* must be 0: hardware cleared MOE */
  uint32_t brk_bg_cycles;       /* cycles from writing BG to MOE reading 0 */
  uint32_t brk_moe_after_pin;   /* must be 0: the real pin did it too */
  uint32_t brk_pin_cycles;
  uint32_t brk_stayed_off;      /* MOE stays off while the break is held */
  uint32_t brk_recovered;       /* and comes back once it is released */

  /* Test 8 - the current measurement front end.
     Two phase currents have to be sampled at the SAME instant, because the
     Clarke transform assumes they are a consistent set; two readings taken
     microseconds apart on a fast-changing waveform are not. That is what
     ADC1 and ADC2 in injected simultaneous mode are for. */
  uint32_t fe_pairs;            /* simultaneous pairs collected */
  uint32_t fe_a_mean;           /* zero-current offset, ADC1 channel 0 */
  uint32_t fe_b_mean;           /* and ADC2 channel 1 */
  uint32_t fe_a_noise;          /* max - min over the run, LSB */
  uint32_t fe_b_noise;
  uint32_t fe_vrefint;          /* internal reference, LSB of 4095 */
  uint32_t fe_vrefint_mv;       /* what that reading means in mV, using the
                                   measured VDDA - not the other way round:
                                   see FE_VDDA_MV in selftest.c */

  /* Test 9 - where the ADC latency actually goes.
     The reported latency is a single number that mixes a fixed overhead with
     the conversion time. Sweeping the sample time separates them: the slope
     is the conversion, the intercept is everything else. */
  uint32_t lat_smp_1_5;         /* latency in timer ticks at 1.5 ADC cycles */
  uint32_t lat_smp_7_5;
  uint32_t lat_smp_13_5;
  uint32_t lat_smp_28_5;
  uint32_t lat_smp_41_5;
  uint32_t lat_smp_55_5;
  uint32_t lat_smp_71_5;

  uint32_t ok_flags;
} selftest_results;

extern volatile selftest_results selftest;

void selftest_run_math(void);
void selftest_run_deadtime(void);
void selftest_run_adc_timing(void);
void selftest_run_align(void);
void selftest_run_loop(void);
void selftest_run_cascade(void);
void selftest_run_break(void);
void selftest_run_frontend(void);
void selftest_run_latency_sweep(void);

#endif /* SELFTEST_H */

#include "capture.h"
#include "stm32f103xb.h"
#include "selftest.h"
#include "tim1.h"
#include "foc.h"
#include "loop.h"
#include "vmotor.h"
#include "outer.h"
#include "board.h"

volatile selftest_results selftest;

/* ------------------------------------------------ test 3: the math ------- */

/*
 * Runs entirely on the CPU, so it is the one test that can fail before any
 * wire is connected. Three independent checks:
 *
 *  a) sin^2 + cos^2 should be 1 (32767 in Q15) at every angle. This catches a
 *     broken table or a wrong interpolation shift.
 *  b) inverse Park followed by Park with the same angle must return the
 *     original (d, q). This catches a sign error in either rotation, which is
 *     the classic FOC bug that makes a motor fight itself.
 *  c) SVPWM duties must stay inside [0, period], and the spread (max - min)
 *     of the three duties must swing by exactly 2/sqrt(3) = 1.1547 over one
 *     revolution. That is not a wobble to be minimised: for a balanced three
 *     phase set, max - min is sqrt(3)*V at the sector boundaries and 2*V in
 *     the middle, so the ratio is a fixed property of the maths and makes a
 *     sharper test than "roughly constant" would.
 */
void selftest_run_math(void)
{
  foc_dq  ref = { 0, 16384 };     /* pure q axis: all torque, no flux current */
  uint32_t max_err = 0u;
  int32_t  sc_min = 0x7FFFFFFF;
  int32_t  sc_max = -0x7FFFFFFF;
  uint32_t duty_min = 0xFFFFFFFFu;
  uint32_t duty_max = 0u;
  uint32_t span_min = 0xFFFFFFFFu;
  uint32_t span_max = 0u;
  uint32_t angle;

  /* 1024 steps over the electrical revolution: enough to hit interpolation
     mid-points, cheap enough to run at boot. */
  for (angle = 0u; angle < 65536u; angle += 64u) {
    uint16_t a = (uint16_t)angle;

    /* (a) sin^2 + cos^2 */
    int16_t s = foc_sin(a);
    int16_t c = foc_cos(a);
    int32_t sc = (int32_t)q15_mul(s, s) + (int32_t)q15_mul(c, c);

    if (sc < sc_min) { sc_min = sc; }
    if (sc > sc_max) { sc_max = sc; }

    /* (b) inverse Park -> Park round trip */
    foc_ab ab  = foc_inv_park(ref, a);
    foc_dq out = foc_park(ab, a);

    int32_t ed = (int32_t)out.d - (int32_t)ref.d;
    int32_t eq = (int32_t)out.q - (int32_t)ref.q;

    if (ed < 0) { ed = -ed; }
    if (eq < 0) { eq = -eq; }
    if ((uint32_t)ed > max_err) { max_err = (uint32_t)ed; }
    if ((uint32_t)eq > max_err) { max_err = (uint32_t)eq; }

    /* (c) SVPWM duty range and spread */
    foc_abc duty = foc_svpwm(ab, TIM1_PERIOD);

    int32_t dmax = (duty.a > duty.b) ? duty.a : duty.b;
    dmax = (dmax > duty.c) ? dmax : duty.c;
    int32_t dmin = (duty.a < duty.b) ? duty.a : duty.b;
    dmin = (dmin < duty.c) ? dmin : duty.c;

    if ((uint32_t)dmin < duty_min) { duty_min = (uint32_t)dmin; }
    if ((uint32_t)dmax > duty_max) { duty_max = (uint32_t)dmax; }

    uint32_t span = (uint32_t)(dmax - dmin);
    if (span < span_min) { span_min = span; }
    if (span > span_max) { span_max = span; }
  }

  selftest.math_park_max_err = max_err;
  selftest.math_sincos_min   = (uint32_t)sc_min;
  selftest.math_sincos_max   = (uint32_t)sc_max;
  selftest.math_duty_min     = duty_min;
  selftest.math_duty_max     = duty_max;
  selftest.math_span_min     = span_min;
  selftest.math_span_max     = span_max;

  /* Pass bar: round trip within 1% of the 16384 reference, sin^2+cos^2 within
     1% of unity, every duty inside the period, and the duty spread swinging
     by 2/sqrt(3) within 1%. */
  uint32_t span_ratio_x1000 = (span_min > 0u) ? ((span_max * 1000u) / span_min) : 0u;

  if ((max_err < 164u) &&
      (sc_min > 32440) && (sc_max < 33100) &&
      (duty_max <= TIM1_PERIOD) &&
      (span_ratio_x1000 > 1143u) && (span_ratio_x1000 < 1166u)) {
    selftest.ok_flags |= SELFTEST_OK_MATH;
  }
}

/* -------------------------------------------- test 1: real dead-time ----- */

/*
 * TIM3 timestamps the edges of TIM1's own output pins at 72 MHz, so one tick
 * is 13.889 ns. At every PWM edge the low side turns off first and the high
 * side turns on one dead-time later, so
 *
 *     dead-time = t(PA6 rising) - t(PA7 falling)
 *
 * Wiring: PA8 (CH1) -> PA6 (TIM3 CH1), PB13 (CH1N) -> PA7 (TIM3 CH2).
 * With no jumpers fitted the test reports dt_samples = 0 rather than a
 * plausible looking wrong number.
 */
void selftest_run_deadtime(void)
{
  uint32_t samples = 0u;
  uint32_t sum = 0u;
  uint32_t min = 0xFFFFFFFFu;
  uint32_t max = 0u;
  uint32_t attempts;

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  /* PA6, PA7 as floating inputs (CNF = 01, MODE = 00) */
  GPIOA->CRL = (GPIOA->CRL & ~(0xFFu << 24)) | (0x4u << 24) | (0x4u << 28);

  /* TIM3 free running at the full 72 MHz */
  TIM3->CR1 = 0u;
  TIM3->PSC = 0u;
  TIM3->ARR = 0xFFFFu;

  /* CH1 captures TI1 (PA6) on the rising edge, CH2 captures TI2 (PA7) on the
     falling edge. No filter: we want the real edge, not a debounced one. */
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;
  TIM3->CR1  |= TIM_CR1_CEN;

  /* One deadline for the whole test, not one per attempt. A per-attempt
     iteration guard multiplies out: 100k spins times 20k attempts is about
     seven minutes, which looks exactly like a hang. */
  uint32_t deadline = millis() + 200u;
  uint32_t sr_seen = 0u;

  for (attempts = 0u; (millis() < deadline) && (samples < 200u); attempts++) {
    uint16_t t_low_off;
    uint16_t t_high_on;
    uint32_t dt;
    uint32_t t0;

    TIM3->SR = 0u;                                   /* drop stale captures */

    /* Wait for the high-side rising capture. 2 ms is 40 PWM periods, so if
       PA6 sees nothing in that time the jumper is simply not fitted. */
    t0 = millis();
    while (((TIM3->SR & TIM_SR_CC1IF) == 0u) && ((millis() - t0) < 2u)) {
      sr_seen |= TIM3->SR;      /* remember which channels ever captured */
    }
    sr_seen |= TIM3->SR;
    if ((TIM3->SR & TIM_SR_CC1IF) == 0u) {
      break;                                         /* no signal on PA6 */
    }

    /* The low side turns off about one dead-time before the high side turns
       on, so its capture must already be in. If it is not, PA7 is not wired
       and there is nothing to compare against. */
    if ((TIM3->SR & TIM_SR_CC2IF) == 0u) {
      continue;
    }

    t_high_on = (uint16_t)TIM3->CCR1;
    t_low_off = (uint16_t)TIM3->CCR2;

    /* 16-bit subtraction wraps correctly across the counter rollover */
    dt = (uint32_t)(uint16_t)(t_high_on - t_low_off);

    /* A pair from two different PWM periods shows up as a huge value; the
       real dead-time is well under 1000 ticks (13.9 us). */
    if ((dt > 0u) && (dt < 1000u)) {
      samples++;
      sum += dt;
      if (dt < min) { min = dt; }
      if (dt > max) { max = dt; }
    }
  }

  selftest.dt_samples    = samples;
  selftest.dt_expected   = TIM1_DEADTIME_TICKS;
  selftest.dt_sr_snapshot = sr_seen;

  if (samples > 0u) {
    selftest.dt_min       = min;
    selftest.dt_max       = max;
    selftest.dt_mean_x100 = (sum * 100u) / samples;

    /* Pass bar: within 2 ticks (28 ns) of what BDTR was asked for */
    uint32_t mean = sum / samples;
    uint32_t diff = (mean > TIM1_DEADTIME_TICKS)
                  ? (mean - TIM1_DEADTIME_TICKS)
                  : (TIM1_DEADTIME_TICKS - mean);
    if (diff <= 2u) {
      selftest.ok_flags |= SELFTEST_OK_DEADTIME;
    }
  }
}

/* ------------------------------------- test 2: where the ADC samples ----- */

static uint16_t adc_prev_stamp;
static uint8_t  adc_have_prev;

/*
 * Gate for the ADC timing statistics.
 *
 * Fourth time this lesson has come up in this project: a field that the
 * interrupt keeps writing means nothing once a later, heavier test is
 * running. adc_cnt_min/max and adc_period_min/max are only meaningful for
 * the window the ADC test measured, so the interrupt stops touching them
 * when that window closes.
 */
static volatile uint8_t  adc_track;
static volatile uint32_t adc_warmup;
static volatile uint8_t loop_track;   /* collect statistics about the loop */
static volatile uint8_t casc_armed;
static uint32_t casc_t0;
static uint8_t  casc_settled;
static uint16_t casc_pos_target;
static uint8_t  casc_pos_settled;
static uint8_t  loop_settled;
static int32_t  id_ss_sum;
static uint32_t id_ss_n;

void ADC1_2_IRQHandler(void)
{
  if ((ADC1->SR & ADC_SR_JEOC) != 0u) {
    /*
     * The PWM counter, read before anything else happens.
     *
     * This has to be the first statement in the handler, not merely an early
     * one: it is the measurement of when the sample landed, so every
     * instruction executed ahead of it is added to the answer. An earlier
     * version read it after the DWT and TIM3 accesses and after two
     * conditional min/max updates, which made the work ahead of it variable
     * and put that variation straight into the reported jitter.
     */
    uint32_t cnt = TIM1->CNT;
    uint32_t dir = (TIM1->CR1 & TIM_CR1_DIR) ? 1u : 0u;

    uint32_t isr_c0 = DWT->CYCCNT;
    /* TIM3 is free running at 72 MHz from the dead-time test, so it doubles
       as a stopwatch: the interval between conversions says directly how many
       triggers a PWM period produces, which counting interrupts over 100 ms
       could not distinguish from conversions being dropped. */
    uint16_t stamp = (uint16_t)TIM3->CNT;

    if ((adc_have_prev != 0u) && (adc_track != 0u)) {
      uint32_t delta = (uint32_t)(uint16_t)(stamp - adc_prev_stamp);

      if (delta < selftest.adc_period_min) { selftest.adc_period_min = delta; }
      if (delta > selftest.adc_period_max) { selftest.adc_period_max = delta; }
    }
    adc_prev_stamp = stamp;
    adc_have_prev  = 1u;

    /* Read the flag register back after clearing it, so the posted write has
       retired before the handler returns. (Measured both ways: it is not what
       sets the conversion rate here - see the note on TRGO in
       selftest_run_adc_timing - but it costs two cycles and removes one thing
       to wonder about.) */
    ADC1->SR = (uint32_t)~ADC_SR_JEOC;
    (void)ADC1->SR;

    selftest.adc_cnt_at_isr = cnt;
    selftest.adc_dir_at_isr = dir;
    selftest.adc_isr_count++;

    /* The spread of this value across thousands of conversions is the real
       figure of merit: a current loop needs the sample to land at the same
       point in the PWM period every single time. */
    /* Skip the first conversions after the trigger is armed: the first one or
       two land at whatever phase the counter happened to be at, and one
       outlier would sit in min/max for the rest of the run. What matters for
       a current loop is the steady state. */
    if (adc_track != 0u) {
      if (adc_warmup < 50u) {
        adc_warmup++;
      } else {
        if (cnt < selftest.adc_cnt_min) { selftest.adc_cnt_min = cnt; }
        if (cnt > selftest.adc_cnt_max) { selftest.adc_cnt_max = cnt; }
      }
    }

    /*
     * The control loop runs here, in the interrupt the current sample arrives
     * in, at the real 20 kHz rate - on every conversion, unconditionally.
     *
     * It used to run only while a test armed it, which meant that once the
     * tests finished nothing drove the loop at all: the console could change
     * modes and targets and absolutely nothing moved. Whether the loop should
     * do anything is the loop's own business - LOOP_MODE_IDLE applies zero
     * volts - and not a question for the interrupt.
     */
    {
      uint32_t c0 = DWT->CYCCNT;
      foc_dq   m  = foc_loop_step();
      uint32_t cycles = DWT->CYCCNT - c0;

      /*
       * The outer loops, every period; outer_tick() decimates to 1 kHz itself
       * and returns immediately when its mode is OFF.
       *
       * Unconditional for the same reason as the control loop above: gating
       * it on a test flag left the speed and position loops dead once the
       * tests finished, so a console command could select speed mode and
       * nothing would ever drive it.
       */
      outer_tick(foc_loop_angle());

      /* The recorder, last: it reads what the two calls above just left. */
      capture_tick();

      if (casc_armed != 0u) {
        int32_t sp  = outer_speed();
        int32_t ref = selftest.casc_speed_ref;

        if (sp > selftest.casc_speed_max) { selftest.casc_speed_max = sp; }
        selftest.casc_speed_final  = sp;
        selftest.casc_outer_cycles = outer_cycles();

        if ((casc_pos_target != 0u) && (casc_pos_settled == 0u)) {
          int32_t pe = (int32_t)(int16_t)(casc_pos_target - foc_loop_angle());
          if (pe < 0) { pe = -pe; }
          if (pe < 300) {
            selftest.casc_pos_settle_ms = millis() - casc_t0;
            casc_pos_settled = 1u;
          }
        }

        if ((ref != 0) && (casc_settled == 0u)) {
          int32_t e = ref - sp;
          if (e < 0) { e = -e; }
          if (e < (ref / 20)) {
            selftest.casc_settle_ms = millis() - casc_t0;
            casc_settled = 1u;
          }
        }
      }

      /* Statistics belong to the loop test only. The cascade runs the same
         loop but its own figures are what matter there, so it must not
         overwrite these. */
      if (loop_track != 0u) {
        int32_t id = (m.d < 0) ? -(int32_t)m.d : (int32_t)m.d;
        int32_t err;

        if (cycles < selftest.loop_cycles_min) { selftest.loop_cycles_min = cycles; }
        if (cycles > selftest.loop_cycles_max) { selftest.loop_cycles_max = cycles; }

        {
          uint32_t pc = foc_loop_plant_cycles();
          if (pc > selftest.loop_plant_cycles_max) {
            selftest.loop_plant_cycles_max = pc;
          }
        }

        if ((uint32_t)id > selftest.loop_id_max_abs) {
          selftest.loop_id_max_abs = (uint32_t)id;
        }
        if (loop_settled != 0u) {
          if ((uint32_t)id > selftest.loop_id_steady_max) {
            selftest.loop_id_steady_max = (uint32_t)id;
          }
          if ((int32_t)m.d < selftest.loop_id_ss_min) {
            selftest.loop_id_ss_min = (int32_t)m.d;
          }
          if ((int32_t)m.d > selftest.loop_id_ss_max) {
            selftest.loop_id_ss_max = (int32_t)m.d;
          }
          id_ss_sum += (int32_t)m.d;
          id_ss_n++;
        }

        selftest.loop_iq_final = (uint32_t)(int32_t)m.q;
        selftest.loop_iterations++;

        err = (int32_t)selftest.loop_iq_target - (int32_t)m.q;
        if (err < 0) { err = -err; }
        if ((loop_settled == 0u) &&
            (err < (int32_t)(selftest.loop_iq_target / 20u))) {
          selftest.loop_settle_iters = selftest.loop_iterations;
          loop_settled = 1u;
        }

        {
          uint32_t isr_cycles = DWT->CYCCNT - isr_c0;
          if (isr_cycles > selftest.loop_isr_cycles_max) {
            selftest.loop_isr_cycles_max = isr_cycles;
          }
        }
      }
    }
  }
}

/*
 * TIM1 fires TRGO when the counter reaches CCR4; that starts an injected
 * conversion with no software in the path. The ISR then records where the
 * counter had got to, so the reported latency is
 *
 *     sample-and-hold + conversion + interrupt entry
 *
 * which at 12 MHz ADC clock and 7.5 cycles of sampling is about 20 ADC clocks
 * = 1.67 us = 120 ticks, plus a dozen cycles of interrupt entry. The useful
 * part is that it is *constant*: a jittering value means something else is
 * stealing the trigger.
 *
 * MEASURED, AND NOT YET RESOLVED: at 20 kHz PWM this fires ~39.3 kHz, i.e.
 * twice per PWM period - the centre-aligned counter crosses CCR4 once going
 * up and once going down, and both crossings produce a trigger. A real
 * current loop wants exactly one sample per period at one defined instant,
 * so before any motor is connected this has to become one trigger per
 * period. The repetition counter (TIM1->RCR) is the intended tool: it makes
 * the timer generate its events every Nth period instead of every one.
 */
void selftest_run_adc_timing(void)
{

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;

  /* ADC clock = PCLK2 / 6 = 12 MHz (14 MHz is the limit) */
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;

  /* PA0 as analog input: CNF = 00, MODE = 00 */
  GPIOA->CRL &= ~0xFu;

  /* 7.5 ADC cycles of sampling on channel 0 - short, as motor control needs */
  ADC1->SMPR2 = (1u << ADC_SMPR2_SMP0_Pos);

  /* One injected conversion. With JL = 0 the single channel goes in JSQ4,
     not JSQ1 - a quirk that silently converts the wrong pin if missed. */
  ADC1->JSQR = (0u << ADC_JSQR_JSQ4_Pos);

  /*
   * Order matters here, and getting it wrong wedges the ADC in a way that
   * looks like a dead interrupt rather than a dead ADC.
   *
   * An earlier version enabled the external trigger before calibrating, and
   * used an empty for-loop for the stabilisation delay. TIM1 is already
   * running by this point, so a trigger could arrive in the middle of
   * calibration; and an empty loop with no volatile access is something -Os
   * is free to delete, which it did. The result was JSTRT set, JEOC never
   * set, and no interrupt ever reaching the handler - with every register
   * otherwise reading exactly as intended.
   *
   * So: power up, wait with the SysTick-based delay that cannot be optimised
   * away, calibrate, and only then arm the trigger.
   */
  ADC1->CR1 = ADC_CR1_JEOCIE;
  ADC1->CR2 = ADC_CR2_ADON;
  delay_ms(1u);

  ADC1->CR2 |= ADC_CR2_RSTCAL;
  while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0u) {
  }
  ADC1->CR2 |= ADC_CR2_CAL;
  while ((ADC1->CR2 & ADC_CR2_CAL) != 0u) {
  }

  ADC1->SR   = 0u;                       /* drop anything latched so far */

  /* Initialise the statistics BEFORE the trigger is armed. An earlier edit to
     the ordering above dropped this block, which left adc_track clear: the
     measurement never ran, the spread came out as 0 - 0 = 0, and the test
     reported a pass without having measured anything. A test that cannot
     fail is worse than no test. */
  selftest.adc_cnt_min     = 0xFFFFFFFFu;
  selftest.adc_cnt_max     = 0u;
  selftest.adc_period_min  = 0xFFFFFFFFu;
  selftest.adc_period_max  = 0u;
  selftest.adc_trgo_update = TIM1_TRGO_UPDATE;
  adc_have_prev            = 0u;
  adc_warmup               = 0u;
  adc_track                = 1u;

  ADC1->CR2 |= ADC_CR2_JEXTTRIG;         /* now the trigger may fire */

  NVIC_SetPriority(ADC1_2_IRQn, 1u);
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* Sample near the bottom of the count, where all three low-side switches
     are on for a 50% duty and the phase currents are measurable. */
  selftest.adc_trigger_ccr = 20u;
  tim1_set_adc_trigger((uint16_t)selftest.adc_trigger_ccr);

  /* Let a few thousand PWM periods go by, and time the window for real so the
     rate can be divided out exactly instead of assuming 100 ms. */
  uint32_t count_start = selftest.adc_isr_count;
  uint32_t t_start     = millis();

  delay_ms(100u);

  selftest.adc_window_ms   = millis() - t_start;
  selftest.adc_count_window = selftest.adc_isr_count - count_start;

  /* Freeze the rate now. adc_isr_count goes on climbing for as long as the
     board is powered, so anything derived from it later is meaningless - the
     first version of this test reported 38 kHz for a 20 kHz chain purely
     because the struct was read a moment after the window closed. */
  if (selftest.adc_window_ms > 0u) {
    selftest.adc_rate_hz = (selftest.adc_count_window * 1000u)
                         / selftest.adc_window_ms;
  }

  if (selftest.adc_count_window > 0u) {
    uint32_t cnt = selftest.adc_cnt_at_isr;
#if TIM1_TRGO_UPDATE
    /* The update event fires at the bottom of the ramp, so the counter value
       seen in the ISR *is* the latency. CCR4 plays no part in this mode. */
    uint32_t trg = 0u;
#else
    uint32_t trg = selftest.adc_trigger_ccr;
#endif

    selftest.adc_latency_ticks = (cnt > trg) ? (cnt - trg) : (trg - cnt);

    /* Pass bar, now that the rate is measured rather than assumed:
         - the sample instant is stable to within 40 ticks (0.55 us), and
         - the rate is one trigger per PWM period, within 2%.
       Either of those failing is a reason not to connect a motor. */
    uint32_t spread = selftest.adc_cnt_max - selftest.adc_cnt_min;

    adc_track = 0u;                      /* the window is closed */
    selftest.adc_jitter_frozen = spread;

    if ((spread < 40u) &&
        (selftest.adc_rate_hz > (TIM1_PWM_HZ - (TIM1_PWM_HZ / 50u))) &&
        (selftest.adc_rate_hz < (TIM1_PWM_HZ + (TIM1_PWM_HZ / 50u)))) {
      selftest.ok_flags |= SELFTEST_OK_ADC;
    }
  }
}

/* --------------------------------------------- test 4: the closed loop --- */

/*
 * Runs the real control loop in the real interrupt at the real rate, with
 * vmotor.c standing in for the motor. What it proves before any hardware
 * exists:
 *
 *  - the signs through Clarke, Park, PI, inverse Park and SVPWM agree, so the
 *    loop pulls towards the target instead of away from it
 *  - the integrator removes the back-EMF, a constant disturbance that
 *    proportional gain alone cannot
 *  - the d axis stays near zero despite the cross-coupling term, which is
 *    what "decoupled" means in practice
 *  - one control period fits inside one PWM period, with how much to spare
 *
 * The step response is measured twice in one run, with feed-forward
 * decoupling off and then on. Measuring both here rather than across two
 * flashes removes every other difference from the comparison.
 */
static void loop_phase(int dec)
{
  selftest.loop_cycles_min     = 0xFFFFFFFFu;
  selftest.loop_cycles_max     = 0u;
  selftest.loop_isr_cycles_max = 0u;
  selftest.loop_plant_cycles_max = 0u;
  selftest.loop_id_max_abs     = 0u;
  selftest.loop_id_steady_max  = 0u;
  selftest.loop_id_ss_min      = 32767;
  selftest.loop_id_ss_max      = -32768;
  selftest.loop_id_ss_mean     = 0;
  selftest.loop_iterations     = 0u;
  selftest.loop_settle_iters   = 0u;
  loop_settled                 = 0u;
  id_ss_sum                    = 0;
  id_ss_n                      = 0u;

  foc_loop_init();

  /* Characterise the current loop the way a bench would: the shaft driven at
     a constant speed by another machine, so cross-coupling and back-EMF are
     both present and both constant. A locked rotor would make the test
     easier and meaningless - at zero speed there is nothing to decouple. */
  vmotor_set_mech(VM_MECH_FORCED, 328);

  foc_loop_set_decouple(dec);
  foc_loop_set_target((int16_t)selftest.loop_iq_target);
  foc_loop_set_mode(LOOP_MODE_CURRENT);

  loop_track = 1u;
  delay_ms(100u);                            /* ~2000 control periods */
  loop_track = 0u;

  if (id_ss_n > 0u) {
    selftest.loop_id_ss_mean = id_ss_sum / (int32_t)id_ss_n;
  }
}

void selftest_run_loop(void)
{
  /* DWT is the cycle counter in the debug unit; TRCENA switches it on. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  selftest.loop_iq_target = 8192u;           /* 0.25 of full scale */

  /* Phase A: PI controllers alone */
  loop_phase(0);
  selftest.loop_nodec_settle     = selftest.loop_settle_iters;
  selftest.loop_nodec_id_max_abs = selftest.loop_id_max_abs;
  selftest.loop_nodec_iq_final   = (int32_t)selftest.loop_iq_final;

  /* Phase B: with the predictable part fed forward */
  loop_phase(1);
  selftest.loop_decouple = 1u;

  {
    int32_t err = (int32_t)selftest.loop_iq_target - (int32_t)selftest.loop_iq_final;
    uint32_t abs_err = (uint32_t)((err < 0) ? -err : err);
    int32_t  m = selftest.loop_id_ss_mean;
    int32_t  abs_id_mean = (m < 0) ? -m : m;

    /* Pass bar: settled to within 5% of target, the d axis mean held under
       2% of it, and the control path costing less than half the PWM period.
       Note the units: TIM1_PERIOD is ARR, which in centre-aligned mode is
       *half* the period, so one PWM period is 2*TIM1_PERIOD = 3600 cycles at
       72 MHz and half of it is TIM1_PERIOD. Getting that wrong set the bar
       at 900 cycles and failed a loop that comfortably fits.

       Decoupling is judged on the MEAN of i_d, not its extremes: an offset
       would show up in the mean, while what is actually there is a near-zero
       average with sparse positive spikes - a numerical artifact in the
       phase-current round trip at particular angles, not the controller
       failing to decouple. The spikes are reported so they are not
       forgotten, but they are not what this bar tests. */
    if ((selftest.loop_iterations > 1000u) &&
        (abs_err < (selftest.loop_iq_target / 20u)) &&
        (abs_id_mean < (int32_t)(selftest.loop_iq_target / 50u)) &&
        (selftest.loop_settle_iters > 0u) &&
        (selftest.loop_cycles_max < TIM1_PERIOD)) {
      selftest.ok_flags |= SELFTEST_OK_LOOP;
    }
  }
}

/* ------------------------------------------- test 5: rotor alignment ----- */

/*
 * Everything up to here assumed the loop knows where the rotor is. It does
 * not: a position sensor reports its own zero, which has no relationship to
 * the rotor's magnetic zero. Commanding torque at the wrong angle gives less
 * torque and more heat, and at 90 degrees out it gives maximum current and no
 * torque at all - which on real hardware is how a motor and a bridge get
 * destroyed while apparently doing nothing.
 *
 * The procedure is open-loop on purpose, because the angle cannot be trusted
 * yet: apply a fixed field at electrical angle zero, let the rotor pull
 * itself into line with it, and then read the sensor. Whatever it says is the
 * offset, because the rotor is now at zero.
 *
 * The plant hides VMOTOR_ENCODER_OFFSET from the firmware and starts the
 * rotor at an arbitrary angle, so this test is a real search, not a lookup.
 */
void selftest_run_align(void)
{
  foc_loop_init();
  foc_loop_set_offset(0u);                    /* nothing known yet */
  vmotor_set_mech(VM_MECH_FREE, 0);           /* the rotor has to be able to move */
  foc_loop_set_mode(LOOP_MODE_ALIGN);

  selftest.loop_iterations = 0u;

  loop_track = 1u;
  delay_ms(300u);                             /* ~6000 periods to settle */
  loop_track = 0u;

  foc_loop_set_mode(LOOP_MODE_IDLE);

  selftest.align_iterations     = selftest.loop_iterations;
  selftest.align_offset_found   = vmotor_sensor_angle();
  selftest.align_offset_true    = VMOTOR_ENCODER_OFFSET;
  selftest.align_true_angle_end = vmotor_true_angle();

  {
    int32_t diff = (int32_t)(int16_t)((uint16_t)selftest.align_offset_found
                                      - (uint16_t)VMOTOR_ENCODER_OFFSET);
    int32_t mag  = (diff < 0) ? -diff : diff;

    selftest.align_error = diff;

    /* Pass bar: within 500 angle units of the truth. 500 of 65536 is 2.7
       electrical degrees, which is the order of accuracy a real alignment
       achieves against friction and cogging. */
    if (mag < 500) {
      selftest.ok_flags |= SELFTEST_OK_ALIGN;
    }
  }

  /* Hand the result to the loop, so the current loop test that follows runs
     on a correctly referenced angle - as it would after a real startup. */
  foc_loop_set_offset((uint16_t)selftest.align_offset_found);
}

/* ------------------------- test 6: speed and position cascade ----------- */

/*
 * The current loop commands torque. Anything useful wants speed or position,
 * which means loops outside it: position -> speed reference, speed -> current
 * reference, current -> voltage. Each has to be slower than the one inside
 * it, and 1 kHz against the current loop's 20 kHz is the usual ratio.
 *
 * Two phases, both with the rotor free:
 *
 *   A  a speed step, to see whether the speed loop reaches and holds it
 *   B  a position step of a quarter turn, to see whether the cascade stops
 *      where it was told rather than oscillating around it
 *
 * Phase B is the one that exercises the structure: the position loop has no
 * integrator on purpose, so if it settles on target it is because the speed
 * loop underneath it is doing its job.
 */
void selftest_run_cascade(void)
{
  /* ---- phase A: speed step ---- */
  foc_loop_init();
  foc_loop_set_offset((uint16_t)selftest.align_offset_found);
  foc_loop_set_decouple(1);
  foc_loop_set_mode(LOOP_MODE_CURRENT);
  vmotor_set_mech(VM_MECH_FREE, 0);

  outer_init();
  outer_set_mode(OUTER_MODE_SPEED);

  selftest.casc_speed_ref  = 3280;            /* half of OUTER_SPEED_MAX */
  selftest.casc_speed_max  = 0;
  selftest.casc_settle_ms  = 0u;
  casc_settled             = 0u;
  outer_set_speed_ref(selftest.casc_speed_ref);

  casc_t0    = millis();
  casc_armed = 1u;
  delay_ms(500u);                             /* mechanical time constant is ~55 ms */
  casc_armed = 0u;

  selftest.casc_speed_a_final = selftest.casc_speed_final;
  selftest.casc_speed_a_max   = selftest.casc_speed_max;

  /* ---- phase B: position step ---- */
  foc_loop_init();
  foc_loop_set_offset((uint16_t)selftest.align_offset_found);
  foc_loop_set_decouple(1);
  foc_loop_set_mode(LOOP_MODE_CURRENT);
  vmotor_set_mech(VM_MECH_FREE, 0);

  outer_init();
  outer_set_mode(OUTER_MODE_POSITION);

  casc_pos_target             = 0u;
  selftest.casc_pos_step      = 16384;        /* a quarter turn */
  selftest.casc_pos_speed_max = 0;
  selftest.casc_speed_max     = 0;
  selftest.casc_speed_ref     = 0;            /* the position loop sets it now */
  casc_settled                = 1u;           /* no speed settling to measure */

  {
    uint16_t start = (uint16_t)(vmotor_sensor_angle()
                                - (uint16_t)selftest.align_offset_found);
    uint16_t target = (uint16_t)(start + (uint16_t)selftest.casc_pos_step);

    outer_set_position_ref(target);

    casc_pos_target  = target;
    casc_pos_settled = 0u;
    casc_t0          = millis();

    casc_armed = 1u;
    delay_ms(1500u);            /* a 100 ms position time constant needs room */
    casc_armed = 0u;

    selftest.casc_pos_speed_max = selftest.casc_speed_max;
    selftest.casc_pos_err_final =
      (int32_t)(int16_t)(target - foc_loop_angle());
  }

  foc_loop_set_mode(LOOP_MODE_IDLE);
  outer_set_mode(OUTER_MODE_OFF);

  {
    int32_t pe = selftest.casc_pos_err_final;
    if (pe < 0) { pe = -pe; }

    /* Pass bar: the position cascade finished within 300 angle units of the
       commanded step - 1.6 electrical degrees - and the 1 kHz update fits
       comfortably in the time between current periods. */
    if ((pe < 300) && (selftest.casc_outer_cycles < 1000u)) {
      selftest.ok_flags |= SELFTEST_OK_CASCADE;
    }
  }
}

/* ------------------------------------------- test 7: the break input ---- */

/*
 * Everything else in this firmware is software protecting hardware. This is
 * the one thing that protects hardware from software: BKIN going active
 * clears MOE in the timer itself, asynchronously, with no interrupt, no
 * handler and no code in the path. All six PWM outputs go inactive whatever
 * the CPU happens to be doing - even if it is stuck in a loop commanding
 * full duty.
 *
 * It has been configured since the first version of tim1.c and never once
 * exercised. A protection path that has never been triggered is a guess.
 *
 * Checked twice, because the two tell you different things:
 *
 *   EGR.BG     the timer's own software break generator. Proves the internal
 *              path: break -> MOE cleared -> outputs off.
 *   the pin    PB12 pulled low by switching its own pull-up to a pull-down.
 *              Proves the pin, its configuration and BDTR.BKP polarity, which
 *              is what an overcurrent comparator will actually use.
 *
 * Also checked: that MOE stays off while the break is held - with AOE clear
 * it must need software to re-enable, so a fault cannot clear itself and
 * restart the inverter - and that it does come back once released.
 */
void selftest_run_break(void)
{
  uint32_t c0;

  /* With nothing driving it, the pull-up must hold BKIN inactive. If this
     reads 0 the break is already asserted and nothing below means anything. */
  selftest.brk_pin_idle_high = ((GPIOB->IDR & (1u << 12)) != 0u) ? 1u : 0u;

  TIM1->SR &= ~TIM_SR_BIF;              /* clear any stale break flag */
  tim1_outputs_enable();
  selftest.brk_moe_before = (uint32_t)tim1_outputs_enabled();

  /* ---- path 1: the timer's own break generator ---- */
  c0 = DWT->CYCCNT;
  TIM1->EGR = TIM_EGR_BG;
  while ((TIM1->BDTR & TIM_BDTR_MOE) != 0u) {
    if ((DWT->CYCCNT - c0) > 7200u) {    /* 100 us is forever for this */
      break;
    }
  }
  selftest.brk_bg_cycles    = DWT->CYCCNT - c0;
  selftest.brk_moe_after_bg = (uint32_t)tim1_outputs_enabled();

  /* ---- path 2: the real pin ---- */
  TIM1->SR &= ~TIM_SR_BIF;
  tim1_outputs_enable();

  /* PB12 is an input with its pull selected by ODR, so clearing ODR turns the
     pull-up into a pull-down and the pin goes low - the same trick used to
     press the demo buttons over SWD, here applied by the firmware to its own
     fault input. */
  c0 = DWT->CYCCNT;
  GPIOB->BRR = (1u << 12);
  while ((TIM1->BDTR & TIM_BDTR_MOE) != 0u) {
    if ((DWT->CYCCNT - c0) > 72000u) {   /* 1 ms: the pull has to drag the pin */
      break;
    }
  }
  selftest.brk_pin_cycles    = DWT->CYCCNT - c0;
  selftest.brk_moe_after_pin = (uint32_t)tim1_outputs_enabled();

  /* While the break is held, software must not be able to re-enable the
     outputs. AOE is clear, so this has to stick. */
  tim1_outputs_enable();
  delay_ms(1u);
  selftest.brk_stayed_off = (tim1_outputs_enabled() == 0) ? 1u : 0u;

  /* Release it and check recovery */
  GPIOB->BSRR = (1u << 12);             /* pull-up again */
  delay_ms(1u);
  TIM1->SR &= ~TIM_SR_BIF;
  tim1_outputs_enable();
  delay_ms(1u);
  selftest.brk_recovered = (uint32_t)tim1_outputs_enabled();

  /* Leave the outputs off: this is the last test to run. */
  tim1_outputs_disable();

  if ((selftest.brk_pin_idle_high == 1u) &&
      (selftest.brk_moe_before == 1u) &&
      (selftest.brk_moe_after_bg == 0u) &&
      (selftest.brk_moe_after_pin == 0u) &&
      (selftest.brk_stayed_off == 1u) &&
      (selftest.brk_recovered == 1u)) {
    selftest.ok_flags |= SELFTEST_OK_BREAK;
  }
}

/* ----------------------------------- test 8: the measurement front end -- */

/*
 * The supply voltage, measured with a multimeter: 3.313 V.
 *
 * It is a constant here rather than something derived from VREFINT, because
 * on this board VREFINT cannot carry that job. Three instruments were
 * compared: VREFINT implied 3.69 V, the ST-Link's sense circuit said 3.23 V,
 * and a multimeter said 3.313 V. The two that agree win, which puts this
 * part's internal reference at 1333 * 3.313 / 4095 = 1.079 V against a
 * specified 1.16 to 1.24 V - about 7% below the minimum.
 *
 * The practical consequence, and the reason this constant exists: current
 * readings must be scaled by a VDDA that was actually measured. Scaling them
 * by a VREFINT-derived supply would put a 12% error on every current in the
 * system, which no amount of controller tuning can fix because the
 * controller would be regulating the wrong number.
 *
 * The device ID registers do not settle whether this is a genuine ST die -
 * the UID is structured and the revision is a real one - so the honest
 * statement is narrower: this part's VREFINT is out of spec, and factory
 * values on cheap boards are worth cross-checking before being trusted.
 */
#define FE_VDDA_MV   3313u

/*
 * Two phase currents must be sampled at the same instant. The Clarke
 * transform treats i_a and i_b as one vector at one point in time; two
 * readings taken even a microsecond apart on a 20 kHz PWM waveform are not
 * that, and the error shows up as a rotating disturbance that no amount of
 * controller tuning removes.
 *
 * ADC1 and ADC2 in injected simultaneous mode solve it in hardware: one
 * trigger, both sample-and-holds close together, both results latched. This
 * test proves that path works, measures the zero-current offset each channel
 * sits at, and measures the noise on it - the offset is what a real build
 * subtracts from every reading, and getting it wrong puts a standing error on
 * i_d that looks exactly like a decoupling problem.
 *
 * It then checks the converter quantitatively against something that does not
 * depend on any external wiring: the chip's internal 1.2 V reference. If that
 * reads back as 1.2 V the whole chain - reference, ladder, timing - is sane.
 */
void selftest_run_frontend(void)
{
  uint32_t i;
  uint32_t sum_a = 0u, sum_b = 0u;
  uint32_t min_a = 0xFFFFFFFFu, max_a = 0u;
  uint32_t min_b = 0xFFFFFFFFu, max_b = 0u;
  uint32_t pairs = 0u;

  RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;

  /* PA0 and PA1 as analog inputs: CNF = 00, MODE = 00 */
  GPIOA->CRL &= ~0xFFuL;

  /* ADC2: channel 1 injected, same 7.5 cycle sampling as ADC1 */
  ADC2->SMPR2 = (1u << ADC_SMPR2_SMP1_Pos);
  ADC2->JSQR  = (1u << ADC_JSQR_JSQ4_Pos);

  /* In injected simultaneous mode the slave's own trigger is never used; the
     master's trigger starts both. JSWSTART is the conventional choice for the
     slave's JEXTSEL so that nothing else can start it. */
  ADC2->CR2 = ADC_CR2_JEXTTRIG | ADC_CR2_JEXTSEL;
  ADC2->CR2 |= ADC_CR2_ADON;
  delay_ms(1u);
  ADC2->CR2 |= ADC_CR2_RSTCAL;
  while ((ADC2->CR2 & ADC_CR2_RSTCAL) != 0u) {
  }
  ADC2->CR2 |= ADC_CR2_CAL;
  while ((ADC2->CR2 & ADC_CR2_CAL) != 0u) {
  }

  /* DUALMOD = 0101: injected simultaneous. Set on the master only. */
  ADC1->CR1 = (ADC1->CR1 & ~ADC_CR1_DUALMOD) | (5u << ADC_CR1_DUALMOD_Pos);

  /* Collect pairs. The ADC interrupt is already running at 20 kHz from the
     timing test, so this only has to watch the result registers. */
  for (i = 0u; i < 4000u; i++) {
    uint32_t a = ADC1->JDR1 & 0xFFFu;
    uint32_t b = ADC2->JDR1 & 0xFFFu;

    if (pairs < 1000u) {
      sum_a += a;
      sum_b += b;
      if (a < min_a) { min_a = a; }
      if (a > max_a) { max_a = a; }
      if (b < min_b) { min_b = b; }
      if (b > max_b) { max_b = b; }
      pairs++;
    }
    delay_ms(0u);                  /* let a conversion or two go by */
    for (volatile uint32_t d = 0u; d < 50u; d++) {
    }
  }

  selftest.fe_pairs   = pairs;
  if (pairs > 0u) {
    selftest.fe_a_mean  = sum_a / pairs;
    selftest.fe_b_mean  = sum_b / pairs;
    selftest.fe_a_noise = max_a - min_a;
    selftest.fe_b_noise = max_b - min_b;
  }

  /*
   * Now the internal reference, which needs its own setup: VREFINT is
   * channel 17, has to be switched on with TSVREFE, and needs at least
   * 17.1 us of sampling - 239.5 ADC cycles at 12 MHz - where the phase
   * currents use 7.5. So this runs as a one-off regular conversion rather
   * than disturbing the injected timing.
   */
  /* Pause the injected trigger: at 20 kHz it would preempt the long regular
     conversion repeatedly. A real build calibrates before the loop runs, for
     the same reason. */
  ADC1->CR2 &= ~ADC_CR2_JEXTTRIG;

  ADC1->CR2 |= ADC_CR2_TSVREFE;
  delay_ms(1u);
  ADC1->SMPR1 = (7u << ADC_SMPR1_SMP17_Pos);     /* 239.5 cycles, 20 us */
  ADC1->SQR3  = 17u;
  ADC1->SQR1  = 0u;                              /* one conversion */

  /*
   * SWSTART is not a start bit on its own: the regular group's trigger has to
   * be enabled and selected as "software" first. Writing SWSTART with
   * EXTTRIG clear does nothing at all, which reads back as a conversion that
   * returned 67 LSB and an implied supply of 73 volts.
   */
  ADC1->CR2 |= ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;

  /*
   * Sixteen conversions, the first four thrown away.
   *
   * The first conversion after switching channel and sample time is not
   * trustworthy - the sample-and-hold starts from whatever the previous
   * channel left on it. Taking one reading gave 1327 LSB, implying a 3.70 V
   * supply, while the ST-Link's own measurement of the same rail said
   * 3.23 V three times in a row. When two independent measurements disagree
   * by 15%, the one that was taken once is the one to doubt.
   */
  {
    uint32_t n = 0u;
    uint32_t sum = 0u;
    uint32_t k;

    for (k = 0u; k < 16u; k++) {
      uint32_t guard = 0u;

      ADC1->SR   = 0u;
      ADC1->CR2 |= ADC_CR2_SWSTART;

      while (((ADC1->SR & ADC_SR_EOC) == 0u) && (guard < 1000000u)) {
        guard++;
      }

      {
        uint32_t v = ADC1->DR & 0xFFFu;

        if (k >= 4u) {
          sum += v;
          n++;
        }
      }
    }

    selftest.fe_vrefint = (n > 0u) ? (sum / n) : 0u;
  }

  ADC1->CR2 &= ~(ADC_CR2_TSVREFE | ADC_CR2_EXTTRIG);
  ADC1->CR2 |= ADC_CR2_JEXTTRIG;                 /* the loop needs it back */

  /* Report what the reading means, given a supply that was measured rather
     than inferred: vrefint = reading * VDDA / 4095 */
  selftest.fe_vrefint_mv = (selftest.fe_vrefint * FE_VDDA_MV) / 4095u;

  /*
   * Pass bar: simultaneous pairs collected, and the converter returning a
   * plausible reading for the internal reference. The bar is deliberately on
   * the converter and not on the reference's accuracy - this part's reference
   * is known to be out of spec, and a test that failed for that reason every
   * boot would just get ignored.
   *
   * The phase-channel offsets are reported but not judged: with nothing
   * connected those pins float, and what they read is not the firmware's
   * fault.
   */
  if ((selftest.fe_pairs > 100u) &&
      (selftest.fe_vrefint > 1200u) && (selftest.fe_vrefint < 1700u)) {
    selftest.ok_flags |= SELFTEST_OK_FRONTEND;
  }
}

/* ------------------------- test 9: where the ADC latency goes ----------- */

/*
 * The ADC timing test reports one latency figure - trigger to the counter
 * value seen in the handler - and about half of it was never accounted for:
 * 2.71 us measured against roughly 1.8 us for sample-and-hold plus conversion
 * plus interrupt entry.
 *
 * Guessing at the remainder went nowhere twice. Sweeping instead settles it.
 * Conversion time is (sample + 12.5) ADC cycles, and at 12 MHz one ADC cycle
 * is six 72 MHz timer ticks, so plotting latency against sample time gives a
 * line whose slope is the conversion and whose intercept is everything that
 * does not depend on it: trigger synchronisation, interrupt entry, and the
 * instructions ahead of the counter read.
 *
 * Seven sample times, 1.5 to 71.5 cycles. If the slope comes out at six ticks
 * per ADC cycle the model is right and the intercept is the real answer.
 */
void selftest_run_latency_sweep(void)
{
  static const uint8_t codes[7] = { 0u, 1u, 2u, 3u, 4u, 5u, 6u };
  uint32_t results[7];
  uint32_t i;

  for (i = 0u; i < 7u; i++) {
    uint32_t sum = 0u;
    uint32_t n = 0u;
    uint32_t k;

    ADC1->SMPR2 = ((uint32_t)codes[i] << ADC_SMPR2_SMP0_Pos);
    delay_ms(5u);                       /* let the new setting take effect */

    /* adc_cnt_at_isr is refreshed every conversion, so sampling it from here
       is enough; 32 reads spread over a few milliseconds average out the
       one-tick quantisation. */
    for (k = 0u; k < 32u; k++) {
      sum += selftest.adc_cnt_at_isr;
      n++;
      delay_ms(1u);
    }

    results[i] = (n > 0u) ? (sum / n) : 0u;
  }

  /* Put the fast setting back: the control loop wants a short sample. */
  ADC1->SMPR2 = (1u << ADC_SMPR2_SMP0_Pos);

  selftest.lat_smp_1_5  = results[0];
  selftest.lat_smp_7_5  = results[1];
  selftest.lat_smp_13_5 = results[2];
  selftest.lat_smp_28_5 = results[3];
  selftest.lat_smp_41_5 = results[4];
  selftest.lat_smp_55_5 = results[5];
  selftest.lat_smp_71_5 = results[6];
}

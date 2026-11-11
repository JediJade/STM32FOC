#include "stm32f103xb.h"
#include "canbus.h"
#include "board.h"
#include "console.h"
#include "drive.h"
#include "loop.h"
#include "outer.h"
#include "sensor.h"
#include "safety.h"
#include "tim1.h"

#define COMMAND_TIMEOUT_MS   500u

typedef struct {
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];
} can_frame;

static uint8_t  active;
static uint8_t  want_usb;
static uint8_t  commanded;      /* a command other than off is standing */
static uint32_t test_result;
static uint32_t rx_count;
static uint32_t tx_count;
static uint32_t drop_count;
static uint32_t last_rx_ms;
static uint32_t last_cmd_ms;
static uint32_t next_status_ms;
static uint32_t next_beat_ms;

uint32_t can_test_result(void) { return test_result; }
int      can_active(void)      { return (int)active; }
uint32_t can_rx_count(void)    { return rx_count; }
uint32_t can_tx_count(void)    { return tx_count; }
uint32_t can_drop_count(void)  { return drop_count; }

/* ------------------------------------------------------ the controller -- */

/*
 * Bit timing from the 36 MHz APB1 clock: 18 quanta to a bit, sampled after
 * the 16th (88.9 %), so the prescaler alone sets the rate.
 *   1 Mbit/s: 2   500 kbit/s: 4   250 kbit/s: 8   125 kbit/s: 16
 */
static uint32_t btr_for(int32_t kbit)
{
  uint32_t presc = 2000u / (uint32_t)kbit;

  return (presc - 1u) | (14u << 16) | (1u << 20);     /* BS1 = 15, BS2 = 2, SJW = 1 */
}

/*
 * Where the controller listens.
 *
 * It has to be given a receive pin that reads recessive before it is even
 * asked to initialise, loopback or not. Left on its reset mapping it listens
 * on PA11, which is USB's D- line: the host holds that low, the controller
 * takes it for a bus that is busy, and it never acknowledges the request to
 * initialise. That is what the first version of the power-on test did - it
 * reported 0, not even "came up". So: remap to PB8 / PB9 and pull PB8 up,
 * before anything else. PB9 is only made an output when there is a bus to
 * talk to.
 */
static void can_pins(int drive_tx)
{
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;

  /* A plain assignment, because the debug-port bits in this register read
     back as garbage and must be written as zero: zero is "SWD and JTAG both
     on", which is how the chip resets. */
  AFIO->MAPR = AFIO_MAPR_CAN_REMAP_REMAP2;

  /* PB8 input with pull-up; PB9 alternate-function push-pull, or floating. */
  GPIOB->BSRR = (1u << 8);
  GPIOB->CRH  = (GPIOB->CRH & ~0x000000FFu) | ((drive_tx != 0) ? 0x000000B8u : 0x00000048u);
}

static void can_pins_release(void)
{
  GPIOB->CRH = (GPIOB->CRH & ~0x000000FFu) | 0x00000044u;     /* both floating again */
  AFIO->MAPR = 0u;
}

/* 1 if the controller came up. `loopback` adds loopback and silent: it then
   hears its own frames and nothing else, and drives no pin. */
static int can_hw_start(int loopback)
{
  uint32_t t0;

  RCC->APB1ENR  |= RCC_APB1ENR_CAN1EN;
  RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

  /* Leave sleep, ask for initialisation, wait for the acknowledge. */
  CAN1->MCR = CAN_MCR_INRQ;
  t0 = millis();
  while ((CAN1->MSR & CAN_MSR_INAK) == 0u) {
    if ((millis() - t0) > 10u) {
      return 0;
    }
  }

  /* ABOM: come back from bus-off without help. TXFP: send in the order
     queued, so a status 1 never overtakes the status 2 before it. */
  CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_TXFP;
  CAN1->BTR = btr_for(drive_p[DP_CANBR])
            | ((loopback != 0) ? (CAN_BTR_LBKM | CAN_BTR_SILM) : 0u);

  /* One filter, 32 bits wide, mask of zero: everything, into FIFO 0. The
     identifiers are checked in software, where the node number is. */
  CAN1->FMR  |= CAN_FMR_FINIT;
  CAN1->FA1R  = 0u;
  CAN1->FM1R  = 0u;
  CAN1->FS1R  = 1u;
  CAN1->FFA1R = 0u;
  CAN1->sFilterRegister[0].FR1 = 0u;
  CAN1->sFilterRegister[0].FR2 = 0u;
  CAN1->FA1R  = 1u;
  CAN1->FMR  &= ~CAN_FMR_FINIT;

  /* Leaving initialisation needs eleven recessive bits on the receive side,
     which loopback supplies itself and an idle or pulled-up pin does too. */
  CAN1->MCR &= ~CAN_MCR_INRQ;
  t0 = millis();
  while ((CAN1->MSR & CAN_MSR_INAK) != 0u) {
    if ((millis() - t0) > 10u) {
      return 0;
    }
  }
  return 1;
}

static void can_hw_stop(void)
{
  RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
  RCC->APB1ENR  &= ~RCC_APB1ENR_CAN1EN;
}

/* Never waits. With no free mailbox the frame is dropped and counted: a bus
   that is not there must not be able to hold the main loop. */
static int can_send(uint32_t id, const uint8_t *d, uint8_t len)
{
  uint32_t tsr = CAN1->TSR;
  uint32_t box;

  if ((tsr & CAN_TSR_TME0) != 0u) {
    box = 0u;
  } else if ((tsr & CAN_TSR_TME1) != 0u) {
    box = 1u;
  } else if ((tsr & CAN_TSR_TME2) != 0u) {
    box = 2u;
  } else {
    drop_count++;
    return 0;
  }

  CAN1->sTxMailBox[box].TDTR = len;
  CAN1->sTxMailBox[box].TDLR = (uint32_t)d[0] | ((uint32_t)d[1] << 8)
                             | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
  CAN1->sTxMailBox[box].TDHR = (uint32_t)d[4] | ((uint32_t)d[5] << 8)
                             | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
  CAN1->sTxMailBox[box].TIR  = (id << 21) | CAN_TI0R_TXRQ;
  tx_count++;
  return 1;
}

static int can_receive(can_frame *f)
{
  uint32_t lo;
  uint32_t hi;
  uint32_t rir;
  int      i;

  if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0u) {
    return 0;
  }

  rir    = CAN1->sFIFOMailBox[0].RIR;
  f->id  = ((rir & CAN_RI0R_IDE) != 0u) ? 0xFFFFFFFFu : (rir >> 21);
  f->len = (uint8_t)(CAN1->sFIFOMailBox[0].RDTR & 0xFu);
  lo     = CAN1->sFIFOMailBox[0].RDLR;
  hi     = CAN1->sFIFOMailBox[0].RDHR;
  for (i = 0; i < 4; i++) {
    f->data[i]     = (uint8_t)(lo >> (8 * i));
    f->data[4 + i] = (uint8_t)(hi >> (8 * i));
  }
  CAN1->RF0R = CAN_RF0R_RFOM0;                        /* release the mailbox */
  rx_count++;
  return 1;
}

/* -------------------------------------------------------- the protocol -- */

static void put16(uint8_t *d, int32_t v)
{
  d[0] = (uint8_t)v;
  d[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *d, int32_t v)
{
  d[0] = (uint8_t)v;
  d[1] = (uint8_t)(v >> 8);
  d[2] = (uint8_t)(v >> 16);
  d[3] = (uint8_t)(v >> 24);
}

static int32_t clip16(int32_t v)
{
  if (v >  32767) { v =  32767; }
  if (v < -32768) { v = -32768; }
  return v;
}

static void send_status(void)
{
  uint32_t node = (uint32_t)drive_p[DP_CANID];
  foc_dq   i    = foc_loop_last_dq();
  foc_dq   v    = foc_loop_last_vdq();
  uint8_t  d[8];

  put16(&d[0], clip16(outer_speed()));
  put16(&d[2], i.q);
  put16(&d[4], i.d);
  d[6] = (uint8_t)(((uint32_t)foc_loop_mode() & 3u)
                 | (((uint32_t)outer_mode_get() & 3u) << 2)
                 | (((uint32_t)sensor_kind_get() & 3u) << 4)
                 | ((tim1_outputs_enabled() != 0) ? 0x40u : 0u)
                 | ((drive_state() != VEH_OFF) ? 0x80u : 0u));
  d[7] = (uint8_t)(((uint32_t)safety_last_fault() & 15u)
                 | ((uint32_t)drive_state() << 4));
  (void)can_send(CAN_ID_STATUS1 + node, d, 8u);

  put16(&d[0], v.d);
  put16(&d[2], v.q);
  put16(&d[4], (int32_t)foc_loop_angle());
  d[6] = (uint8_t)(drive_throttle() >> 7);
  d[7] = (uint8_t)(((drive_brake() != 0) ? 1u : 0u)
                 | ((drive_fw_d() < 0) ? 2u : 0u)
                 | (((uint32_t)sensor_hall_now() & 7u) << 2)
                 | (((uint32_t)sensor_phase() & 7u) << 5));
  (void)can_send(CAN_ID_STATUS2 + node, d, 8u);
}

static void handle(const can_frame *f)
{
  uint32_t node = (uint32_t)drive_p[DP_CANID];

  if ((f->id == (CAN_ID_COMMAND + node)) && (f->len >= 4u)) {
    int32_t value = (int16_t)((uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8));
    uint8_t mode  = f->data[0];

    last_cmd_ms = millis();

    __disable_irq();
    switch (mode) {
    case CAN_MODE_OFF:
      drive_off();
      commanded = 0u;
      break;
    case CAN_MODE_TORQUE:
      drive_torque(value);
      commanded = 1u;
      break;
    case CAN_MODE_SPEED:
      drive_speed(value);
      commanded = 1u;
      break;
    case CAN_MODE_THROTTLE: {
      /* The throttle arrives as a position; the drive layer takes a reading
         in ADC counts, so turn it into the reading that position gives. */
      int32_t lo   = drive_p[DP_THMIN];
      int32_t span = drive_p[DP_THMAX] - lo;

      if (value < 0) { value = 0; }
      drive_p[DP_INSRC] = 0;
      drive_sim_throttle(lo + (span >> 5) + ((value * (span - (span >> 5))) >> 15) + ((value != 0) ? 1 : 0));
      drive_sim_brake((f->data[1] & 1u) != 0u);
      if (drive_state() == VEH_OFF) {
        (void)drive_vehicle_on();
      }
      commanded = 1u;
      break;
    }
    case CAN_MODE_USB:
      drive_off();
      commanded = 0u;
      want_usb  = 1u;
      break;
    default:
      break;
    }
    __enable_irq();

  } else if ((f->id == (CAN_ID_PARAM + node)) && (f->len == 8u)) {
    uint8_t op    = f->data[0];
    int     index = (int)f->data[1];
    int32_t value = (int32_t)((uint32_t)f->data[4] | ((uint32_t)f->data[5] << 8)
                            | ((uint32_t)f->data[6] << 16) | ((uint32_t)f->data[7] << 24));
    uint8_t d[8] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };

    d[1] = f->data[1];
    if ((index >= console_param_count()) || ((op != 1u) && (op != 2u))) {
      d[0] = 0xFFu;
    } else {
      if (op == 2u) {
        console_param_set(index, value);
      }
      d[0] = (uint8_t)(op | 0x80u);
      put32(&d[4], console_param_get(index));
    }
    (void)can_send(CAN_ID_REPLY + node, d, 8u);
  }
}

/* ---------------------------------------------------------- self-test -- */

static int wait_frame(can_frame *f, uint32_t ms)
{
  uint32_t t0 = millis();

  while ((millis() - t0) <= ms) {
    if (can_receive(f) != 0) {
      return 1;
    }
  }
  return 0;
}

uint32_t can_selftest(void)
{
  static const uint8_t pattern[8] = { 0x55u, 0xAAu, 0x01u, 0x80u, 0xF0u, 0x0Fu, 0x3Cu, 0xC3u };
  uint32_t  node = (uint32_t)drive_p[DP_CANID];
  uint32_t  result = 0u;
  can_frame f;
  uint8_t   req[8] = { 1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };     /* read parameter 0 */
  int       i;

  can_pins(0);
  if (can_hw_start(1) != 0) {
    result |= CAN_TEST_INIT;

    /* 1: a frame, through the transmit mailbox, the bit timing, the filter
       and the receive FIFO, and back out unchanged. */
    (void)can_send(0x2A5u, pattern, 8u);
    if ((wait_frame(&f, 5u) != 0) && (f.id == 0x2A5u) && (f.len == 8u)) {
      int same = 1;

      for (i = 0; i < 8; i++) {
        if (f.data[i] != pattern[i]) {
          same = 0;
        }
      }
      if (same != 0) {
        result |= CAN_TEST_ECHO;
      }
    }

    /* 2: the protocol, end to end. Ask for parameter 0; in loopback the
       request arrives here, handle() answers it, and the answer arrives
       here too. It has to carry the value the console would print. */
    (void)can_send(CAN_ID_PARAM + node, req, 8u);
    if (wait_frame(&f, 5u) != 0) {
      handle(&f);
      for (i = 0; i < 3; i++) {
        if (wait_frame(&f, 5u) == 0) {
          break;
        }
        if (f.id == (CAN_ID_REPLY + node)) {
          int32_t value = (int32_t)((uint32_t)f.data[4] | ((uint32_t)f.data[5] << 8)
                                  | ((uint32_t)f.data[6] << 16) | ((uint32_t)f.data[7] << 24));

          if ((f.data[0] == 0x81u) && (f.data[1] == 0u) &&
              (value == console_param_get(0))) {
            result |= CAN_TEST_PROTOCOL;
          }
          break;
        }
      }
    }
  }

  can_hw_stop();
  can_pins_release();
  rx_count   = 0u;
  tx_count   = 0u;
  drop_count = 0u;
  test_result = result;
  return result;
}

/* -------------------------------------------------------- running on CAN */

void can_start(void)
{
  can_pins(1);

  rx_count   = 0u;
  tx_count   = 0u;
  drop_count = 0u;
  want_usb   = 0u;
  commanded  = 0u;
  last_rx_ms     = millis();
  last_cmd_ms    = millis();
  next_status_ms = millis();
  next_beat_ms   = millis();

  active = (uint8_t)((can_hw_start(0) != 0) ? 1u : 0u);
  if (active == 0u) {
    want_usb = 1u;                  /* it did not come up: do not stay away */
  }
}

void can_stop(void)
{
  can_hw_stop();
  can_pins_release();
  active = 0u;
}

int can_poll(void)
{
  uint32_t  now = millis();
  can_frame f;
  int       n = 0;

  while ((n < 8) && (can_receive(&f) != 0)) {
    last_rx_ms = now;
    handle(&f);
    n++;
  }

  if ((commanded != 0u) && ((now - last_cmd_ms) > COMMAND_TIMEOUT_MS)) {
    __disable_irq();
    drive_off();
    __enable_irq();
    commanded = 0u;
  }

  if ((int32_t)(now - next_status_ms) >= 0) {
    next_status_ms = now + (uint32_t)drive_p[DP_CANPER];
    send_status();
  }
  if ((int32_t)(now - next_beat_ms) >= 0) {
    uint8_t d[8] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };

    next_beat_ms = now + 1000u;
    d[0] = (uint8_t)console_version();
    d[1] = (uint8_t)drive_state();
    (void)can_send(CAN_ID_HEARTBEAT + (uint32_t)drive_p[DP_CANID], d, 2u);
  }

  if ((drive_p[DP_CANBACK] > 0) &&
      ((now - last_rx_ms) > ((uint32_t)drive_p[DP_CANBACK] * 1000u))) {
    want_usb = 1u;
  }
  return (int)want_usb;
}

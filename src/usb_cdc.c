/*
 * STM32F103 USB CDC-ACM device, register level. See usb_cdc.h for what this
 * adds over the HID stack it came from.
 *
 * The three things that bite at this level, unchanged from that project:
 *
 * 1. PMA is 512 bytes of dedicated SRAM seen by the CPU as 16-bit words in
 *    32-bit slots, so a buffer at offset o lives at 0x40006000 + o*2, and
 *    only 16-bit accesses work.
 *
 * 2. USB_EPnR mixes toggle-write bits (STAT_RX, STAT_TX, DTOG_*) with rc_w0
 *    bits (CTR_RX, CTR_TX). Setting a status means writing current ^ wanted,
 *    and every write must put 1 in the CTR bits or a completed transfer is
 *    silently lost. Only ep_set_*_status() may touch EPnR, except when
 *    configuring a fresh endpoint.
 *
 * 3. SET_ADDRESS must not take effect until after its status stage.
 */

#include "stm32f103xb.h"
#include "usb_cdc.h"
#include "board.h"

/* ------------------------------------------------------------------ PMA --- */

#define PMA_BASE          0x40006000UL
#define PMA16(off)        (*(volatile uint16_t *)(PMA_BASE + (uint32_t)(off) * 2U))

#define BT_ADDR_TX(ep)    PMA16((ep) * 8U + 0U)
#define BT_COUNT_TX(ep)   PMA16((ep) * 8U + 2U)
#define BT_ADDR_RX(ep)    PMA16((ep) * 8U + 4U)
#define BT_COUNT_RX(ep)   PMA16((ep) * 8U + 6U)

/* BTABLE 0x00..0x3F, then the buffers: 336 bytes of the 512 in use */
#define EP0_TX_OFF        0x40U
#define EP0_RX_OFF        0x80U
#define EP1_TX_OFF        0xC0U      /* bulk IN  */
#define EP1_RX_OFF        0x100U     /* bulk OUT */
#define EP2_TX_OFF        0x140U     /* interrupt IN, never used */

#define EP0_SIZE          64U
#define BULK_SIZE         64U
#define NOTIFY_SIZE       16U

/* ------------------------------------------------------------- EPnR ------- */

#define EPR(ep)           (*(volatile uint16_t *)(USB_BASE + 4U * (uint32_t)(ep)))

#define EP_KEEP_MASK      (USB_EP0R_CTR_RX | USB_EP0R_SETUP | USB_EP0R_EP_TYPE | \
                           USB_EP0R_EP_KIND | USB_EP0R_CTR_TX | USB_EP0R_EA)

#define EP_TX_DIS         0x0000U
#define EP_TX_STALL       USB_EP0R_STAT_TX_0
#define EP_TX_NAK         USB_EP0R_STAT_TX_1
#define EP_TX_VALID       USB_EP0R_STAT_TX

#define EP_RX_DIS         0x0000U
#define EP_RX_STALL       USB_EP0R_STAT_RX_0
#define EP_RX_NAK         USB_EP0R_STAT_RX_1
#define EP_RX_VALID       USB_EP0R_STAT_RX

#define EP_TYPE_CONTROL   USB_EP0R_EP_TYPE_0
#define EP_TYPE_BULK      0x0000U
#define EP_TYPE_INTERRUPT USB_EP0R_EP_TYPE

static void ep_set_tx_status(uint8_t ep, uint16_t stat)
{
  uint16_t v = (uint16_t)((EPR(ep) & (EP_KEEP_MASK | USB_EP0R_STAT_TX)) ^ stat);
  EPR(ep) = (uint16_t)(v | USB_EP0R_CTR_RX | USB_EP0R_CTR_TX);
}

static void ep_set_rx_status(uint8_t ep, uint16_t stat)
{
  uint16_t v = (uint16_t)((EPR(ep) & (EP_KEEP_MASK | USB_EP0R_STAT_RX)) ^ stat);
  EPR(ep) = (uint16_t)(v | USB_EP0R_CTR_RX | USB_EP0R_CTR_TX);
}

/*
 * Clear one CTR bit and leave the other alone - which means writing 1 to the
 * other, not writing back what was read.
 *
 * Both are rc_w0: writing 0 clears, writing 1 does nothing. These two used
 * to write the other bit back as it was read. If that read said 0 and the
 * transfer in the other direction finished before the write landed, the
 * write cleared a CTR bit nobody had seen yet. For a lost CTR_RX that is
 * the end of the endpoint: the hardware has already set it to NAK, the
 * handler that would have taken the packet and let the application re-arm
 * it never runs, and every later write from the host times out.
 *
 * It needs an OUT packet to complete inside the few cycles between the read
 * and the write of an IN completion, so it wants traffic in both directions
 * at once. A console answering one request at a time almost never does
 * that; a host sending two requests back to back does it within seconds.
 * Caught on the board: EP1R read 0x2021 - STAT_RX NAK, CTR_RX clear - with
 * rx_len still 0.
 */
static void ep_clear_ctr_rx(uint8_t ep)
{
  EPR(ep) = (uint16_t)((EPR(ep) & EP_KEEP_MASK & ~USB_EP0R_CTR_RX) | USB_EP0R_CTR_TX);
}

static void ep_clear_ctr_tx(uint8_t ep)
{
  EPR(ep) = (uint16_t)((EPR(ep) & EP_KEEP_MASK & ~USB_EP0R_CTR_TX) | USB_EP0R_CTR_RX);
}

static uint16_t rx_blocks(uint16_t size)
{
  if (size > 62U) {
    return (uint16_t)(USB_COUNT0_RX_BLSIZE | (((size / 32U) - 1U) << 10));
  }
  return (uint16_t)(((size + 1U) / 2U) << 10);
}

static void pma_write(uint16_t off, const uint8_t *src, uint16_t len)
{
  uint16_t i = 0U;

  while ((uint16_t)(i + 1U) < len) {
    PMA16(off + i) = (uint16_t)(src[i] | ((uint16_t)src[i + 1U] << 8));
    i = (uint16_t)(i + 2U);
  }
  if (i < len) {
    PMA16(off + i) = src[i];
  }
}

static void pma_read(uint16_t off, uint8_t *dst, uint16_t len)
{
  uint16_t i;

  for (i = 0U; i < len; i = (uint16_t)(i + 2U)) {
    uint16_t w = PMA16(off + i);
    dst[i] = (uint8_t)w;
    if ((uint16_t)(i + 1U) < len) {
      dst[i + 1U] = (uint8_t)(w >> 8);
    }
  }
}

/* ------------------------------------------------------ descriptors ------- */

#define USB_VID           0x0483U
#define USB_PID           0x5752U   /* distinct from the HID builds, so Windows
                                       cannot reuse a cached descriptor set */

#define DESC_DEVICE       0x01U
#define DESC_CONFIG       0x02U
#define DESC_STRING       0x03U

static const uint8_t device_desc[18] = {
  18, DESC_DEVICE,
  0x00, 0x02,             /* bcdUSB 2.00 */
  0x02,                   /* bDeviceClass: CDC */
  0x00, 0x00,
  EP0_SIZE,
  (uint8_t)USB_VID, (uint8_t)(USB_VID >> 8),
  (uint8_t)USB_PID, (uint8_t)(USB_PID >> 8),
  0x00, 0x02,             /* bcdDevice 2.00 */
  0x01, 0x02, 0x03,       /* iManufacturer, iProduct, iSerialNumber */
  0x01,
};

static const uint8_t config_desc[67] = {
  /* configuration */
  0x09, DESC_CONFIG, 67, 0x00, 0x02, 0x01, 0x00, 0xA0, 0x32,

  /* interface 0: CDC communication, ACM, AT command protocol */
  0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

  /* CDC header functional, spec 1.10 */
  0x05, 0x24, 0x00, 0x10, 0x01,
  /* call management: none, data interface 1 */
  0x05, 0x24, 0x01, 0x00, 0x01,
  /* ACM: supports Set_Line_Coding and Serial_State */
  0x04, 0x24, 0x02, 0x02,
  /* union: control interface 0, subordinate 1 */
  0x05, 0x24, 0x06, 0x00, 0x01,

  /* notification endpoint, 16 bytes - required, never used here */
  0x07, 0x05, 0x82, 0x03, NOTIFY_SIZE, 0x00, 0xFF,

  /* interface 1: CDC data */
  0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

  /* bulk OUT and bulk IN, 64 bytes each */
  0x07, 0x05, 0x01, 0x02, BULK_SIZE, 0x00, 0x00,
  0x07, 0x05, 0x81, 0x02, BULK_SIZE, 0x00, 0x00,
};

static const uint8_t lang_desc[4] = { 4, DESC_STRING, 0x09, 0x04 };

static const char str_manufacturer[] = "STMicroelectronics";
static const char str_product[]      = "STM32 FOC Console";
static const char str_serial[]       = "FOC000001";

static uint8_t str_buf[64];

static uint16_t make_string_desc(const char *s)
{
  uint16_t n = 2U;

  while ((*s != '\0') && (n < (uint16_t)(sizeof(str_buf) - 1U))) {
    str_buf[n++] = (uint8_t)*s++;
    str_buf[n++] = 0x00U;
  }
  str_buf[0] = (uint8_t)n;
  str_buf[1] = DESC_STRING;
  return n;
}

/* --------------------------------------------------------- transfers ----- */

typedef struct {
  uint8_t  bmRequestType;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} setup_packet;

enum { CTL_IDLE = 0, CTL_IN_DATA, CTL_OUT_DATA, CTL_STATUS_IN };

static const uint8_t *ctl_ptr;
static uint16_t       ctl_left;
static uint8_t        ctl_zlp;
static uint8_t        ctl_state;
static uint16_t       ctl_out_len;      /* bytes still expected from the host */

static uint8_t  pending_address;
static uint8_t  have_pending_address;
static uint8_t  configured;

/* CDC state. Stored so GET_LINE_CODING can echo it back; nothing uses it. */
static uint8_t  line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 };
static uint8_t  control_lines;

/*
 * Enumeration counters, readable over SWD. Guessing why a USB device does not
 * appear is unproductive; these say whether the host is talking to us at all,
 * and what it asked for last.
 */
volatile uint32_t usb_dbg_resets;
volatile uint32_t usb_dbg_setups;
volatile uint32_t usb_dbg_ctr;
volatile uint32_t usb_dbg_in_done;
volatile uint32_t usb_dbg_stalls;
volatile uint8_t  usb_dbg_last_setup[8];

/* Application-facing buffers */
static volatile uint8_t  tx_busy;
static uint8_t           rx_buf[BULK_SIZE];
static volatile uint16_t rx_len;
static volatile uint16_t rx_pos;

static void ctl_send_chunk(void)
{
  uint16_t n = (ctl_left > EP0_SIZE) ? EP0_SIZE : ctl_left;

  pma_write(EP0_TX_OFF, ctl_ptr, n);
  BT_COUNT_TX(0) = n;
  ctl_ptr  += n;
  ctl_left  = (uint16_t)(ctl_left - n);
  ep_set_tx_status(0, EP_TX_VALID);
}

static void ctl_send(const uint8_t *data, uint16_t len, uint16_t requested)
{
  if (len > requested) {
    len = requested;
  }
  ctl_ptr   = data;
  ctl_left  = len;
  ctl_zlp   = ((len < requested) && (len != 0U) && ((len % EP0_SIZE) == 0U)) ? 1U : 0U;
  ctl_state = CTL_IN_DATA;
  ctl_send_chunk();
}

static void ctl_status(void)
{
  BT_COUNT_TX(0) = 0U;
  ctl_state = CTL_STATUS_IN;
  ep_set_tx_status(0, EP_TX_VALID);
}

static void ctl_stall(void)
{
  usb_dbg_stalls++;
  ctl_state = CTL_IDLE;
  ep_set_tx_status(0, EP_TX_STALL);
  ep_set_rx_status(0, EP_RX_VALID);
}

/* Accept an OUT data stage of wLength bytes on EP0. */
static void ctl_receive(uint16_t len)
{
  ctl_out_len = len;
  ctl_state   = CTL_OUT_DATA;
  ep_set_rx_status(0, EP_RX_VALID);
}

static void open_data_endpoints(void)
{
  BT_ADDR_TX(1)  = EP1_TX_OFF;
  BT_COUNT_TX(1) = 0U;
  BT_ADDR_RX(1)  = EP1_RX_OFF;
  BT_COUNT_RX(1) = rx_blocks(BULK_SIZE);

  EPR(1) = (uint16_t)(EP_TYPE_BULK | 1U);
  ep_set_rx_status(1, EP_RX_VALID);
  ep_set_tx_status(1, EP_TX_NAK);

  BT_ADDR_TX(2)  = EP2_TX_OFF;
  BT_COUNT_TX(2) = 0U;
  EPR(2) = (uint16_t)(EP_TYPE_INTERRUPT | 2U);
  ep_set_rx_status(2, EP_RX_DIS);
  ep_set_tx_status(2, EP_TX_NAK);

  tx_busy = 0U;
  rx_len  = 0U;
  rx_pos  = 0U;
}

static void handle_get_descriptor(const setup_packet *s)
{
  uint8_t type  = (uint8_t)(s->wValue >> 8);
  uint8_t index = (uint8_t)(s->wValue);

  switch (type) {
  case DESC_DEVICE:
    ctl_send(device_desc, sizeof(device_desc), s->wLength);
    break;
  case DESC_CONFIG:
    ctl_send(config_desc, sizeof(config_desc), s->wLength);
    break;
  case DESC_STRING:
    switch (index) {
    case 0: ctl_send(lang_desc, sizeof(lang_desc), s->wLength); break;
    case 1: ctl_send(str_buf, make_string_desc(str_manufacturer), s->wLength); break;
    case 2: ctl_send(str_buf, make_string_desc(str_product), s->wLength); break;
    case 3: ctl_send(str_buf, make_string_desc(str_serial), s->wLength); break;
    default: ctl_stall(); break;
    }
    break;
  default:
    ctl_stall();
    break;
  }
}

static void handle_standard(const setup_packet *s)
{
  static uint8_t reply[2];

  switch (s->bRequest) {
  case 0x00: /* GET_STATUS */
    reply[0] = 0U;
    reply[1] = 0U;
    ctl_send(reply, 2U, s->wLength);
    break;

  case 0x01: /* CLEAR_FEATURE */
  case 0x03: /* SET_FEATURE */
    ctl_status();
    break;

  case 0x05: /* SET_ADDRESS, applied after the status stage */
    pending_address      = (uint8_t)(s->wValue & 0x7FU);
    have_pending_address = 1U;
    ctl_status();
    break;

  case 0x06:
    handle_get_descriptor(s);
    break;

  case 0x08: /* GET_CONFIGURATION */
    reply[0] = configured;
    ctl_send(reply, 1U, s->wLength);
    break;

  case 0x09: /* SET_CONFIGURATION */
    configured = (uint8_t)(s->wValue & 0xFFU);
    if (configured != 0U) {
      open_data_endpoints();
    }
    ctl_status();
    break;

  case 0x0A: /* GET_INTERFACE */
    reply[0] = 0U;
    ctl_send(reply, 1U, s->wLength);
    break;

  case 0x0B: /* SET_INTERFACE */
    ctl_status();
    break;

  default:
    ctl_stall();
    break;
  }
}

static void handle_class(const setup_packet *s)
{
  switch (s->bRequest) {
  case 0x20: /* SET_LINE_CODING: seven bytes arrive on EP0 OUT */
    ctl_receive(s->wLength);
    break;

  case 0x21: /* GET_LINE_CODING */
    ctl_send(line_coding, sizeof(line_coding), s->wLength);
    break;

  case 0x22: /* SET_CONTROL_LINE_STATE: DTR and RTS, no data stage */
    control_lines = (uint8_t)s->wValue;
    ctl_status();
    break;

  case 0x23: /* SEND_BREAK */
    ctl_status();
    break;

  default:
    ctl_stall();
    break;
  }
}

static void handle_setup(void)
{
  uint8_t raw[8];
  setup_packet s;

  pma_read(EP0_RX_OFF, raw, 8U);

  usb_dbg_setups++;
  for (uint8_t i = 0U; i < 8U; i++) {
    usb_dbg_last_setup[i] = raw[i];
  }

  s.bmRequestType = raw[0];
  s.bRequest      = raw[1];
  s.wValue        = (uint16_t)(raw[2] | ((uint16_t)raw[3] << 8));
  s.wIndex        = (uint16_t)(raw[4] | ((uint16_t)raw[5] << 8));
  s.wLength       = (uint16_t)(raw[6] | ((uint16_t)raw[7] << 8));

  ctl_state   = CTL_IDLE;
  ctl_left    = 0U;
  ctl_zlp     = 0U;
  ctl_out_len = 0U;

  switch (s.bmRequestType & 0x60U) {
  case 0x00U: handle_standard(&s); break;
  case 0x20U: handle_class(&s);    break;
  default:    ctl_stall();         break;
  }
}

static void ep0_in_done(void)
{
  switch (ctl_state) {
  case CTL_IN_DATA:
    if (ctl_left != 0U) {
      ctl_send_chunk();
    } else if (ctl_zlp != 0U) {
      ctl_zlp = 0U;
      BT_COUNT_TX(0) = 0U;
      ep_set_tx_status(0, EP_TX_VALID);
    } else {
      ctl_state = CTL_IDLE;
      ep_set_rx_status(0, EP_RX_VALID);
    }
    break;

  case CTL_STATUS_IN:
    if (have_pending_address != 0U) {
      USB->DADDR = (uint16_t)(USB_DADDR_EF | pending_address);
      have_pending_address = 0U;
    }
    ctl_state = CTL_IDLE;
    ep_set_rx_status(0, EP_RX_VALID);
    break;

  default:
    ep_set_rx_status(0, EP_RX_VALID);
    break;
  }
}

static void ep0_out_done(void)
{
  uint16_t count = (uint16_t)(BT_COUNT_RX(0) & 0x3FFU);

  if (ctl_state == CTL_OUT_DATA) {
    uint8_t buf[16];
    uint16_t n = (count > sizeof(buf)) ? (uint16_t)sizeof(buf) : count;

    pma_read(EP0_RX_OFF, buf, n);

    /* The only OUT request this device accepts is SET_LINE_CODING */
    if (n >= sizeof(line_coding)) {
      uint16_t i;
      for (i = 0U; i < sizeof(line_coding); i++) {
        line_coding[i] = buf[i];
      }
    }

    ctl_out_len = 0U;
    ep_set_rx_status(0, EP_RX_VALID);
    ctl_status();                 /* the device acknowledges with a ZLP IN */
  } else {
    /* status stage of an IN transfer: nothing to read */
    ep_set_rx_status(0, EP_RX_VALID);
  }
}

/* ------------------------------------------------------------- driver ---- */

static void usb_reset(void)
{
  configured = 0U;
  ctl_state  = CTL_IDLE;
  tx_busy    = 0U;
  rx_len     = 0U;
  rx_pos     = 0U;
  have_pending_address = 0U;

  USB->BTABLE = 0U;

  BT_ADDR_TX(0)  = EP0_TX_OFF;
  BT_COUNT_TX(0) = 0U;
  BT_ADDR_RX(0)  = EP0_RX_OFF;
  BT_COUNT_RX(0) = rx_blocks(EP0_SIZE);

  EPR(0) = (uint16_t)(EP_TYPE_CONTROL | 0U);
  ep_set_rx_status(0, EP_RX_VALID);
  ep_set_tx_status(0, EP_TX_NAK);

  USB->DADDR = USB_DADDR_EF;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  uint16_t istr = USB->ISTR;

  if ((istr & USB_ISTR_RESET) != 0U) {
    USB->ISTR = (uint16_t)~USB_ISTR_RESET;
    usb_dbg_resets++;
    usb_reset();
    return;
  }

  if ((istr & USB_ISTR_SUSP) != 0U) {
    USB->ISTR = (uint16_t)~USB_ISTR_SUSP;
    USB->CNTR |= USB_CNTR_FSUSP;
  }

  if ((istr & USB_ISTR_WKUP) != 0U) {
    USB->ISTR  = (uint16_t)~USB_ISTR_WKUP;
    USB->CNTR &= (uint16_t)~USB_CNTR_FSUSP;
  }

  while (((istr = USB->ISTR) & USB_ISTR_CTR) != 0U) {
    uint8_t ep = (uint8_t)(istr & USB_ISTR_EP_ID);

    usb_dbg_ctr++;

    if (ep == 0U) {
      if ((istr & USB_ISTR_DIR) == 0U) {
        ep_clear_ctr_tx(0);
        usb_dbg_in_done++;
        ep0_in_done();
      } else {
        uint16_t epr = EPR(0);

        ep_clear_ctr_rx(0);
        if ((epr & USB_EP0R_SETUP) != 0U) {
          handle_setup();
        } else {
          ep0_out_done();
        }
      }
    } else if (ep == 1U) {
      if ((istr & USB_ISTR_DIR) == 0U) {
        ep_clear_ctr_tx(1);
        tx_busy = 0U;
      } else {
        ep_clear_ctr_rx(1);
        rx_len = (uint16_t)(BT_COUNT_RX(1) & 0x3FFU);
        rx_pos = 0U;
        if (rx_len > 0U) {
          pma_read(EP1_RX_OFF, rx_buf, rx_len);
        }
        /* Re-arm only once the application has drained it, so a burst of
           commands cannot overwrite one that has not been read yet. */
        if (rx_len == 0U) {
          ep_set_rx_status(1, EP_RX_VALID);
        }
      }
    } else {
      ep_clear_ctr_tx(ep);
    }
  }
}

void USBWakeUp_IRQHandler(void)
{
  EXTI->PR = EXTI_PR_PR18;
}

void usb_cdc_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;

  /*
   * Leave power down, then hold the peripheral in reset for the analog
   * block's startup time before releasing it.
   *
   * The delay has to be a real one. This was copied from the HID project as
   * an empty for loop, which -Os is entitled to delete and does: FRES then
   * gets released a couple of cycles after PDWN clears, the transceiver is
   * not ready, and the result is a device that answers the USB reset,
   * configures EP0 correctly, and then fails every transfer - ERR latched in
   * ISTR, DADDR still 0, and nothing in the host's device list. Exactly the
   * same mistake as in the ADC calibration path, in code that happened to
   * survive it there.
   */
  USB->CNTR = USB_CNTR_FRES;
  delay_ms(1U);

  USB->CNTR   = 0U;
  USB->ISTR   = 0U;
  USB->BTABLE = 0U;
  USB->CNTR   = USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;

  /*
   * Below the ADC interrupt, which runs the control loop.
   *
   * This was briefly set to 0 while chasing an enumeration failure that
   * turned out to be an unplugged cable, and the higher priority stayed
   * behind as a real regression: USB is busiest during enumeration, which is
   * exactly when the ADC timing test measures, and a USB handler preempting
   * the control interrupt pushed the measured sample jitter from a few ticks
   * to 148 - 2 us of uncertainty on when the current is sampled.
   *
   * A console must never be able to delay a current loop.
   */
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2U);
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

void usb_cdc_stop(void)
{
  /* The interrupt first: its vector is shared with the CAN controller's
     receive interrupt, and after this the CAN controller may be next. */
  NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

  USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
  USB->ISTR = 0U;
  RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;
  NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);

  configured = 0U;
  tx_busy    = 0U;
  rx_len     = 0U;
  rx_pos     = 0U;
}

int usb_cdc_configured(void)
{
  return (configured != 0U) ? 1 : 0;
}

int usb_cdc_write_ready(void)
{
  return ((configured != 0U) && (tx_busy == 0U)) ? 1 : 0;
}

uint16_t usb_cdc_write(const uint8_t *data, uint16_t len)
{
  if ((configured == 0U) || (tx_busy != 0U)) {
    return 0U;
  }
  /*
   * One byte short of a full packet, always.
   *
   * A bulk transfer ends at a packet shorter than the endpoint size. A
   * packet of exactly 64 bytes says "more follows", and the host holds it
   * back until whatever comes next - which for a console answering one
   * request at a time is the reply to the next request. The state line is
   * 64 characters for some values (it was for "ang 54078  spd 0  id -20
   * iq -1  iqref 1076  moe 0  fault none"), and then every answer arrived
   * one request late: the board had sent it, DTOG_TX had toggled, and the
   * host application saw nothing.
   *
   * The other cure is a zero-length packet after each full one. Never
   * sending a full one needs no state and no second interrupt.
   */
  if (len > (BULK_SIZE - 1U)) {
    len = BULK_SIZE - 1U;
  }

  tx_busy = 1U;
  pma_write(EP1_TX_OFF, data, len);
  BT_COUNT_TX(1) = len;
  ep_set_tx_status(1, EP_TX_VALID);

  return len;
}

uint16_t usb_cdc_puts(const char *s)
{
  const char *p = s;
  uint16_t n = 0U;

  while ((*p != '\0') && (n < BULK_SIZE)) {
    p++;
    n++;
  }
  return usb_cdc_write((const uint8_t *)s, n);
}

int usb_cdc_read_byte(uint8_t *out)
{
  if (rx_pos >= rx_len) {
    return 0;
  }

  *out = rx_buf[rx_pos++];

  if (rx_pos >= rx_len) {
    rx_len = 0U;
    rx_pos = 0U;
    ep_set_rx_status(1, EP_RX_VALID);   /* ready for the next packet */
  }
  return 1;
}

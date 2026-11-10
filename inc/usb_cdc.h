/*
 * USB CDC-ACM device for the STM32F103, on the registers.
 *
 * Ported from the bare metal HID stack in the USBHIDBare project - same PMA
 * handling, same EPnR toggle-write rules, same control transfer engine - with
 * three changes that CDC forces and HID did not:
 *
 *  1. Two interfaces, not one: a communication interface carrying the class's
 *     functional descriptors, and a data interface with the bulk endpoints.
 *  2. Three endpoints on top of EP0: bulk IN and bulk OUT for the data, plus
 *     an interrupt IN that exists only because the descriptor requires it.
 *  3. An EP0 *OUT* data stage. SET_LINE_CODING sends seven bytes to the
 *     device, and the HID stack never had to receive anything on EP0 - it
 *     stalled every request with an OUT payload. A host will not finish
 *     enumerating a CDC device that does that.
 *
 * The line coding itself is stored and ignored: there is no UART behind this,
 * so baud rate and parity are whatever the host would like them to be.
 */
#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

void usb_cdc_init(void);

/* 1 once the host has configured the device */
int  usb_cdc_configured(void);

/* Queue a byte range for transmission. Returns the number of bytes accepted,
   which is 0 if the previous packet has not gone out yet. */
uint16_t usb_cdc_write(const uint8_t *data, uint16_t len);

/* Same, for a zero-terminated string. */
uint16_t usb_cdc_puts(const char *s);

/* 1 when the IN endpoint is free. */
int  usb_cdc_write_ready(void);

/* Pull one received byte. Returns 0 when nothing is waiting. */
int  usb_cdc_read_byte(uint8_t *out);

#endif /* USB_CDC_H */

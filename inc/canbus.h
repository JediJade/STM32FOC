/*
 * CAN, on the registers, and the small protocol this drive speaks over it.
 *
 * ONE THING FIRST. On the STM32F103 the CAN controller and the USB device
 * share a block of memory, so they cannot both be switched on. This firmware
 * has a USB console. So CAN is something it changes over to, in software:
 *
 *   `can on`           the console says so, USB is shut down (the host sees
 *                      the device unplug), CAN comes up on PB8 / PB9.
 *   back to USB        on a command frame with mode 0x55, or by itself when
 *                      no frame has arrived for `canback` seconds - so a
 *                      board with nothing on its CAN pins comes back.
 *
 * The parameters and the loops keep their state across both changes; only
 * the console is away.
 *
 * And so that the controller is not something that has only ever been
 * compiled: before USB is started at power-on, it is put in loopback, where
 * it hears itself without any pins, and made to send a frame, receive it,
 * then ask itself for a parameter and check the answer (can_selftest).
 *
 * THE PROTOCOL. 11-bit identifiers, node number n = `canid`, little-endian.
 *
 *   0x100+n  in    command     [mode, flags, value lo, value hi]
 *                    mode 0 off, 1 torque current (Q15), 2 speed (units/ms),
 *                    3 throttle (0..32767, flags bit0 = brake),
 *                    0x55 back to USB
 *   0x200+n  in    parameter   [op, index, 0, 0, value int32]   op 1 read, 2 write
 *   0x300+n  out   status 1    speed, iq, id (int16), state, fault
 *   0x400+n  out   status 2    vd, vq (int16), angle (uint16), throttle/128, flags
 *   0x500+n  out   reply       [op | 0x80, index, 0, 0, value]  op 0xFF = no such index
 *   0x700+n  out   heartbeat   [console version, vehicle state], once a second
 *
 * Status frames go out every `canper` ms. A drive that was told to turn and
 * then hears no command for half a second stops: a cable that falls off must
 * not leave the last command standing. The outputs cannot be armed over CAN.
 *
 * Pins: PB8 receive, PB9 transmit (the remapped pair, so the USB connector
 * can stay plugged in for power). A transceiver is needed to reach a bus.
 */
#ifndef CANBUS_H
#define CANBUS_H

#include <stdint.h>

#define CAN_ID_COMMAND    0x100u
#define CAN_ID_PARAM      0x200u
#define CAN_ID_STATUS1    0x300u
#define CAN_ID_STATUS2    0x400u
#define CAN_ID_REPLY      0x500u
#define CAN_ID_HEARTBEAT  0x700u

#define CAN_MODE_OFF      0u
#define CAN_MODE_TORQUE   1u
#define CAN_MODE_SPEED    2u
#define CAN_MODE_THROTTLE 3u
#define CAN_MODE_USB      0x55u

/* Bits of can_selftest()'s result. 7 = all three. */
#define CAN_TEST_INIT     1u    /* the controller entered and left initialisation */
#define CAN_TEST_ECHO     2u    /* a frame came back as it was sent */
#define CAN_TEST_PROTOCOL 4u    /* a parameter request to itself was answered right */

/* Power-on only, before USB is started. Leaves the controller off. */
uint32_t can_selftest(void);
uint32_t can_test_result(void);

/* USB must already be shut down. */
void     can_start(void);
void     can_stop(void);
int      can_active(void);

/* From the main loop while active. Returns 1 when it is time to go back to
   USB: asked to by a frame, or nothing heard for `canback` seconds. */
int      can_poll(void);

uint32_t can_rx_count(void);
uint32_t can_tx_count(void);
uint32_t can_drop_count(void);

#endif /* CANBUS_H */

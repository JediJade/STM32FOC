/*
 * A virtual motor, so the control loop has something to regulate before any
 * real hardware exists.
 *
 * The important structural point: the plant keeps its electrical state in the
 * *rotor's own* frame, at its *own* angle, and is driven with stator-frame
 * voltages. The controller never gets handed d and q for free. If the
 * controller's idea of the rotor angle is wrong, the plant behaves exactly as
 * a real motor would - the commanded torque lands on the wrong axis - which
 * is what makes the alignment procedure testable at all.
 *
 * Electrical, in the real rotor frame:
 *
 *   i_d += a*(v_d - i_d) + c*i_q
 *   i_q += a*(v_q - i_q) - c*i_d - a*ke*omega
 *
 * Mechanical, when the rotor is free:
 *
 *   omega += (k*i_q - friction*omega)
 *   theta += omega
 *
 * so torque comes from whatever current ends up on the real q axis. A field
 * applied at the wrong angle produces less torque and more heating, with no
 * special case needed to model it.
 *
 * The sensor reports theta plus a fixed offset that the firmware does not
 * know. Finding that offset is what align mode is for.
 */
#ifndef VMOTOR_H
#define VMOTOR_H

#include <stdint.h>
#include "foc.h"

/* The offset the firmware has to discover. Any value; this one is just not a
   round number, so an accidentally-zero result cannot look like a pass. */
#define VMOTOR_ENCODER_OFFSET   12345u

void vmotor_reset(void);

/*
 * How the shaft behaves. All three are things a real test bench can do:
 *
 *   FREE    torque moves the rotor. What alignment needs.
 *   LOCKED  omega held at zero, as if the shaft were clamped.
 *   FORCED  omega held at a constant value, as if another machine were
 *           driving the shaft. This is how a current loop is characterised in
 *           reality, and it keeps cross-coupling and back-EMF present and
 *           constant while the current loop is measured.
 */
typedef enum {
  VM_MECH_FREE = 0,
  VM_MECH_LOCKED,
  VM_MECH_FORCED
} vm_mech;

void vmotor_set_mech(vm_mech mode, int32_t omega_step);

/* Advance the plant by one sample with the stator-frame voltages applied. */
void vmotor_step(int16_t v_alpha, int16_t v_beta);

/* What current sensors on phases A and B would read right now. */
void vmotor_phase_currents(int16_t *ia, int16_t *ib);

/* What the position sensor reports: the truth plus the unknown offset. */
uint16_t vmotor_sensor_angle(void);

/* Ground truth, for the test to check against - not for the controller. */
uint16_t vmotor_true_angle(void);
int32_t  vmotor_true_speed(void);
foc_dq   vmotor_true_dq(void);

#endif /* VMOTOR_H */

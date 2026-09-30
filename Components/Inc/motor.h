#ifndef RM2027AT_MOTOR_H
#define RM2027AT_MOTOR_H

/* Task context only (or after initialization before scheduler startup).
 * Disable all TT outputs and clear all registered DJI current targets.
 * DJI zero commands are submitted by the service on its next cycle.
 * This is repeatable and does not latch: explicit new control can restart.
 */
void motor_shutdown(void);

#endif

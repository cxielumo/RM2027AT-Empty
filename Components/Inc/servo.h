#ifndef RM2027AT_SERVO_H
#define RM2027AT_SERVO_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"

typedef enum {
  SERVO_1,
  SERVO_2,
  SERVO_3,
  SERVO_4,
  SERVO_5,
  SERVO_6,
  LAST_SERVO
} servo_id_t;

typedef struct {
    uint32_t pulse_min_us;
    uint32_t pulse_max_us;
    uint32_t initial_pulse_us;
    float angle_min_deg;
    float angle_max_deg;
    bool invert;
    bool calibrated;
} servo_config_t;

void servo_enable(servo_id_t id);
void servo_disable(servo_id_t id);

void servo_setPulse(servo_id_t id, uint32_t pulse_us);
void servo_setAngle(servo_id_t id, float angle_deg);

#endif /* RM2027AT_SERVO_H */

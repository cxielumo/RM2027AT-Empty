#ifndef RM2027AT_MOTOR_TT_H
#define RM2027AT_MOTOR_TT_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"

typedef enum {
  MOTOR_TT_1,
  MOTOR_TT_2,
  MOTOR_TT_3,
  MOTOR_TT_4,
  MOTOR_TT_5,
  MOTOR_TT_6,
  LAST_MOTOR_TT
} motor_tt_id_t;

typedef struct {
    bool invert;
} motor_tt_config_t;

void motor_tt_enable(motor_tt_id_t id);
void motor_tt_disable(motor_tt_id_t id);

void motor_tt_setDuty(motor_tt_id_t id, float ratio);

#endif

#ifndef RM2027AT_PWM_H
#define RM2027AT_PWM_H

#include "bsp.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  PWM_MOTOR_1_IN1,
  PWM_MOTOR_1_IN2,
  PWM_MOTOR_2_IN1,
  PWM_MOTOR_2_IN2,
  PWM_MOTOR_3_IN1,
  PWM_MOTOR_3_IN2,
  PWM_MOTOR_4_IN1,
  PWM_MOTOR_4_IN2,
  PWM_MOTOR_5_IN1,
  PWM_MOTOR_5_IN2,
  PWM_MOTOR_6_IN1,
  PWM_MOTOR_6_IN2,
  PWM_SERVO_1,
  PWM_SERVO_2,
  PWM_SERVO_3,
  PWM_SERVO_4,
  PWM_SERVO_5,
  PWM_SERVO_6,
  LAST_PWM_CHANNEL
} pwm_channel_t;

typedef enum {
  PWM_GROUP_TMR1,
  PWM_GROUP_TMR2,
  PWM_GROUP_TMR3,
  PWM_GROUP_TMR4,
  PWM_GROUP_TMR12,
  PWM_GROUP_TMR13,
  PWM_GROUP_TMR14,
  LAST_PWM_GROUP
} pwm_group_t;

/* Initializes all PWM outputs in the disabled state. */
/* Sets one PWM channel's duty in [0, 1]. */
void pwm_setDuty(pwm_channel_t channel, float duty);

/* Disables one servo channel or one complete motor pair. */
void pwm_disable(pwm_channel_t channel);

/* Immediately disables every PWM output; safe before initialization. */
void pwm_disableAll(void);

#endif /* RM2027AT_PWM_H */

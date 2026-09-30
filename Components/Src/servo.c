#include "servo.h"

#include <math.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "pwm.h"
#include "task.h"

#define SERVO_PULSE_MIN_US 500U
#define SERVO_PULSE_MAX_US 2500U
#define SERVO_INITIAL_PULSE_US 1500U
#define SERVO_ANGLE_MIN_DEG 0.0f
#define SERVO_ANGLE_MAX_DEG 180.0f
#define SERVO_PERIOD_US 20000.0f

typedef struct {
    uint32_t pulse_us;
    float target_angle_deg;
    bool enabled;
} servo_state_t;

static servo_state_t s_servo_state[6];
static bool s_initialized;

static int servo_id_to_index(servo_id_t id, size_t *index)
{
    if ((unsigned int)id >= (unsigned int)LAST_SERVO) {
        return -1;
    }
    *index = (size_t)(unsigned int)id;
    return 0;
}

static pwm_channel_t servo_pwm_channel(size_t index)
{
    return (pwm_channel_t)((unsigned int)PWM_SERVO_1 + (unsigned int)index);
}

static float servo_angle_from_pulse(uint32_t pulse_us)
{
    double pulse_fraction = ((double)pulse_us - SERVO_PULSE_MIN_US) /
                            (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    double angle = SERVO_ANGLE_MIN_DEG +
                   pulse_fraction * (SERVO_ANGLE_MAX_DEG -
                                     SERVO_ANGLE_MIN_DEG);
    return (float)angle;
}

void servo_init(void)
{
    size_t index;

    if (s_initialized) {
        return;
    }

    for (index = 0U; index < 6U; ++index) {
        pwm_disable(servo_pwm_channel(index));
    }

    taskENTER_CRITICAL();
    for (index = 0U; index < 6U; ++index) {
        s_servo_state[index].pulse_us = 0U;
        s_servo_state[index].target_angle_deg = 0.0f;
        s_servo_state[index].enabled = false;
    }
    s_initialized = true;
    taskEXIT_CRITICAL();
}

void servo_enable(servo_id_t id)
{
    size_t index;

    if (!s_initialized) {
        return;
    }
    if (servo_id_to_index(id, &index)) {
        return;
    }
    pwm_setDuty(servo_pwm_channel(index),
                (float)SERVO_INITIAL_PULSE_US / SERVO_PERIOD_US);

    taskENTER_CRITICAL();
    s_servo_state[index].pulse_us = SERVO_INITIAL_PULSE_US;
    s_servo_state[index].target_angle_deg =
        servo_angle_from_pulse(SERVO_INITIAL_PULSE_US);
    s_servo_state[index].enabled = true;
    taskEXIT_CRITICAL();
}

void servo_disable(servo_id_t id)
{
    size_t index;

    if (!s_initialized) {
        return;
    }
    if (servo_id_to_index(id, &index)) {
        return;
    }

    pwm_disable(servo_pwm_channel(index));

    taskENTER_CRITICAL();
    s_servo_state[index].pulse_us = 0U;
    s_servo_state[index].target_angle_deg = 0.0f;
    s_servo_state[index].enabled = false;
    taskEXIT_CRITICAL();
}

void servo_setPulse(servo_id_t id, uint32_t pulse_us)
{
    size_t index;

    if (!s_initialized) {
        return;
    }
    if (servo_id_to_index(id, &index)) {
        return;
    }
    if (pulse_us < SERVO_PULSE_MIN_US || pulse_us > SERVO_PULSE_MAX_US) {
        return;
    }
    if (!s_servo_state[index].enabled) {
        return;
    }

    pwm_setDuty(servo_pwm_channel(index), (float)pulse_us / SERVO_PERIOD_US);

    taskENTER_CRITICAL();
    s_servo_state[index].pulse_us = pulse_us;
    s_servo_state[index].target_angle_deg =
        servo_angle_from_pulse(pulse_us);
    taskEXIT_CRITICAL();
}

void servo_setAngle(servo_id_t id, float angle_deg)
{
    size_t index;
    double fraction;
    double pulse;
    uint32_t pulse_us;

    if (!s_initialized) {
        return;
    }
    if (servo_id_to_index(id, &index)) {
        return;
    }
    if (!isfinite(angle_deg)) {
        return;
    }
    if (angle_deg < SERVO_ANGLE_MIN_DEG ||
        angle_deg > SERVO_ANGLE_MAX_DEG) {
        return;
    }
    if (!s_servo_state[index].enabled) {
        return;
    }

    fraction = ((double)angle_deg - SERVO_ANGLE_MIN_DEG) /
               (SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG);
    pulse = SERVO_PULSE_MIN_US +
            fraction * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    pulse_us = (uint32_t)floor(pulse + 0.5);
    if (pulse_us < SERVO_PULSE_MIN_US || pulse_us > SERVO_PULSE_MAX_US) {
        return;
    }

    pwm_setDuty(servo_pwm_channel(index), (float)pulse_us / SERVO_PERIOD_US);

    taskENTER_CRITICAL();
    s_servo_state[index].pulse_us = pulse_us;
    s_servo_state[index].target_angle_deg =
        servo_angle_from_pulse(pulse_us);
    taskEXIT_CRITICAL();
}


void servo_setDuty(servo_id_t id, float duty)
{
    if (!isfinite(duty) || duty < 0.025f || duty > 0.125f) return;
    servo_setPulse(id, (uint32_t)floor((double)duty * SERVO_PERIOD_US + 0.5));
}

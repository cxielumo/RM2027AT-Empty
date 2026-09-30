#include "motor_tt.h"

#include <math.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "pwm.h"
#include "task.h"

typedef struct {
    pwm_channel_t in1;
    pwm_channel_t in2;
    motor_tt_config_t config;
    bool enabled;
} motor_tt_device_t;

/* CN10 output order is reversed on the board, so MOTOR_TT_6 is inverted. */
static motor_tt_device_t motor_devices[6] = {
    { PWM_MOTOR_1_IN1, PWM_MOTOR_1_IN2, { false }, false },
    { PWM_MOTOR_2_IN1, PWM_MOTOR_2_IN2, { false }, false },
    { PWM_MOTOR_3_IN1, PWM_MOTOR_3_IN2, { false }, false },
    { PWM_MOTOR_4_IN1, PWM_MOTOR_4_IN2, { false }, false },
    { PWM_MOTOR_5_IN1, PWM_MOTOR_5_IN2, { false }, false },
    { PWM_MOTOR_6_IN1, PWM_MOTOR_6_IN2, { true }, false }
};

static bool motor_tt_initialized;

static bool motor_tt_is_id_valid(motor_tt_id_t id)
{
    return (unsigned int)id < (unsigned int)LAST_MOTOR_TT;
}

static motor_tt_device_t *motor_tt_device(motor_tt_id_t id)
{
    return &motor_devices[(uint32_t)id];
}

static void motor_tt_apply_duty(motor_tt_device_t *device, float ratio)
{
    bool forward = ratio > 0.0f;
    float duty = fabsf(ratio);

    if (device->config.invert) {
        forward = !forward;
    }

    if (ratio == 0.0f) {
        pwm_disable(device->in1);
        pwm_disable(device->in2);
    } else if (forward) {
        /* RZ7889: Fi=PWM, Bi=0 alternates forward drive and coast. */
        pwm_disable(device->in2);
        pwm_setDuty(device->in1, duty);
    } else {
        /* RZ7889: Fi=0, Bi=PWM alternates reverse drive and coast. */
        pwm_disable(device->in1);
        pwm_setDuty(device->in2, duty);
    }
}

void motor_tt_init(void)
{
    size_t i;

    if (motor_tt_initialized) {
        return;
    }

    for (i = 0U; i < sizeof(motor_devices) / sizeof(motor_devices[0]); ++i) {
        pwm_disable(motor_devices[i].in1);
        pwm_disable(motor_devices[i].in2);
        motor_devices[i].enabled = false;
    }

    motor_tt_initialized = true;
}

void motor_tt_enable(motor_tt_id_t id)
{
    motor_tt_device_t *device;

    if (!motor_tt_is_id_valid(id)) {
        return;
    }
    if (!motor_tt_initialized) {
        return;
    }

    device = motor_tt_device(id);
    taskENTER_CRITICAL();
    motor_tt_apply_duty(device, 0.0f);
    device->enabled = true;
    taskEXIT_CRITICAL();
}

void motor_tt_setDuty(motor_tt_id_t id, float ratio)
{
    motor_tt_device_t *device;

    if (!motor_tt_is_id_valid(id) || !isfinite(ratio) ||
        ratio < -1.0f || ratio > 1.0f) {
        return;
    }
    if (!motor_tt_initialized) {
        return;
    }

    device = motor_tt_device(id);
    taskENTER_CRITICAL();
    if (device->enabled) {
        motor_tt_apply_duty(device, ratio);
    }
    taskEXIT_CRITICAL();
}

void motor_tt_disable(motor_tt_id_t id)
{
    motor_tt_device_t *device;

    if (!motor_tt_is_id_valid(id)) {
        return;
    }
    if (!motor_tt_initialized) {
        return;
    }

    device = motor_tt_device(id);
    taskENTER_CRITICAL();
    motor_tt_apply_duty(device, 0.0f);
    device->enabled = false;
    taskEXIT_CRITICAL();
}


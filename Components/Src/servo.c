#include "servo.h"

#include <math.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "pwm.h"
#include "task.h"

/* Calibration must come from the selected actuator datasheets and mechanics. */
static const servo_config_t s_servo_config[6] = {
    { 0U, 0U, 0U, 0.0f, 0.0f, false, false },
    { 0U, 0U, 0U, 0.0f, 0.0f, false, false },
    { 0U, 0U, 0U, 0.0f, 0.0f, false, false },
    { 0U, 0U, 0U, 0.0f, 0.0f, false, false },
    { 0U, 0U, 0U, 0.0f, 0.0f, false, false },
    { 0U, 0U, 0U, 0.0f, 0.0f, false, false }
};

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

static bool servo_is_config_valid(const servo_config_t *config)
{
    return config->calibrated && config->pulse_min_us > 0U &&
           config->pulse_min_us < config->pulse_max_us &&
           config->pulse_max_us < 20000U &&
           config->initial_pulse_us >= config->pulse_min_us &&
           config->initial_pulse_us <= config->pulse_max_us &&
           isfinite(config->angle_min_deg) &&
           isfinite(config->angle_max_deg) &&
           config->angle_min_deg < config->angle_max_deg;
}

static float servo_angle_from_pulse(const servo_config_t *config,
                                    uint32_t pulse_us)
{
    double pulse_fraction =
        ((double)pulse_us - (double)config->pulse_min_us) /
        ((double)config->pulse_max_us - (double)config->pulse_min_us);
    double angle_fraction = config->invert ? 1.0 - pulse_fraction
                                          : pulse_fraction;
    double angle = (double)config->angle_min_deg +
                   angle_fraction * ((double)config->angle_max_deg -
                                     (double)config->angle_min_deg);
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
    const servo_config_t *config;
    uint32_t initial_pulse;

    if (!s_initialized) {
        return;
    }
    if (servo_id_to_index(id, &index)) {
        return;
    }
    config = &s_servo_config[index];
    if (!servo_is_config_valid(config)) {
        return;
    }

    initial_pulse = config->initial_pulse_us;
    pwm_setDuty(servo_pwm_channel(index), (float)initial_pulse / 20000.0f);

    taskENTER_CRITICAL();
    s_servo_state[index].pulse_us = initial_pulse;
    s_servo_state[index].target_angle_deg =
        servo_angle_from_pulse(config, initial_pulse);
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
    const servo_config_t *config;

    if (!s_initialized) {
        return;
    }
    if (servo_id_to_index(id, &index)) {
        return;
    }
    config = &s_servo_config[index];
    if (!servo_is_config_valid(config)) {
        return;
    }
    if (pulse_us < config->pulse_min_us || pulse_us > config->pulse_max_us) {
        return;
    }
    if (!s_servo_state[index].enabled) {
        return;
    }

    pwm_setDuty(servo_pwm_channel(index), (float)pulse_us / 20000.0f);

    taskENTER_CRITICAL();
    s_servo_state[index].pulse_us = pulse_us;
    s_servo_state[index].target_angle_deg =
        servo_angle_from_pulse(config, pulse_us);
    taskEXIT_CRITICAL();
}

void servo_setAngle(servo_id_t id, float angle_deg)
{
    size_t index;
    const servo_config_t *config;
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
    config = &s_servo_config[index];
    if (!servo_is_config_valid(config)) {
        return;
    }
    if (angle_deg < config->angle_min_deg ||
        angle_deg > config->angle_max_deg) {
        return;
    }
    if (!s_servo_state[index].enabled) {
        return;
    }

    fraction = ((double)angle_deg - (double)config->angle_min_deg) /
               ((double)config->angle_max_deg -
                (double)config->angle_min_deg);
    if (config->invert) {
        fraction = 1.0 - fraction;
    }
    pulse = (double)config->pulse_min_us +
            fraction * ((double)config->pulse_max_us -
                        (double)config->pulse_min_us);
    pulse_us = (uint32_t)floor(pulse + 0.5);
    if (pulse_us < config->pulse_min_us || pulse_us > config->pulse_max_us) {
        return;
    }

    pwm_setDuty(servo_pwm_channel(index), (float)pulse_us / 20000.0f);

    taskENTER_CRITICAL();
    s_servo_state[index].pulse_us = pulse_us;
    s_servo_state[index].target_angle_deg =
        servo_angle_from_pulse(config, pulse_us);
    taskEXIT_CRITICAL();
}


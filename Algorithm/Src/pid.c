#include "pid.h"

#include <math.h>
#include <stddef.h>

static bool pid_is_config_valid(const pid_config_t *config)
{
    return config != NULL &&
           isfinite(config->kp) &&
           isfinite(config->ki) &&
           isfinite(config->kd) &&
           isfinite(config->output_min) &&
           isfinite(config->output_max) &&
           isfinite(config->integral_min) &&
           isfinite(config->integral_max) &&
           config->output_min <= config->output_max &&
           config->integral_min <= 0.0f &&
           config->integral_max >= 0.0f &&
           config->integral_min <= config->integral_max;
}

static float pid_clamp(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

int pid_init(pid_t *pid, const pid_config_t *config)
{
    if (pid == NULL || !pid_is_config_valid(config)) {
        return -1;
    }

    pid->config = *config;
    pid->integral = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->has_previous = false;
    pid->initialized = true;
    return 0;
}

int pid_reset(pid_t *pid, float integral)
{
    float limited_integral;

    if (pid == NULL || !pid->initialized || !isfinite(integral) ||
        !pid_is_config_valid(&pid->config)) {
        return -1;
    }

    limited_integral = pid_clamp(integral,
                                 pid->config.integral_min,
                                 pid->config.integral_max);

    pid->integral = limited_integral;
    pid->previous_measurement = 0.0f;
    pid->has_previous = false;
    return 0;
}

int pid_step(pid_t *pid, float target, float measurement,
              float dt_s, float *output)
{
    float error;
    float proportional;
    float derivative;
    float integral_delta;
    float candidate_integral;
    float candidate_output;
    float accepted_integral;
    float raw_output;
    float limited_output;

    if (pid == NULL || output == NULL || !pid->initialized ||
        !isfinite(target) || !isfinite(measurement) ||
        !isfinite(dt_s) || dt_s <= 0.0f ||
        !pid_is_config_valid(&pid->config) ||
        !isfinite(pid->integral) ||
        (pid->has_previous && !isfinite(pid->previous_measurement))) {
        return -1;
    }

    error = target - measurement;
    if (!isfinite(error)) {
        return -1;
    }

    proportional = pid->config.kp * error;
    if (!isfinite(proportional)) {
        return -1;
    }

    if (pid->has_previous) {
        float measurement_delta = measurement - pid->previous_measurement;
        float measurement_rate;

        if (!isfinite(measurement_delta)) {
            return -1;
        }
        measurement_rate = measurement_delta / dt_s;
        if (!isfinite(measurement_rate)) {
            return -1;
        }
        derivative = -pid->config.kd * measurement_rate;
    } else {
        derivative = 0.0f;
    }
    if (!isfinite(derivative)) {
        return -1;
    }

    integral_delta = pid->config.ki * error;
    if (!isfinite(integral_delta)) {
        return -1;
    }
    integral_delta *= dt_s;
    if (!isfinite(integral_delta)) {
        return -1;
    }

    candidate_integral = pid->integral + integral_delta;
    if (!isfinite(candidate_integral)) {
        return -1;
    }
    candidate_integral = pid_clamp(candidate_integral,
                                   pid->config.integral_min,
                                   pid->config.integral_max);

    candidate_output = proportional + candidate_integral;
    if (!isfinite(candidate_output)) {
        return -1;
    }
    candidate_output += derivative;
    if (!isfinite(candidate_output)) {
        return -1;
    }

    accepted_integral = candidate_integral;
    if ((candidate_output > pid->config.output_max &&
         candidate_integral > pid->integral) ||
        (candidate_output < pid->config.output_min &&
         candidate_integral < pid->integral)) {
        accepted_integral = pid->integral;
    }

    raw_output = proportional + accepted_integral;
    if (!isfinite(raw_output)) {
        return -1;
    }
    raw_output += derivative;
    if (!isfinite(raw_output)) {
        return -1;
    }

    limited_output = pid_clamp(raw_output,
                               pid->config.output_min,
                               pid->config.output_max);

    pid->integral = accepted_integral;
    pid->previous_measurement = measurement;
    pid->has_previous = true;
    *output = limited_output;
    return 0;
}

#ifndef RMCB_PID_H
#define RMCB_PID_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
} pid_config_t;

typedef struct {
    pid_config_t config;
    float integral;
    float previous_measurement;
    bool has_previous;
    bool initialized;
} pid_t;

int pid_init(pid_t *pid, const pid_config_t *config);
int pid_reset(pid_t *pid, float integral);
int pid_step(pid_t *pid,
              float target,
              float measurement,
              float dt_s,
              float *output);

#endif /* RMCB_PID_H */

#ifndef RAMP_H
#define RAMP_H

#include <stdbool.h>

typedef struct {
    float value;
    float rise_per_s;
    float fall_per_s;
    bool initialized;
} ramp_t;

int ramp_init(ramp_t *ramp,
               float initial,
               float rise_per_s,
               float fall_per_s);
int ramp_reset(ramp_t *ramp, float value);
int ramp_step(ramp_t *ramp, float target, float dt_s, float *output);

#endif /* RAMP_H */

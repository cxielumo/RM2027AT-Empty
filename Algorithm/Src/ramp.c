#include "ramp.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

static bool ramp_is_finite_nonnegative(float value) {
  return isfinite(value) && value >= 0.0F;
}

int ramp_init(ramp_t *ramp, float initial, float rise_per_s,
               float fall_per_s) {
  if (ramp == NULL || !isfinite(initial) ||
      !ramp_is_finite_nonnegative(rise_per_s) ||
      !ramp_is_finite_nonnegative(fall_per_s)) {
    return -1;
  }

  ramp->value = initial;
  ramp->rise_per_s = rise_per_s;
  ramp->fall_per_s = fall_per_s;
  ramp->initialized = true;
  return 0;
}

int ramp_reset(ramp_t *ramp, float value) {
  if (ramp == NULL || !ramp->initialized || !isfinite(value)) {
    return -1;
  }

  ramp->value = value;
  return 0;
}

int ramp_step(ramp_t *ramp, float target, float dt_s, float *output) {
  double current;
  double goal;
  double max_delta;
  double next;
  float rate;

  if (ramp == NULL || output == NULL || !ramp->initialized ||
      !isfinite(ramp->value) ||
      !ramp_is_finite_nonnegative(ramp->rise_per_s) ||
      !ramp_is_finite_nonnegative(ramp->fall_per_s) || !isfinite(target) ||
      !isfinite(dt_s) || dt_s <= 0.0F) {
    return -1;
  }

  current = (double)ramp->value;
  goal = (double)target;
  if (goal > current) {
    rate = ramp->rise_per_s;
  } else if (goal < current) {
    rate = ramp->fall_per_s;
  } else {
    rate = 0.0F;
  }

  max_delta = (double)rate * (double)dt_s;
  if (!isfinite(max_delta) || max_delta < 0.0) {
    return -1;
  }

  if (goal > current) {
    const double distance = goal - current;
    next = (distance <= max_delta) ? goal : current + max_delta;
  } else if (goal < current) {
    const double distance = current - goal;
    next = (distance <= max_delta) ? goal : current - max_delta;
  } else {
    next = current;
  }

  if (!isfinite(next) || next > (double)FLT_MAX || next < -(double)FLT_MAX) {
    return -1;
  }

  {
    const float result = (float)next;
    if (!isfinite(result)) {
      return -1;
    }
    ramp->value = result;
    *output = result;
  }

  return 0;
}

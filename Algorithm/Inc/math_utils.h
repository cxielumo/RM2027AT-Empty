#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>

static inline float clampf(float x, float lower, float upper)
{
    return x < lower ? lower : (x > upper ? upper : x);
}

static inline float deadzone(float x, float width)
{
    return fabsf(x) <= width ? 0.0f : x;
}

#define DEG2RAD(deg) ((deg) * 0.01745329251994329577f)
#define RAD2DEG(rad) ((rad) * 57.295779513082320876f)

#endif /* MATH_UTILS_H */

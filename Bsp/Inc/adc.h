#ifndef RM2027AT_BSP_ADC_H
#define RM2027AT_BSP_ADC_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"

typedef struct {
    uint16_t raw;
    uint16_t full_scale;
    uint32_t timestamp_ms;
} adc_sample_t;

int adc_sample(adc_sample_t *out, uint32_t timeout_ms);

#endif /* RM2027AT_BSP_ADC_H */

#include "adc.h"

#include "os.h"
#include "wk_adc.h"

#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>

#define ADC_RAW_FULL_SCALE 4095U

static StaticSemaphore_t s_adc_mutex_storage;
static SemaphoreHandle_t s_adc_mutex;
static TickType_t adc_timeout_ticks(uint32_t timeout_ms)
{
    uint64_t ticks;

    ticks = ((uint64_t)timeout_ms * (uint64_t)configTICK_RATE_HZ + 999ULL) / 1000ULL;
    if (ticks == 0ULL) {
        ticks = 1ULL;
    }
    if (ticks >= (uint64_t)portMAX_DELAY) {
        ticks = (uint64_t)portMAX_DELAY - 1ULL;
    }

    return (TickType_t)ticks;
}

static uint32_t adc_elapsed_ms(uint32_t start_ms)
{
    return (uint32_t)(os_getTime() - start_ms);
}

void adc_init(void)
{
    if (s_adc_mutex == NULL) {
        s_adc_mutex = xSemaphoreCreateMutexStatic(&s_adc_mutex_storage);
        if (s_adc_mutex == NULL) {
            panic(FAULT_INIT);
        }
    }

    /* wk_adc1_init owns peripheral setup/calibration and must run before bsp_init. */
    if (adc_flag_get(ADC1, ADC_RDY_FLAG) != SET) {
        panic(FAULT_INIT);
    }
}

int adc_sample(adc_sample_t *out, uint32_t timeout_ms)
{
    TickType_t lock_timeout;
    uint32_t start_ms;
    uint16_t raw;
    bool sample_ready = false;

    if ((out == NULL) || (timeout_ms == 0U)) {
        return -1;
    }
    if (s_adc_mutex == NULL) {
        return -1;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return -1;
    }

    start_ms = os_getTime();
    lock_timeout = adc_timeout_ticks(timeout_ms);
    if (xSemaphoreTake(s_adc_mutex, lock_timeout) != pdTRUE) {
        return -1;
    }

    if (adc_elapsed_ms(start_ms) >= timeout_ms) {
        goto cleanup;
    }

    /* A timed-out conversion may have completed after its caller returned. */
    if (adc_flag_get(ADC1, ADC_OCCS_FLAG) == SET) {
        goto cleanup;
    }
    if (adc_flag_get(ADC1, ADC_OCCE_FLAG) == SET) {
        (void)adc_ordinary_conversion_data_get(ADC1);
    }
    if (adc_flag_get(ADC1, ADC_OCCO_FLAG) == SET) {
        adc_flag_clear(ADC1, ADC_OCCO_FLAG);
    }

    adc_ordinary_software_trigger_enable(ADC1, TRUE);
    for (;;) {
        /* The caller's deadline covers mutex wait and conversion alike. */
        if (adc_elapsed_ms(start_ms) >= timeout_ms) {
            /* Deassert the request; a still-active conversion is detected next time. */
            adc_ordinary_software_trigger_enable(ADC1, FALSE);
            if (adc_flag_get(ADC1, ADC_OCCE_FLAG) == SET) {
                (void)adc_ordinary_conversion_data_get(ADC1);
            }
            if (adc_flag_get(ADC1, ADC_OCCO_FLAG) == SET) {
                adc_flag_clear(ADC1, ADC_OCCO_FLAG);
            }
            goto cleanup;
        }
        if (adc_flag_get(ADC1, ADC_OCCE_FLAG) == SET) {
            break;
        }
        vTaskDelay(1U);
    }

    /* EOC can become set during the delay; the deadline still takes precedence. */
    if (adc_elapsed_ms(start_ms) >= timeout_ms) {
        adc_ordinary_software_trigger_enable(ADC1, FALSE);
        if (adc_flag_get(ADC1, ADC_OCCE_FLAG) == SET) {
            (void)adc_ordinary_conversion_data_get(ADC1);
        }
        if (adc_flag_get(ADC1, ADC_OCCO_FLAG) == SET) {
            adc_flag_clear(ADC1, ADC_OCCO_FLAG);
        }
        goto cleanup;
    }

    raw = adc_ordinary_conversion_data_get(ADC1);
    adc_ordinary_software_trigger_enable(ADC1, FALSE);
    if (adc_flag_get(ADC1, ADC_OCCO_FLAG) == SET) {
        adc_flag_clear(ADC1, ADC_OCCO_FLAG);
        goto cleanup;
    }

    out->raw = raw;
    out->full_scale = (uint16_t)ADC_RAW_FULL_SCALE;
    out->timestamp_ms = os_getTime();
    sample_ready = true;

cleanup:
    (void)xSemaphoreGive(s_adc_mutex);
    return sample_ready ? 0 : -1;
}

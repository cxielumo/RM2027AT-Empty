#include "led.h"
#include "bsp.h"
#include "at32f423_wk_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static int led_pin_mask(led_id_t id, uint16_t *pin_mask)
{
    if ((pin_mask == NULL) || ((unsigned int)id >= (unsigned int)LAST_LED)) {
        return -1;
    }
    /* LED_1..LED_3 are zero-based and drive PC13..PC15. */
    *pin_mask = (uint16_t)(1U << (13U + (unsigned int)id));
    return 0;
}

static void led_write(led_id_t id, bool on)
{
    uint16_t pin_mask;
    if (led_pin_mask(id, &pin_mask)) {
        return;
    }
    if (on) {
        gpio_bits_set(GPIOC, pin_mask);
    } else {
        gpio_bits_reset(GPIOC, pin_mask);
    }
}

void led_init(void)
{
    gpio_bits_reset(GPIOC, GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15);
}

void led_set(led_id_t id, bool on)
{
    led_write(id, on);
}

void led_on(led_id_t id)
{
    led_set(id, true);
}

void led_off(led_id_t id)
{
    led_set(id, false);
}

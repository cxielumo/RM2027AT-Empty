#ifndef RM2027AT_LED_H
#define RM2027AT_LED_H

#include <stdbool.h>

#include "bsp.h"

typedef enum {
  LED_1,
  LED_2,
  LED_3,
  LAST_LED
} led_id_t;

void led_on(led_id_t id);
void led_off(led_id_t id);
void led_set(led_id_t id, bool on);

#endif

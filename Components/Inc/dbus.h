#ifndef RM2027AT_DBUS_H
#define RM2027AT_DBUS_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"

typedef enum {
  DBUS_SWITCH_UNKNOWN,
  DBUS_SWITCH_UP,
  DBUS_SWITCH_MIDDLE,
  DBUS_SWITCH_DOWN,
  LAST_DBUS_SWITCH
} dbus_switch_t;

typedef struct {
    int16_t channel_raw[4];
    float channel[4];
    dbus_switch_t switch_left;
    dbus_switch_t switch_right;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    bool mouse_left;
    bool mouse_right;
    uint16_t keys;
    uint16_t wheel_raw;
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool valid;
    bool online;
} dbus_t;

typedef struct {
    int16_t center_raw;
    uint16_t span_raw;
    uint16_t deadzone_raw;
    uint32_t timeout_ms;
} dbus_config_t;

int dbus_get(dbus_t *dbus);

#endif

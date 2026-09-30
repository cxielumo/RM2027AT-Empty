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

/* Protocol bit masks, usable directly with dbus_t.keys. */
typedef enum {
    DBUS_KEY_W     = 1U << 0,
    DBUS_KEY_S     = 1U << 1,
    DBUS_KEY_D     = 1U << 2,
    DBUS_KEY_A     = 1U << 3,
    DBUS_KEY_SHIFT = 1U << 4,
    DBUS_KEY_CTRL  = 1U << 5,
    DBUS_KEY_Q     = 1U << 6,
    DBUS_KEY_E     = 1U << 7,
    DBUS_KEY_R     = 1U << 8,
    DBUS_KEY_F     = 1U << 9,
    DBUS_KEY_G     = 1U << 10,
    DBUS_KEY_Z     = 1U << 11,
    DBUS_KEY_X     = 1U << 12,
    DBUS_KEY_C     = 1U << 13,
    DBUS_KEY_V     = 1U << 14,
    DBUS_KEY_B     = 1U << 15
} dbus_key_t;

typedef struct {
    float channel[4];
    dbus_switch_t switch_left;
    dbus_switch_t switch_right;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    bool mouse_left;
    bool mouse_right;
    uint16_t keys;
    float wheel; /* Centered at 1024, deadzone 33, normalized to [-1, 1]. */
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool valid;
    bool online;
} dbus_t;

/* Wait for a valid frame published after entry; task context only.
 * Returns elapsed milliseconds (uint32 wrap); UINT32_MAX means unavailable.
 * Each caller waits independently. No timeout: disconnect keeps waiting. */
uint32_t dbus_waitData(void);
int dbus_get(dbus_t *dbus);

/* Call after dbus_init(), while the input source is idle.
 * true: normal TTL UART RX at 100000 baud, 8N2 (PC simulator).
 * false: inverted DBUS RX at 100000 baud, 8E2 (DR16, default).
 * dbus_init() always configures the default DR16 mode.
 * Both modes use the same 18-byte payload and idle frame boundary.
 */
void dbus_useUartInstead(bool use_uart);

#endif

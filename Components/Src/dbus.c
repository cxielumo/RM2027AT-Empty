#include "dbus.h"
#include "daemon.h"

#include <stddef.h>

#include "at32f423_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math_utils.h"
#include "os.h"
#include "uart.h"

#define DBUS_FRAME_SIZE       18U
#define DBUS_PROTOCOL_MIN    (-660)
#define DBUS_PROTOCOL_MAX      660

#define DBUS_CENTER_RAW  1024
#define DBUS_SPAN_RAW    660U
#define DBUS_DEADZONE_RAW 33U
#define DBUS_TIMEOUT_MS  100U

static bool dbus_registered;
static dbus_t dbus_snapshot;
static bool dbus_init_called;

static uint16_t read_u16_le(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
}

static int16_t read_i16_le(const uint8_t *bytes)
{
    uint16_t value = read_u16_le(bytes);
    int32_t signed_value = (value < 0x8000U) ? (int32_t)value
                                               : (int32_t)value - 65536L;
    return (int16_t)signed_value;
}

static dbus_switch_t decode_switch(uint8_t raw)
{
    switch (raw) {
    case 1U:
        return DBUS_SWITCH_UP;
    case 2U:
        return DBUS_SWITCH_DOWN;
    case 3U:
        return DBUS_SWITCH_MIDDLE;
    default:
        return DBUS_SWITCH_UNKNOWN;
    }
}

static int decode_frame(const uint8_t frame[DBUS_FRAME_SIZE],
                        dbus_t *decoded)
{
    uint16_t channel_bits[4];
    uint8_t switch_left_raw;
    uint8_t switch_right_raw;
    channel_bits[0] = (uint16_t)(((uint16_t)frame[0] |
                                  ((uint16_t)frame[1] << 8U)) & 0x07FFU);
    channel_bits[1] = (uint16_t)((((uint16_t)frame[1] >> 3U) |
                                  ((uint16_t)frame[2] << 5U)) & 0x07FFU);
    channel_bits[2] = (uint16_t)((((uint16_t)frame[2] >> 6U) |
                                  ((uint16_t)frame[3] << 2U) |
                                  ((uint16_t)frame[4] << 10U)) & 0x07FFU);
    channel_bits[3] = (uint16_t)((((uint16_t)frame[4] >> 1U) |
                                  ((uint16_t)frame[5] << 7U)) & 0x07FFU);

    for (size_t i = 0U; i < 4U; ++i) {
        int16_t raw = (int16_t)((int32_t)channel_bits[i] -
                                (int32_t)DBUS_CENTER_RAW);
        float normalized;

        if (raw < DBUS_PROTOCOL_MIN || raw > DBUS_PROTOCOL_MAX) {
            return -1;
        }

        normalized = (float)raw / (float)DBUS_SPAN_RAW;
        if (!isfinite(normalized)) {
            panic(FAULT_ASSERT);
        }
        decoded->channel[i] = clampf(
            deadzone(normalized,
                     (float)DBUS_DEADZONE_RAW /
                         (float)DBUS_SPAN_RAW),
            -1.0f, 1.0f);
    }

    switch_left_raw = (uint8_t)((frame[5] >> 6U) & 0x03U);
    switch_right_raw = (uint8_t)((frame[5] >> 4U) & 0x03U);
    decoded->switch_left = decode_switch(switch_left_raw);
    decoded->switch_right = decode_switch(switch_right_raw);
    if ((decoded->switch_left == DBUS_SWITCH_UNKNOWN) ||
        (decoded->switch_right == DBUS_SWITCH_UNKNOWN)) {
        return -1;
    }

    decoded->mouse_x = read_i16_le(&frame[6]);
    decoded->mouse_y = read_i16_le(&frame[8]);
    decoded->mouse_z = read_i16_le(&frame[10]);
    if ((frame[12] > 1U) || (frame[13] > 1U)) {
        return -1;
    }
    decoded->mouse_left = (frame[12] != 0U);
    decoded->mouse_right = (frame[13] != 0U);
    decoded->keys = read_u16_le(&frame[14]);
    decoded->wheel = clampf(
        deadzone(((float)read_u16_le(&frame[16]) - (float)DBUS_CENTER_RAW) /
                     (float)DBUS_SPAN_RAW,
                 (float)DBUS_DEADZONE_RAW / (float)DBUS_SPAN_RAW),
        -1.0f, 1.0f);
    decoded->valid = true;
    decoded->online = true;
    return 0;
}

static void publish_frame(const uint8_t frame[DBUS_FRAME_SIZE],
                          uint32_t timestamp_ms)
{
    dbus_t decoded = {0};

    if (decode_frame(frame, &decoded)) {
        return;
    }

    decoded.timestamp_ms = timestamp_ms;
    taskENTER_CRITICAL();
    decoded.sequence = dbus_snapshot.sequence + 1U;
    dbus_snapshot = decoded;
    taskEXIT_CRITICAL();
}

static void dbus_receive(const uart_frame_t *frame, void *context)
{
    (void)context;
    if (frame->length == DBUS_FRAME_SIZE && frame->error_flags == 0U)
        publish_frame(frame->data, frame->timestamp_ms);
}

uint32_t dbus_waitData(void)
{
    uint32_t sequence, start;
    if (!dbus_registered || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
        return UINT32_MAX;
    taskENTER_CRITICAL();
    sequence = dbus_snapshot.sequence;
    start = os_getTime();
    taskEXIT_CRITICAL();
    for (;;) {
        bool changed;
        taskENTER_CRITICAL();
        changed = dbus_snapshot.sequence != sequence;
        taskEXIT_CRITICAL();
        if (changed) return os_getTime() - start;
        vTaskDelay(1);
    }
}

void dbus_useUartInstead(bool use_uart)
{
    confirm_state was_enabled = USART6->ctrl1_bit.uen ? TRUE : FALSE;

    usart_enable(USART6, FALSE);
    usart_parity_selection_config(USART6, USART_PARITY_NONE);
    usart_init(USART6, 100000,
               use_uart ? USART_DATA_8BITS : USART_DATA_9BITS,
               USART_STOP_2_BIT);
    usart_parity_selection_config(USART6,
                                  use_uart ? USART_PARITY_NONE : USART_PARITY_EVEN);
    usart_receive_pin_polarity_reverse(USART6, use_uart ? FALSE : TRUE);
    usart_enable(USART6, was_enabled);
}

void dbus_init(void)
{
    if (dbus_init_called) {
        return;
    }
    dbus_init_called = true;

    dbus_useUartInstead(false);

    if (uart_register(UART_6, dbus_receive, NULL) != 0) panic(FAULT_INIT);
    dbus_registered = true;
}

int dbus_get(dbus_t *out)
{
    dbus_t snapshot;
    uint32_t now_ms;

    if (out == NULL) {
        return -1;
    }
    if (!dbus_init_called || !dbus_registered) {
        return -1;
    }

    taskENTER_CRITICAL();
    snapshot = dbus_snapshot;
    taskEXIT_CRITICAL();
    now_ms = os_getTime();

    if (!snapshot.valid ||
        ((uint32_t)(now_ms - snapshot.timestamp_ms) >= DBUS_TIMEOUT_MS)) {
        snapshot.online = false;
    }
    *out = snapshot;
    if (snapshot.valid && snapshot.online) {
        daemon_reload();
    }
    return 0;
}

#ifndef RM2027AT_UART_H
#define RM2027AT_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp.h"

typedef enum {
  UART_1,
  UART_6,
  UART_7,
  LAST_UART_ID
} uart_id_t;

#define UART_FRAME_CAPACITY 64U

typedef struct {
    uint8_t data[UART_FRAME_CAPACITY];
    size_t length;
    uint32_t timestamp_ms;
    uint32_t error_flags;
} uart_frame_t;

#define UART_ERROR_PARITY   (1U << 0)
#define UART_ERROR_FRAMING  (1U << 1)
#define UART_ERROR_OVERRUN  (1U << 2)
#define UART_ERROR_OVERFLOW (1U << 3)

int uart_receiveFrame(uart_id_t id, uart_frame_t *out, uint32_t timeout_ms);

size_t uart_write(uart_id_t id, const uint8_t *data, size_t length);
size_t uart_read(uart_id_t id, uint8_t *data, size_t capacity);

#endif

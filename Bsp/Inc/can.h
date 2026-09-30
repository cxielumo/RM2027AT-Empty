#ifndef RM2027AT_CAN_H
#define RM2027AT_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"

typedef enum {
  CAN_BUS_1,
  CAN_BUS_2,
  LAST_CAN_BUS
} can_bus_t;

typedef struct {
    uint32_t id;
    bool extended;
    bool remote;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t timestamp_ms;
} can_frame_t;

typedef void (*can_callback_t)(const can_frame_t *frame, void *context);
/* Exact ID match; one owner per bus/ID/type, max 16 registrations.
 * Callback runs in task context; frame is borrowed for the callback only. */
int can_register(can_bus_t bus, uint32_t id, bool extended,
                 can_callback_t callback, void *context);

void can_send(can_bus_t bus, const can_frame_t *frame);
/* Empty queue is distinct from bus faults so consumers can stop safely. */
#define CAN_RECEIVE_EMPTY (-1)
#define CAN_RECEIVE_ERROR (-2)
int can_receive(can_bus_t bus, can_frame_t *out);

#endif

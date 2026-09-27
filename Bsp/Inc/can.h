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

void can_send(can_bus_t bus, const can_frame_t *frame);
int can_receive(can_bus_t bus, can_frame_t *out);

#endif

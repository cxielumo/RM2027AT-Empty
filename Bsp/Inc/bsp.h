#ifndef RM2027AT_BSP_H
#define RM2027AT_BSP_H

typedef enum {
  FAULT_INIT,
  FAULT_ASSERT,
  FAULT_STACK_OVERFLOW,
  FAULT_SCHEDULER,
  FAULT_HARDWARE,
  LAST_FAULT_REASON
} fault_reason_t;

void panic(fault_reason_t reason);

#endif

#ifndef RM2027AT_OS_H
#define RM2027AT_OS_H

#include "bsp.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*os_taskEntry_t)(void *args);
typedef TaskHandle_t os_task_t;

#define OS_WAIT_FOREVER UINT32_MAX

typedef enum {
  OS_TASK_PRIORITY_LOW,
  OS_TASK_PRIORITY_NORMAL,
  OS_TASK_PRIORITY_HIGH,
  LAST_OS_TASK_PRIORITY
} os_task_priority_t;

/* Dynamic task creation and deletion are reserved for Application. */
os_task_t os_createTask(const char *name,
                        os_taskEntry_t entry,
                        void *args,
                        uint32_t priority,
                        size_t stack_words);

void os_deleteTask(os_task_t task);

uint32_t os_getTime(void);

void os_delay(uint32_t delay_ms);
void os_delayUntil(uint32_t *last_wake, uint32_t period_ms);

#endif

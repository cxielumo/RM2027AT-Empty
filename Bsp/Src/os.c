#include "os.h"

#include "bsp.h"

#include <stdint.h>

/* FreeRTOS requires application-owned idle memory when static allocation is
 * enabled.  Keep it here, with the kernel integration code. */
static StaticTask_t idle_task_object;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **task_object,
                                   StackType_t **task_stack,
                                   configSTACK_DEPTH_TYPE *stack_size)
{
    *task_object = &idle_task_object;
    *task_stack = idle_task_stack;
    *stack_size = (configSTACK_DEPTH_TYPE)configMINIMAL_STACK_SIZE;
}

static int os_ms2ticks(uint32_t milliseconds, TickType_t *ticks)
{
    uint64_t converted;

    if (ticks == NULL) {
        return -1;
    }

    converted = ((uint64_t)milliseconds * (uint64_t)configTICK_RATE_HZ +
                 UINT64_C(999)) / UINT64_C(1000);
    /* Reserve portMAX_DELAY for OS_WAIT_FOREVER in blocking APIs. */
    if (converted >= (uint64_t)portMAX_DELAY) {
        return -1;
    }

    *ticks = (TickType_t)converted;
    return 0;
}

os_task_t os_createTask(const char *name, os_taskEntry_t entry,
                        void *args, uint32_t priority, size_t stack_words)
{
    TaskHandle_t task = NULL;

    if ((name == NULL) || (entry == NULL) || (stack_words == 0U) ||
        (stack_words > (size_t)UINT32_MAX) ||
        (priority >= (uint32_t)configMAX_PRIORITIES)) {
        return NULL;
    }

    if (xTaskCreate(entry, name, (configSTACK_DEPTH_TYPE)stack_words, args,
                    (UBaseType_t)priority, &task) != pdPASS) {
        return NULL;
    }

    return task;
}

void os_delay(uint32_t delay_ms)
{
    TickType_t delay_ticks;

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }

    if (delay_ms == 0U) {
        taskYIELD();
        return;
    }

    if (os_ms2ticks(delay_ms, &delay_ticks)) {
        delay_ticks = (TickType_t)(portMAX_DELAY - (TickType_t)1U);
    }
    if (delay_ticks == 0U) {
        delay_ticks = (TickType_t)1U;
    }
    vTaskDelay(delay_ticks);
}

void os_delayUntil(uint32_t *last_wake, uint32_t period_ms)
{
    TickType_t period_ticks;
    TickType_t last_wake_tick;

    if ((last_wake == NULL) || (period_ms == 0U) ||
        os_ms2ticks(period_ms, &period_ticks) || (period_ticks == 0U)) {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }

    last_wake_tick = (TickType_t)*last_wake;
    if (xTaskDelayUntil(&last_wake_tick, period_ticks) == pdFALSE) {
        last_wake_tick = xTaskGetTickCount();
    }
    *last_wake = (uint32_t)last_wake_tick;
}

uint32_t os_getTime(void)
{
    static TickType_t previous_tick;
    static uint64_t accumulated_ticks;
    TickType_t current_tick;
    uint64_t elapsed_ticks;
    uint64_t milliseconds;

    taskENTER_CRITICAL();
    current_tick = xTaskGetTickCount();
    /* Unsigned subtraction extends a wrapping tick counter from its zero epoch,
     * as long as this function is called at least once per full counter span. */
    elapsed_ticks = (TickType_t)(current_tick - previous_tick);
    accumulated_ticks += elapsed_ticks;
    previous_tick = current_tick;
    milliseconds = (accumulated_ticks * UINT64_C(1000)) /
                   (uint64_t)configTICK_RATE_HZ;
    taskEXIT_CRITICAL();

    return (uint32_t)milliseconds;
}

void os_deleteTask(os_task_t task)
{
    if (task == NULL) {
        return;
    }
    vTaskDelete(task);
}

void os_assertFailed(void)
{
    panic(FAULT_ASSERT);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    panic(FAULT_STACK_OVERFLOW);
}

void vApplicationMallocFailedHook(void)
{
    panic(FAULT_SCHEDULER);
}

#include "daemon.h"
#include <stddef.h>

#include "os.h"
/* Internal DJI shutdown entry; TT outputs are independent of daemon. */
void motor_dji_shutdown(void);

static uint32_t timeout_ms;
static uint32_t last_reload_ms;
static bool enabled;
static bool online;
static bool timeout_armed;
static daemon_callback_t offline_callback;
static void *callback_context;
static StaticTask_t monitor_control;
static StackType_t monitor_stack[384];
static TaskHandle_t monitor_handle;

static void monitor_task(void *args)
{
    (void)args;
    for (;;) {
        daemon_callback_t callback = NULL;
        void *context = NULL;
        taskENTER_CRITICAL();
        /* Keep timeout detection and shutdown atomic with application reload. */
        if (enabled && !daemon_isOnline()) motor_dji_shutdown();
        if (enabled && timeout_armed &&
            (uint32_t)(os_getTime() - last_reload_ms) >= timeout_ms) {
            timeout_armed = false;
            callback = offline_callback;
            context = callback_context;
        }
        taskEXIT_CRITICAL();
        if (callback != NULL) callback(context);
        vTaskDelay(1);
    }
}

void daemon_registerCallback(daemon_callback_t callback, void *context)
{
    taskENTER_CRITICAL();
    offline_callback = callback;
    callback_context = context;
    taskEXIT_CRITICAL();
}

/* Internal output gate. Before enable, existing motor behavior is preserved. */
bool daemon_motorAllowed(void)
{
    bool allowed;
    taskENTER_CRITICAL();
    allowed = !enabled || daemon_isOnline();
    taskEXIT_CRITICAL();
    return allowed;
}

int daemon_enable(uint32_t timeout)
{
    if (timeout == 0U || timeout > INT32_MAX) return -1;
    taskENTER_CRITICAL();
    if (monitor_handle == NULL) {
        monitor_handle = xTaskCreateStatic(monitor_task, "daemon", 384, NULL, 4,
                                          monitor_stack, &monitor_control);
        if (monitor_handle == NULL) {
            taskEXIT_CRITICAL();
            return -1;
        }
    }
    timeout_ms = timeout;
    last_reload_ms = os_getTime();
    online = false;
    timeout_armed = true;
    enabled = true;
    motor_dji_shutdown();
    taskEXIT_CRITICAL();
    return 0;
}

void daemon_reload(void)
{
    taskENTER_CRITICAL();
    if (enabled) {
        last_reload_ms = os_getTime();
        online = true;
        timeout_armed = true;
    }
    taskEXIT_CRITICAL();
}

bool daemon_isOnline(void)
{
    bool result;
    taskENTER_CRITICAL();
    if (enabled && (uint32_t)(os_getTime() - last_reload_ms) >= timeout_ms)
        online = false;
    result = enabled && online;
    taskEXIT_CRITICAL();
    return result;
}

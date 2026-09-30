#include "motor_dji.h"

#include <limits.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"

/* Internal daemon output gate. */
bool daemon_motorAllowed(void);

static motor_dji_config_t configs[MOTOR_DJI_MAX_DEVICES];
static size_t device_count;
#define GROUPS_PER_BUS 4U
#define GROUP_COUNT (LAST_CAN_BUS * GROUPS_PER_BUS)
#define RX_BUDGET 32U
#define FEEDBACK_TIMEOUT_MS 100U
#define COMMAND_TIMEOUT_MS 100U

typedef struct {
    motor_dji_t *owner;
    int16_t target;
    uint32_t command_ms;
    uint16_t feedback_id;
    uint8_t group, slot;
} device_t;

static device_t devices[MOTOR_DJI_MAX_DEVICES];
static bool active_groups[GROUP_COUNT];
static bool active_buses[LAST_CAN_BUS];
static bool initialized;
static StaticTask_t task_control;
static StackType_t task_stack[768];
static const uint16_t group_ids[GROUPS_PER_BUS] = { 0x200, 0x1FF, 0x1FE, 0x2FE };

static uint32_t now_ms(void)
{
    return (uint32_t)(((uint64_t)xTaskGetTickCount() * 1000U) / configTICK_RATE_HZ);
}

static int32_t current_limit(motor_dji_model_t model)
{
    switch (model) {
    case MOTOR_DJI_M2006: return 10000;
    case MOTOR_DJI_M3508:
    case MOTOR_DJI_GM6020: return 16384;
    default: return 0;
    }
}

static device_t *find_device(const motor_dji_t *motor, size_t *index)
{
    if (!initialized || motor == NULL || motor->slot == 0U ||
        motor->slot > device_count) return NULL;
    *index = (size_t)motor->slot - 1U;
    return devices[*index].owner == motor ? &devices[*index] : NULL;
}
static void stop_device(device_t *d)
{
    d->owner->measure.enabled = false;
    d->target = 0;
}

static void expire(size_t i, uint32_t now)
{
    device_t *d = &devices[i];
    if (!d->owner->measure.valid ||
        (uint32_t)(now - d->owner->measure.timestamp_ms) >= FEEDBACK_TIMEOUT_MS) {
        d->owner->measure.online = false;
        d->owner->measure.angle_continuous = false;
        stop_device(d);
    }
    if (d->owner->measure.enabled &&
        (uint32_t)(now - d->command_ms) >= COMMAND_TIMEOUT_MS) {
        stop_device(d);
    }
}

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static int16_t read_i16(const uint8_t *p)
{
    uint16_t value = read_u16(p);
    return (int16_t)(value <= INT16_MAX ? (int32_t)value : (int32_t)value - 65536);
}

static int16_t feedback_direction(int16_t value, bool invert)
{
    if (!invert) return value;
    /* +32768 cannot be represented by the signed 16-bit status fields. */
    return value == INT16_MIN ? INT16_MAX : (int16_t)(-(int32_t)value);
}

static void accept_frame(can_bus_t bus, const can_frame_t *frame, uint32_t now)
{
    size_t i;
    uint16_t encoder;
    if (frame->extended || frame->remote || frame->dlc != 8U) return;
    encoder = read_u16(frame->data);
    if (encoder > 8191U) return;
    for (i = 0; i < device_count; ++i) {
        device_t *d = &devices[i];
        volatile motor_dji_status_t *s = &d->owner->measure;
        int32_t delta;
        if (configs[i].bus != bus || d->feedback_id != frame->id) continue;
        expire(i, now);
        if ((uint32_t)(now - frame->timestamp_ms) >= FEEDBACK_TIMEOUT_MS) return;
        if (s->online) {
            delta = (int32_t)encoder - s->encoder_raw;
            if (delta >= 4096) delta -= 8192;
            if (delta < -4096) delta += 8192;
            if (configs[i].invert) delta = -delta;
            if ((delta > 0 && s->angle_counts > INT64_MAX - delta) ||
                (delta < 0 && s->angle_counts < INT64_MIN - delta)) {
                s->angle_counts = 0;
                s->angle_continuous = false;
            } else {
                s->angle_counts += delta;
                s->angle_continuous = true;
            }
        } else {
            s->angle_counts = 0;
            s->angle_continuous = false;
        }
        s->encoder_raw = encoder;
        s->speed_rpm = feedback_direction(read_i16(&frame->data[2]), configs[i].invert);
        s->current_raw = feedback_direction(read_i16(&frame->data[4]), configs[i].invert);
        s->temperature_valid = configs[i].model != MOTOR_DJI_M2006;
        s->temperature_c = s->temperature_valid ? frame->data[6] : 0U;
        s->timestamp_ms = frame->timestamp_ms;
        ++s->sequence;
        s->valid = true;
        s->online = true;
        return;
    }
}

static void service_task(void *args)
{
    unsigned int first_group = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1U) > 0U ? pdMS_TO_TICKS(1U) : 1U;
    (void)args;
    for (;;) {
        unsigned int bus, n;
        size_t i;
        for (bus = 0; bus < LAST_CAN_BUS; ++bus) {
            if (!active_buses[bus]) continue;
            for (n = 0; n < RX_BUDGET; ++n) {
                can_frame_t frame;
                int result = can_receive((can_bus_t)bus, &frame);
                if (result != 0) {
                    if (result == CAN_RECEIVE_ERROR) {
                        taskENTER_CRITICAL();
                        for (i = 0; i < device_count; ++i) {
                            if ((unsigned int)configs[i].bus == bus) {
                                stop_device(&devices[i]);
                                devices[i].owner->measure.online = false;
                                devices[i].owner->measure.angle_continuous = false;
                            }
                        }
                        taskEXIT_CRITICAL();
                    }
                    break;
                }
                taskENTER_CRITICAL();
                accept_frame((can_bus_t)bus, &frame, now_ms());
                taskEXIT_CRITICAL();
            }
        }
        for (n = 0; n < GROUP_COUNT; ++n) {
            unsigned int group = (first_group + n) % GROUP_COUNT;
            can_frame_t frame = { 0 };
            uint32_t now;
            if (!active_groups[group]) continue;
            frame.id = group_ids[group % GROUPS_PER_BUS];
            frame.dlc = 8;
            taskENTER_CRITICAL();
            now = now_ms();
            for (i = 0; i < device_count; ++i) {
                device_t *d = &devices[i];
                uint16_t value;
                if (d->group != group) continue;
                expire(i, now);
                value = d->owner->measure.enabled ?
                    (uint16_t)(configs[i].invert ? -(int32_t)d->target : d->target) : 0U;
                frame.data[2U * d->slot] = (uint8_t)(value >> 8);
                frame.data[2U * d->slot + 1U] = (uint8_t)value;
            }
            /* Keep stop/target updates atomic with the mailbox submission. */
            can_send((can_bus_t)(group / GROUPS_PER_BUS), &frame);
            taskEXIT_CRITICAL();
        }
        first_group = (first_group + 1U) % GROUP_COUNT;
        if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
            /* A missed deadline must not turn this high-priority task into
             * an unbounded catch-up loop. Block, then establish a new phase. */
            vTaskDelay(1U);
            last_wake = xTaskGetTickCount();
        }
    }
}

void motor_dji_init(void)
{
    /* Zero-initialized storage is retained on repeated initialization. */
    initialized = true;
}

int motor_dji_register(motor_dji_t *motor, const motor_dji_config_t *config)
{
    motor_dji_config_t c;
    device_t candidate = { 0 };
    size_t j;
    if (!initialized || motor == NULL || config == NULL) return -1;
    c = *config;
    if (current_limit(c.model) == 0) return -1;
    if (c.id == 0 || (unsigned int)c.bus >= LAST_CAN_BUS ||
        c.id > (c.model == MOTOR_DJI_GM6020 ? 7U : 8U)) return -1;
    candidate.owner = motor;
    candidate.feedback_id = (uint16_t)((c.model == MOTOR_DJI_GM6020 ? 0x204U : 0x200U) + c.id);
    candidate.slot = (uint8_t)((c.id - 1U) % 4U);
    candidate.group = (uint8_t)((unsigned int)c.bus * GROUPS_PER_BUS +
        (c.model == MOTOR_DJI_GM6020 ? 2U : 0U) +
        (c.id - 1U) / 4U);

    taskENTER_CRITICAL();
    if (device_count >= MOTOR_DJI_MAX_DEVICES) {
        taskEXIT_CRITICAL();
        return -1;
    }
    for (j = 0; j < device_count; ++j) {
        if (devices[j].owner == motor || (configs[j].bus == c.bus &&
            (devices[j].feedback_id == candidate.feedback_id ||
             (devices[j].group == candidate.group && devices[j].slot == candidate.slot)))) {
            taskEXIT_CRITICAL();
            return -1;
        }
    }
    /* The new task cannot execute until this critical section exits. No
     * published state changes on failure, so registration can be retried. */
    if (device_count == 0U && xTaskCreateStatic(service_task, "motor_dji", 768U,
        NULL, 5U, task_stack, &task_control) == NULL) {
        taskEXIT_CRITICAL();
        return -1;
    }
    { const motor_dji_status_t initial = { 0 }; motor->measure = initial; }
    configs[device_count] = c;
    devices[device_count] = candidate;
    active_groups[candidate.group] = true;
    active_buses[c.bus] = true;
    ++device_count;
    motor->slot = (uint8_t)device_count;
    taskEXIT_CRITICAL();
    return 0;
}
void motor_dji_stop(const motor_dji_t *motor)
{
    size_t i;
    device_t *d = find_device(motor, &i);
    if (d == NULL) return;
    taskENTER_CRITICAL();
    stop_device(d);
    taskEXIT_CRITICAL();
}

/* Called only by motor.c while holding the task critical section. */
void motor_dji_shutdown(void)
{
    size_t i;
    for (i = 0U; i < device_count; ++i) {
        stop_device(&devices[i]);
    }
}

void motor_dji_setCurrent(const motor_dji_t *motor, int16_t value)
{
    size_t i;
    device_t *d = find_device(motor, &i);
    if (d == NULL ||
        value > current_limit(configs[i].model) || value < -current_limit(configs[i].model)) return;
    taskENTER_CRITICAL();
    expire(i, now_ms());
    if (!daemon_motorAllowed()) {
        stop_device(d);
    } else if (d->owner->measure.online) {
        d->owner->measure.enabled = true;
        d->target = value;
        d->command_ms = now_ms();
    }
    taskEXIT_CRITICAL();
}

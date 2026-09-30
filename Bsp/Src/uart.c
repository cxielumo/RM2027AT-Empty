#include "uart.h"

#include "at32f423_wk_config.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <string.h>

#define UART6_DMA_CAPACITY 128U
#define UART_FRAME_SLOT_COUNT 2U

extern volatile uint8_t usart6_rx_dma_buffer[UART6_DMA_CAPACITY];

typedef struct {
    uart_frame_t frame;
    bool reserved;
} uart_frame_slot_t;

static uart_frame_slot_t frame_slots[UART_FRAME_SLOT_COUNT];
static StaticQueue_t ready_queue_object;
static StaticQueue_t free_queue_object;
static uint8_t ready_queue_storage[UART_FRAME_SLOT_COUNT * sizeof(uint8_t)];
static uint8_t free_queue_storage[UART_FRAME_SLOT_COUNT * sizeof(uint8_t)];
static QueueHandle_t ready_queue;
static QueueHandle_t free_queue;

typedef struct {
    uart_id_t id;
    uart_callback_t callback;
    void *context;
} uart_registration_t;
static uart_registration_t uart_registration;
static TaskHandle_t uart_callback_handle;
static StaticTask_t uart_callback_control;
static StackType_t uart_callback_stack[384];
static bool uart_registered;

static void uart_callback_task(void *args)
{
    uart_frame_t frame;
    (void)args;
    for (;;) {
        if (uart_receiveFrame(UART_6, &frame, UINT32_MAX) == 0)
            uart_registration.callback(&frame, uart_registration.context);
    }
}

int uart_register(uart_id_t id, uart_callback_t callback, void *context)
{
    const uart_registration_t registration = { id, callback, context };
    const uart_registration_t *config = &registration;
    int result = -1;
    if (config->id != UART_6 || config->callback == NULL) return -1;
    taskENTER_CRITICAL();
    if (!uart_registered && ready_queue != NULL) {
        uart_registration = *config;
        uart_callback_handle = xTaskCreateStatic(uart_callback_task, "uart_rx",
            384, NULL, 5, uart_callback_stack, &uart_callback_control);
        if (uart_callback_handle != NULL) { uart_registered = true; result = 0; }
    }
    taskEXIT_CRITICAL();
    return result;
}

static uint16_t dma_read_position;
static int8_t active_slot = -1;
static bool discard_until_idle;
static uint32_t active_error_flags;

static uint32_t uart_tick_ms_from_isr(void)
{
    uint64_t ticks = (uint64_t)xTaskGetTickCountFromISR();
    return (uint32_t)((ticks * UINT64_C(1000)) /
                      (uint64_t)configTICK_RATE_HZ);
}

static int ms_to_ticks(uint32_t milliseconds, TickType_t *ticks)
{
    uint64_t converted;

    if (ticks == NULL) {
        return -1;
    }
    converted = ((uint64_t)milliseconds * (uint64_t)configTICK_RATE_HZ +
                 UINT64_C(999)) / UINT64_C(1000);
    if (converted >= (uint64_t)portMAX_DELAY) {
        return -1;
    }
    *ticks = (TickType_t)converted;
    if ((milliseconds != 0U) && (*ticks == 0U)) {
        *ticks = (TickType_t)1U;
    }
    return 0;
}

static void release_slot_from_isr(uint8_t slot,
                                  BaseType_t *higher_priority_woken)
{
    BaseType_t result;

    if (slot >= UART_FRAME_SLOT_COUNT) {
        return;
    }
    frame_slots[slot].reserved = false;
    result = xQueueSendFromISR(free_queue, &slot, higher_priority_woken);
    if (result != pdPASS) {
        /* A duplicate free token is an internal queue consistency failure. */
        panic(FAULT_ASSERT);
    }
}

static int reserve_slot_from_isr(BaseType_t *higher_priority_woken)
{
    uint8_t slot;

    if (xQueueReceiveFromISR(free_queue, &slot, higher_priority_woken) !=
        pdPASS) {
        return -1;
    }
    if ((slot >= UART_FRAME_SLOT_COUNT) || frame_slots[slot].reserved) {
        panic(FAULT_ASSERT);
        return -1;
    }
    frame_slots[slot].reserved = true;
    (void)memset(&frame_slots[slot].frame, 0,
                 sizeof(frame_slots[slot].frame));
    active_slot = (int8_t)slot;
    return 0;
}

static void append_byte_from_isr(uint8_t value, uint32_t error_flags,
                                 BaseType_t *higher_priority_woken)
{
    if (discard_until_idle) {
        active_error_flags |= error_flags;
        return;
    }
    if (active_slot < 0) {
        active_error_flags |= error_flags;
        if (reserve_slot_from_isr(higher_priority_woken)) {
            discard_until_idle = true;
            return;
        }
    }

    active_error_flags |= error_flags;
    if (frame_slots[(uint8_t)active_slot].frame.length >=
        UART_FRAME_CAPACITY) {
        frame_slots[(uint8_t)active_slot].frame.length = 0U;
        active_error_flags |= UART_ERROR_OVERFLOW;
        discard_until_idle = true;
        return;
    }
    frame_slots[(uint8_t)active_slot].frame.data[
        frame_slots[(uint8_t)active_slot].frame.length] = value;
    frame_slots[(uint8_t)active_slot].frame.length++;
}

static void publish_frame_from_isr(BaseType_t *higher_priority_woken)
{
    uint8_t slot;
    BaseType_t result;

    if ((active_slot < 0) && (active_error_flags != 0U) &&
        !discard_until_idle) {
        (void)reserve_slot_from_isr(higher_priority_woken);
    }
    if (active_slot >= 0) {
        slot = (uint8_t)active_slot;
        frame_slots[slot].frame.error_flags = active_error_flags;
        frame_slots[slot].frame.timestamp_ms = uart_tick_ms_from_isr();
        result = xQueueSendFromISR(ready_queue, &slot,
                                   higher_priority_woken);
        if (result != pdPASS) {
            release_slot_from_isr(slot, higher_priority_woken);
        } else {
            frame_slots[slot].reserved = true;
        }
    }

    active_slot = -1;
    discard_until_idle = false;
    active_error_flags = 0U;
}

static uint16_t dma_write_position(void)
{
    uint16_t remaining = (uint16_t)DMA1_CHANNEL1->dtcnt;

    if (remaining > UART6_DMA_CAPACITY) {
        remaining = UART6_DMA_CAPACITY;
    }
    return (uint16_t)((UART6_DMA_CAPACITY - remaining) &
                      (UART6_DMA_CAPACITY - 1U));
}

static void drain_dma_from_isr(BaseType_t *higher_priority_woken)
{
    uint16_t write_position = dma_write_position();

    /*
     * CNDTR exposes only a modulo-128 producer position. The configured DMA
     * half/full interrupts bound normal service latency, but if both are
     * masked long enough for an entire 128-byte lap, hardware provides no
     * remaining count with which this layer could detect the overwritten
     * bytes. USART overrun and frame-slot overflow are reported separately.
     */
    while (dma_read_position != write_position) {
        append_byte_from_isr(usart6_rx_dma_buffer[dma_read_position], 0U,
                             higher_priority_woken);
        dma_read_position = (uint16_t)((dma_read_position + 1U) &
                                       (UART6_DMA_CAPACITY - 1U));
    }
}

static uint32_t status_to_error_flags(uint32_t status)
{
    uint32_t result = 0U;

    if ((status & USART_PERR_FLAG) != 0U) {
        result |= UART_ERROR_PARITY;
    }
    if (((status & USART_FERR_FLAG) != 0U) ||
        ((status & USART_NERR_FLAG) != 0U)) {
        result |= UART_ERROR_FRAMING;
    }
    if ((status & USART_ROERR_FLAG) != 0U) {
        result |= UART_ERROR_OVERRUN;
    }
    return result;
}

void uart_init(void)
{
    uint8_t slot;
    QueueHandle_t new_ready_queue;
    QueueHandle_t new_free_queue;

    if ((ready_queue != NULL) && (free_queue != NULL)) {
        return;
    }
    if ((ready_queue != NULL) || (free_queue != NULL)) {
        panic(FAULT_INIT);
    }

    (void)memset(frame_slots, 0, sizeof(frame_slots));
    new_ready_queue = xQueueCreateStatic(UART_FRAME_SLOT_COUNT, sizeof(uint8_t),
                                         ready_queue_storage,
                                         &ready_queue_object);
    new_free_queue = xQueueCreateStatic(UART_FRAME_SLOT_COUNT, sizeof(uint8_t),
                                        free_queue_storage,
                                        &free_queue_object);
    if ((new_ready_queue == NULL) || (new_free_queue == NULL)) {
        panic(FAULT_INIT);
    }
    for (slot = 0U; slot < UART_FRAME_SLOT_COUNT; ++slot) {
        if (xQueueSend(new_free_queue, &slot, 0U) != pdPASS) {
            panic(FAULT_INIT);
        }
    }

    dma_read_position = dma_write_position();
    active_slot = -1;
    discard_until_idle = false;
    active_error_flags = 0U;
    taskENTER_CRITICAL();
    ready_queue = new_ready_queue;
    free_queue = new_free_queue;
    taskEXIT_CRITICAL();
}

size_t uart_read(uart_id_t id, uint8_t *data, size_t capacity)
{
    if ((id != UART_1) && (id != UART_6) && (id != UART_7)) {
        return 0U;
    }
    if ((capacity > 0U) && (data == NULL)) {
        return 0U;
    }
    if ((id == UART_1) || (id == UART_7)) {
        return 0U;
    }
    if (ready_queue == NULL) {
        return 0U;
    }
    return 0U;
}

int uart_receiveFrame(uart_id_t id, uart_frame_t *out,
                      uint32_t timeout_ms)
{
    if (uart_registered && xTaskGetCurrentTaskHandle() != uart_callback_handle) return -1;
    TickType_t timeout_ticks = 0U;
    uint8_t slot;

    if ((id != UART_1) && (id != UART_6) && (id != UART_7)) {
        return -1;
    }
    if (out == NULL) {
        return -1;
    }
    if ((id == UART_1) || (id == UART_7)) {
        return -1;
    }
    if (ready_queue == NULL) {
        return -1;
    }
    if (timeout_ms == 0U) {
        timeout_ticks = 0U;
    } else if (timeout_ms == UINT32_MAX) {
        timeout_ticks = portMAX_DELAY;
    } else if (ms_to_ticks(timeout_ms, &timeout_ticks)) {
        return -1;
    }
    if ((timeout_ticks != 0U) &&
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)) {
        return -1;
    }
    if (xQueueReceive(ready_queue, &slot, timeout_ticks) != pdPASS) {
        return -1;
    }
    if ((slot >= UART_FRAME_SLOT_COUNT) || !frame_slots[slot].reserved) {
        panic(FAULT_ASSERT);
        return -1;
    }

    (void)memcpy(out, &frame_slots[slot].frame, sizeof(*out));
    /* Publish the free token only after its slot metadata is ready. */
    frame_slots[slot].reserved = false;
    if (xQueueSend(free_queue, &slot, 0U) != pdPASS) {
        panic(FAULT_ASSERT);
        return -1;
    }
    return 0;
}

size_t uart_write(uart_id_t id, const uint8_t *data, size_t length)
{
    if ((id != UART_1) && (id != UART_6) && (id != UART_7)) {
        return 0U;
    }
    if ((length > 0U) && (data == NULL)) {
        return 0U;
    }
    if ((id == UART_1) || (id == UART_7)) {
        return 0U;
    }
    if (ready_queue == NULL) {
        return 0U;
    }
    return 0U;
}

void uart_irq_handler(uart_id_t id)
{
    BaseType_t higher_priority_woken = pdFALSE;
    uint32_t initial_status;
    uint32_t clear_status;
    uint32_t error_flags;
    bool idle;
    bool needs_clear;

    if (id != UART_6) {
        return;
    }

    initial_status = USART6->sts;
    idle = (initial_status & USART_IDLEF_FLAG) != 0U;
    error_flags = status_to_error_flags(initial_status);
    needs_clear = idle || (error_flags != 0U);

    if ((ready_queue == NULL) || (free_queue == NULL)) {
        if (needs_clear) {
            usart_dma_receiver_enable(USART6, FALSE);
            dma_channel_enable(DMA1_CHANNEL1, FALSE);
            clear_status = USART6->sts;
            (void)USART6->dt;
            dma_channel_enable(DMA1_CHANNEL1, TRUE);
            usart_dma_receiver_enable(USART6, TRUE);
            (void)clear_status;
        }
        return;
    }

    if (needs_clear) {
        bool pending_data;
        uint8_t pending_byte;

        /*
         * IDLE/error clearing on this AT32 USART is an STS read followed by
         * a DT read. Pause DMAR and the DMA channel, drain bytes already
         * committed by DMA, then capture a still-pending RDBF byte before
         * performing that clear sequence. This recovers the usual DMA-vs-CPU
         * final-byte race. The
         * silicon does not make the STS/DT sequence atomic with a new start
         * bit, so a byte arriving inside that tiny window cannot be proven
         * lossless; any RDBF byte observed is preserved and reported.
         */
        usart_dma_receiver_enable(USART6, FALSE);
        dma_channel_enable(DMA1_CHANNEL1, FALSE);
        drain_dma_from_isr(&higher_priority_woken);
        clear_status = USART6->sts;
        pending_data = (clear_status & USART_RDBF_FLAG) != 0U;
        pending_byte = (uint8_t)USART6->dt;
        error_flags |= status_to_error_flags(clear_status);
        if (pending_data) {
            append_byte_from_isr(pending_byte, error_flags,
                                 &higher_priority_woken);
        }
        active_error_flags |= error_flags;
        if (idle) {
            publish_frame_from_isr(&higher_priority_woken);
        }
        dma_channel_enable(DMA1_CHANNEL1, TRUE);
        usart_dma_receiver_enable(USART6, TRUE);
    } else {
        drain_dma_from_isr(&higher_priority_woken);
    }

    portYIELD_FROM_ISR(higher_priority_woken);
}

#include "can.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "at32f423_can.h"
#include "at32f423_wk_config.h"

#include <stddef.h>
#include <stdint.h>

#define CAN_RX_QUEUE_LENGTH 32U

typedef struct {
    can_type *peripheral;
    QueueHandle_t queue;
    StaticQueue_t queue_control;
    can_frame_t queue_storage[CAN_RX_QUEUE_LENGTH];
    volatile uint32_t rx_queue_dropped;
    volatile uint32_t rx_hardware_overruns;
    volatile bool error_pending;
} can_context_t;

static can_context_t can_contexts[2] = {
    { .peripheral = CAN1 },
    { .peripheral = CAN2 }
};

static can_context_t *can_context_get(can_bus_t bus)
{
    switch (bus) {
    case CAN_BUS_1:
        return &can_contexts[0];
    case CAN_BUS_2:
        return &can_contexts[1];
    default:
        return NULL;
    }
}

static uint32_t can_tick_to_ms(TickType_t tick)
{
    return (uint32_t)(((uint64_t)tick * UINT64_C(1000)) /
                      (uint64_t)configTICK_RATE_HZ);
}

/* Observe errors from task context as well as the CAN IRQ entry. */
static bool can_is_bus_off(can_context_t *context)
{
    can_type *peripheral = context->peripheral;
    bool bus_off = (can_flag_get(peripheral, CAN_BOF_FLAG) == SET);
    bool error_record = (can_flag_get(peripheral, CAN_ETR_FLAG) == SET);

    if (bus_off || error_record) {
        context->error_pending = true;
        if (error_record) {
            can_flag_clear(peripheral, CAN_ETR_FLAG);
        } else {
            can_flag_clear(peripheral, CAN_EOIF_FLAG);
        }
    }
    return bus_off;
}

void can_init(void)
{
    size_t i;

    for (i = 0U; i < (sizeof(can_contexts) / sizeof(can_contexts[0])); ++i) {
        can_context_t *context = &can_contexts[i];

        if (context->queue == NULL) {
            context->queue = xQueueCreateStatic(
                (UBaseType_t)CAN_RX_QUEUE_LENGTH,
                (UBaseType_t)sizeof(can_frame_t),
                (uint8_t *)context->queue_storage,
                &context->queue_control);
            if (context->queue == NULL) {
                panic(FAULT_INIT);
            }
        }
    }

    /* wk_can1_init()/wk_can2_init() already configure clocks, pins, bitrate,
     * and the generated FIFO0 catch-all filters before this function runs. */
    can_interrupt_enable(CAN1,
                         CAN_RF0MIEN_INT | CAN_RF0FIEN_INT | CAN_RF0OIEN_INT |
                             CAN_ETRIEN_INT,
                         TRUE);
    can_interrupt_enable(CAN2,
                         CAN_RF0MIEN_INT | CAN_RF0FIEN_INT | CAN_RF0OIEN_INT |
                             CAN_ETRIEN_INT,
                         TRUE);
}

void can_send(can_bus_t bus, const can_frame_t *frame)
{
    can_context_t *context = can_context_get(bus);
    can_tx_message_type message = { 0 };
    uint8_t mailbox;
    bool bus_off;

    if ((context == NULL) || (frame == NULL) || (frame->dlc > 8U) ||
        ((frame->extended && (frame->id > UINT32_C(0x1FFFFFFF))) ||
         (!frame->extended && (frame->id > UINT32_C(0x7FF))))) {
        return;
    }
    if (context->queue == NULL) {
        return;
    }

    bus_off = can_is_bus_off(context);
    if (bus_off || context->error_pending) {
        return;
    }

    message.id_type = frame->extended ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
    message.frame_type = frame->remote ? CAN_TFT_REMOTE : CAN_TFT_DATA;
    message.dlc = frame->dlc;
    if (frame->extended) {
        message.extended_id = frame->id;
    } else {
        message.standard_id = frame->id;
    }
    if (!frame->remote) {
        size_t i;
        for (i = 0U; i < (size_t)frame->dlc; ++i) {
            message.data[i] = frame->data[i];
        }
    }

    mailbox = can_message_transmit(context->peripheral, &message);
    if (mailbox == CAN_TX_STATUS_NO_EMPTY) {
        return;
    }
    if (mailbox > CAN_TX_MAILBOX2) {
        return;
    }
}

int can_receive(can_bus_t bus, can_frame_t *out)
{
    can_context_t *context = can_context_get(bus);
    BaseType_t received;

    if ((context == NULL) || (out == NULL)) {
        return -1;
    }
    if (context->queue == NULL) {
        return -1;
    }

    (void)can_is_bus_off(context);
    if (context->error_pending) {
        taskENTER_CRITICAL();
        context->error_pending = false;
        taskEXIT_CRITICAL();
        return -1;
    }

    received = xQueueReceive(context->queue, out, (TickType_t)0U);
    return (received == pdPASS) ? 0 : -1;
}

void can_irq_handler(can_bus_t bus)
{
    can_context_t *context = can_context_get(bus);
    can_type *peripheral;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (context == NULL) {
        return;
    }
    peripheral = context->peripheral;

    if (can_flag_get(peripheral, CAN_RF0OF_FLAG) == SET) {
        context->rx_hardware_overruns++;
        can_flag_clear(peripheral, CAN_RF0OF_FLAG);
        context->error_pending = true;
    }
    (void)can_is_bus_off(context);

    /* The SDK's receive helper also releases FIFO0 after copying the frame. */
    while (can_receive_message_pending_get(peripheral, CAN_RX_FIFO0) != 0U) {
        can_rx_message_type rx_message = { 0 };
        can_frame_t frame = { 0 };

        can_message_receive(peripheral, CAN_RX_FIFO0, &rx_message);
        if (rx_message.dlc > 8U) {
            context->error_pending = true;
            continue;
        }

        frame.id = (rx_message.id_type == CAN_ID_EXTENDED)
                       ? rx_message.extended_id
                       : rx_message.standard_id;
        frame.extended = (rx_message.id_type == CAN_ID_EXTENDED);
        frame.remote = (rx_message.frame_type == CAN_TFT_REMOTE);
        frame.dlc = rx_message.dlc;
        if (!frame.remote) {
            size_t i;
            for (i = 0U; i < (size_t)frame.dlc; ++i) {
                frame.data[i] = rx_message.data[i];
            }
        }
        frame.timestamp_ms = can_tick_to_ms(xTaskGetTickCountFromISR());

        if ((context->queue != NULL) &&
            (xQueueSendFromISR(context->queue, &frame,
                               &higher_priority_task_woken) != pdPASS)) {
            context->rx_queue_dropped++;
        }
    }

    /* FIFO full is a level flag; clear it after draining pending frames. */
    if (can_flag_get(peripheral, CAN_RF0FF_FLAG) == SET) {
        can_flag_clear(peripheral, CAN_RF0FF_FLAG);
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

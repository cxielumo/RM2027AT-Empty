#include "motor.h"

#include "motor_tt.h"
#include "FreeRTOS.h"
#include "task.h"

/* Internal component composition entry; not an application API. */
void motor_dji_shutdown(void);

void motor_shutdown(void)
{
    unsigned int i;

    /* Prevent a control task from updating targets halfway through shutdown. */
    taskENTER_CRITICAL();
    for (i = 0U; i < (unsigned int)LAST_MOTOR_TT; ++i) {
        motor_tt_disable((motor_tt_id_t)i);
    }
    motor_dji_shutdown();
    taskEXIT_CRITICAL();
}

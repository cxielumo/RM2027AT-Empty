#include "bsp.h"

#include "at32f423_wk_config.h"
#include "adc.h"
#include "can.h"
#include "led.h"
#include "pwm.h"
#include "uart.h"

/* Driver initialization is private to BSP composition. */
void adc_init(void);
void can_init(void);
void led_init(void);
void pwm_init(void);
void uart_init(void);

void bsp_init(void)
{
    pwm_init();
    led_init();
    can_init();
    uart_init();
    adc_init();
}

void panic(fault_reason_t reason)
{
    (void)reason;

    /* Freeze task and interrupt activity before forcing outputs safe, so no
     * later software path can restore a PWM compare mode after this point. */
    __disable_irq();
    pwm_shutdown();

    /* GPIO CLR writes the output latch directly and is safe without driver or
     * scheduler state. The board's three LEDs are active high on PC13..PC15. */
    GPIOC->clr = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;

    for (;;) {
    }
}

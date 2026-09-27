#include "pwm.h"

#include "at32f423.h"
#include "at32f423_crm.h"
#include "at32f423_gpio.h"
#include "at32f423_tmr.h"

#include <math.h>
#include <stddef.h>

#define PWM_SERVO_HZ       50UL
#define PWM_MOTOR_HZ       20000UL
#define PWM_SERVO_COUNTS   20000UL
#define PWM_MOTOR_COUNTS   7500UL
#define PWM_CHANNEL_COUNT  ((unsigned int)LAST_PWM_CHANNEL)

typedef enum {
  PWM_GPIO_PORT_A,
  PWM_GPIO_PORT_B,
  PWM_GPIO_PORT_C,
  PWM_GPIO_PORT_F,
  LAST_PWM_GPIO_PORT
} pwm_gpio_port_t;

typedef struct {
    pwm_gpio_port_t port;
    uint8_t pin;
} pwm_pin_desc_t;

typedef struct {
    pwm_pin_desc_t pin;
    pwm_group_t group;
    uint8_t timer_channel;
    uint8_t alternate_function;
    uint32_t target_hz;
} pwm_channel_desc_t;

#define PWM_DESC(port_, pin_, group_, channel_, mux_, frequency_) \
    { { (port_), (pin_) }, (group_), (channel_), (mux_), (frequency_) }

static const pwm_channel_desc_t s_channels[PWM_CHANNEL_COUNT] = {
    [PWM_MOTOR_1_IN1] = PWM_DESC(PWM_GPIO_PORT_B, 8U, PWM_GROUP_TMR4, 3U, 2U, 20000U),
    [PWM_MOTOR_1_IN2] = PWM_DESC(PWM_GPIO_PORT_B, 9U, PWM_GROUP_TMR4, 4U, 2U, 20000U),
    [PWM_MOTOR_2_IN1] = PWM_DESC(PWM_GPIO_PORT_B, 7U, PWM_GROUP_TMR4, 2U, 2U, 20000U),
    [PWM_MOTOR_2_IN2] = PWM_DESC(PWM_GPIO_PORT_B, 6U, PWM_GROUP_TMR4, 1U, 2U, 20000U),
    [PWM_MOTOR_3_IN1] = PWM_DESC(PWM_GPIO_PORT_B, 5U, PWM_GROUP_TMR3, 2U, 2U, 50U),
    [PWM_MOTOR_3_IN2] = PWM_DESC(PWM_GPIO_PORT_B, 4U, PWM_GROUP_TMR3, 1U, 2U, 50U),
    [PWM_MOTOR_4_IN1] = PWM_DESC(PWM_GPIO_PORT_A, 10U, PWM_GROUP_TMR1, 3U, 1U, 20000U),
    [PWM_MOTOR_4_IN2] = PWM_DESC(PWM_GPIO_PORT_A, 9U, PWM_GROUP_TMR1, 2U, 1U, 20000U),
    [PWM_MOTOR_5_IN1] = PWM_DESC(PWM_GPIO_PORT_B, 15U, PWM_GROUP_TMR12, 2U, 9U, 20000U),
    [PWM_MOTOR_5_IN2] = PWM_DESC(PWM_GPIO_PORT_B, 14U, PWM_GROUP_TMR12, 1U, 9U, 20000U),
    [PWM_MOTOR_6_IN1] = PWM_DESC(PWM_GPIO_PORT_B, 10U, PWM_GROUP_TMR2, 3U, 1U, 50U),
    [PWM_MOTOR_6_IN2] = PWM_DESC(PWM_GPIO_PORT_B, 2U, PWM_GROUP_TMR2, 4U, 1U, 50U),
    [PWM_SERVO_1] = PWM_DESC(PWM_GPIO_PORT_B, 1U, PWM_GROUP_TMR3, 4U, 2U, 50U),
    [PWM_SERVO_2] = PWM_DESC(PWM_GPIO_PORT_B, 0U, PWM_GROUP_TMR3, 3U, 2U, 50U),
    [PWM_SERVO_3] = PWM_DESC(PWM_GPIO_PORT_A, 7U, PWM_GROUP_TMR14, 1U, 9U, 50U),
    [PWM_SERVO_4] = PWM_DESC(PWM_GPIO_PORT_A, 6U, PWM_GROUP_TMR13, 1U, 9U, 50U),
    [PWM_SERVO_5] = PWM_DESC(PWM_GPIO_PORT_A, 1U, PWM_GROUP_TMR2, 2U, 1U, 50U),
    [PWM_SERVO_6] = PWM_DESC(PWM_GPIO_PORT_A, 0U, PWM_GROUP_TMR2, 1U, 1U, 50U)
};

#undef PWM_DESC

static int pwm_get_desc(pwm_channel_t channel, pwm_channel_desc_t *out)
{
    if ((out == NULL) ||
        ((unsigned int)channel >= PWM_CHANNEL_COUNT)) {
        return -1;
    }
    *out = s_channels[(unsigned int)channel];
    return 0;
}

typedef struct {
    pwm_group_t group;
    tmr_type *timer;
    uint32_t clock_hz;
    uint32_t period_counts;
    uint8_t apb_bus;
} pwm_timer_t;

static pwm_timer_t s_timers[] = {
    { PWM_GROUP_TMR1, TMR1, 0UL, 0UL, 2U },
    { PWM_GROUP_TMR2, TMR2, 0UL, 0UL, 1U },
    { PWM_GROUP_TMR3, TMR3, 0UL, 0UL, 1U },
    { PWM_GROUP_TMR4, TMR4, 0UL, 0UL, 1U },
    { PWM_GROUP_TMR12, TMR12, 0UL, 0UL, 1U },
    { PWM_GROUP_TMR13, TMR13, 0UL, 0UL, 1U },
    { PWM_GROUP_TMR14, TMR14, 0UL, 0UL, 1U }
};

static tmr_channel_select_type pwm_channel_select(uint8_t channel)
{
    switch (channel) {
    case 1U: return TMR_SELECT_CHANNEL_1;
    case 2U: return TMR_SELECT_CHANNEL_2;
    case 3U: return TMR_SELECT_CHANNEL_3;
    default: return TMR_SELECT_CHANNEL_4;
    }
}

static tmr_type *pwm_timer_for_group(pwm_group_t group)
{
    switch (group) {
    case PWM_GROUP_TMR1:  return TMR1;
    case PWM_GROUP_TMR2:  return TMR2;
    case PWM_GROUP_TMR3:  return TMR3;
    case PWM_GROUP_TMR4:  return TMR4;
    case PWM_GROUP_TMR12: return TMR12;
    case PWM_GROUP_TMR13: return TMR13;
    case PWM_GROUP_TMR14: return TMR14;
    default: return (tmr_type *)0;
    }
}

static pwm_timer_t *pwm_timer_state_for_group(pwm_group_t group)
{
    uint8_t i;
    for (i = 0U; i < (uint8_t)(sizeof(s_timers) / sizeof(s_timers[0])); ++i) {
        if (s_timers[i].group == group) {
            return &s_timers[i];
        }
    }
    return (pwm_timer_t *)0;
}

static void pwm_force_low(tmr_type *timer, uint8_t channel)
{
    tmr_channel_select_type select = pwm_channel_select(channel);
    tmr_channel_value_set(timer, select, 0UL);
    tmr_output_channel_mode_select(timer, select, TMR_OUTPUT_CONTROL_FORCE_LOW);
}

static uint32_t pwm_timer_clock(const pwm_timer_t *timer,
                               const crm_clocks_freq_type *clocks)
{
    uint32_t pclk = (timer->apb_bus == 2U) ? clocks->apb2_freq : clocks->apb1_freq;

    if (pclk == 0UL || clocks->ahb_freq == 0UL || clocks->ahb_freq < pclk ||
        (clocks->ahb_freq % pclk) != 0UL) {
        return 0UL;
    }
    if (clocks->ahb_freq == pclk) {
        return pclk;
    }
    /* AT32F423 timers receive PCLK when APB is undivided, otherwise 2*PCLK. */
    if ((uint64_t)pclk * 2ULL <= UINT32_MAX) {
        return pclk * 2UL;
    }
    return 0UL;
}

static int pwm_clock_plan(pwm_timer_t *timer, uint32_t hz,
                           uint32_t *prescaler, uint32_t *period)
{
    uint32_t counts = (hz == PWM_SERVO_HZ) ? PWM_SERVO_COUNTS : PWM_MOTOR_COUNTS;
    uint32_t counter_hz = hz * counts;

    if (timer->clock_hz == 0UL || counter_hz == 0UL ||
        (timer->clock_hz % counter_hz) != 0UL) {
        return -1;
    }
    *prescaler = timer->clock_hz / counter_hz;
    if (*prescaler == 0UL || *prescaler > 65536UL || counts > 65536UL) {
        return -1;
    }
    --(*prescaler);
    *period = counts - 1UL;
    return 0;
}

static int pwm_group_frequency(pwm_group_t group, uint32_t *hz)
{
    uint8_t i;
    bool found = false;
    uint32_t frequency = 0UL;

    for (i = 0U; i < PWM_CHANNEL_COUNT; ++i) {
        pwm_channel_desc_t desc;
        if (pwm_get_desc((pwm_channel_t)i, &desc)) {
            return -1;
        }
        if (desc.group == group) {
            if (!found) {
                frequency = desc.target_hz;
                found = true;
            } else if (frequency != desc.target_hz) {
                return -1;
            }
        }
    }
    if (!found || (frequency != PWM_SERVO_HZ && frequency != PWM_MOTOR_HZ)) {
        return -1;
    }
    *hz = frequency;
    return 0;
}

static gpio_type *pwm_gpio_for_port(pwm_gpio_port_t port)
{
    switch (port) {
    case PWM_GPIO_PORT_A: return GPIOA;
    case PWM_GPIO_PORT_B: return GPIOB;
    case PWM_GPIO_PORT_C: return GPIOC;
    case PWM_GPIO_PORT_F: return GPIOF;
    default: return (gpio_type *)0;
    }
}

static void pwm_configure_pin(const pwm_channel_desc_t *desc)
{
    gpio_type *gpio = pwm_gpio_for_port(desc->pin.port);
    gpio_init_type config;
    uint32_t pin_mask = 1UL << desc->pin.pin;

    gpio_default_para_init(&config);
    gpio_pin_mux_config(gpio, (gpio_pins_source_type)desc->pin.pin,
                        (gpio_mux_sel_type)desc->alternate_function);
    config.gpio_pins = pin_mask;
    config.gpio_mode = GPIO_MODE_MUX;
    config.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    config.gpio_pull = GPIO_PULL_NONE;
    config.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(gpio, &config);
}

static void pwm_enable_gpio_clocks(void)
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
}

static void pwm_enable_timer_clocks(void)
{
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR12_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR13_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR14_PERIPH_CLOCK, TRUE);
}

static void pwm_configure_timer(pwm_timer_t *timer, uint32_t prescaler,
                                uint32_t period)
{
    uint8_t i;

    tmr_counter_enable(timer->timer, FALSE);
    tmr_cnt_dir_set(timer->timer, TMR_COUNT_UP);
    tmr_clock_source_div_set(timer->timer, TMR_CLOCK_DIV1);
    tmr_base_init(timer->timer, period, prescaler);
    tmr_period_buffer_enable(timer->timer, TRUE);
    tmr_primary_mode_select(timer->timer, TMR_PRIMARY_SEL_RESET);

    for (i = 0U; i < PWM_CHANNEL_COUNT; ++i) {
        pwm_channel_desc_t desc;
        tmr_output_config_type output;
        if (pwm_get_desc((pwm_channel_t)i, &desc)) {
            panic(FAULT_INIT);
        }
        if (pwm_timer_for_group(desc.group) != timer->timer) {
            continue;
        }
        tmr_output_default_para_init(&output);
        output.oc_mode = TMR_OUTPUT_CONTROL_FORCE_LOW;
        output.oc_output_state = TRUE;
        output.occ_output_state = FALSE;
        output.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        output.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
        output.oc_idle_state = FALSE;
        output.occ_idle_state = FALSE;
        tmr_output_channel_config(timer->timer,
                                  pwm_channel_select(desc.timer_channel), &output);
        tmr_channel_value_set(timer->timer,
                              pwm_channel_select(desc.timer_channel), 0UL);
        tmr_output_channel_buffer_enable(timer->timer,
                                         pwm_channel_select(desc.timer_channel), TRUE);
        tmr_channel_enable(timer->timer,
                           pwm_channel_select(desc.timer_channel), TRUE);
    }

    /* Advanced-output gate exists on these timer instances (not TMR2/3/4). */
    if (timer->timer == TMR1 || timer->timer == TMR12 ||
        timer->timer == TMR13 || timer->timer == TMR14) {
        tmr_output_enable(timer->timer, TRUE);
    }
    tmr_event_sw_trigger(timer->timer, TMR_OVERFLOW_SWTRIG);
    tmr_counter_enable(timer->timer, TRUE);
}

static uint32_t pwm_compare(const pwm_channel_desc_t *desc, float duty)
{
    pwm_timer_t *timer = pwm_timer_state_for_group(desc->group);
    if (timer == (pwm_timer_t *)0) {
        return 0UL;
    }
    return (uint32_t)((double)duty * (double)timer->period_counts + 0.5);
}

static bool pwm_is_valid_duty(float duty)
{
    return isfinite(duty) && duty >= 0.0f && duty <= 1.0f;
}

static uint32_t pwm_irq_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void pwm_irq_unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

void pwm_init(void)
{
    crm_clocks_freq_type clocks;
    uint32_t prescalers[7];
    uint32_t periods[7];
    uint8_t i;

    crm_clocks_freq_get(&clocks);
    for (i = 0U; i < (uint8_t)(sizeof(s_timers) / sizeof(s_timers[0])); ++i) {
        uint32_t hz;
        if (pwm_group_frequency(s_timers[i].group, &hz)) {
            panic(FAULT_INIT);
        }
        s_timers[i].clock_hz = pwm_timer_clock(&s_timers[i], &clocks);
        if (pwm_clock_plan(&s_timers[i], hz, &prescalers[i], &periods[i])) {
            panic(FAULT_INIT);
        }
        s_timers[i].period_counts = periods[i] + 1UL;
    }

    pwm_enable_gpio_clocks();
    pwm_enable_timer_clocks();
    for (i = 0U; i < PWM_CHANNEL_COUNT; ++i) {
        pwm_channel_desc_t desc;
        if (pwm_get_desc((pwm_channel_t)i, &desc)) {
            panic(FAULT_INIT);
        }
        pwm_configure_pin(&desc);
    }
    for (i = 0U; i < (uint8_t)(sizeof(s_timers) / sizeof(s_timers[0])); ++i) {
        pwm_configure_timer(&s_timers[i], prescalers[i], periods[i]);
    }
}

void pwm_setDuty(pwm_channel_t channel, float duty)
{
    pwm_channel_desc_t desc;
    tmr_type *timer;
    pwm_timer_t *timer_state;
    uint32_t primask;
    if ((unsigned int)channel >= PWM_CHANNEL_COUNT ||
        !pwm_is_valid_duty(duty)) {
        return;
    }
    if (pwm_get_desc(channel, &desc)) {
        panic(FAULT_ASSERT);
    }
    timer = pwm_timer_for_group(desc.group);
    timer_state = pwm_timer_state_for_group(desc.group);
    if ((timer == (tmr_type *)0) || (timer_state == (pwm_timer_t *)0) ||
        (timer_state->period_counts == 0UL)) {
        return;
    }
    primask = pwm_irq_lock();
    tmr_channel_value_set(timer, pwm_channel_select(desc.timer_channel),
                          pwm_compare(&desc, duty));
    tmr_output_channel_mode_select(timer,
                                   pwm_channel_select(desc.timer_channel),
                                   TMR_OUTPUT_CONTROL_PWM_MODE_A);
    pwm_irq_unlock(primask);
}

void pwm_disable(pwm_channel_t channel)
{
    pwm_channel_desc_t desc;
    tmr_type *timer;
    pwm_timer_t *timer_state;
    uint32_t primask;
    if ((unsigned int)channel >= PWM_CHANNEL_COUNT) {
        return;
    }
    if (pwm_get_desc(channel, &desc)) {
        panic(FAULT_ASSERT);
    }
    timer = pwm_timer_for_group(desc.group);
    timer_state = pwm_timer_state_for_group(desc.group);
    if ((timer == (tmr_type *)0) || (timer_state == (pwm_timer_t *)0) ||
        (timer_state->period_counts == 0UL)) {
        return;
    }
    primask = pwm_irq_lock();
    pwm_force_low(timer, desc.timer_channel);
    pwm_irq_unlock(primask);
}

void pwm_disableAll(void)
{
    uint8_t i;
    for (i = 0U; i < PWM_CHANNEL_COUNT; ++i) {
        pwm_channel_desc_t desc;
        tmr_type *timer;
        if (pwm_get_desc((pwm_channel_t)i, &desc)) {
            continue;
        }
        timer = pwm_timer_for_group(desc.group);
        if (timer != (tmr_type *)0) {
            pwm_force_low(timer, desc.timer_channel);
        }
    }
}

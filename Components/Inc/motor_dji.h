#ifndef RM2027AT_MOTOR_DJI_H
#define RM2027AT_MOTOR_DJI_H

#include "can.h"

#define MOTOR_DJI_MAX_DEVICES 16U
typedef enum {
    MOTOR_DJI_M2006 = 1, MOTOR_DJI_M3508, MOTOR_DJI_GM6020
} motor_dji_model_t;

typedef struct {
    uint8_t id; /* Physical ESC ID: C610/C620 1..8, GM6020 1..7. */
    can_bus_t bus;
    motor_dji_model_t model;
    bool invert; /* Reverse command and signed feedback; default false. */
} motor_dji_config_t;

typedef struct {
    uint16_t encoder_raw; /* Physical encoder value, unaffected by invert. */
    int16_t speed_rpm;
    int16_t current_raw;
    uint8_t temperature_c;
    bool temperature_valid;
    int64_t angle_counts;
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool valid, online, enabled, angle_continuous;
} motor_dji_status_t;

typedef struct {
    volatile motor_dji_status_t measure; /* Updated by driver; application reads only. */
    uint8_t slot; /* Driver-owned token; zero is invalid. */
} motor_dji_t;

/* Initialized by main. Other APIs are task-context only, or may be called
 * after init before starting the scheduler. The service owns used RX buses. */
/* Copies config; returns 0 on success, -1 for invalid/conflicting/full config,
 * uninitialized driver or task creation failure. No unregister operation.
 * Writes the object only on success; failure leaves it unchanged.
 * Object address must remain valid for the driver lifetime; do not copy or move it.
 * Direct measure reads are live; protect multi-field and 64-bit reads with
 * a task critical section when a consistent snapshot is required.
 * First registration starts the service; motors remain disabled. */
int motor_dji_register(motor_dji_t *motor, const motor_dji_config_t *config);
void motor_dji_stop(const motor_dji_t *motor);
/* M2006/C610: +/-10000; M3508/C620 and GM6020 current mode: +/-16384. */
/* Online feedback is required. Accepting a command activates output;
 * stop/timeouts clear it. Recovery requires a new command. */
void motor_dji_setCurrent(const motor_dji_t *motor, int16_t current);

#endif

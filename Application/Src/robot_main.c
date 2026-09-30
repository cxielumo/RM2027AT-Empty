#include "os.h"
#include "motor_dji.h"
#include "pid.h"

motor_dji_t motor;
pid_t speed_pid;
int target_rpm = 500;
int current;

void robot_main(void *args)
{
    const motor_dji_config_t motor_config = {
        .id = 1,
        .bus = CAN_BUS_1,
        .model = MOTOR_DJI_M3508,
        .invert = false
    };
    const pid_config_t pid_config = {
        .kp = 2.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .output_min = -500.0f,
        .output_max = 500.0f,
        .integral_min = -250.0f,
        .integral_max = 250.0f
    };
    float output;

    (void)args;
    motor_dji_register(&motor, &motor_config);
    pid_init(&speed_pid, &pid_config);

    for (;;) {
        pid_step(&speed_pid, (float)target_rpm, (float)motor.measure.speed_rpm, 0.01f, &output);
        motor_dji_setCurrent(&motor, (int16_t)output);
        os_delay(10);
    }
}

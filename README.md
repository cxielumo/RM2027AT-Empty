# RM2027AT

RM2027AT 是面向 AT32F423 的机器人控制框架.

## 框架结构

```text
Application → Components → Bsp → ThirdParty + ATWP生成
       └──────────────→ Algorithm（独立）
```

- **Application**：默认用户入口为 `void robot_main(void *args)`，位于 `Application/Src/robot_main.c`。在此实现机器人业务；启动装配在 `project/src/main.c`。
- **Components**：遥控器 DBUS、TT 直流电机及舵机，位于 `Components/Inc`、`Components/Src`；初始化由 `main()` 直接装配。
- **Bsp**：板级映射、PWM、CAN、UART、ADC、LED、OS 线程接口，位于 `Bsp/Inc`、`Bsp/Src`。
- **Algorithm**：PID、斜坡、数学工具和环形缓冲，不依赖硬件或上层模块，位于 `Algorithm/Inc`、`Algorithm/Src`。
- **ThirdParty + ATWP生成**：FreeRTOS、AT32 外设库、生成的启动/外设代码和中断接入，位于 `ThirdParty`、`libraries`、`project`。

自维护的应用、组件、BSP 与算法代码按 C99 兼容方式编写。工程默认 C 标准配置保持原样。

## 板载模块

- 六路 TT 电机：`MOTOR_TT_1` 至 `MOTOR_TT_6`。
- 六路舵机：`SERVO_1` 至 `SERVO_6`，使用定时器 PWM。
- 双 CAN 总线、USART6 DBUS 接收、三路编号 LED（LED_1..3，板上均为红灯）及 ADC。

## 开始使用

1. 在 `Application/Src/robot_main.c` 编写用户业务入口。可通过 `dbus_snapshot()` 获取遥控器快照，并使用 `os_delay()` 安排任务执行。
2. 查看 [API 设计说明](docs/API设计说明.md) 了解接口、错误处理、资源和调用约束。
3. 查看 [框架设计方案](docs/框架设计方案.md) 及 [功能引脚对应](docs/功能引脚对应.md) 了解分层和板级映射。

## 构建入口

FreeRTOS V11.3.1 源码已作为普通目录提交，直接下载 GitHub ZIP 即包含内核，无需初始化子模块。

- **CMake / GCC**：根目录提供 `CMakePresets.json`，可用 `cmake --preset Debug` 配置、`cmake --build --preset Debug` 构建；Release 对应 `--preset Release`。工具链定义在 `cmake/gcc-arm-none-eabi.cmake`。
- **Keil**：使用 `project/MDK_V5/RM2027AT.uvprojx` 打开工程。

# Algorithm / Bsp / Components API 设计说明

日期：2026-09-27。状态：设计说明与当前源码并存。Algorithm 的 PID、ramp、ringBuffer 和 math_utils 已存在；Bsp 当前包含 PWM、CAN、UART、LED、OS、ADC 等实现；Components 已实现 DBUS，motor_tt 与 servo 接口存在但默认不可启用，motor_dji 已实现三种型号协议，通过配置结构体注册。本文后续各节保留完整目标 API 设计，不能据此推断每个设计能力都已在当前代码中实现。

依据：[框架设计方案](框架设计方案.md)、[功能引脚对应](功能引脚对应.md)。当前可见头文件与源文件数为 Algorithm 7 个、Bsp 15 个、Components 10 个，共 32 个；I2C 仅保留在文档设计中，暂不创建文件或实现。每节定义头文件的公共 API 与源文件的实现责任；math_utils 仅保留头文件，不增加框架模块。代码按 C99 编写，不修改工程默认 C 标准。本文仅核对文件与接口状态，未据此声称已构建或测试。

## 当前实现限制

- PID、ramp、ringBuffer、math_utils 已有源文件或头文件；Bsp 的 PWM、CAN、UART、LED、OS、ADC 模块及 Components 的 DBUS 已有实现。文件存在不表示硬件行为已在板上验证。
- `motor_tt` 当前识别六路映射，但因 RZ7889 驱动真值表、换向安全规则及逐电机标定尚未确认，`enable` 和占空比控制遇到未配置状态时直接忽略；保持默认关闭。
- `servo` 的六路接口存在，但脉宽、角度和初始位置标定尚未配置，默认不允许启用或输出。
- `motor_dji` 当前通过 motor_dji_register 复制配置结构体注册电机；已实现 M2006/C610、M3508/C620 和 GM6020 电流控制协议，默认未注册实际设备，不会启动设备处理任务或发送控制帧。
- I2C 明确暂不实现，也不创建 `i2c.h` / `i2c.c` 文件。此处描述的是当前源码状态；其余章节中超出已实现范围的函数签名与行为仍属设计约定，不能视为已有实现。

## 1. 公共约定

- 自维护代码使用 C99；不修改工程默认 C 标准。头文件自行包含所需的 `stdint.h`、`stddef.h`、`stdbool.h`，有唯一 include guard。
- Algorithm 只依赖 C99 标准库，实例及存储由调用者提供。Bsp、Components 的长期资源、任务栈、任务控制块、队列和同步对象全部静态；只有 Application 可通过os创建动态线程，或直接使用FreeRTOS动态队列/同步对象及堆内存接口。
- Bsp、Components 不使用统一状态码：不可恢复的配置、资源或内部错误调用 `panic()`；可恢复的无效参数、未就绪、忙或超时由操作接口忽略，有结果的接口以int返回：成功为0，失败为-1；只有is/has谓词返回bool；math_utils函数直接返回float计算值，其他有结果的算法接口使用int，成功返回0、失败返回-1；数据量接口返回数量。
- 除明确说明，指针不得为空，输出指针指向调用者可写内存；失败不修改输出参数、不改变原有效配置和目标。输入缓冲在函数返回后可复用；异步接口必须先复制数据。
- 除 `*_irq_handler`、`*_from_isr`、`panic` 及特别标注的接口，其余接口只从任务调用；初始化接口允许在调度器启动前调用。Algorithm 可在 ISR 调用，但计算预算、实例同步由调用者负责。
- 运行期驱动/组件 API 在初始化前忽略请求，有结果的接口以int返回：成功为0，失败为-1；只有is/has谓词返回bool。初始化成功后的重复调用直接返回，不重建对象、不清除目标；不可恢复的初始化错误直接调用 `panic()`。
- 超时单位 ms；`0` 表示立即返回；`OS_WAIT_FOREVER` 表示无限等待，仅适用于明确允许等待的 OS/接收接口。硬件事务只接受有限超时。总超时包含等锁与执行时间，不能逐步重置。
- `uint32_t` 毫秒时间戳可回绕；比较采用无符号差值，有效超时间隔小于 `2^31 ms`。时间戳是接收/采样时刻，不能用查询时刻替代。
- 浮点参数必须有限。返回int的算法接口遇到非法输入返回-1；is/has谓词返回bool；math_utils要求调用者满足有限输入及各函数前置条件，设备接口忽略非法输入。DBUS 初始化配置错误及不可恢复的浮点计算异常调用 `panic(FAULT_ASSERT)`。范围外的有效指令默认拒绝，只有明确标注的接口会限幅。
- 设备控制默认关闭。`enable` 不恢复旧指令。带共享状态的查询复制一致快照；不返回内部可变指针。不用 volatile 代替同步。
- 本文的函数原型为公共接口全集；源文件私有辅助函数使用 `static`，不作为跨模块 API。

## 2. Algorithm

### 2.1 ringBuffer.h / ringBuffer.c

**文件：** `Algorithm/Inc/ringBuffer.h` 声明字节环形缓冲；`Algorithm/Src/ringBuffer.c` 实现索引、复制、容量和溢出统计。

```c
typedef struct {
    uint8_t *storage;
    size_t capacity;
    size_t read_pos;
    size_t write_pos;
    size_t used;
    uint32_t dropped_bytes;
} ringBuffer_t;

int ringBuffer_init(ringBuffer_t *rb, uint8_t *storage, size_t capacity);
void ringBuffer_reset(ringBuffer_t *rb);
size_t ringBuffer_write(ringBuffer_t *rb, const uint8_t *src, size_t length);
size_t ringBuffer_read(ringBuffer_t *rb, uint8_t *dst, size_t capacity);
size_t ringBuffer_size(const ringBuffer_t *rb);
size_t ringBuffer_free(const ringBuffer_t *rb);
uint32_t ringBuffer_dropped(const ringBuffer_t *rb);
```

- `init`：容量必须大于 0，全部 capacity 字节可用；storage 在整个实例生命周期有效，初始化不分配内存。
- `write/read`：返回实际写入/读出字节数；允许部分完成。长度为 0 时允许数据指针为空。满时丢弃新数据，不覆盖旧数据；丢弃字节计入饱和到 UINT32_MAX 的计数。
- `reset`：清空内容及统计，不擦除存储、不释放内存；必须在收发双方停用或共同受保护时调用。
- 单生产者/单消费者使用模型，**不是无锁实现**。`used` 和索引都是共享状态，读、写及查询均由外层共同临界区保护；大数据须分段复制，控制 ISR 延迟。
- 源文件不引用 os、中断或芯片头文件；不持有静态实例。Bsp/Components 静态提供对象和存储。

### 2.2 math_utils.h

**文件：** `Algorithm/Inc/math_utils.h` 定义无状态的static inline函数及转换宏；不再需要 `Algorithm/Src/math_utils.c`。

```c
static inline float clampf(float x, float lower, float upper);
static inline float deadzone(float x, float width);

#define DEG2RAD(deg) ((deg) * 0.01745329251994329577f)
#define RAD2DEG(rad) ((rad) * 57.295779513082320876f)
```

| 接口 | 精确语义 |
|---|---|
| clampf | 前置条件 lower ≤ upper；返回闭区间内的值 |
| deadzone | 前置条件 width ≥ 0；abs(x) ≤ width 返回 0，其他值原样返回、不重缩放 |
| DEG2RAD(deg) | 度转弧度，表达式直接返回转换值 |
| RAD2DEG(rad) | 弧度转度，表达式直接返回转换值 |

上述两个函数直接返回float计算结果，不使用状态返回值或out参数。调用者负责保证输入为有限值、`lower <= upper` 且 `width >= 0`；这些前置条件不满足时结果未定义。函数不检查或构造NaN作为错误标记；需要终止运行的上层组件应在调用前校验输入并调用 `panic(FAULT_ASSERT)`。

`DEG2RAD`和`RAD2DEG`在math_utils.h中定义，不再提供对应转换函数；宏参数及整个表达式加括号，参数只求值一次，使用浮点系数避免整数除法，不依赖非标准M_PI。宏不返回状态、不检查有限性，调用者负责输入与结果有效性；系数采用float精度。模块无全局可变状态，可并发调用。

### 2.3 pid.h / pid.c

**文件：** `Algorithm/Inc/pid.h` 定义参数/状态；`Algorithm/Src/pid.c` 实现带积分限制及条件积分抗饱和的位置式 PID。

```c
typedef struct {
    float kp, ki, kd;
    float output_min, output_max;
    float integral_min, integral_max;
} pid_config_t;

typedef struct {
    pid_config_t config;
    float integral;
    float previous_measurement;
    bool has_previous;
    bool initialized;
} pid_t;

int pid_init(pid_t *pid, const pid_config_t *config);
int pid_reset(pid_t *pid, float integral);
int pid_step(pid_t *pid, float target, float measurement,
              float dt_s, float *output);
```

- 配置必须有限，min ≤ max；积分范围须包含 0。`init` 复制配置并清零状态；运行期改参数通过再次 init，明确重置历史。
- integral 保存积分项的**输出贡献**；候选值为 integral + ki × error × dt_s，随后限制到积分范围。reset 参数按该单位传入并限幅，清除微分历史。
- `dt_s > 0`；微分对测量值求导，D = -kd × (measurement - previous_measurement) / dt_s，首步 D=0。
- 当候选积分的变化使饱和方向进一步恶化时拒绝本次积分变化；否则接收，再对总输出限幅。判断使用积分贡献变化的符号，不能假设 ki 必为正。
- 非法输入或中间计算溢出时返回-1；成功返回0，实例和 output 均保持原值。每个控制对象独占实例；不内置周期、任务或反馈来源。

### 2.4 ramp.h / ramp.c

**文件：** `Algorithm/Inc/ramp.h` 定义状态；`Algorithm/Src/ramp.c` 实现显式 dt 的变化率限制。

```c
typedef struct {
    float value;
    float rise_per_s;
    float fall_per_s;
    bool initialized;
} ramp_t;

int ramp_init(ramp_t *ramp, float initial,
               float rise_per_s, float fall_per_s);
int ramp_reset(ramp_t *ramp, float value);
int ramp_step(ramp_t *ramp, float target, float dt_s, float *output);
```

变化率必须有限且 ≥0，dt_s 必须有限且 >0。上升/下降分别最多变化对应速率 × dt_s，不越过目标；速率为 0 表示该方向保持。reset 立即修改当前值，保留速率。非法输入/计算失败不改变状态；无任务、无隐式时钟。

## 3. Bsp

### 3.1 bsp.h / bsp.c

**文件：** `Bsp/Inc/bsp.h` 仅定义故障类型及板级入口；`Bsp/Src/bsp.c` 持有固定资源表、初始化顺序和故障锁存。

```c
void bsp_init(void);

typedef enum {
    FAULT_INIT=1, FAULT_ASSERT, FAULT_STACK_OVERFLOW,
    FAULT_SCHEDULER, FAULT_HARDWARE
} fault_reason_t;
void panic(fault_reason_t reason);
```

- 类型按模块归属：motor_id_t在motor_tt.h，servo_id_t在servo.h；LED/CAN/UART及PWM的ID在对应驱动头文件，gpio_port_t和pin_desc_t在gpio.h。各驱动头文件包含bsp.h取得返回值，需要引脚描述时再包含gpio.h；bsp.h不反向包含这些头文件。资源描述和bsp_get_*声明放在对应驱动头文件，实现仍在bsp.c；bsp.c包含各驱动头文件，不引用Components头文件。
- 描述查询只复制静态表，可在初始化前调用；0 和未列出的 ID 无效。描述供驱动使用，不开放改引脚、改组频率或运行期重新分配资源。
- `bsp_init`：依次准备 pwm → led → can → uart → adc；I2C暂不实现，首版不调用i2c初始化。仅管理运行期驱动状态，不重复 ATWP 基础初始化；不创建 robot_main、不启动调度器。
- PWM 固定映射按框架方案：MOTOR_TT_1..6 对应 CN9/CN5/CN6/CN7/CN8/CN10；TMR2/3/13/14 为 50Hz，TMR1/4/12 为 20kHz。
- `panic`：可从初始化、任务或故障/中断入口调用；锁存原因并禁止再次使能本地 PWM，关闭本地输出后进入不返回的故障停留。允许重复进入；不等待锁、不发送日志、不承诺 CAN 远端停机。即使初始化未完成也须能走最小寄存器关闭路径。
- 描述内 MUX、计时及停机参数须依据已核对的硬件配置填写；未落实的安全输出配置导致初始化失败。

### 3.2 os.h / os.c

**文件：** `Bsp/Inc/os.h` 只封装线程创建、调度、延时、时间查询和线程通知；`Bsp/Src/os.c` 转发FreeRTOS任务接口，并保留Idle静态内存及内核hooks。不封装队列、互斥、信号量、事件组、临界区或堆内存。

线程类型集中在os.h：`os_task_t`适配TaskHandle_t，`os_tick_t`适配TickType_t。不定义os_queue_t/os_mutex_t及对应存储类型，不建立隐藏任务槽池。

```c
typedef void (*os_task_entry_t)(void *args);
#define OS_WAIT_FOREVER UINT32_MAX

/* 仅供Application动态创建线程。 */
os_task_t os_createTask(const char *name, os_task_entry_t entry,
    void *args, uint32_t priority, size_t stack_words);
void os_scheduler_start(void);
void os_delay(uint32_t delay_ms);
os_tick_t os_tick_now(void);
void os_delay_until(os_tick_t *last_wake, uint32_t period_ms);
uint32_t os_now_ms(void);

void os_task_notify(os_task_t task);
void os_task_notify_from_isr(os_task_t task, bool *higher_priority_woken);
int os_task_wait(uint32_t timeout_ms);
void os_yield_from_isr(bool higher_priority_woken);

/* 仅供Application删除自己的动态线程。 */
void os_task_delete_dynamic(os_task_t task);
```

- 无os_init，也不提供静态线程创建封装或静态栈/控制块类型别名。os_createTask通过xTaskCreate动态创建，直接返回os_task_t，参数非法或分配失败返回NULL；stack_words单位为word。仅Application可调用。框架组件及main中的默认robot_main继续直接使用xTaskCreateStatic，由所属.c静态持有StackType_t栈和StaticTask_t控制块，创建后检查句柄。动态删除只接受Application自己的动态线程，删除前须停止其他引用者。
- os_scheduler_start成功后不返回；若返回则封装直接调用panic。组件不得启动调度器。
- os_delay仅调度后使用；非零ms向上取整到tick，0只让出执行权，不能保证低优先级线程运行。
- os_delay_until使用原生tick保存节拍，周期必须非零；正常按绝对周期推进，超期时将基准重置为当前tick；无效参数时忽略调用。
- os_tick_now/os_now_ms提供线程周期及设备超时所需的时间查询。os_now_ms按uint32_t回绕，实现须处理原生tick回绕；不封装通用os_elapsed，调用方按无符号差值判断超时。
- 线程通知仅用于唤醒，允许合并；数据由使用方自己的队列保存。wait消耗当前任务通知，收到通知返回0，到期或参数无效返回-1；该通知槽不得与其他语义混用。
- FromISR接口不阻塞，只将woken置true、不清除已有true；IRQ尾部统一yield，须满足内核中断优先级约束。
- Idle静态内存回调、栈溢出hook和断言适配保留在os.c，属于内核接入，故障转panic，不作为公共业务API。

**非线程资源由使用方直接使用FreeRTOS：**

| 需求 | 使用方式与约束 |
|---|---|
| 队列 | 模块.c持有StaticQueue_t及静态数据区，调用xQueueCreateStatic、xQueueSend/xQueueReceive及需要的FromISR接口 |
| 互斥/信号量 | 模块.c持有StaticSemaphore_t，调用对应Static创建接口；互斥只用于任务，不能从ISR等待 |
| 临界区 | 在使用方调用任务/ISR各自的临界区宏，ISR保存并恢复原屏蔽状态；临界区内不等待、不阻塞 |
| App动态队列/同步对象 | Application直接调用FreeRTOS动态创建与匹配删除接口，框架内部禁止使用 |
| App业务内存 | Application直接调用pvPortMalloc/vPortFree；统一heap_4，不与C库堆混用，不直接释放RTOS句柄 |

FreeRTOS非线程接口及静态线程创建接口是层依赖的显式例外，局限在使用方实现文件；普通组件公共头文件不暴露队列/互斥句柄。驱动内部处理原生错误码和ms超时；不向框架API暴露统一状态码。原生等待以tick为单位，OS_WAIT_FOREVER仅用于本文封装API，不能直接作为任意原生接口的等待值。

### 3.3 pwm.h / pwm.c

**文件：** `Bsp/Inc/pwm.h` 声明固定通道操作；`Bsp/Src/pwm.c` 实现时钟换算、比较寄存器提交和关闭。通道/组类型定义在pwm.h。

```c
typedef enum {
    PWM_MOTOR_1_IN1, PWM_MOTOR_1_IN2,
    PWM_MOTOR_2_IN1, PWM_MOTOR_2_IN2,
    PWM_MOTOR_3_IN1, PWM_MOTOR_3_IN2,
    PWM_MOTOR_4_IN1, PWM_MOTOR_4_IN2,
    PWM_MOTOR_5_IN1, PWM_MOTOR_5_IN2,
    PWM_MOTOR_6_IN1, PWM_MOTOR_6_IN2,
    PWM_SERVO_1, PWM_SERVO_2, PWM_SERVO_3,
    PWM_SERVO_4, PWM_SERVO_5, PWM_SERVO_6, LAST_PWM_CHANNEL
} pwm_channel_t;
typedef enum {
    PWM_GROUP_TMR1, PWM_GROUP_TMR2, PWM_GROUP_TMR3,
    PWM_GROUP_TMR4, PWM_GROUP_TMR12,
    PWM_GROUP_TMR13, PWM_GROUP_TMR14, LAST_PWM_GROUP
} pwm_group_t;
typedef struct {
    pin_desc_t pin;
    pwm_group_t group;
    uint8_t timer_channel;
    uint8_t alternate_function;
    uint32_t target_hz;
} pwm_desc_t;
int bsp_get_pwm_desc(pwm_channel_t channel, pwm_desc_t *out);

/* 组件使用的操作接口，仅保留以下六个。 */
void pwm_init(void);
void pwm_set_duty(pwm_channel_t channel, float duty);
void pwm_set_pair(pwm_channel_t first, float first_duty,
                      pwm_channel_t second, float second_duty);
void pwm_disable(pwm_channel_t channel);
void pwm_disable_pair(pwm_channel_t first, pwm_channel_t second);
void pwm_disable_all(void);
```

- **设置即输出：** duty为有限[0,1]。set_duty设置单个舵机通道，set_pair同时设置同一电机的两路输入；驱动负责在合法周期边界更新并开启输出，不再分为“暂存目标”和“enable”两步。初始化仍保持全部关闭。
- **组件负责使能状态：** motor_tt/servo只有在自身已使能时才调用set；上层enable从安全零目标或标定初始脉宽开始，不能重放旧指令。panic()会关闭所有PWM输出并停机，不会返回。
- **成对更新只为桥驱动：** set_pair/disable_pair只接受固定映射中同一电机的两个输入，先验证全部参数再整体执行。禁止用set_duty/disable单独操作电机半桥输入，误用时忽略该请求。
- **关闭即撤销输出：** disable用于单舵机通道，disable_pair用于电机输入对；清除待生效旧目标并强制已确认的关闭电平，不停共享计数器、不影响其他设备。pwm_disable_all供panic使用，无锁、无等待，初始化未完成时也须安全；它本身不建立永久故障锁存。
- **单位只保留占空比：** servo.c负责将标定脉宽换算为duty（50Hz时pulse_us/20000.0f）；motor_tt.c负责方向与双输入占空比。pwm不解析设备语义，不提供脉宽、角度、改频或时序查询API。
- **硬件细节留在.c：** 计数频率、周期计数、比较值换算、预装载和双输入同步提交全部内部处理。配置必须满足约定频率及脉宽精度，不能静默采用偏离约定的周期。成对写入处理跨更新边界问题，不重置同组其他通道的相位。
- 正常返回表示已接受更新，不保证已输出一个完整周期；50Hz更新仍可能等待20ms。调用方按设备保持单一逻辑写者，驱动短临界区保护寄存器提交。

pwm_group_t、pwm_desc_t和bsp_get_pwm_desc仅用于bsp.c与pwm.c之间的固定映射接入，不属于组件控制接口；组件只使用通道ID和上述操作，不操作组或资源描述。不增加独立配置、运行期资源分配或PWM后台任务。

### 3.4 can.h / can.c

**文件：** `Bsp/Inc/can.h` 只提供原始帧收发和IRQ入口；`Bsp/Src/can.c` 持有每总线静态RX队列，直接使用硬件TX邮箱，不建软件发送队列、latest槽或后台线程。

```c
typedef enum { CAN_BUS_1, CAN_BUS_2, LAST_CAN_BUS } can_bus_t;
typedef struct {
    uint32_t id;
    bool extended;
    bool remote;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t timestamp_ms;
} can_frame_t;

void can_init(void);
void can_send(can_bus_t bus, const can_frame_t *frame);
int can_receive(can_bus_t bus, can_frame_t *out);
void can_irq_handler(can_bus_t bus);

/* 仅供bsp.c/can.c固定映射接入，组件不调用。 */
typedef struct { pin_desc_t tx, rx; uint32_t bitrate; } can_desc_t;
int bsp_get_can_desc(can_bus_t bus, can_desc_t *out);
```

- send/receive均非阻塞，每次处理一帧。bus只通过参数指定，不在帧内重复存储；can.h不依赖os线程句柄。
- send验证ID、帧型及dlc≤8后尝试写入空闲硬件邮箱；无空邮箱或参数无效时忽略，不缓存、不自动排队。数据在返回前复制，timestamp_ms在发送时忽略。
- receive从指定总线静态队列取帧，空队列返回CAN_RECEIVE_EMPTY (-1)，错误返回CAN_RECEIVE_ERROR (-2)，成功时返回0并携带ISR接收时间戳。每总线默认32帧，单消费者；队列满丢新帧，内部累计溢出计数用于调试，不因新增统计改变控制状态。
- IRQ只搬运RX帧、清标志和处理硬件错误；不绑定组件任务、不解析协议。发送完成不需要推进软件队列。硬件滤波由固定配置决定。
- bus-off时驱动中止尚未完成的硬件发送，记录错误并按固定策略恢复，不重放旧帧。未处理的总线错误阻止send并忽略该次请求；receive优先向唯一消费者返回CAN_RECEIVE_ERROR (-2)，使其撤销该总线旧目标。已上总线的帧不能撤回，远端停机不作保证。
- 无任务绑定、状态查询、latest发送或取消软件待发接口。寄存器/队列并发保护与恢复过程留在驱动内部，不能无限等待。

### 3.5 uart.h / uart.c

**文件：** `Bsp/Inc/uart.h` 提供流式/帧式读取和发送；`Bsp/Src/uart.c` 管理USART6循环DMA接收、空闲边界和静态帧存储；USART1发送暂缓。

```c
typedef enum { UART_1, UART_6, UART_7, LAST_UART_ID } uart_id_t;
typedef struct {
    pin_desc_t tx, rx;
    uint32_t baudrate;
    uint8_t data_bits, stop_bits;
    bool even_parity;
    bool enabled;
} uart_desc_t;
int bsp_get_uart_desc(uart_id_t id, uart_desc_t *out);

#define UART_FRAME_CAPACITY 64U
typedef struct {
    uint8_t data[UART_FRAME_CAPACITY];
    size_t length;
    uint32_t timestamp_ms;
    uint32_t error_flags;
} uart_frame_t;
typedef struct {
    uint32_t rx_dropped, tx_rejected;
    uint32_t parity_errors, framing_errors, overrun_errors;
} uart_state_t;

void uart_init(void);
size_t uart_read(uart_id_t id, uint8_t *data, size_t capacity);
int uart_receive_frame(uart_id_t id, uart_frame_t *out, uint32_t timeout_ms);
size_t uart_write(uart_id_t id, const uint8_t *data, size_t length);
void uart_get_state(uart_id_t id, uart_state_t *out);
void uart_irq_handler(uart_id_t id);
```

- UART_1暂缓实现，保留ID及115200 8N1参数设计，不启用业务收发、中断或TX DMA，不分配TX缓冲，调用调用被忽略；UART_6 为帧模式，100000、8 数据位+偶校验+1 停止位；UART_7 默认关闭，调用调用被忽略。
- 模式固定，不能对同一端口同时 stream read 与 receive_frame，模式不符时忽略请求或返回0。read 非阻塞且可部分读取，无数据返回0。
- 帧 API 根据空闲边界发布帧，空闲不等同“合法遥控帧”；dbus 自行验证 18 字节长度。超长帧丢弃到下一个空闲边界，错误/溢出通过帧错误标志传播，不能把截断帧作为合法数据。
- error_flags 定义命名位：UART_ERROR_PARITY、UART_ERROR_FRAMING、UART_ERROR_OVERRUN、UART_ERROR_OVERFLOW。非零错误帧可交上层统计，但不作为合法输入。
- USART6 双 64 字节帧槽由 ISR 填充、完成后冻结；消费者复制完成才归还。两槽均占用时丢弃新帧并计数，不能覆盖正在解析的数据。允许任务有限等待；无数据或超时返回-1，成功返回0。
- write 是全有或全无的入队，长度超过总容量或当前余量不足时忽略该请求。数据复制后返回写入字节数，不表示线上的发送已结束。USART1发送暂缓，该端口优先调用被忽略；后续启用时再分配发送缓冲。已启用端口长度0允许空指针且正常返回。
- 源文件使用 Algorithm/ringBuffer，临界区保护各操作；UART ISR 不解析协议、不打印日志。无默认后台线程；当前仅接入USART6 RX DMA，USART1发送及其DMA暂缓。

### 3.6 adc.h / adc.c

**文件：** `Bsp/Inc/adc.h` 声明单路原始采样；`Bsp/Src/adc.c` 实现 ADC1_IN2 校准、采样和超时清理。

```c
typedef struct { pin_desc_t pin; uint8_t channel; } adc_desc_t;
int bsp_get_adc_desc(adc_desc_t *out);

typedef struct {
    uint16_t raw;
    uint16_t full_scale;
    uint32_t timestamp_ms;
} adc_sample_t;
void adc_init(void);
int adc_sample(adc_sample_t *out, uint32_t timeout_ms);
```

仅开放 PA2/ADC1_IN2，不为 PA3 添加无来源数据。raw 范围 0..full_scale，full_scale 是配置分辨率对应最大码值。sample 在调度后调用，有限超时必须 >0，内部静态互斥防止转换重入；时间戳取完成时刻。超时恢复转换状态并返回-1，成功返回0；不保留部分输出。初始化中的校准等待使用有限硬件时限，不依赖尚未启动的任务延时。不包含分压倍率、电压滤波或欠压业务决策。

### 3.7 i2c.h / i2c.c（暂不实现）

**状态：暂不实现。** 以下文件、类型及接口仅保留设计，不属于首版实现或可调用API；不创建占位实现，不纳入编译，不分配互斥及缓冲，不接入bsp_init。后续有明确外设需求时再实现。

**预留文件：** `Bsp/Inc/i2c.h` 用于I2C2寄存器读写声明；`Bsp/Src/i2c.c` 用于静态互斥、事务和错误恢复。下列语义均为后续实现约定。

```c
typedef struct { pin_desc_t scl, sda; uint32_t bitrate; bool enabled; } i2c_desc_t;
int bsp_get_i2c_desc(i2c_desc_t *out);

void i2c_init(void);
void i2c_read_reg(uint8_t address_7bit, uint16_t reg,
    uint8_t reg_bytes, uint8_t *data, size_t length, uint32_t timeout_ms);
void i2c_write_reg(uint8_t address_7bit, uint16_t reg,
    uint8_t reg_bytes, const uint8_t *data, size_t length, uint32_t timeout_ms);
```

固定 I2C2 PF6/PB11，首版不启用。后续实现并启用后， address_7bit 不含读写位，接受 0x08..0x77；reg_bytes 为1或2，两字节高字节先发，1字节时reg≤255。length>0、timeout_ms>0且有限。读采用写寄存器地址后 repeated START，完成后 STOP。

同步阻塞接口仅调度后任务可用；NACK、总线错误或超时返回-1，成功返回0。读失败时 data 可能被部分写入，全部内容须丢弃（公共失败输出规则的例外）；写失败可能已对外设产生部分作用，不能自动重试有副作用的寄存器操作。恢复不得无限等待，不引入额外线程。

### 3.8 led.h / led.c

**文件：** `Bsp/Inc/led.h` 声明三个编号 LED；`Bsp/Src/led.c` 按静态映射写 GPIO。

```c
typedef enum { LED_1, LED_2, LED_3, LAST_LED } led_id_t;
int bsp_get_led_pin(led_id_t id, pin_desc_t *out);

void led_init(void);
void led_set(led_id_t id, bool on);
```

LED_1/2/3 分别 PC13/14/15，均为红灯，高电平点亮；初始化全部关闭。set 同步完成寄存器写入；无灯效、无线程、无颜色语义，也不新增 toggle 读改写接口。


### 3.9 gpio.h

**文件：** `Bsp/Inc/gpio.h` 仅定义GPIO端口和引脚描述类型，包含stdint.h；无对应.c、初始化函数或线程。

```c
typedef enum { GPIO_PORT_A, GPIO_PORT_B, GPIO_PORT_C, GPIO_PORT_F } gpio_port_t;
typedef struct { gpio_port_t port; uint8_t pin; } pin_desc_t;
```

## 4. Components

### 4.1 组件初始化

组件不设统一初始化汇总模块。启动入口在 `main.c` 中自行声明并依次调用 `motor_tt_init()`、`servo_init()` 和 `dbus_init()`。

前置条件 bsp_init 成功；顺序 motor_tt → servo → dbus。不可恢复错误由对应模块直接调用 panic()；可恢复问题忽略。只创建各组件的静态资源，不调用 bsp_init、不创建 robot_main、不启动调度器。

### 4.2 dbus.h / dbus.c

**文件：** `Components/Inc/dbus.h` 提供遥控输入快照；`Components/Src/dbus.c` 持有配置、解码、快照及私有 dbus_task。

```c
typedef enum {
    DBUS_SWITCH_UNKNOWN=0,
    DBUS_SWITCH_UP,
    DBUS_SWITCH_MIDDLE,
    DBUS_SWITCH_DOWN
} dbus_switch_t;
typedef struct {
    float channel[4];
    dbus_switch_t switch_left, switch_right;
    int16_t mouse_x, mouse_y, mouse_z;
    bool mouse_left, mouse_right;
    uint16_t keys;
    float wheel;
    uint32_t timestamp_ms;
    uint32_t sequence;
    bool valid;
    bool online;
} dbus_t;

int dbus_get(dbus_t *out);
void dbus_useUartInstead(bool use_uart);
```

- DBUS 解码参数使用实现内部常量：中位 1024、幅度 660、死区 33、离线超时 100 ms；init 不接收外部配置。
- channel 是减中心、应用死区并除以 span 的 [-1,1] 值。合法协议范围内的端点可限幅；非法原始值拒绝整帧。死区内为0，死区外不重缩放。
- 左/右开关的报文字段对应须依遥控器实物确认，不能仅凭位位置猜测。keys 为协议位图，头文件按已确认协议提供键位掩码；wheel 按中心1024、跨度660和死区33归一化，限幅至[-1,1]，死区外不重新缩放。DBUS_KEY_W/S/D/A/SHIFT/CTRL/Q/E/R/F/G/Z/X/C/V/B 为对应协议位掩码，可直接与keys进行按位运算。
- 私有任务通过 uart_receiveFrame(UART_6,...) 接收，18字节长度、错误标志、开关及通道范围全部通过后发布，sequence递增。不能把结构检查描述为CRC校验。
- 初次未接收：valid=false、online=false、数值清零/开关UNKNOWN。掉线保留最后合法数据用于诊断，online=false；valid表示曾有合法数据，不能代替online。
- 默认100ms掉线；snapshot查询依据当前时间重新判断online，避免线程调度延迟产生过期在线状态。查询初始化后正常返回，即使无有效数据也通过标志表达。
- 任务静态栈384 words、优先级5；UART帧队列提供等待和唤醒，超时等待有界。快照短临界区复制，不直接关闭其他组件输出。

### 4.3 motor_dji.h / motor_dji.c

**文件：** `Components/Inc/motor_dji.h` 提供配置类型、反馈与原始电流命令；`Components/Src/motor_dji.c` 持有注册配置、协议规则、成组缓冲和 motor_dji_task。

```c
typedef enum {
    MOTOR_DJI_M2006=1, MOTOR_DJI_M3508, MOTOR_DJI_GM6020
} motor_dji_model_t;
typedef struct {
    uint8_t id;
    can_bus_t bus;
    motor_dji_model_t model;
    bool invert;
} motor_dji_config_t;
typedef struct {
    uint16_t encoder_raw;
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
    volatile motor_dji_status_t measure;
    uint8_t slot;
} motor_dji_t;

int motor_dji_register(motor_dji_t *motor, const motor_dji_config_t *config);
void motor_dji_stop(const motor_dji_t *motor);
void motor_dji_setCurrent(const motor_dji_t *motor, int16_t raw_current);
```

- id 直接表示硬件电调编号；注册通过 bus、model、id 组合识别硬件，控制/查询通过返回的 motor_dji_t 句柄指针定位设备，不同总线或无协议冲突的不同型号可使用相同编号。应用通过配置结构体调用 motor_dji_register，驱动复制配置到静态容量16的存储中。首次成功注册创建服务任务，可运行期追加，禁止ISR调用；不支持注销或覆盖。未注册设备时不创建任务。注册失败返回-1且不修改已有设备和输出句柄，成功返回0并写入句柄。对象须在驱动生命周期内保持有效且地址稳定，不支持复制、移动或重复注册；配置复制后不依赖调用者存储。应用可直接读取 motor->measure，多字段及64位角度一致性由应用使用任务临界区读取保证。
- 已实现 M2006/C610（电流 ±10000）、M3508/C620（电流 ±16384）、GM6020（电流 ±16384）。GM6020 要求开启支持该功能的固件电流环。配置包含 id、bus、model、invert；invert 默认 false，反向时命令和带符号反馈均翻转，encoder_raw 保留物理值，16位反馈反向溢出时饱和至32767；电流限值由型号推导，反馈和命令超时统一100ms。配置及接线示例见 [大疆电机驱动](大疆电机驱动.md)。
- 注册校验硬件id范围、同总线反馈ID唯一、分组槽不冲突、电流限值按型号自动确定。成组ID由型号和id推导，禁止应用随意填裸命令ID绕过校验。
- 每个组8字节四个有符号16位命令，高字节在前；未配置、未使能、反馈离线或命令超时的槽填0。双总线组状态相互独立；组件每轮依据最新目标重新打包，调用can_send(bus, &frame)。邮箱BUSY时放弃该次发送，下一轮重取最新目标，不保留旧帧队列；各命令组轮转尝试，避免固定顺序使后续组长期饥饿。
- set只接受反馈在线的设备并激活输出，否则忽略；命令越限时忽略，不静默裁剪。任务是CAN提交的唯一写者，应用仅修改目标缓冲。
- setCurrent 要求已收到有效在线反馈，并更新目标、命令时间和 enabled 状态。stop立即在组件状态中撤销使能、清零目标；服务任务下一轮按新目标提交全组值，不能把同组其他设备误清零。stop不保证邮箱旧帧撤回或远端立即停机。
- 默认反馈/命令超时均100ms。超时撤销使能、清零目标，恢复反馈后须提交新电流命令。CAN bus-off也执行此规则，不自动恢复旧电流。
- encoder_raw范围0..8191，speed_rpm/current_raw使用协议原始单位，符号按invert转换；没有减速比信息时不称为输出轴转速。温度字段按型号有效性标注。
- 首帧angle_counts=0，以后差值折回[-4096,4095]累计；首次/重连首帧angle_continuous=false，后续连续合法帧可为true。此标志仅表示当前接收段内连续，不表示跨掉线保持位置；超过半圈/采样仍有不可检测歧义。用宽整数并处理累计溢出，不依赖有符号溢出行为。
- 无合法反馈valid=false；离线保留最后反馈并online=false。在线和超时状态由后台任务维护，应用直接读取 motor->measure。
- 静态任务优先级5、栈768 words，使用xTaskDelayUntil按1ms周期调度（低tick频率时至少1 tick），错过周期时阻塞1 tick并重设基准；每轮按限定帧数预算调用can_receive读取两总线，再服务最新命令和超时，随后等待下个周期，超期时实际阻塞1 tick，不能无限排空。can_receive返回CAN_RECEIVE_EMPTY (-1)时结束本轮接收；返回CAN_RECEIVE_ERROR (-2)时撤销该总线全部电机使能并清零目标，BSP同时清空故障前排队帧。栈和控制块由.c持有。

### 4.4 motor_tt.h / motor_tt.c

**文件：** `Components/Inc/motor_tt.h` 提供六路TT电机开环控制；`Components/Src/motor_tt.c` 持有方向配置、使能状态和RZ7889私有输出规则。

```c
typedef enum { MOTOR_TT_1, MOTOR_TT_2, MOTOR_TT_3, MOTOR_TT_4, MOTOR_TT_5, MOTOR_TT_6, LAST_MOTOR_TT } motor_tt_id_t;

typedef struct { bool invert; } motor_tt_config_t;
typedef struct {
    float duty;
    bool enabled;
} motor_tt_state_t;
void motor_tt_init(void);
void motor_tt_enable(motor_id_t id);
void motor_tt_set_duty(motor_id_t id, float ratio);
void motor_tt_stop(motor_id_t id);
int motor_tt_get_state(motor_id_t id, motor_tt_state_t *out);
```

- id为MOTOR_1..6，依次CN9/CN5/CN6/CN7/CN8/CN10；映射只从Bsp取得，组件不写GPIO名称。invert静态配置修正安装/接线方向，尤其CN10差异；duty反馈为应用坐标系命令。
- ratio为有限[-1,1]，表示输出比例；无速度或位置反馈，不提供get_speed。set在未使能时调用被忽略，不保存将来自动执行的指令。
- enable置安全零目标，按确认的桥规则启用；再次enable不恢复此前非零指令。stop撤销使能并关闭对应输入对，可重复调用。
- 零占空比的使能态与stop的未使能态明确区分；具体制动/滑行、电平和换向步骤须依据RZ7889确认。未确认前不能给出虚构真值表；实现不得默认“双低就是某种制动”。
- 换向需过渡等待时，首版保持同步、有界处理，由同一调用任务完成；不得在临界区忙等。若硬件要求引入后台状态机，需另行更新任务/时间语义与预算。
- get_state是已接受目标/使能状态，不是实测电机运动。源文件无默认线程、无自主命令超时；每设备指定一个任务写者，组件短保护保证查询一致，禁止多业务任务竞相控制。
- MOTOR_TT_3/6为50Hz，其余20kHz；set不修改基础频率。

### 4.5 servo.h / servo.c

**文件：** `Components/Inc/servo.h` 提供脉宽/角度接口；`Components/Src/servo.c` 保存六路标定、使能状态并调用pwm。

```c
typedef enum { SERVO_1, SERVO_2, SERVO_3, SERVO_4, SERVO_5, SERVO_6, LAST_SERVO } servo_id_t;

typedef struct {
    uint32_t pulse_min_us, pulse_max_us, initial_pulse_us;
    float angle_min_deg, angle_max_deg;
    bool invert;
    bool calibrated;
} servo_config_t;
typedef struct {
    uint32_t pulse_us;
    float target_angle_deg;
    bool enabled;
} servo_state_t;
void servo_init(void);
void servo_enable(servo_id_t id);
void servo_disable(servo_id_t id);
void servo_set_pulse_us(servo_id_t id, uint32_t pulse_us);
void servo_set_angle_deg(servo_id_t id, float angle_deg);
int servo_get_state(servo_id_t id, servo_state_t *out);
```

- SERVO_1..6对应H1.1..6，全部50Hz。配置存在servo.c静态表；不默认所有舵机均是0..180°或500..2500µs。
- 标定要求pulse_min>0、pulse_min<pulse_max<20000、angle_min<angle_max且有限，initial位于脉宽区间。calibrated=false的通道允许整体框架初始化，但enable/set调用被忽略，保持关闭。
- enable从initial_pulse_us开始，可能使舵机运动；用户须选择合理初始位置。disable取消脉冲并清除旧运行目标，重新enable回到initial，不恢复旧目标。
- set只允许已使能通道，范围外拒绝；angle线性映射到标定脉宽，invert交换映射端点，脉宽按最近整数µs舍入。直接set_pulse仍检查标定边界。
- state记录已接受的脉宽及通过同一标定反算的目标角度，不是反馈实测角度；50Hz目标可被新值覆盖。查询与更新短保护；每通道单一写者。
- 无默认线程，不承诺无人调用时自动执行命令超时。若未来需要自主超时关闭，由本组件新增静态任务并更新设计。

## 5. 生命周期、资源与调用示例

### 5.1 唯一启动装配点

生成main的用户代码区依次执行：ATWP基础初始化 → bsp_init → motor_tt_init → servo_init → dbus_init → 使用main静态栈/控制块直接调用xTaskCreateStatic创建robot_main → os_scheduler_start。初始化不可恢复错误或任务创建失败时直接panic。Application仍只有 `void robot_main(void *args)` 用户入口，不新增应用初始化模块。

### 5.2 静态资源归属表

| 源文件 | 持有资源 | 默认线程 |
|---|---|---|
| ringBuffer.c / pid.c / ramp.c；math_utils.h | 无全局实例；状态由调用者持有，math_utils无状态 | 无 |
| bsp.c | const物理映射、初始化/故障状态 | 无 |
| os.c | Idle栈/控制块、内核hook数据 | Idle |
| pwm.c | 通道状态及暂存比较值 | 无 |
| can.c | 每总线静态RX32帧、硬件邮箱及内部错误状态；无软件TX队列 | 无 |
| uart.c | 静态ringBuffer、USART6 DMA循环缓冲128字节、双64字节帧槽、帧等待对象 | 无 |
| adc.c | 静态互斥与状态 | 无 |
| i2c.h / i2c.c（预留） | 暂不实现，首版不占用运行期资源 | 无 |
| led.c | 初始化状态 | 无 |
| dbus.c | 参数、快照、静态栈与TCB | dbus_task 384 words |
| motor_dji.c | 注册配置、反馈/目标/组状态、静态栈与TCB | motor_dji_task 768 words；空配置不开 |
| motor_tt.c / servo.c | 各六项配置和状态 | 无 |

CAN接收队列及驱动同步对象须计入实施时RAM预算；此表不是链接结果或栈安全证明。私有函数、线程入口、协议解析器不在公共头文件中暴露。

### 5.3 用户入口中的设备调用片段

```c
/* 仅展示API组合；放入robot_main的业务逻辑，不额外创建Application文件。 */
dbus_snapshot_t input;

if (dbus_get_snapshot(&input) == 0 && input.valid && input.online) {
    /* motor_tt_enable(MOTOR_TT_1)应在用户明确的使能动作中调用一次。 */
    motor_tt_set_duty(MOTOR_TT_1, input.channel[1]);
} else {
    motor_tt_stop(MOTOR_TT_1);
}
/* 遥控恢复后不会在此片段中自动重新使能。 */
```

Application可直接调用os/led/uart基础接口，设备控制使用Components，计算使用Algorithm。大疆电机配置由应用构造结构体并注册；其他组件的配置仍在各组件.c集中修改。

## 6. 实现前需要落实的接口细节

1. RZ7889停机/换向真值表及允许PWM范围，决定motor_tt私有映射和pwm关闭电平。
2. 实际DJI设备清单及型号协议，决定应用的motor_dji注册配置、字段有效性与分组ID，不能仅凭型号名称推定全部参数。
3. 六舵机标定、遥控器开关字段和滚轮解释；未确认通道保持禁用或原始数据形式。
4. AT32定时器双输入统一提交机制、实际时钟及中断优先级；影响底层实现，不开放应用任意改频接口。
5. OS适配与厂商库实际符号冲突检查：公共名若与厂商函数冲突，优先采用具体操作名称调整并同步本文，禁止通过脆弱宏覆盖厂商符号。

本文为API设计交付，未添加源代码、未执行编译或测试。

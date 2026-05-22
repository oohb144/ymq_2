# ymq_2 — STM32H750 麦克纳姆轮四轮驱动控制程序

## 1. 项目概述

基于 STM32H750VBT6 的麦克纳姆轮（Mecanum）四轮全向移动机器人控制程序。使用 STM32CubeMX + Keil MDK-ARM 开发，裸机（无 RTOS）50Hz 超级循环控制。

**硬件核心：**

- MCU：STM32H750VBT6（Cortex-M7, 480MHz）
- 电机驱动：4 路直流电机 + TB6612/DRV8833 等 H 桥驱动
- 编码器：4 路正交编码器（TIM3/TIM4/TIM5/TIM8）
- PWM 输出：TIM2 四通道（20kHz）

**控制架构：**
```
编码器读取 → 正运动学(里程计) → 逆运动学(目标分解) → PID 速度环 → 电机输出
```

---

## 2. 目录结构

```
ymq_2/
├── Core/
│   ├── Inc/
│   │   ├── main.h                    # CubeMX 生成，引脚宏定义
│   │   ├── mecanum_kinematics.h      # 运动学参数与接口（用户可调）
│   │   ├── mecanum_motor.h           # 电机驱动参数与接口（用户可调）
│   │   ├── mecanum_pid.h             # PID 参数与接口（用户可调）
│   │   ├── gpio.h / tim.h            # CubeMX 生成的外设头文件
│   │   └── stm32h7xx_hal_conf.h      # HAL 库配置
│   └── Src/
│       ├── main.c                    # 主入口 + MecanumControl_50Hz() 控制逻辑
│       ├── mecanum_kinematics.c      # 正/逆运动学实现
│       ├── mecanum_motor.c           # 4 路电机 PWM + 方向控制
│       ├── mecanum_pid.c             # 4 路增量式 PID
│       ├── gpio.c / tim.c            # CubeMX 生成的外设初始化
│       └── stm32h7xx_it.c            # 中断服务函数
├── Drivers/                          # HAL 库 + CMSIS（自动生成，勿改）
├── MDK-ARM/                          # Keil 工程文件
│   └── ymq_2.uvprojx                # 双击打开工程
├── 硬件资源表.md                      # 引脚分配文档
└── README.md                         # 本文档
```

---

## 3. 硬件资源分配

### 3.1 电机通道映射

| 通道 | PWM 输出 | 方向引脚 | 编码器定时器 | 极性 |
|------|---------|---------|------------|------|
| 电机A（左前） | TIM2_CH1 (PA15) | AIN1(PA4), AIN2(PC4) | TIM3 (PA6/PA7) | -1 |
| 电机B（右后） | TIM2_CH2 (PB3) | BIN1(PB7), BIN2(PB8) | TIM4 (PD12/PD13) | -1 |
| 电机C（左后） | TIM2_CH3 (PA2) | CIN1(PC10), CIN2(PC11) | TIM5 (PA0/PA1) | -1 |
| 电机D（右前） | TIM2_CH4 (PA3) | DIN1(PE14), DIN2(PE15) | TIM8 (PC6/PC7) | -1 |

### 3.2 时钟与定时器

| 外设 | 时钟源 | 配置 | 用途 |
|------|--------|------|------|
| SYSCLK | PLL (HSI 64MHz) | 480MHz | 系统主时钟 |
| TIM2 | APB1 × 2 = 240MHz | Prescaler=0, ARR=11999 → 20kHz PWM | 电机 PWM |
| TIM3/4/8 | - | 16位，ARR=65535 | 编码器模式 |
| TIM5 | - | 32位，ARR=0xFFFFFFFF | 编码器模式 |

### 3.3 指示灯

- PC13（LED）：主循环心跳灯，每 500ms 翻转一次，常闪 = 程序正常运行

---

## 4. 编译与烧录

### 4.1 开发环境

- **IDE**：Keil MDK-ARM V5.32 或以上
- **代码生成**：STM32CubeMX 6.14.1（修改引脚/时钟时使用）
- **下载器**：ST-Link / J-Link

### 4.2 编译步骤

1. 双击 `MDK-ARM/ymq_2.uvprojx` 打开 Keil 工程
2. 点击 **Build (F7)** 编译
3. 确认 0 Error, 0 Warning

### 4.3 烧录步骤

1. 连接 ST-Link 到开发板
2. Keil 中点击 **Options for Target → Debug → Use ST-Link Debugger**
3. 点击 **Download (F8)** 烧录
4. 点击 **Debug (Ctrl+F5)** 可在线调试

### 4.4 首次烧录检查清单

- [ ] 确认 4 个电机驱动板供电正常（推荐独立电源，与 MCU 共地）
- [ ] 确认 4 组编码器信号线连接正确
- [ ] **断开轮子与地面的接触**（防止意外高速旋转）
- [ ] 烧录后观察 LED 是否闪烁（确认程序跑起来了）

---

## 5. 参数调校指南

### 5.1 第一步：确认电机方向（开环测试）

打开 `Core/Src/main.c`，找到 `MecanumControl_50Hz()` 函数：

```c
#define TEST_OPEN_LOOP  0    // ← 改为 1，进入开环测试模式
#define TEST_PWM_VALUE  3000 // ← 测试 PWM 值，建议从 2000 开始
```

- 改为 `TEST_OPEN_LOOP = 1` 后重新编译烧录
- 观察四轮转向：向前运动时应全部同向旋转
- 若某路方向反了，修改 `Core/Inc/mecanum_motor.h` 中对应的极性：

```c
#define MOTOR_A_POLARITY    (-1)  // 改为 1 或 -1 切换方向
#define MOTOR_B_POLARITY    (-1)
#define MOTOR_C_POLARITY    (-1)
#define MOTOR_D_POLARITY    (-1)
```

- 确认无误后改回 `TEST_OPEN_LOOP = 0`

### 5.2 第二步：调 PID 参数

修改 `Core/Inc/mecanum_pid.h`：

```c
/* 电机A */
#define MOTOR_A_KP      2000    // 比例：响应速度，越大越快但易震荡
#define MOTOR_A_KI      0       // 积分：消除稳态误差，一般从小值开始
#define MOTOR_A_KD      500     // 微分：抑制震荡，阻尼作用

/* 电机B/C/D 同理 */
```

**调参经验（仅供参考，需根据实际硬件调整）：**

| 步骤 | 操作 | 现象判断 |
|------|------|---------|
| 1 | KP=1000, KI=0, KD=0 | 电机应能缓慢响应 |
| 2 | 逐步增大 KP（+500/次） | 直到响应足够快但不震荡 |
| 3 | 加入 KD（100~500） | 抑制 KP 过大引起的震荡 |
| 4 | 逐步加入 KI（10~100） | 消除目标速度与实际速度的稳态误差 |

### 5.3 第三步：调运动学参数

修改 `Core/Inc/mecanum_kinematics.h`：

```c
#define MECANUM_WHEEL_TRACK         0.375f    // 等效轮距 = 前后轴距/2 + 左右轮距/2（米）
#define MECANUM_WHEEL_DIAMETER      0.100f    // 轮径（米）
#define MECANUM_ENCODER_RESOLUTION  827.2f    // 编码器每转脉冲数（TI12 模式下）
#define MECANUM_LINEAR_CORRECTION   1.0f      // 线速度校正（里程计偏大则调大此值）
#define MECANUM_ANGULAR_CORRECTION  1.0f      // 角速度校正（旋转偏大则调大此值）
```

### 5.4 第四步：设定目标速度

当前目标速度在 `main.c` 中硬编码：

```c
g_target_speed[0] = -500;  // vx = -500 mm/s（向前）
g_target_speed[1] = 0;     // vy = 0（无横移）
g_target_speed[2] = 0;     // w  = 0（无旋转）
```

**速度单位说明：**

| 索引 | 含义 | 单位 | 说明 |
|------|------|------|------|
| `[0]` | vx | mm/s | 前进为正（当前极性为 -1 表示向前） |
| `[1]` | vy | mm/s | 右移为正 |
| `[2]` | w | mrad/s | 逆时针为正 |

---

## 6. 后续功能扩展指南

以下是后续可能需要添加的功能及对应的方法指引。

### 6.1 串口遥控（UART 接收速度指令）

**目标：** 通过串口发送 `vx, vy, w` 指令控制小车运动。

**步骤：**

1. 在 CubeMX 中开启 USART，配置波特率（如 115200）
2. 在 `main.c` 的 `USER CODE BEGIN Includes` 中添加：

```c
#include "usart.h"
```

3. 在 `main.c` 中添加串口接收解析（以 DMA + 空闲中断为例）：

```c
/* USER CODE BEGIN PV */
extern uint8_t usart_rx_buf[64];  // DMA 接收缓冲区
extern volatile uint8_t usart_rx_flag;
/* USER CODE END PV */

/* USER CODE BEGIN 3 — 在主循环中添加 */
if (usart_rx_flag) {
    usart_rx_flag = 0;
    /* 解析格式: "VX,VY,W\n"  例如 "-500,0,0\n" */
    int16_t vx, vy, w;
    if (sscanf((char*)usart_rx_buf, "%hd,%hd,%hd", &vx, &vy, &w) == 3) {
        g_target_speed[0] = vx;
        g_target_speed[1] = vy;
        g_target_speed[2] = w;
    }
}
/* USER CODE END 3 */
```

4. PC 端发送测试：`-500,0,0\n`（向前 500mm/s），`0,500,0\n`（右移 500mm/s）

### 6.2 遥控器（PS2 / SBUS / 蓝牙手柄）

**目标：** 用手柄实时控制小车方向和速度。

**核心思路：**
- 读取手柄摇杆值（通常映射到 0~255 或 -128~127）
- 线性映射到 `g_target_speed[0/1/2]`
- 在主循环中定时读取并更新

**示例（PS2 手柄 SPI 通信）：**

```c
/* 假设已读取摇杆值 joystick_lx, joystick_ly, joystick_rx */
int16_t max_speed = 1000;  // 最大速度 mm/s

g_target_speed[0] = (int16_t)((joystick_ly - 128) * max_speed / 128);  // 前后
g_target_speed[1] = (int16_t)((joystick_lx - 128) * max_speed / 128);  // 左右
g_target_speed[2] = (int16_t)((joystick_rx - 128) * max_speed / 128);  // 旋转
```

**注意：** 手柄死区处理——摇杆在中心附近会有微小偏移，需要加死区判断（如 ±10 以内视为 0）。

### 6.3 里程计反馈（输出到上位机）

**目标：** 将 `g_odom` 里程计数据通过串口发送给 PC/ROS 上位机。

```c
/* 在主循环中，每 N 个周期发送一次里程计 */
static uint32_t odom_send_cnt = 0;
if (++odom_send_cnt >= 5) {  // 每 100ms 发一次（50Hz × 5）
    odom_send_cnt = 0;
    /* 格式: "ODOM x y yaw vx vy w\n" */
    printf("ODOM %d %d %d %d %d %d\r\n",
           g_odom[0], g_odom[1], g_odom[2],
           g_odom[3], g_odom[4], g_odom[5]);
}
```

### 6.4 ROS 集成（micro-ROS 或 自定义协议）

**目标：** 让小车接入 ROS 导航栈（Nav2），发布 `/odom` 话题，订阅 `/cmd_vel`。

**方案 A — 自定义串口协议（推荐新手）：**
- 上位机（ROS 节点）订阅 `/cmd_vel`，将 `Twist` 消息解析为 `vx, vy, w`，通过串口下发
- 下位机（STM32）接收后更新 `g_target_speed`，同时定时上发 `g_odom`

**方案 B — micro-ROS（进阶）：**
- 将 micro-ROS agent 直接跑在 STM32 上
- 优点：原生 ROS2 通信，无需手动写协议
- 缺点：需要 FreeRTOS，资源占用较大

### 6.5 位置闭环（目标位置 → 速度控制）

**目标：** 给定目标坐标 (x, y, θ)，小车自动运动到位。

**核心思路：**

```c
/* 简单的位置 P 控制器（可替换为更复杂的轨迹规划） */
float err_x = target_x - (float)g_odom[0];    // x 误差 (mm)
float err_y = target_y - (float)g_odom[1];    // y 误差 (mm)
float err_yaw = target_yaw - (float)g_odom[2]; // yaw 误差 (mrad)

g_target_speed[0] = (int16_t)(Kp_pos * err_x);     // 比例控制
g_target_speed[1] = (int16_t)(Kp_pos * err_y);
g_target_speed[2] = (int16_t)(Kp_yaw * err_yaw);

/* 到达判断 */
if (fabs(err_x) < 10 && fabs(err_y) < 10 && fabs(err_yaw) < 50) {
    g_target_speed[0] = g_target_speed[1] = g_target_speed[2] = 0;
}
```

### 6.6 添加硬件看门狗（推荐尽快添加）

在 CubeMX 中开启 IWDG（独立看门狗），超时时间设为 100ms~500ms。在主循环中定期喂狗：

```c
/* USER CODE BEGIN 3 */
HAL_IWDG_Refresh(&hiwdg);  // 喂狗
/* 正常循环逻辑 */
/* USER CODE END 3 */
```

如果主循环卡死（如某个函数死循环），看门狗超时会自动复位 MCU，避免电机失控。

---

## 7. 代码在哪里写

本项目由 CubeMX 自动生成外设初始化代码，用户只能在特定区域编写自己的代码，否则 CubeMX 重新生成时会被覆盖。

### 7.1 USER CODE 区域规则

CubeMX 生成的每个 `.c` 文件中都有这种标记：

```c
/* USER CODE BEGIN X */
    // ← 这里写你的代码，CubeMX 不会动
/* USER CODE END X */
```

**红线：绝对不要在 `BEGIN / END` 之外写任何用户代码。**

### 7.2 各文件的 USER CODE 位置说明

以 `Core/Src/main.c` 为例，你实际能写代码的位置：

| 区域 | 位置（行号附近） | 用途 | 示例 |
|------|----------------|------|------|
| `USER CODE BEGIN Includes` | 文件顶部 | 添加 `#include` 头文件 | `#include "mecanum_motor.h"` |
| `USER CODE BEGIN PTD` | Includes 下方 | 自定义 `typedef` 类型 | `typedef struct { ... } MyType;` |
| `USER CODE BEGIN PD` | PTD 下方 | 自定义 `#define` 宏 | `#define MAX_SPEED 5000` |
| `USER CODE BEGIN PM` | PD 下方 | 自定义宏函数 | `#define ABS(x) ((x)>0?(x):-(x))` |
| `USER CODE BEGIN PV` | 变量区 | 定义全局/静态变量 | `int16_t g_target_speed[3];` |
| `USER CODE BEGIN PFP` | 函数声明区 | 声明自己的函数原型 | `static void MecanumControl_50Hz(void);` |
| `USER CODE BEGIN 0` | main 函数前 | 全局函数实现（很少用） | - |
| `USER CODE BEGIN 2` | 外设初始化之后 | 硬件初始化补充 | `Motor_Init(); Kinematics_Init();` |
| `USER CODE BEGIN 3` | `while(1)` 内 | **主循环逻辑（最常用）** | 控制、通信、状态机 |
| `USER CODE BEGIN 4` | main 函数之后 | **自定义函数实现（最常用）** | `MecanumControl_50Hz()` 完整实现 |

### 7.3 添加新模块文件的最佳实践

如果功能复杂，不要全写在 `main.c` 里，应该新建独立的 `.c/.h` 文件：

**步骤：**

1. 在 `Core/Inc/` 下新建头文件，如 `my_comm.h`：

```c
#ifndef __MY_COMM_H
#define __MY_COMM_H

#include "main.h"

/* 声明对外接口 */
void MyComm_Init(void);
void MyComm_Process(void);

#endif
```

2. 在 `Core/Src/` 下新建源文件，如 `my_comm.c`：

```c
#include "my_comm.h"

void MyComm_Init(void)
{
    /* 初始化代码 */
}

void MyComm_Process(void)
{
    /* 处理逻辑 */
}
```

3. 在 Keil 工程中添加文件：右键 Source Group 1 → Add Existing Files → 选中 `.c` 文件

4. 在 `main.c` 的 `USER CODE BEGIN Includes` 中 `#include "my_comm.h"`

5. 在 `USER CODE BEGIN 2` 中调用 `MyComm_Init()`

6. 在 `USER CODE BEGIN 3` 中调用 `MyComm_Process()`

**本项目现有的三个模块就是这样组织的：**

```
mecanum_motor.h / .c      → 电机驱动（GPIO + PWM 操作）
mecanum_pid.h / .c        → PID 控制（纯算法，无硬件依赖）
mecanum_kinematics.h / .c → 运动学（纯数学，无硬件依赖）
```

### 7.4 CubeMX 重新生成代码后的处理

当你用 CubeMX 修改引脚/时钟并重新生成代码后：

- `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间的代码 **会被保留**
- 其他区域会被 CubeMX 覆盖为新配置
- **你无需做任何额外操作**，用户代码自动恢复

所以养成习惯：**所有用户代码必须写在 USER CODE 区域内。**

---

## 8. 函数怎么用（完整示例）

本节通过实际场景，演示如何组合使用三个模块的函数。

### 8.1 函数调用关系总览

```
main() 初始化阶段:
  Motor_Init()          → 启动 PWM，电机静止
  Kinematics_Init()     → 计算运动学常量

while(1) 每 20ms:
  读编码器
  → Kinematics_Forward()  → 得到里程计
  → Kinematics_Inverse()  → 得到各轮目标脉冲
  → PID_MotorA/B/C/D()    → 得到各轮 PWM 值
  → Motor_A/B/C/D_SetSpeed() → 输出到电机
```

### 8.2 场景一：让小车直线前进

最简单的控制——只改 `g_target_speed`，其他全部自动：

```c
/* main.c → USER CODE BEGIN 2（初始化之后） */
g_target_speed[0] = -500;  // vx: 向前 500 mm/s
g_target_speed[1] = 0;     // vy: 不横移
g_target_speed[2] = 0;     // w:  不旋转
```

**解释：**
- `g_target_speed` 是外部全局变量，直接赋值即可
- `MecanumControl_50Hz()` 每 20ms 自动读取这个变量，完成逆运动学 → PID → 电机输出
- 修改值后下一个控制周期自动生效，**无需额外调用任何函数**

### 8.3 场景二：让小车走正方形

在 `while(1)` 中用一个简单的状态机切换目标速度：

```c
/* main.c → USER CODE BEGIN PV */
static uint32_t g_state_timer = 0;   // 状态计时器（ms）
static uint8_t  g_state = 0;         // 当前状态

/* main.c → USER CODE BEGIN 3（主循环内） */
uint32_t now = HAL_GetTick();

/* 每个状态持续 2 秒，循环切换 */
if (now - g_state_timer >= 2000) {
    g_state_timer = now;
    g_state = (g_state + 1) % 4;
}

switch (g_state) {
    case 0:  /* 向前走 */
        g_target_speed[0] = -500;
        g_target_speed[1] = 0;
        g_target_speed[2] = 0;
        break;
    case 1:  /* 右移 */
        g_target_speed[0] = 0;
        g_target_speed[1] = 500;
        g_target_speed[2] = 0;
        break;
    case 2:  /* 向后走 */
        g_target_speed[0] = 500;
        g_target_speed[1] = 0;
        g_target_speed[2] = 0;
        break;
    case 3:  /* 左移 */
        g_target_speed[0] = 0;
        g_target_speed[1] = -500;
        g_target_speed[2] = 0;
        break;
}
```

### 8.4 场景三：直接控制单个电机（绕过运动学）

调试阶段有时需要单独控制某一路电机，直接调用 `Motor_x_SetSpeed()`：

```c
/* main.c → USER CODE BEGIN 3（主循环内） */

/* 直接控制电机A正转，PWM=3000 */
Motor_A_SetSpeed(3000);

/* 电机B反转 */
Motor_B_SetSpeed(-2000);

/* 停止电机C */
Motor_C_SetSpeed(0);

/* 全部停止 */
Motor_SetAllStop();
```

**注意：** 直接调用 `Motor_x_SetSpeed()` 不经过 PID，相当于开环控制。正常运行时不应使用此方式。

### 8.5 场景四：单独使用 PID 模块（不经过运动学）

如果你有自己的目标值来源（如位置环输出），可以直接调用 PID：

```c
/* main.c → USER CODE BEGIN 3（主循环内） */

/* 假设你已经通过其他方式得到了各轮目标值 */
int32_t my_target_a = 25;   // 目标：每个周期走 25 个脉冲
int32_t my_actual_a = g_encoder_delta[0];  // 实际编码器增量

/* 调用 PID 计算 PWM */
int32_t pwm_a = PID_MotorA(my_target_a, my_actual_a);

/* 输出到电机 */
Motor_A_SetSpeed(pwm_a);
```

**PID 函数签名：**
```c
int32_t PID_MotorA(int32_t target, int32_t current);
// target:  本周期期望的编码器脉冲增量
// current: 本周期实际读到的编码器脉冲增量
// 返回值:  PWM 值（-11999 ~ +11999）
```

### 8.6 场景五：读取里程计判断当前位置

正运动学自动更新 `g_odom` 数组，直接读取即可：

```c
/* main.c → USER CODE BEGIN 3（主循环内） */

int16_t pos_x   = g_odom[0];  // X 位置 (mm)
int16_t pos_y   = g_odom[1];  // Y 位置 (mm)
int16_t yaw     = g_odom[2];  // 航向角 (mrad, ±3141 对应 ±π)
int16_t vel_x   = g_odom[3];  // X 速度 (mm/s)
int16_t vel_y   = g_odom[4];  // Y 速度 (mm/s)
int16_t vel_w   = g_odom[5];  // 角速度 (mrad/s)

/* 例：前进超过 500mm 后停止 */
if (pos_x < -500) {  // 注意极性，实际值取决于方向
    g_target_speed[0] = 0;
    g_target_speed[1] = 0;
    g_target_speed[2] = 0;
}
```

### 8.7 场景六：急停后重新启动

PID 内部有积分累积，急停后直接重启可能导致输出突变。正确做法：

```c
/* 急停 */
Motor_SetAllStop();
PID_ResetAll();                // ← 必须重置 PID 内部状态
g_target_speed[0] = 0;
g_target_speed[1] = 0;
g_target_speed[2] = 0;

/* 重新启动（例如 1 秒后） */
HAL_Delay(1000);
g_target_speed[0] = -500;     // 重新设定目标
```

### 8.8 场景七：串口接收指令控制（完整示例）

将串口接收集成到主循环，实现 PC 发指令控制小车：

```c
/* main.c → USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "usart.h"

/* main.c → USER CODE BEGIN PV */
#define RX_BUF_SIZE  64
static uint8_t  g_rx_buf[RX_BUF_SIZE];
static volatile uint8_t g_rx_flag = 0;
static uint16_t g_rx_len = 0;

/* main.c → USER CODE BEGIN 2（初始化阶段） */
/* 开启串口空闲中断 + DMA 接收 */
extern UART_HandleTypeDef huart1;
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_rx_buf, RX_BUF_SIZE);

/* main.c → USER CODE BEGIN 3（主循环内） */
if (g_rx_flag) {
    g_rx_flag = 0;
    /* 解析格式: "vx,vy,w\n"  例如 "-500,0,0\n" */
    int16_t vx, vy, w;
    if (sscanf((char*)g_rx_buf, "%hd,%hd,%hd", &vx, &vy, &w) == 3) {
        /* 限幅保护 */
        if (vx >  5000) vx =  5000;
        if (vx < -5000) vx = -5000;
        if (vy >  5000) vy =  5000;
        if (vy < -5000) vy = -5000;
        if (w >  5000)  w =  5000;
        if (w < -5000)  w = -5000;
        g_target_speed[0] = vx;
        g_target_speed[1] = vy;
        g_target_speed[2] = w;
    }
    /* 重新开启接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_rx_buf, RX_BUF_SIZE);
}
```

**注意：** 串口接收回调需要在 `stm32h7xx_it.c` 或用户文件中实现：

```c
/* 在 USER CODE 区域添加 DMA 空闲中断回调 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        g_rx_len = Size;
        g_rx_flag = 1;
    }
}
```

### 8.9 完整调用流程图（速查）

```
┌─────────────────────────────────────────────────────┐
│  main() 初始化                                      │
│                                                     │
│  HAL_Init() → SystemClock_Config() → 外设初始化     │
│       ↓                                             │
│  Motor_Init()         ← 启动 PWM，电机归零          │
│  Kinematics_Init()    ← 预计算运动学常量             │
│  g_target_speed[0] = -500  ← 设定初始速度            │
│       ↓                                             │
│  启动编码器定时器（TIM3/4/5/8），置中值 32768        │
└──────────────────────┬──────────────────────────────┘
                       ↓
┌─────────────────────────────────────────────────────┐
│  while(1) 主循环  ← 每 20ms 执行一次控制逻辑        │
│                                                     │
│  ① 读编码器增量（减中值法）                          │
│  ② 限幅过滤（ENC_MAX_DELTA = 150）                  │
│  ③ 累计编码器值                                     │
│  ④ Kinematics_Forward(g_encoder_acc, g_odom)        │
│  ⑤ Kinematics_Inverse(g_target_speed, g_target_enc) │
│  ⑥ PID_MotorA/B/C/D(g_target_enc, g_encoder_delta)  │
│  ⑦ Motor_A/B/C/D_SetSpeed(pwm)                      │
│                                                     │
│  同时：LED 翻转（500ms 心跳）                        │
└─────────────────────────────────────────────────────┘
```

---

## 10. API 速查表

### 10.1 运动学模块 (`mecanum_kinematics.h`)

```c
void Kinematics_Init(void);
// 初始化运动学参数，必须在使用前调用一次

void Kinematics_Inverse(int16_t *input, int16_t *output);
// 逆运动学：目标速度 → 各轮目标脉冲
// input[3]:  {vx(mm/s), vy(mm/s), w(mrad/s)}
// output[4]: 各轮每 PID 周期的期望编码器脉冲增量

void Kinematics_Forward(int16_t *input, int16_t *output);
// 正运动学：编码器累计值 → 里程计
// input[4]:  各轮编码器累计脉冲
// output[6]: {x(mm), y(mm), yaw(mrad), vx, vy, w}
```

### 10.2 电机驱动模块 (`mecanum_motor.h`)

```c
void Motor_Init(void);              // 初始化 PWM 通道，全部停止
void Motor_A_SetSpeed(int32_t speed); // 电机A：正值正转，负值反转，0停止
void Motor_B_SetSpeed(int32_t speed);
void Motor_C_SetSpeed(int32_t speed);
void Motor_D_SetSpeed(int32_t speed);
void Motor_SetAllStop(void);         // 紧急停止所有电机
```

### 10.3 PID 模块 (`mecanum_pid.h`)

```c
int32_t PID_MotorA(int32_t target, int32_t current);  // 电机A PID，返回 PWM 值
int32_t PID_MotorB(int32_t target, int32_t current);
int32_t PID_MotorC(int32_t target, int32_t current);
int32_t PID_MotorD(int32_t target, int32_t current);
void    PID_ResetAll(void);           // 重置所有 PID 状态（换方向/急停后调用）
```

### 10.4 全局变量

```c
int16_t g_target_speed[3];  // 目标速度 {vx, vy, w}，可直接写入
                              // 修改后下一周期自动生效

static int16_t g_odom[6];   // 里程计 {x, y, yaw, vx, vy, w}（只读）
                              // 由 MecanumControl_50Hz() 自动更新
```

---

## 11. 常见问题

### Q1：烧录后电机不转？
- 检查 `TEST_OPEN_LOOP` 是否为 1（开环测试模式）
- 检查 `g_target_speed[0]` 是否为非零值
- 检查电机驱动板供电是否独立且与 MCU 共地
- 用万用表测量 PWM 引脚是否有波形

### Q2：电机乱转 / 震荡？
- 检查编码器 A/B 相是否接反
- 检查 `MOTOR_x_POLARITY` 极性是否正确
- 降低 KP 值，增大 KD 值

### Q3：某个电机完全不动？
- 检查该路编码器信号——用示波器看是否有脉冲
- 检查该路 PWM 引脚——进入开环模式看是否有波形
- 检查该路方向引脚——GPIO 是否正常输出

### Q4：想修改引脚怎么办？
- 用 CubeMX 打开 `ymq_2.ioc`
- 修改引脚分配
- 重新生成代码（**注意：会覆盖 Core/Src/tim.c、gpio.c 等自动生成文件**）
- 用户代码在 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间，CubeMX 不会覆盖

---

## 12. 已知限制

1. **TIM5 编码器计数器为 32 位**，但当前统一按 16 位处理，长时间运行可能溢出
2. **目标速度硬编码**在 `main.c` 中，尚无外部输入接口
3. **无硬件看门狗**，主循环卡死时电机将失控
4. **PID 参数全部为宏定义**，无法运行时在线修改
5. **无速度限幅保护**，`g_target_speed` 赋值过大时无保护

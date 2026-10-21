# OpenCTR H60 V3.2 固件（PlatformIO）

本工程使用 PlatformIO 构建 STM32F407VET6 固件。当前固件源码在 `src/`，头文件在 `include/`；FreeRTOS、SPL 和 CMSIS 所需源码也已包含在这两个目录中。旧 Keil 工程及其重复的源码目录已移除。

## 如何打开

1. 在 VS Code 选择“文件 → 打开文件夹”，选择**本文件夹** `OpenCTR_H60V32_4W_FW_V1.00.231009_copy`。
2. 安装或启用 PlatformIO IDE 扩展。根目录的 `platformio.ini` 会使它识别为 PlatformIO 工程。
3. 点击底部状态栏 **✓ Build** 编译。ST-LINK 连接好且决定烧录时，再点击 **→ Upload**。

在 Linux 终端进入本仓库目录后：

```bash
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload
```

`pio run -t upload` 会改写单片机 Flash。编译产物在 `.pio/build/openctr_h60_v32/`。当前 `src/` 和 `include/` 中的中文注释已统一为 UTF-8，VS Code 可直接阅读。本地保留的 `四轮小车/` 厂商资料包不属于当前固件源码，不随源码仓库发布；其中的原始示例可能使用 GBK。

## 哪里修改功能

| 目录 | 功能 |
| --- | --- |
| `src/app/` | 初始化、运行任务、状态与电池保护 |
| `src/chassis/` | 四轮电机、编码器、速度控制、运动学 |
| `src/control/` | 遥控输入处理 |
| `src/communication/` | UART、CAN、PS2、SBUS |
| `src/sensors/` | MPU6050、电池电压采集、XURSC 测距驱动 |
| `src/actuators/` | 舵机驱动和 PB8/SG90 控制 |
| `src/ui/` | OLED、RGB、LED、按键、拨码、蜂鸣器 |
| `src/system/`、`src/platform/` | 延时、时钟、中断与启动 |
| `src/rtos/` | FreeRTOS 9.0.0 内核与 GCC Cortex-M4F 移植层 |
| `src/vendor/spl/`、`include/vendor/` | SPL、CMSIS 及其他随工程提供的依赖 |

## STM32F407VET6 引脚使用表

下表按 **PA → PB → PC → PD → PE**、同一端口内按编号从小到大排列。硬件连接以本地厂商资料包 `四轮小车/4 OpenCTR控制器资料/OpenCTR_H60_原理图_V3.2.pdf` 的 **STM32F407** 部分为准；该资料包不随源码仓库发布。外设功能和当前启用状态再用实际参与 PlatformIO 构建的 `src/` 代码及 `src/app/main.c` 初始化流程核对。表中的“固件已配置”只表示 STM32 会初始化该引脚；如果它只通向扩展排针，还需要接上相应扩展板，才能使用所写的外设功能。未初始化的引脚会在用途栏中明确标注。

### GPIOA

| 引脚 | STM32 外设功能/方向 | 使用位置 | 实际用途 |
| --- | --- | --- | --- |
| PA0 | `TIM5_CH1`，复用输入 | 编码器 C | 采集 C 电机编码器 A 相 |
| PA1 | `TIM5_CH2`，复用输入 | 编码器 C | 采集 C 电机编码器 B 相 |
| PA2 | GPIO 高电平输出 | 扩展接口 2 `EXP2`/单基站 RX | 当前不发送数据，保持已连接基站接收线的空闲电平 |
| PA3 | `USART2_RX`，复用输入 | 扩展接口 2 `EXP2`/单基站 TX | 以 115200 波特率接收单基站串口数据 |
| PA4 | GPIO 推挽输出 | OLED（经扩展接口 1 `EXP1`） | D/C：数据/命令选择；主程序启动时会初始化 |
| PA5 | `SPI1_SCK`，复用输出 | OLED（经扩展接口 1 `EXP1`） | SCK：SPI 时钟；主程序启动时会初始化 |
| PA6 | GPIO 推挽输出 | OLED（经扩展接口 1 `EXP1`） | RES/RST：屏幕复位；主程序启动时会初始化 |
| PA7 | `SPI1_MOSI`，复用输出 | OLED（经扩展接口 1 `EXP1`） | DIN/MOSI：向屏幕发送数据；主程序启动时会初始化 |
| PA9 | `USART1_TX`，复用输出 | USB-DEBUG/调试串口 | 230400 波特率调试串口发送，同时作为 `printf` 输出口 |
| PA10 | `USART1_RX`，复用输入 | USB-DEBUG/调试串口 | 接收调试串口协议数据 |
| PA11 | GPIO/复用功能预留 | 扩展接口 2 `EXP2` | 已连接到扩展排针，当前固件未初始化 |
| PA12 | GPIO/复用功能预留 | 扩展接口 2 `EXP2` | 已连接到扩展排针，当前固件未初始化 |
| PA13 | `SWDIO`，系统调试接口 | ST-LINK | 程序下载和在线调试；不是业务代码初始化的普通 GPIO |
| PA14 | `SWCLK`，系统调试接口 | ST-LINK | 程序下载和在线调试时钟；不是业务代码初始化的普通 GPIO |
| PA15 | `TIM2_CH1`，复用输入 | 编码器 A | 采集 A 电机编码器 A 相 |

**蓝牙屏幕扩展板一共使用 8 路连接：PA4、PA5、PA6、PA7、PC10、PC11、3.3V、GND。**四轮小车用户手册第 12～13 页画的是整块扩展板，其中 OLED 使用四根信号线及电源，蓝牙模块使用另外两根串口信号。按当前固件和控制器原理图，可以确认以下用途：

| 扩展板信号 | 控制器连接 | 用途 |
| --- | --- | --- |
| OLED D/C | PA4（`EXP1`） | 区分显示命令和数据，GPIO 输出 |
| OLED SCK | PA5（`EXP1`） | SPI1 时钟输出 |
| OLED RES/RST | PA6（`EXP1`） | 屏幕复位，GPIO 输出 |
| OLED DIN/MOSI | PA7（`EXP1`） | SPI1 显示数据输出 |
| 蓝牙 RX | PC10（`EXP1`，UART4_TX） | 与 OLED 同在扩展板上，但属于蓝牙模块 |
| 蓝牙 TX | PC11（`EXP1`，UART4_RX） | 与 OLED 同在扩展板上，但属于蓝牙模块 |
| 电源 3.3V | `EXP1` 的 3.3V | 给 OLED 和蓝牙模块供电，不是 STM32 GPIO |
| 电源 GND | `EXP1` 的 GND | OLED 和蓝牙模块共地，不是 STM32 GPIO |

`src/app/main.c` 启动时会调用 `AX_OLED_Init()`，所以 PA4～PA7 在当前固件中均已配置。SPI1 采用内部软件 NSS，但程序没有控制外部 OLED CS 的 GPIO；若单独接裸屏，还要确认该屏的 CS 接法。**资料存在引脚标注矛盾**：用户手册第 13 页的扩展板图将 3.3V 一行对到 PA7、OLED D/C 一行对到 PA3，与控制器原理图的 `EXP1` 供电/引脚以及当前 OLED 代码不一致。上表记录的是当前代码预期的连接，实际接线前需要核对所用扩展板的引脚或原理图。USART2_RX 则是 PA3，不属于当前代码的 OLED 信号。

### GPIOB

| 引脚 | STM32 外设功能/方向 | 使用位置 | 实际用途 |
| --- | --- | --- | --- |
| PB0 | GPIO 上拉输入 | 拨码开关 S1 | 选择麦轮底盘或四轮差速底盘 |
| PB1 | GPIO 上拉输入 | 拨码开关 S2 | 读取第二位拨码状态 |
| PB2 | `BOOT1`，启动配置输入 | 启动模式配置 | 原理图通过 10 kΩ 电阻下拉，正常从主 Flash 启动 |
| PB3 | `TIM2_CH2`，复用输入 | 编码器 A | 采集 A 电机编码器 B 相 |
| PB4 | `TIM3_CH1`，复用输入 | 编码器 B | 采集 B 电机编码器 A 相 |
| PB5 | `TIM3_CH2`，复用输入 | 编码器 B | 采集 B 电机编码器 B 相 |
| PB6 | GPIO 推挽输出，软件 I²C | MPU6050 | 软件 I²C 时钟 `SCL` |
| PB7 | GPIO 双向，软件 I²C | MPU6050 | 软件 I²C 数据 `SDA`，程序会在输入、输出模式之间切换 |
| PB8 | 启动时 GPIO 低电平；收到角度指令后 `TIM10_CH1` PWM | 舵机接口 S1 / SG90 | SG90 橙线接此脚，红线接 5 V，棕线接 GND |
| PB9 | 当前未初始化 | 舵机接口 S2 | 原机械臂功能关闭，当前不输出舵机脉冲 |
| PB10 | `USART3_TX` 可选复用 | 扩展接口 1 `EXP1` | 原理图标为 `UART3_TX`，当前固件未配置；当前 USART3 实际使用 PD8/PD9 |
| PB11 | `USART3_RX` 可选复用 | 扩展接口 1 `EXP1` | 原理图标为 `UART3_RX`，当前固件未配置；当前 USART3 实际使用 PD8/PD9 |
| PB12 | `CAN2_RX` 可选复用 | 扩展接口 1 `EXP1` | 已引出备用 CAN2 接收脚，当前固件未初始化 CAN2 |
| PB13 | `CAN2_TX` 可选复用 | 扩展接口 1 `EXP1` | 已引出备用 CAN2 发送脚，当前固件未初始化 CAN2 |
| PB14 | `TIM12_CH1`，PWM 输出 | 电机 D 驱动 | D 电机 H 桥第一路 20 kHz PWM |
| PB15 | `TIM12_CH2`，PWM 输出 | 电机 D 驱动 | D 电机 H 桥第二路 20 kHz PWM；与 PB14 配合控制方向和速度 |

### GPIOC

| 引脚 | STM32 外设功能/方向 | 使用位置 | 实际用途 |
| --- | --- | --- | --- |
| PC0 | `ADC2_IN10`，模拟输入 | 电池电压检测 | 采集经 1:11 分压后的 VIN；源码旧注释写成 PB1，但实际代码配置的是 PC0 |
| PC2 | `ADC_IN12` 可选模拟输入 | 扩展接口 1 `EXP1` | 已引出模拟采集通道，当前固件未初始化 |
| PC3 | `ADC_IN13` 可选模拟输入 | 扩展接口 1 `EXP1` | 已引出模拟采集通道，当前固件未初始化 |
| PC4 | `ADC_IN14` 可选模拟输入 | 扩展接口 1 `EXP1` | 已引出模拟采集通道，当前固件未初始化 |
| PC5 | `ADC_IN15` 可选模拟输入 | 扩展接口 1 `EXP1` | 已引出模拟采集通道，当前固件未初始化 |
| PC6 | `TIM8_CH1`，输入捕获 | 舵机接口 S3 / XURSC ECHO | 接 XURSC 回波；不再输出舵机 PWM |
| PC7 | GPIO 推挽输出 | 舵机接口 S4 / XURSC TRIG | 向 XURSC 发出测距触发脉冲 |
| PC8 | 当前未初始化 | 舵机接口 S5 | 原机械臂功能关闭 |
| PC9 | 当前未初始化 | 舵机接口 S6 | 原机械臂功能关闭 |
| PC10 | `UART4_TX`，复用输出 | `EXP1` 蓝牙屏幕扩展板的蓝牙模块 | 以 115200 波特率向蓝牙模块 RX 发送数据，不是 OLED 信号 |
| PC11 | `UART4_RX`，复用输入 | `EXP1` 蓝牙屏幕扩展板的蓝牙模块 | 接收蓝牙模块 TX 的手机 APP 控制帧；当前 `OPENCTR_ENABLE_APP_CONTROL=1`，不是 OLED 信号 |
| PC12 | `SPI3_MOSI`，单线输出 | RGB 彩灯 | 发送 RGB 灯珠颜色与灯效数据 |

### GPIOD

| 引脚 | STM32 外设功能/方向 | 使用位置 | 实际用途 |
| --- | --- | --- | --- |
| PD0 | `CAN1_RX`，复用输入 | CAN 接口 | CAN 总线接收脚；CAN 外设会初始化，但当前控制接收中断关闭 |
| PD1 | `CAN1_TX`，复用输出 | CAN 接口 | 发送底盘遥测 CAN 帧，波特率 1 Mbit/s |
| PD2 | `UART5_RX` 可选复用输入 | 扩展接口 1/SBUS | SBUS 接收脚；`OPENCTR_ENABLE_SBUS_CONTROL=0`，当前启动时不初始化 |
| PD3 | GPIO 推挽输出，低电平点亮 | 板载红色 LED | 红色状态指示灯 |
| PD4 | GPIO 推挽输出，低电平点亮 | 板载绿色 LED | 绿色状态指示灯 |
| PD5 | `USART2_TX` 可选复用 | 扩展接口 1 `EXP1` | 原理图标为 `UART2_TX`，当前固件未配置；单基站仅使用 PA3 接收 |
| PD6 | `USART2_RX` 可选复用 | 扩展接口 1 `EXP1` | 原理图标为 `UART2_RX`，当前固件未配置；单基站仅使用 PA3 接收 |
| PD7 | GPIO/外部中断输入 | MPU6050 | 板上已连接 MPU6050 的 `INT`；当前固件采用轮询读取，未初始化该中断脚 |
| PD8 | `USART3_TX`，复用输出 | USB-COM 通信口 | 以 230400 波特率发送底盘遥测数据 |
| PD9 | `USART3_RX`，复用输入 | USB-COM 通信口 | 接收电脑或上位机发送的速度控制帧 |
| PD12 | `TIM4_CH1`，复用输入 | 编码器 D | 采集 D 电机编码器 A 相 |
| PD13 | `TIM4_CH2`，复用输入 | 编码器 D | 采集 D 电机编码器 B 相 |
| PD14 | GPIO/复用功能预留 | 扩展接口 2 `EXP2` | 已连接到扩展排针，当前固件未初始化 |
| PD15 | GPIO/复用功能预留 | 扩展接口 2 `EXP2` | 已连接到扩展排针，当前固件未初始化 |

### GPIOE

| 引脚 | STM32 外设功能/方向 | 使用位置 | 实际用途 |
| --- | --- | --- | --- |
| PE0 | GPIO 上拉输入 | 板载按键 KEY | 低电平表示按键按下 |
| PE1 | GPIO 推挽输出，高电平鸣叫 | 蜂鸣器 | 控制蜂鸣器开、关和翻转 |
| PE2 | GPIO 输出，软件 I²C | 板载 EEPROM | EEPROM 时钟 `SCL`；原厂旧驱动中有实现，当前 PlatformIO 业务源码未初始化 |
| PE3 | GPIO 双向，软件 I²C | 板载 EEPROM | EEPROM 数据 `SDA`；原厂旧驱动中有实现，当前 PlatformIO 业务源码未初始化 |
| PE5 | `TIM9_CH1`，PWM 输出 | 电机 C 驱动 | C 电机 H 桥第一路 20 kHz PWM |
| PE6 | `TIM9_CH2`，PWM 输出 | 电机 C 驱动 | C 电机 H 桥第二路 20 kHz PWM；与 PE5 配合控制方向和速度 |
| PE7 | GPIO 推挽输出 | PS2 手柄接收器 | PS2 通信时钟 `CLK` |
| PE8 | GPIO 推挽输出 | PS2 手柄接收器 | PS2 片选/触发信号 `CS/ATT` |
| PE9 | `TIM1_CH1`，PWM 输出 | 电机 A 驱动 | A 电机 H 桥第一路 20 kHz PWM |
| PE10 | GPIO 推挽输出 | PS2 手柄接收器 | PS2 命令信号 `CMD` |
| PE11 | `TIM1_CH2`，PWM 输出 | 电机 A 驱动 | A 电机 H 桥第二路 20 kHz PWM；与 PE9 配合控制方向和速度 |
| PE12 | GPIO 下拉输入 | PS2 手柄接收器 | PS2 数据输入 `DATA/DI`；当前 `OPENCTR_ENABLE_PS2_CONTROL=1` |
| PE13 | `TIM1_CH3`，PWM 输出 | 电机 B 驱动 | B 电机 H 桥第一路 20 kHz PWM |
| PE14 | `TIM1_CH4`，PWM 输出 | 电机 B 驱动 | B 电机 H 桥第二路 20 kHz PWM；与 PE13 配合控制方向和速度 |

按原理图，PA～PE 中未列出的引脚为 PA8、PC1、PC13～PC15、PD10～PD11、PE4、PE15：这些脚既没有连接到板载功能或扩展排针，也没有被当前业务代码初始化。`BOOT0` 和 `NRST` 是专用系统引脚，不属于 PA～PE；8 MHz 外部晶振使用的 `PH0/OSC_IN`、`PH1/OSC_OUT` 也不属于本表要求的 PA～PE 范围。

`platformio.ini` 指定 STM32F407VET6、ST-LINK、8 MHz 外部晶振、原 SPL 编译宏及硬浮点参数。`ld/stm32f407ve_flash.ld` 是 GCC 链接脚本。FreeRTOS 内核和原业务功能保持原版本；Keil 专用的 ARMCC 移植层由同版本的 [FreeRTOS 官方 GCC Cortex-M4F 移植层](https://github.com/FreeRTOS/FreeRTOS/tree/V9.0.0/FreeRTOS/Source/portable/GCC/ARM_CM4F)替换。

四轮行驶、麦轮/差速选择、编码器速度控制、USB-COM、MPU6050、OLED 和电池保护均在新构建中。手机 APP 蓝牙控制和 PS2 手柄控制已恢复；USB-COM 控制仍可使用。SG90 和 XURSC 使用独立模块，原机械臂功能关闭。XURSC 在 SCAN 或 USB-COM 手动测距时回传数据；PS2 FOLLOW 只使用 AOA，不读取超声波或控制舵机。单基站 USART2 接收诊断已配置，原 TTL 控制协议、SBUS 和 CAN 的控制接收入口仍关闭。

**编译和烧录成功不等于实车功能验证。**本次恢复 APP 和 PS2、补齐电机定时器初始化字段的版本已通过 PlatformIO/ST-LINK 烧录，OpenOCD 报告 `Verified OK`；手机、手柄和前右轮运动状态尚未完成实车验证。若仍有单个轮子持续转动，请先断电并检查该轮电机与编码器接线。

## 控制入口：手机 APP、PS2 手柄和 USB-COM

手机 APP 通信代码在 `src/communication/ax_uart4.c` 和 `src/control/ax_control.c`：UART4 接收 APP 控制帧，`AX_CTL_App()` 将其转成速度目标。PS2 手柄代码在 `src/communication/ax_ps2.c`、`src/app/ax_robot.c` 的 `Ps2_Task()` 和 `src/control/ax_control.c` 的 `AX_CTL_Ps2()`。PS2 接收器通过 GPIO 与主控通信，并不通过 APP 的 UART4 蓝牙链路。

`platformio.ini` 中 `OPENCTR_ENABLE_APP_CONTROL=1` 会初始化 UART4（115200 波特率）并恢复 APP 控制处理；收到有效 APP 模式切换帧后进入 APP 模式。`OPENCTR_ENABLE_PS2_CONTROL=1` 会初始化 PS2 接口、创建手柄读取任务并恢复 PS2 控制处理；原厂代码通过手柄 START 键及左摇杆动作切入 PS2 模式。PS2 接收器通过 GPIO 与主控通信，不走 APP 的 UART4 蓝牙链路。

`OPENCTR_ENABLE_SBUS_CONTROL=0` 仍关闭 SBUS 接收；`OPENCTR_ENABLE_TTL_CONTROL=0` 关闭原 TTL 控制协议，USART2 已改为单基站接收诊断；`OPENCTR_ENABLE_CAN_CONTROL=0` 关闭 CAN 接收中断，保留 CAN 遥测发送。USB-COM 的 USART3 仍可接收速度指令，并可将控制模式切回 ROS。恢复固件入口并不等于已经验证手机配对、手柄接收器连接或电机运动。

PS2 控制来源内有四个子模式：手动驾驶、自动跟随、避障跟随和手动环视测距。按原有方式用 START 加左摇杆上推进入 PS2 后，默认手动驾驶；每次按下再松开 SELECT，按 `DRIVE → FOLLOW → FOLLOW_AVOID → SCAN → DRIVE` 循环切换，OLED 第 4 行显示当前模式。切换时底盘速度目标清零。离开 PS2 控制来源后，子模式重置为手动驾驶。手柄扫描超过 200 ms 未更新时按无效输入停车。

自动跟随只在 PS2 的 FOLLOW 子模式运行，控制规则在 `src/control/ax_follow_policy.c`。AOA 定位超过 300 ms 未更新即停车。跟随只使用报文中的方位角和距离，不使用仰角，也不对距离做水平投影。距离为 0 cm 或超过 300 cm 时停车。先判断方位角：偏差达到 ±12° 时原地转向，回到 ±5° 内后再按距离控制前进与停车；较大角度不再因超过 ±60° 而直接停车。当前报文解析和跟随策略都没有前后判定；目标在车后时若仍收到有效角度与距离，也会进入跟随决策。

避障跟随在 PS2 的 FOLLOW_AVOID 子模式运行，沿用 AOA 跟随目标，并使用 SG90 转动 XURSC 测量前方。底盘拨码开关 S1 需要选择四轮差速；其他底盘设置下该模式保持停车。在 `TRACK` 且仍需接近目标时，舵机从 0° 向 +15°、再向 -15°、再返回 0°，每隔 5° 测一次，持续往返，换角等待 60 ms；换角和等回波时继续按 AOA 正常跟随，任一方向测得低于 350 mm 就停车并进入侧扫。目标已足够近时舵机保持 0°。若测距超过 250 ms 没有更新，则停车回中，等待回中后新触发的前方测距。侧扫时舵机从 +90° 到 -90° 每隔 15° 测一次并记录 13 个方向的距离；在测距有效且不小于 800 mm 的方向中，选距离最小者，距离相同时选转角较小者。若选中 0°，先回中复测前方；选中其他方向则先把舵机回中，再让底盘转过对应角度，等角速度降至 100 mrad/s 以下，并用停稳后触发的新测距确认前方至少 650 mm，才分段前进；转角超出目标 100 mrad 时停车重扫。全程侧扫期间底盘停车；没有合格方向或 8000 ms 内未完成扫描时，舵机回中、停车等待 2000 ms 后重试。转角后的新测距仍受阻，或完成一段前进后前方不足 650 mm 时，停车重新扫描。绕障转向前的回中、完整侧扫与手动扫描换角后仍等待 250 ms 再测。绕行累计距离和累计时间的配置值均为 0，表示可反复绕行，没有这两项总量上限；单次转向超时、扫描超时和前进卡住会停车回中并按恢复流程处理。绕行速度不超过 100 mm/s，每段最多 150 mm；AOA、测距或运动估计无效时停车。到达目标方向后，需要两条不同的清晰前向测距样本才恢复跟随。该模式只前进和原地转向，不能检查车尾或覆盖整条通道，适用于宽敞室内和静态小障碍，实车阈值需按制动距离校准。

避障跟随只在仍需接近信标时启动或继续绕行：AOA 距离达到 110 cm 后认为需要接近，缩短到 100 cm 后停止接近，连续有效定位的 100～110 cm 区间保持原状态；AOA 失效时清除接近需求。信标已足够近时，即使前方有近物，也不会左右扫描或启动绕行，舵机保持正前方；前方通畅时仍可按跟随策略原地转向。绕行途中达到目标距离会立即取消绕行并回到前向等待。是否需要接近按 AOA 距离判断，不能用当前前进速度判断，因为对准信标的原地转向阶段前进速度同样为零。

绕行后的 `TRY_REJOIN` 会根据最新 AOA 方位角持续原地转向，直到偏差进入 ±12°；原地转向期间不以超声波读数决定是否继续转，也不因累计转角超过 90°、用时超过 5 秒或单次控制循环稍慢而锁定停车。对准后才检查正前方测距，并须取得两次不同的、至少 650 mm 的前向测距样本才恢复 `TRACK`；对准后测距无效则停车等待，前方受阻则停车重新扫描。AOA 无效时停车。`TURN_OUT` 的单次转向超时仍保留。

避障参数集中在 `include/config/openctr_avoid_config.h`。调试超声波对障碍物的反应，先看这几项（距离单位 mm，时间单位 ms）：

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `OPENCTR_AVOID_FRONT_STOP_MM` | 350 | TRACK 任一探测方向低于此值时停车并侧扫；调大可以更早避障。 |
| `OPENCTR_AVOID_FRONT_RELEASE_MM` | 650 | 正前方达到此值才允许恢复前进；必须大于停车距离。 |
| `OPENCTR_AVOID_SIDE_MIN_MM` | 800 | 侧扫读数达到此值才把该方向当作可绕行候选。 |
| `OPENCTR_AVOID_PAN_SETTLE_MS` | 250 | 完整侧扫、回中及手动扫描的换角等待时间。 |
| `OPENCTR_AVOID_TRACK_PROBE_ANGLE_DEG` | 15 | TRACK 往返探测的两侧端点角度。 |
| `OPENCTR_AVOID_TRACK_PROBE_STEP_DEG` | 5 | TRACK 每次换角和测距的角度间隔。 |
| `OPENCTR_AVOID_TRACK_PROBE_SETTLE_MS` | 60 | TRACK 中每次 5° 换角的等待时间；需实车确认读数稳定。 |
| `OPENCTR_AVOID_ULTRASONIC_INTERVAL_MS` | 100 | 两次超声波触发的最短间隔；底层还要求至少 60 ms。 |
| `OPENCTR_AVOID_RESCAN_WAIT_MS` | 2000 | 未找到合格方向时停车等待多久再试。 |

这些是编译时参数，修改后要重新编译、烧录。测距年龄上限、扫描超时、转向和前进速度也在同一配置文件。距离阈值只决定何时触发和放行，实际反应还受测量周期、舵机换角时间及车速影响。

FOLLOW_AVOID 运行时通过 USB-COM 的 `0x16` 帧报告当前状态、停车/等待原因、AOA 与超声波有效性及数值、舵机角度、目标与实测前进/转向速度、阶段计时和绕行进度。状态或原因变化时立即发送，状态不变时每 200 ms 发送一次。运行 `python3 tools/range_pan_serial.py --port /dev/ttyACM0 --seconds 30` 可读出文字诊断；`RESCAN_WAIT / WAIT_RESCAN` 表示停车等待下一轮扫描。转向超时或前进卡住时显示 `WAIT_FRONT / TURN_TIMEOUT` 或 `WAIT_FRONT / ADVANCE_STALL`，车辆回中等待故障后新触发的前方测距，再重新扫描；`HOLD` 仅保留给配置错误等无法靠重新观测修复的故障。若没有 `0x16` 帧，先确认 OLED 显示 `Mode: AVOID` 和 USB-COM 连接。

角度对正后，原 FOLLOW 策略在距离小于 80 cm 时计算倒退目标，达到 95 cm 时停止；距离达到 110 cm 时计算前进目标，前进后在 100 cm 处停止。当前 FOLLOW 在速度提交前禁止自动倒退，前进最高限制为 300 mm/s；这两项限制不依赖超声波。转向目标速度按方位角绝对值以 50 mrad/s 每度增加，限制在 300～2000 mrad/s。车轮编码器速度仍由现有 PID 闭环控制。控制器不会横移。

距离阈值、角度阈值、速度上下限、比例增益和底盘 X 轴限速集中在 `include/config/openctr_follow_config.h` 配置；底盘 X 轴限速适用于所有控制来源。已按当前安装测得的“信标在车头右侧时方位角为负”将 `platformio.ini` 中 `OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN` 设为 `1`：正角左转，负角右转。若改变基站安装方向，需要重新确认这个符号；设为 `0` 时 FOLLOW 始终停车。

SCAN 模式将底盘目标速度和电机 PWM 都置零，并清除麦轮速度 PID 的旧累计输出。进入时 SG90 指向 0°；手柄 X 每按一次减 15°，B 每按一次加 15°，范围为 -90° 到 +90°。按住不会连续转，两键同时按也不会转；每次重新进入 SCAN 都从 0° 开始。XURSC 持续测量当前方向，转角后等待 250 ms 再测；USB-COM 的 `0x15` 帧包含有符号角度（2 字节）、测距状态（1 字节）和距离毫米（2 字节）。运行 `python3 tools/range_pan_serial.py --port /dev/ttyACM0 --seconds 20` 可查看测距结果。退出 SCAN 后停止主动测距并释放舵机 PWM。USB-COM 独立角度指令只在 PS2 控制来源以外可用，SCAN 和 FOLLOW 都不会继承手动测距状态；FOLLOW_AVOID 则由避障控制器独占舵机和测距。

机械臂输入处理在 `src/control/ax_legacy_arm_input.c`，关节数据和舵机输出在 `src/control/ax_legacy_arm.c`；默认配置关闭旧机械臂。`src/control/ax_obstacle_guard.c` 保留为未接入控制模式的纯决策模块。FOLLOW 只使用 AOA，不使用超声波或舵机；FOLLOW_AVOID 使用两者绕过有限的静态障碍；SCAN 只采集距离，不控制底盘绕障。实车行驶时应预留空间并保持可随时停车。

控制来源切换时会清除上一来源的目标速度并停止电机输出；各来源的三轴速度会原子提交。运动学使用目标快照及版本号，若计算期间收到新指令，会丢弃旧轮速结果。

## 单基站串口接收诊断

`OPENCTR_ENABLE_AOA_RECEIVER=1` 时，EXP2/J5 上的 USART2 专用于单基站：J5-5/PA3 接基站 TX，J5-6/PA2 接基站 RX，J5-3 或 4 接转接板 VCC，J5-1 或 2 接 GND。USART2 以 115200、8N1 接收；PA2 保持高电平，不发送原底盘遥测。原有底盘遥测仍从 USB-COM 发送。

单片机的 AOA 模块分为接收和协议两层：`src/communication/ax_uart2.c` 接收串口字节，`src/communication/ax_aoa_protocol.c` 校验并解包 `0x2001` 定位报文，`src/communication/ax_aoa.c` 保存最新距离、方位角、仰角、基站/标签 ID 和接收时间。任务代码可调用 `AX_AOA_GetLatest(&sample, max_age_ms)`；超过指定时间没有新定位帧就返回无效。协议书中的版本号与实测报文不同，所以解包不按版本号过滤，只校验长度、命令和异或值。`0x2002` 离线心跳不更新定位，已有数据会按时间过期。

为方便诊断，模块仍通过 USB-COM 转发原始字节（帧号 `0x13`），并每约 1 秒报告累计接收字节数及缓冲溢出数（帧号 `0x14`）。烧录后可用 `python3 tools/aoa_serial.py --port /dev/ttyACM0 --seconds 8` 查看。若接收字节数一直为 0，检查转接板供电、基站 TX 到 PA3 的连线、信标是否在线。

协议解包的独立 C 测试在 `tests/test_aoa_protocol.c`。编译和运行该测试可用：`cc -std=c99 -Wall -Wextra -Isrc/communication tests/test_aoa_protocol.c src/communication/ax_aoa_protocol.c -o /tmp/test_aoa_protocol && /tmp/test_aoa_protocol`。

跟随决策的独立 C 测试在 `tests/test_follow_policy.c`。编译和运行可用：`cc -std=c99 -Wall -Wextra -Isrc/control tests/test_follow_policy.c src/control/ax_follow_policy.c -o /tmp/test_follow_policy && /tmp/test_follow_policy`。

避障跟随决策测试在 `tests/test_follow_avoid_policy.c` 和 `tests/test_follow_avoid_sweep.c`；可分别用 `cc -std=c99 -Wall -Wextra -Werror -Isrc/control -Isrc/sensors -Iinclude/config tests/test_follow_avoid_policy.c src/control/ax_follow_avoid_policy.c -o /tmp/test_follow_avoid_policy && /tmp/test_follow_avoid_policy` 与 `cc -std=c99 -Wall -Wextra -Werror -Isrc/control -Isrc/sensors -Iinclude/config tests/test_follow_avoid_sweep.c src/control/ax_follow_avoid_policy.c -o /tmp/test_follow_avoid_sweep && /tmp/test_follow_avoid_sweep` 编译运行。

跟随配置与速度下发的 C 测试在 `tests/test_follow_step.c`。编译和运行可用：`cc -std=c99 -Wall -Wextra -Werror -DOPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN=1 -Itests/follow_stubs -Isrc/app -Isrc/communication -Isrc/control -Iinclude/config tests/test_follow_step.c src/control/ax_follow.c src/control/ax_follow_policy.c -o /tmp/test_follow_step && /tmp/test_follow_step`。独立防撞决策模块的测试在 `tests/test_obstacle_guard.c`；模式循环测试在 `tests/test_ps2_mode.c`；SCAN 的手动 15° 步进、测距与退出行为测试在 `tests/test_range_pan.c`；零速度时 PID 清零测试在 `tests/test_speed_stop.c`。

需要持续查看基站测量结果时，运行 `python3 tools/aoa_distance_elevation.py --port /dev/ttyACM0`，按 Ctrl+C 退出。脚本只显示校验正确的 `0x2001` 定位帧中的距离（cm）、方位角（度）、左右角度文字和仰角（度）；当前安装下正角标“左侧角度”、负角标“右侧角度”、0° 标“左右居中”，均提示“前后未知”。目标在正后方或斜后方时，±90° 内的角度不证明它在车前。没有定位帧时会继续等待。若实际 USB-COM 端口不是 `/dev/ttyACM0`，将参数改为对应设备名。

USB-COM 接口仍支持速度指令和遥测。该接口按帧格式接收指令，并不识别电脑身份；接入该口的其他设备若发送同样的有效帧，也能控制底盘。当前固件没有命令超时自动停车机制，实车行驶前应在 STM32 侧加入超时清零目标速度的保护。

## 用电脑串口发送前进、后退、停车指令

`tools/drive_serial.py` 通过板子的 **USB-COM 通信口**发送 USART3 速度帧；ST-LINK 仅用于烧录和调试，USB-DEBUG 也不是这个脚本使用的通信口。串口参数为 230400 波特率。前进或后退前，脚本必须先收到一帧校验正确的 `AA 55 19 10` 遥测；单独发送停车指令无须等待遥测。

先安装 Python 串口库（如系统尚未安装）：`python3 -m pip install pyserial`。在本文件夹的终端执行：

```bash
python3 tools/drive_serial.py forward --dry-run
python3 tools/drive_serial.py forward --port /dev/ttyACM0
python3 tools/drive_serial.py backward --port /dev/ttyACM0
python3 tools/drive_serial.py stop --port /dev/ttyACM0
```

`/dev/ttyACM0` 只是示例；可用 `python3 -m serial.tools.list_ports -v` 找到实际端口。前进、后退默认以 100 mm/s 发送 0.3 秒，然后发送三次零速度帧；例如 `--speed 50 --duration 0.2`，速度范围是 20～200 mm/s，时长范围是 0.05～1.0 秒。`--dry-run` 只显示帧字节，不打开串口。收到有效速度帧后，固件会切换到 ROS 控制模式。

## XURSC 测距和 SG90 控制

PlatformIO 固件将 XURSC 的 ECHO 接 PC6、TRIG 接 PC7，SG90 的橙色信号线接 PB8/S1；两模块共用控制器地线，电源接 5 V。`include/config/openctr_features.h` 中两个模块默认启用，原机械臂功能关闭。SG90 在收到角度指令时开始输出 PWM，角度范围为相对中位的 -90～90 度。舵机稳定后，XURSC 每约 100 ms 测距一次。

USB-COM 仍用 230400 波特率及 `AA 55 长度 类型 数据 校验和` 帧格式。原有 `0x10` 底盘遥测保持原样。新增 `0x11` 测距帧，数据为 `状态(1 字节) + 距离毫米(大端 2 字节)`；状态 `3` 表示有效距离，`4` 表示超时，`5` 表示超出 20～4000 mm 范围，后两种状态的距离为 0。向固件发送 `0x61` 指令，数据为有符号大端 16 位角度（度）；固件执行后回传 `0x12` 帧，数据为实际执行的角度（度）。角度指令和测距回传不参与 FOLLOW 控制，也不会改变底盘控制模式。

烧录新版固件后，在本机终端运行：

```bash
python3 tools/range_pan_serial.py --port /dev/ttyACM0
python3 tools/range_pan_serial.py --port /dev/ttyACM0 --angle 30
python3 tools/range_pan_serial.py --port /dev/ttyACM0 --angle 0
```

第一个命令显示距离或测距失败状态；第二个命令转到 30 度并等待舵机回执；第三个命令回到中位。程序会先确认收到原有底盘遥测，再发送角度指令。只能由一个程序占用 USB-COM 串口；运行这些命令时先关闭其他串口监视器。

## 按住键盘方向键控制底盘

`tools/drive_keyboard.py` 在 Linux 本机读取键盘的 `/dev/input/*-event-kbd` 事件，可以区分真正的按下和松开。普通终端的方向键转义序列不包含松开事件，因此脚本需要当前用户能读取键盘 event 设备。在本文件夹的本机终端执行：

```bash
python3 tools/drive_keyboard.py --dry-run
python3 tools/drive_keyboard.py --port /dev/ttyACM0
```

先用 `--dry-run` 验证键盘状态，不打开串口。正常运行时，先等到有效遥测，再通过 USB-COM 每 50 毫秒发送一次速度帧。按住 ↑ 前进、↓ 后退、← 左转、→ 右转；可以同时按前进和转向键。松开方向键后，下一帧对应速度立即变为 0；空格发送停车，Q 或 Ctrl+C 退出并尝试连发三帧停车。默认前后速度为 100 mm/s、角速度为 300 mrad/s，可通过 `--speed`、`--turn` 调整。

如果自动查找不到键盘，先查看 `ls -l /dev/input/by-id/*-event-kbd /dev/input/by-path/*-event-kbd`，再使用 `--keyboard-device /dev/input/by-id/你的键盘-event-kbd` 指定。若显示权限不足，需要给当前用户配置该设备的读取权限。通过 SSH 远程终端操作时，远端通常无法读取你本机的键盘 event 设备。

**首次测试请先让四轮离地，并保持电源开关可触及。**现有固件没有串口断线自动停车机制；如果运动帧发出后线缆脱落或程序异常终止，停车帧可能无法送达。脚本正常结束或按 Ctrl+C 时会尝试发停车帧，但不能代替固件侧的断线保护。这两个脚本目前只通过本地模拟测试，尚未向实车发送运动命令。

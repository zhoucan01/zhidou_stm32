# AT32F415 机器控制固件

> **【强制规范】代码与文档同步**：任何代码修改（新增/修改/删除文件、引脚映射、定时器/中断配置、API 接口、构建流程、时钟配置等）都必须同步更新本 README 对应章节。AI Agent 在完成代码修改后，必须检查并更新 README，保持文档与代码一致。

本目录是 **AT32F415（Artery Cortex-M4）** 的机器控制器固件工程，负责控制多路步进电机、阀门、泵、电流采集与状态检测。本文档面向编程助手（AI Agent），说明代码框架、分层结构与开发流程，便于快速定位与修改代码。

---

## 1. 项目概览

| 项目 | 说明 |
| --- | --- |
| MCU | AT32F415RBT7（Cortex-M4，Flash 128KB，RAM 32KB） |
| 主时钟 | 150MHz（HICK → PLL），AHB=150MHz，APB1/APB2=75MHz |
| 编译器 | Keil MDK ARMCC V5.06（AC5，C90 语法约束） |
| 工程入口 | `project/MDK_V5/code.uvprojx`（实际构建用 Keil UV4，非 CMake） |
| 当前资源占用 | Flash 约 46.77KB / 128KB（36.54%），RAM 约 26.28KB / 32KB（82.13%） |

> ⚠️ **RAM 余量已不足 6KB**：其中 FreeRTOS `ucHeap` 占 16KB，`modbusData_handle` 队列白占约 2KB（见 §9 第 13 条）。

设备型号字符串 `AT32F415RBT7` 在 `build.bat` 与 `flash.bat` 中用于烧录工具参数。

---

## 2. 目录与框架分层

```
code/
├─ libraries/                 # 芯片底层库（厂商 SDK，勿改动）
│  ├─ cmsis/                  #   CMSIS 内核 + 启动文件
│  │  ├─ cm4/core_support/
│  │  └─ cm4/device_support/  #   system_at32f415.c 等时钟定义
│  └─ drivers/                #   外设驱动库（at32f415_*.c/.h）
│     ├─ inc/                 #   at32f415_adc.h ... at32f415_wwdt.h
│     └─ src/                 #   对应 .c 实现
│
├─ project/                   # BSP（由 AT32 WorkBench 生成的板级配置）
│  ├─ inc/
│  │  ├─ at32f415_wk_config.h #   ★ 所有引脚宏定义（核心映射表）
│  │  ├─ at32f415_conf.h      #   外设使能开关
│  │  └─ wk_*.h               #   wk_adc / wk_gpio / wk_tmr / wk_usart ...
│  ├─ src/
│  │  ├─ at32f415_wk_config.c #   系统时钟 + 外设时钟 + NVIC 配置
│  │  ├─ main.c               #   ★ 程序入口，初始化顺序 + 主循环
│  │  ├─ at32f415_int.c       #   ★ 中断服务函数（ISR）定义处
│  │  └─ wk_*.c               #   各外设 init（GPIO/ADC/DMA/TMR/USART...）
│  └─ MDK_V5/                 #   Keil 工程 + 构建产物
│     ├─ code.uvprojx         #   ★ Keil 工程文件
│     ├─ objects/             #   .obj/.axf/.hex/.bin/.htm/.map 输出
│     └─ listings/code.map    #   ★ 链接器 map（RAM/Flash 用量分析源）
│
├─ task/                      # ★ RTOS 任务 / 通信协议层（**当前主用**）
│  ├─ modbusTask.c/.h         #   ★★ **标准 Modbus RTU 从站**（USART3 RS485，地址 2，100000）
│  │                          #       重命名自 modbus.c（旧工程显示名 modbus_rtu.c），产出 modbusTask.o
│  ├─ getState.c/.h           #   ★ 传感器缓存 20ms 刷新（强符号 my_task02_func）
│  └─ actionRun.c/.h          #   动作任务，**未加入构建**
│
├─ app/                       # ★ 应用层模块（业务逻辑，主要在此开发）
│  ├─ moter28.c/.h            #   28 步进电机（阀门），8 拍 GPIO 驱动
│  ├─ moter42.c/.h            #   42 步进双电机（正弦微步 PWM，当前启用）
│  ├─ moter57.c/.h            #   57 步进电机（正弦微步 PWM，当前启用）
│  ├─ totolCtl.c/.h           #   总控制：电机使能/复位（moterEN/DIS、moterRstSet）
│  ├─ AT8236.c/.h             #   泵驱动（清水/卤水泵 S 曲线加减速，软管泵线性斜坡同 main；pumpDecSpdTick）
│  ├─ current.c/.h            #   电流采样（ADC 原始值 → 安培）
│  ├─ tmux1308.c/.h           #   8 通道复用器扫描（状态/电流/NTC原始ADC值）
│  ├─ getBusVol.c/.h          #   母线电压读取（ADC1_IN5）
│  ├─ enc.c/.h                #   编码器计数（正交/ABZ）
│  ├─ LED.c/.h                #   指示灯控制
│  ├─ ws2812.c/.h             #   WS2812 灯带驱动
│  ├─ FT.c/.h                 #   流量计脉冲计数 + 清水定量停泵（计数超过 waterFtMaxNum 即停泵并清流量计）；点卤只计数清零，停泵由主站控制
│  └─ hostif.c/.h             #   ⚠ 已废弃：旧 UART5 ASCII 行协议，**符号已被链接器全部移除**
│
│  > 以上 15 个 .c 均在 `code.uvprojx` 的 app 组里（含 3 个 `task/` 下的文件），
│  > 但 `hostif.c` 的全部符号**会被链接器移除**（无人调用）。
│
├─ tools/                     # ★ 上位机工具
│  └─ motor57_gui.py          #   已迁移为 **Modbus RTU 主站**的上位机（tkinter+matplotlib+pyserial）
│
├─ build.bat                  # 增量/全量编译（UV4 命令行 + map 分析）
├─ flash.bat                  # 编译 + 烧录（pyOCD/CMSIS-DAP 全盘擦除后烧录 / JLink / STLink / MSD）
├─ analyze_map.ps1            # ★ map 文件解析，输出 Flash/RAM 用量
├─ keilkill.bat               # 清理编译中间产物（慎用，会删 .map）
└─ CMakeLists.txt / CMakePresets.json  # 存在但当前构建不用（用 Keil UV4）
```

### 分层原则

```
task/        RTOS 任务与通信协议（Modbus RTU 从站、传感器刷新）
   │
app/         业务模块（调用 libraries 驱动 + project 里的 wk_* 配置）
   │
project/     BSP 板级配置（wk_* init 函数、ISR、main 入口、引脚宏）
   │
libraries/   芯片厂商 SDK（外设驱动、CMSIS，不要修改）
```

**约定**：

- `wk_*.c` 由 AT32 WorkBench 工具生成，用户代码写在 `/* add user code begin ... */` 注释块之间（生成器重新生成时只保留这些块）。
- `app/` 是纯手写业务代码，不受生成器影响。
- `task/` **不在编译器的 include 路径里**，因此其中的 `.c` 必须用相对路径包含头文件（如 `#include "../../app/modbus.h"`）。
- ✅ `code.uvprojx` 的 `<FileName>` 曾长期为**历史遗留显示名**（如 `modbus_rtu.c` 实际构建 `task/modbus.c`）。现已**统一重命名**：`task/modbus.c` → **`task/modbusTask.c`**，工程条目 `<FileName>`/`<FilePath>` 均已同步，目标文件为 `modbusTask.o`。

---

## 3. 开发流程（构建 → 烧录 → 分析）

### 3.1 编译

```bat
build.bat              :: 增量编译（UV4 -b）
build.bat -Rebuild     :: 全量重编译（UV4 -r）
build.bat -Clean       :: 清理中间产物
```

- 实际调用 `D:\application\KEIL\ARM\UV4\UV4.exe`（若 `C:\Keil_v5` 不存在）。
- 编译输出重定向到 `project/MDK_V5/build_log.txt`。
- 编译成功后**自动调用 `analyze_map.ps1`** 分析 `project/MDK_V5/listings/code.map`，打印 Flash/RAM 用量。

### 3.2 烧录

```bat
flash.bat              :: 编译 + pyOCD/CMSIS-DAP 烧录（默认，不擦除）
flash.bat daplink      :: 同上（显式指定 pyOCD/CMSIS-DAP）
flash.bat jlink        :: J-Link（烧录前先 erase）
flash.bat stlink       :: ST-Link（经 UV4 下载）
flash.bat msd          :: 旧 DAPLink U 盘拖拽烧录（不擦除）
flash.bat -SkipBuild   :: 跳过编译直接烧录
flash.bat -Erase       :: 烧录前先全盘擦除
flash.bat -Rebuild     :: 全量重编译后再烧录
```

- **默认 daplink 模式（pyOCD/CMSIS-DAP）**：直接用 `pyocd flash --erase=auto` 烧录 `objects/code.hex`（只擦除待写入的扇区），烧录成功后自动 `pyocd reset` 复位运行。
  - 未接探测器时**快速报错退出**（不挂死）：脚本会解析 `pyocd list -p` 输出判断是否找到 CMSIS-DAP 探测器（`pyocd list` 无探测器时退出码仍为 0，不能只看返回值），`flash`/`reset` 均带 `-W` 禁止无限等待。
  - 依赖：Python 3.10+ 与 `pyocd`（`pip install pyocd`），以及 Keil DFP 包 `Keil.AT32F4xx_DFP`（提供 `AT32F415_128.FLM` 烧录算法）。
  - 目标名 `at32f415rbt7`，SWD 时钟默认 1 MHz（遇 ACK 错误可在 `flash.bat` 中调低 `PYOCD_FREQ`）。
  - 旧版"U 盘拖拽"方式保留为 `flash.bat msd`（检测 `INFO_UF2.TXT`/`DETAILS.TXT` 盘符后复制 `code.bin`）。
- J-Link 模式：在 `loadfile` 前插入 `erase` 指令实现全盘擦除。

### 3.3 内存用量分析

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File analyze_map.ps1 -MapFile "project\MDK_V5\listings\code.map"
```

脚本解析 map 中的 `LR_IROM1`（Flash）与 `RW_IRAM1`（RAM）区域，打印用量与百分比条形图。

### 3.4 在 VS Code 中触发（`tasks.json`）

工作区已配置 `shell: build` 任务，依赖 `saveAll`（先保存所有文件再编译）。

---

## 4. 时钟与外设基址

系统时钟配置见 `project/src/at32f415_wk_config.c` 的 `wk_system_clock_config()`：

```
HICK → PLL(pll_ns=150, pll_ms=1, pll_fr=4) → 150MHz SCLK
AHB   = 150MHz (AHB_DIV_1)
APB1  = 75MHz  (APB1_DIV_2)
APB2  = 75MHz  (APB2_DIV_2)
Flash wait cycle = 4
```

- SysTick 时钟源 = AHB/8（`wk_system.c` 的 `wk_timebase_init()`），`wk_delay_ms/us` 基于此实现。
- ADC 时钟分频 = `CRM_ADC_DIV_4`。

---

## 5. 定时器与中断映射

| 定时器 | 周期 / 用途 | 备注 |
| --- | --- | --- |
| TMR1 | 65535，编码器（正交计数） | CH1=PA8/CH2=PA9 正交，CH3=PA10 B 相输入捕获 |
| TMR2 | 14999，泵 PWM（4 路） | CH1=PA0 软管泵A，CH2=PA1 软管泵B，CH3=PA2 水泵，CH4=PA3 卤水泵 |
| TMR3 | 65535，编码器（正交） | CH3=PC8 B 相捕获；encoder_mode_A |
| **TMR4** | **3749 → 40kHz**，步进 PWM | CH1=PB6 / CH2=PB7（moter42 电机1），**CH3=PB8 / CH4=PB9**（moter57/压制电机），OVF 中断已开；`sts=1` 主机：TRGOUT 输出 OVF，**同时**触发 ADC1 注入组与驱动 TMR5 从机复位 |
| **TMR5** | **3749 → 40kHz**，步进 PWM | **CH1=PF4 / CH2=PF5**（右抬升电机），`TMR5_GMUX_001` 重映射；**TMR4 从机**（`TMR_SUB_RESET_MODE` + `ITR3`，每周期被 TMR4 OVF 复位 → 与 TMR4 载波锁相），OVF 中断已关 |
| TMR10 | 分频寄存器 11（实际 12 分频），周期寄存器从 49999 降到 11249；2448 步（约 6 秒）内从 4 ms/步加速到 0.9 ms/步，重新启动或换向重置加速，同向重复指令保持速度 | OVF 中断驱动 `moter28RunStep()` |

> 定时器时钟 = 150MHz，`period=3749` 时更新频率 = 150MHz / 3750 = **40kHz**。
> 步进驱动每 40kHz 中断刷新一次 PWM 相位。

### 中断服务函数（`project/src/at32f415_int.c`）

| ISR | 服务内容 |
| --- | --- |
| `TMR1_OVF_TMR10_IRQHandler` | TMR10 OVF → `moter28RunStep()`（阀门步进） |
| `TMR4_GLOBAL_IRQHandler` | TMR4 OVF → **`StepperPulse42_Tick()`（moter42 双电机）+ `StepperPulse57_8412_Tick()`（moter57）** |
| `TMR5_GLOBAL_IRQHandler` | TMR5 OVF → **当前为空**（moter42 电机2 的 PWM 由 TMR4 中断中的 `StepperPulse42_Tick()` 统一驱动；TMR5 已配置为 TMR4 从机锁相载波，`wk_tmr5_init` 已关其 OVF 中断，此 ISR 仅为兜底） |
| `ADC1_IRQHandler` | ADC 注入组转换完成（PCCE）→ `updateMoter57CurAdc()`（读 `ADC1->pdt1/pdt2/pdt3` 锁存 57 电机绕组电流原始值） |
| `USART3_IRQHandler` | 空闲中断（IDLEF）→ **`modbusUsart3IdleIsr()`（关 DMA → 读 dtcnt → 清 IDLE → 回调 → 重载 → 重启）** |
| `UART5_IRQHandler` | 仅清 IDLEF；**UART5 旧协议已停用**（既不调 `modbusUart5IdleIsr()` 也不调 `hostifUart5IdleIsr()`） |

---

## 6. 电机控制架构

### 6.1 正弦微步进原理（moter42 / moter57）

两相步进电机使用 **256 点正弦表** 进行微步细分，两相电流相差 90°：

```
绕组 A = sin(θ)        → PWM CCR
绕组 B = sin(θ + 90°)  → PWM CCR（相位偏移 64 个采样点 = 256/4）
```

- `PWM_CENTER = 1875`（50% 占空比 = 零电流），PWM 范围 0~3750（对应 `period=3749`）。
- 幅值缩放：`CCR = CENTER + (sin - CENTER) * amplitude / 100`。
- 转速换算（×64 定点分数步进，累加器 ≥64 才推进一步）：

$$
\text{RPM} = \frac{60000 \times \text{StepRate}}{2048 \times \text{Divider}}, \quad \text{StepRate} \in [64, 512]
$$

- S 形加减速：`smoothstep s = 3x² - 2x³`（Q16 定点），每 10ms（`RAMP_TICKS=400` 次 40kHz 中断）更新一次。

### 6.2 moter57 —— 57 步进电机（`app/moter57.c`，已接入 main）

- 驱动 **TMR4 CH3/CH4**（PB8/PB9），由 `TMR4_GLOBAL_IRQHandler` 调用 `StepperPulse57_8412_Tick()`。
- 带转速-电流幅值查表（`SP_57_8412_AmpTable[]`，低速小电流、高速大电流，**区间内按转速线性插值**）：0~175RPM 由 20% 渐升到 38%（折点按 25RPM 加密，便于低速调参）；200RPM 起整体抬高（52%），到 350RPM 前为 92%；**360RPM 以上直接满幅 100%**（标定期间先保证不丢步，标定完成后再逐点瘦表）；查表结果钳位到 ≤100%，避免标记行在超范围时返回 101%。2026-10-19：机械上限 `PRESS_MOTER_MAX_RPM_X10` 收到 350RPM，表内 350RPM 以上档位不再可达。
- 支持步数模式（`MoveToAngle`）与持续运行（`RunAtSpeed`）。
- **ADC 零点标定**：`StepperPulse57_8412_Init()` 在电机静止时对 ADC1_IN11(PC1)/ADC1_IN12(PC2)（压制电机绕组 A/B 电流检测，`pressMoterCurrA/B`）各采样 **20 次求平均**，存入 `StepperPulse57_8412_CurrA/B_Zero` 作为零电流基准；运行时实际电流 = 当前采样 − 零点基准（`getMoter57Curr()` 更新全局变量 `moter57CurrA/B`，供 Modbus 输入寄存器上报读取）。
- **电流采样链路**：TMR4TRGOUT 触发 ADC 注入组（通道 11/12/8）→ `ADC1_IRQHandler`（PCCE）调用 `updateMoter57CurAdc()` 锁存 `ADC1->pdt1/pdt2/pdt3` → `getMoter57Curr()` 换算（系数 0.002015）。
- 关键 API：`StepperPulse57_8412_Init/RunAtSpeed/Run/Stop/MoveToAngle/GetPosition/GetCurrentA/GetCurrentB`、`getMoter57Curr`、`updateMoter57CurAdc`。
- **电流表标定入口**：`StepperPulse57_8412_RunAutoTable(rpm_x10)` / `RunAutoTableEx(rpm_x10, accelTime_ms)` 以查表模式持续运行（与 `pressMoterUp/Down` 同一条路径，幅值传查表标记值 >100；Ex 版速度按 `PRESS_MOTER_MAX_RPM_X10` 钳位）；上位机用 `NB+m57=run,<rpm>,auto` 触发，用 `NB+m57=watch,on|<周期ms>` 周期打印 `cur/amp/pos/denc/adcA/adcB/zA/zB/ia/ib`（`adcA/zA` 用于核对电流零点：静止时 `adcA` 应≈`zA`；`ia/ib` 为绕组电流，趋势判据），或用 `NB+m57=sweep,<rpm>,<加速ms>,<保持ms>,<循环次数>` 自动往返扫描，观察丢步与抖动对应的转速区间。
- **停机斜坡**：`StepperPulse57_8412_StopRamped()` = `SP_57_8412_STOP_RAMP_TIME_MS`（**1000ms**，原 300ms）的 S 曲线减速到 `SP_57_8412_STOP_COAST_RPM_X10`(50RPM) + `SP_57_8412_STOP_FADE_TIME_MS`(60ms) 电流淡出。原 300ms 在 170RPM 停机实测丢步（指令 `pos` 仍推进、编码器 `denc`≈0），低速（≤100RPM）时 300ms 尚能跟上。
- ⚠️ 头文件注释里提到的 "STM32F103 TIM3" 是旧平台遗留说明，**实际代码已迁移到 AT32F415 + TMR4 CH3/CH4**。
- **限位保护（保护下沉到底层，已与 main 统一）**：低位（`pressMoterLowPosition`，压制编码器绝对值 ≥ `moter57LowEncNum`）只允许上转，高位（`pressMoterHighPosition`，PB12）只允许下转。`pressMoterPositionUpdate()` 只依据 PB12 与编码器目标刷新位置状态，**不随运动命令清零**；`StepperPulse57_8412_RunAtSpeed()` 在 `__disable_irq()` 临界区内自检越限并直接停机（实际启动走 `SP_57_8412_RunAtSpeedUnchecked()`），因此**任何**入口（`pressMoterUp/Down`、调试口 `NB+m57=run/sweep`）都受保护；`pressMoterMonitor()`（每 40kHz 由 `StepperPulse57_8412_Tick()` 调用）负责运行中撞限位停机，碰高位时把压制编码器清零为机械零点。
- 调试口只做**只读**限位查询（`usrDebugM57LimitBlocked()`，源为 `pressMoterHighPositionGet()/LowPositionGet()`），用于回 `blocked`/`sweep,abort`；原先自定义的 `pressMoterLimitAllow()` 重复保护已删除。
- ⚠️ 压制编码器上电清零（`encInit()`），只有碰过一次 PB12 高位后才建立"距上死点的行程"绝对值，因此上电时若机构已在底部，低位保护要等首次上行到顶之后才生效。

### 6.3 moter42 —— 双电机（`app/moter42.c`，已接入 main，当前启用）

| 电机 | 定时器通道 | 引脚 | 说明 |
| --- | --- | --- | --- |
| 电机 1 | TMR4 CH3 + CH4 | PB8 + PB9 | 正转 |
| 电机 2 | TMR5 CH1 + CH2 | PF4 + PF5 | **与电机 1 同时运动、方向相反** |

- **控制模式完整复刻自 moter57**：256 点正弦表微步、`SP_42_CalcRPM` 转速换算（RPM = 60000×StepRate/(2048×Divider)）、转速-电流查表 `SP_42_AmpTable[]`、S 形加减速（smoothstep，Q16 定点，每 10ms 由 TMR4 中断自动更新）、步数模式（`MoveToAngle`）与持续运行（`RunAtSpeed`）。
- **单一函数驱动双电机**：`StepperPulse42_Tick()` 在 TMR4 OVF 中断中被调用一次，同时刷新电机1（TMR4 CH3/CH4）与电机2（TMR5 CH1/CH2）的 PWM。TMR5 中断无需调用 Tick，仅清标志维持时基。
- **方向自动辨识**：操作函数只需输入带符号速度 `rpm_x10`（或角度），符号即电机1方向；电机2 通过 `idx2 = (0 - idx1) & 0xFF` 相位取反实现自动反向，无需单独输入方向。
- 位置/角度 API 以电机1为视角，电机2位置 = -电机1位置。
- 关键 API：`StepperPulse42_Init/Tick/RunAtSpeed/Run/Stop/MoveToAngle/GetPosition/SetAmplitude/GetCurrentRPM/IsRamping`。
- **抬升 42 底层已整体回退到 origin 版（2026-10-21）**：`riseMoterRun()` 直接调 `StepperPulse42_Run/RunSameDirection`（内部即 `RunAtSpeed`，加速 5000ms、起步 300），幅值用 `RISE_MOTER_LEFT/RIGHT_AMPLITUDE` 的固定值（低档 15%）；“查表 / 手动”来源切换已删除（`riseMoterSetAmpTableEnable/GetAmpTableEnable` 与 `NB+rise=auto,on|off` 一并移除）。`SP_42_Left/RightAmpTable[]`（origin 的 **62.5RPM 一档阶梯表**，20%~99%，标记行 `{6000,0}`）与阶梯取值版 `SP_42_CalcAmplitude()`（无插值、无 ≤100 钳位）仅在显式传入 >100 幅值时经 `SP_42_ApplyRPM()` 使用。抬升默认速度回到 `RISE_MOTER_RPM_X10`=600（60rpm，作用于 `riseMoterUp/Down()` 及上/下段动作）。上行专用 3 秒堵转保护已删除，只保留 origin 的下行堵转保护。
- **抬升保护层**：`riseMoterUp/Down/Stop`、`riseMoterUpAtSpeed/DownAtSpeed` 在 `__disable_irq()` 临界区内原子更新状态并启动驱动；重复下行只更新速度、不重建计程起点；低位已锁存时拒绝下行；重跑上行会清除上一轮的到位/段状态。`riseMoterMonitor()`（TMR4 OVF 中断内）按左右编码器位移判到位，并做**堵转保护**：`RISE_MOTER_STALL_TICKS`(120000) 窗口内任一侧位移 ≤ `RISE_MOTER_STALL_PULSES`(200) 判定卡死并停机。
- **下降一段**：`riseMoterDownSegment(counts)` / `riseMoterDownSegmentDefault()`（段长 `RISE_MOTER_SEGMENT_COUNTS`=14409 编码器计数，保持寄存器 `0x0003` 写 -1000 触发）。只能从高位启动，左右两侧都走满段长后零电流停止、位置靠丝杆自锁；RS485 重传幂等；途中触发低位按故障处理（`riseMoterSegmentActive/Fault/CountsGet` 查询）。
- **抬升电流档（低/高，2026-10-21）**：抬升默认走**低档**（`RISE_MOTER_LEFT/RIGHT_AMPLITUDE`=15%），用于上位机手动控制与开机自检（`main.c`：下行 8s → 上行到高位 → 再下行到最低位，上电后停在最低位）；执行**段动作**（`0x0003` 写 -1000 下降一段 / -601 上行一段，压制流程带载定位）时，启动后立即切**高档**（`RISE_MOTER_LEFT/RIGHT_AMPLITUDE_LOADED`=40%）并保持；**压制流程全程抬升带载**，所以由保持寄存器 `0x0004`（0=低档、非 0=高档）统一切档，覆盖归位下行、-601/-1000 段动作、保压后回高位上行，流程结束写回低档。档位粘滞保存（段动作一定带载，也会强制切高档）。实现：`riseMoterSetLoadGear()`（档位状态）与 `riseMoterApplyLoadGear()`（`riseMoterUpAtSpeed/DownAtSpeed` 起步按档位套幅值），复用 `riseMoterSetAmplitudeLeftRight()` 的“运行中立即生效”。协议不变。
- ⚠️ moter42 与 moter57 **都占用 TMR4**（moter42 电机1 用 CH1/CH2，moter57 用 CH3/CH4），当前 main.c 两者同时启用（`StepperPulse42_Init()` + `StepperPulse57_8412_Init()`），TMR4 OVF 中断中同时调用 `StepperPulse42_Tick()` 与 `StepperPulse57_8412_Tick()`。

### 6.4 moter28 —— 阀门 28 步进（`app/moter28.c`）

- 8 拍整步驱动表 `moter28State[8][4]`，用 4 个 GPIO（`valveGPIOA/B/C/D`）直接输出相序。
- 由 `TMR10` 溢出中断调用 `moter28RunStep()` 推进节奏。
- API：`moter28Forward/Backward/Stop`、`valveOn/valveOff`、`getMoter28Fault`、`clearFault`。
- 开阀走固定步数（`totalTurns = 20*4096`），关阀检测 `valveOFFPos` 到位开关。

### 6.5 上位机通信接口 —— 标准 Modbus RTU 从站（`task/modbusTask.c/.h`，**当前主用**）

**硬件**：**USART3**（TX=PB10，RX=PB11）经 RS485 收发器，**1000000** 8N1；从站地址 **2**。

> ⚠️ AT32F415 **没有硬件 RS485 DE/RE**（SDK 无 rs485/DEM API），方向控制靠 GPIO 手动翻转：
> **PB2**（`USART3SEL_PIN`）**高=发送 / 低=接收**。发送完必须等 `USART_TDC_FLAG` **再额外延时 3 个字符时间**才能拉低，否则末字节被截断。
> 波特率由 AT32 Workbench 生成（`wk_usart.c` 的 `usart_init(USART3, ...)` + `code.ATWP`），
> 改它必须同步 `task/modbusTask.c` 的 `MB_BAUD_RATE`。

**接收链路**：USART3 RX 走 **DMA1_CH1（非循环模式）** + **IDLE 空闲中断**。

⚠️ **与常规 RTU 从站不同**：RTU 标准要求 3.5 字符时间静默判帧，但 USART IDLE 只有 **1 个字符时间**。
采用 **关 DMA → 读 dtcnt → 清 IDLE → 回调 → 重载 → 重启 DMA** 的模式（参考 NUVELL_V1 工程）：
IDLE 中断里先 `dma_channel_enable(DMA1_CHANNEL1, FALSE)` 停止 DMA，再用 `rxLength = MODBUS_RX_BUFFER_SIZE - DMA1_CHANNEL1->dtcnt` 算出本次收到的字节数，
**然后才** `usart_flag_clear(USART3, USART_IDLEF_FLAG)` 清 IDLE 标志（该函数会读 STS 再读 DT，必须在 DMA 停止后执行，否则读 DT 会从接收寄存器偷走字节导致 DMA buffer 内容损坏），
接着调用 `Modbus_RxIdleCallback(buf, rxLength)` 把整段数据 memcpy 到接收队列槽位，然后重载 `dtcnt = MODBUS_RX_BUFFER_SIZE` 与 `maddr`，最后 `dma_channel_enable(DMA1_CHANNEL1, TRUE)` 重启 DMA 等待下一帧。

> 📌 **为何不用循环 DMA + dtcnt 反算增量**：循环模式下靠 `dtcnt` 反算自上次以来新增字节数并累积重组的方案，在 1Mbaud 高波特率下会出现**字节错位**
> （如 `02 04 00 00 00 28 F0 27` 被错位成 `00 00 00 28 F0 27 02 04`，CRC 失败 stages=C）。根因是循环 DMA 的写指针回绕与 ISR 读取 dtcnt 的时序竞争不可靠。
> 改为非循环 + 关DMA读重载重启后，每次 IDLE 都拿到一段**连续、确定长度**的数据，无需在 ISR 内做帧拼接，从根本上消除错位。

**任务**：`my_task01_func`（1024 字栈，prio 10）阻塞在 `xQueueReceive(modbusData_handle, ...)` 上，收到帧槽位号后调 `processBurst()` 解析并应答。
`processBurst()` 采用**逐字节游标推进**解析：用 `offset` 从缓冲区起始逐字节扫描，遇到地址不匹配/不支持的功能码/CRC 失败等错误时只 `offset++` 跳过 1 字节继续找下一帧起始（地址字节），
而非 `break` 退出——这样能在一次 IDLE 收到的多帧突发数据中正确拆出每一帧。`supportedFunction()` 白名单：0x01/02/03/04/05/06/0F/10。

**UART5 日志**（PC12 TX，115200 8N1）：统一使用 `project/inc/log.h` 的 `log_d(...)`（普通日志）和 `log_e(...)`（错误/报警）。格式为 `[时间戳|等级]函数名.行号 :: 数据`；时间戳是芯片 1 ms timebase 的低 8 位十进制数字，不足 8 位补零。底层格式化与发送由 `project/src/wk_debug.c` 实现。

Modbus 每条未过滤请求在任务上下文只输出一行，例如：

```text
[00001234|D]my_task01_func.1140 :: Modbus RX[8]=02 04 00 00 00 28 F0 27 stages=QTVSD
```

阶段字符串按处理阶段汇总：

| 阶段 | 代码位置 | 含义 |
| --- | --- | --- |
| `Q` | `Modbus_RxIdleCallback()` | 完整请求已成功放入 Modbus 队列 |
| `T` | `my_task01_func()` | Modbus 任务已从队列取出请求 |
| `C` / `V` | `processBurst()` | `C` = CRC 校验失败；`V` = CRC 校验通过 |
| `S` / `D` | `processBurst()` | `S` = 开始调用 RS485 应答发送；`D` = 发送函数返回 |

默认静默过滤请求 `02 04 00 00 00 28 F0 27`，避免周期轮询刷屏。要更换过滤目标，编辑 `task/modbusTask.c` 顶部的 `MODBUS_DEBUG_FILTER_BYTES`；将 `MODBUS_DEBUG_FILTER_ENABLE` 设为 `0U` 可关闭过滤、显示所有请求。过滤按完整帧长度和字节逐一精确匹配，命中的请求不输出日志。

诊断时看 `stages` 中最后出现的阶段：缺少 `Q` 表示帧未入队；有 `Q` 无 `T` 表示任务未消费；`C` 表示 CRC 不匹配；有 `V` 无 `S` 表示请求处理未产生应答；有 `S`、`D` 但主站收不到数据，则检查 PB2/DE 方向、收发器和总线。日志在 Modbus 任务中一次性输出，不在 ISR 中阻塞打印。

状态刷新由 `task/getState.c` 的 `my_task02_func` 每 **20ms** 完成（母线电压/电流、TMUX 三颗全通道扫描、泵加减速 tick），Modbus 应答只读**缓存快照**，不在应答路径重扫 ADC。

**完整协议（寄存器映射、功能码、示例帧）见 [`通信协议说明.md`](通信协议说明.md) 及参考文档 `ESP32S3/docs/站号2_AT32F415CBT7_Modbus协议.md`**。要点：

| 区 | 数量 | 地址范围 | 说明 |
| --- | --- | --- | --- |
| 线圈 Coils（FC01/05/0F） | 10 | 0x0000~0x0009 | 清水泵/点卤泵/蠕动泵/阀门/LED/预留×3/压制命令状态/抬升命令状态（泵"开"的速度=编译期宏 `PUMP_DEFAULT_WATER/BRINE/HOSE_PERCENT`，见 `app/AT8236.h`，改宏重编译即改默认转速） |
| 离散输入（FC02） | 16 | 0x0000~0x000F | 抬升高位/抬升低位/压制高位/压制低位/压制头在位/水箱在位/水箱高液位/水箱低液位/废水箱在位/阀门开到位/阀门关到位/点卤盒在位/溢出报警/上盖在位/滤网在位/成型盒在位 |
| 保持寄存器（FC03/06/10） | 103 | 0x0000~0x0066 | 0x0000清水泵(0~1000，写入值/10=速度百分比)/0x0001蠕动泵(0~255，>127 经 (int8_t) 变为负值=反向)/0x0002压制电机int16（**调试期：数值直接=转速rpm**，0~350 有效、>350 按350rpm钳位（2026-10-19 收上限：实测 373rpm 起必失步）；|值|<50 时该值本身即 rpm×10（0~49→0~4.9rpm，2026-10-22 边界 100→50））；开关 `PRESS_CMD_VALUE_IS_RPM`@`task/modbusTask.c`，置0恢复“数值=转速×10”，1000=100rpm）/0x0003抬升电机int16（**数值直接=转速rpm**（2026-10-19 重新打开速度接口）：|值|×10 交 `riseMoterUp/DownAtSpeed()`，上限 `RISE_MOTER_MAX_RPM_X10`=6000（600rpm）@`app/moter42.c`；写 -1000 触发固定段长“下降一段”，段长度写在下位机。注意主站 ESP32 的 `riseMoterUp/Down(23)` 现在=23rpm（原为固定 90rpm））/0x0004~0x0066参数预留 |
| 输入寄存器（FC04） | 80 | 0x0000~0x004F | 0x0000~0x0027文档标准(40个)+0x0028~0x004F V2追加(40个) |
| 输入寄存器波形窗口（FC04） | — | 0x1F40~0x33FF | 57绕组A/B的40kHz ADC波形(只读，不占用快照) |

> 📌 **寄存器布局严格遵循 `ESP32S3/docs/站号2_AT32F415CBT7_Modbus协议.md`**：0x0000~0x0027（输入寄存器）、0x0000~0x0009（线圈）、0x0000~0x000F（离散输入）、0x0000~0x0003（保持寄存器）必须与参考文档完全一致；V2 特有功能（多路电流、编码器位置、NTC2/3/Back、流量计、ADC采集状态等）追加在 0x0028 之后。

**上位机工具**：`tools/motor57_gui.py` 已迁移为 **Modbus RTU 主站**（`ModbusRtuMaster` 类，从站 2，CRC poly `0xA001` init `0xFFFF`），
即本工程的**官方参考上位机**。它包含一套 DMA/IDLE 压力测试（`MODBUS_DMA_CASES`）用于验证接收重组逻辑。

> ⚠️ 已知小问题：GUI 的 TMUX **D0 标签**用的是 `SENS_D0_NAMES`（抑制在位/卤水盒在位/…），
> 与固件 `readDiscrete()` 的离散输入 0x0000~0x000F 顺序（抬升高位/抬升低位/压制高位/…）不对应——**标签与实际位不对应**，属 GUI 侧待修问题。离散输入顺序现已严格遵循参考协议文档。

### 6.6 已停用的旧接口（保留代码但**未编译进去**）

- `app/hostif.c/.h`：UART5（PC12/PD2）ASCII 行协议上位机接口（`SPD:` / `AMP:` / `STREAM:` / `INFO` / `CUR,...`）。
  **`hostifInit()` 从未被调用**，main 也不调 `wk_uart5_init()`，链接器 map 里 `hostif.o` 的全部函数与 `.bss`/`.data` 均被 `Removing` —— 属死代码。
- ~~`app/modbus.c/.h`~~：**已删除**。原为 UART5（PC12/PD2）的"屏幕自定义帧协议"（非 RTU，CRC16 多项式 `0x04C11DB7`，与 Modbus 的 `0xA001` **完全不同，切勿混用**）。因命名误导（文件名/函数名/宏名均以 modbus 开头，但实际是屏幕口自定义帧，不是 Modbus RTU）且全部符号被链接器移除，已从工程中彻底删除。**当前工程唯一的 Modbus 实现是 `task/modbusTask.c`（USART3 RTU 从站）**。
- `UART5_IRQHandler` 只清 IDLE 标志，不调任何业务回调。

> 如需恢复 UART5 通信：必须在 `main.c` 里调 `wk_uart5_init()` + 相应 `xxxInit()`，并在 ISR 里挂回调，否则永远不会收到数据。

---

## 7. 引脚映射（核心，完整见 `at32f415_wk_config.h`）

### 步进电机 PWM

| 功能 | 引脚 | 宏 |
| --- | --- | --- |
| 压制电机绕组 B / 57 电机 | PB8 | `pressMoterPWMB_PIN` |
| 压制电机绕组 A / 57 电机 | PB9 | `pressMoterPWMA_PIN` |
| 右抬升电机绕组 A | PF4 | `riseMoterRPWMA_PIN` |
| 右抬升电机绕组 B | PF5 | `riseMoterRPWMB_PIN` |
| 左抬升电机绕组 B / A | PB6 / PB7 | `riseMoterLPWMB/LPWMA_PIN` |

### 控制 / 状态 / 采样

| 功能 | 引脚 | 宏 |
| --- | --- | --- |
| 电机停止 | PC14 | `moterStop_PIN` |
| 电机复位 | PC10 | `moterRST_PIN`（用户要求忽略复位脚） |
| 阀门复位 | PC13 | `vavleRST_PIN` |
| 阀门驱动 A/B/C/D | PA6/PA7/PB15/PB14 | `valveGPIOA/B/C/D_PIN` |
| 阀门关到位 | PF7 | `valveOFFPos_PIN` |
| LED1 | PC15 | `LED1_PIN` |
| RS485 方向（USART3） | PB2 | `USART3SEL_PIN`（**高=发送，低=接收**） |
| TMUX1308 地址线 A0/A1/A2 | PC11 / PB3 / PB4 | `tmux1308A0/A1/A2_PIN` |
| TMUX1308 D0（数字） | PB13 | `tmux1308IOData0_PIN` |
| TMUX1308 D1（ADC） | PC3 | `tmux1308ADCData1_PIN`（ADC1_IN13） |
| TMUX1308 D2（ADC） | PC4 | `tmux1308ADCData2_PIN`（ADC1_IN14） |
| 母线电流 | PA4 | `busCurr_PIN`（ADC1_IN4） |
| 母线电压 | PA5 | `busVol_PIN`（ADC1_IN5） |
| 应变片电流 | PB0 | `strainGage_PIN`（ADC1_IN8） |
| LED 电流 / 抱闸电流 | PC0 | `moterAndLED_PIN`（ADC1_IN10） |

### 编码器

| 信号 | 引脚 |
| --- | --- |
| 编码器1 A / B | PA11 / PA10（TMR1_CH3 捕获） |
| 编码器2 A / B | PC9 / PC8（TMR3_CH3 捕获） |

- 压制（57）编码器计数由 EXINT9 按 A/B 相方向增减：`encPressMoterREncPositionGet()` 返回**真实脉冲数**，`getPressMoterTurns()` 是它的别名，Modbus 输入寄存器 `0x0007~0x0008` 上报 `getPressMoterTurns()/1000`（float32 圈数）。

---

## 8. 应用模块接口速查

| 模块 | 关键函数 |
| --- | --- |
| `moter57` | `StepperPulse57_8412_Init/Tick/RunAtSpeed/Run/RunAutoTable/RunAutoTableEx/Stop/MoveToAngle/GetPosition/GetCurrentA/GetCurrentB`、`getMoter57Curr`、`updateMoter57CurAdc`、`getMoter57AdcA/B`（当前启用） |
| `moter42` | `StepperPulse42_Init/Tick/RunAtSpeed/Run/Stop/MoveToAngle/GetPosition/SetAmplitude/IsRamping/GetCurrentRPM`；抬升保护层 `riseMoterUp/Down/Stop`、`riseMoterUpAtSpeed/DownAtSpeed`、`riseMoterSetAmplitude(LeftRight)`、`riseMoterSetLoadGear/GetLoadGear`、`riseMoterSetSameDirection`、`riseMoterDownSegment`、`riseMoterDownSegmentDefault`（0x0003 写 -1000 触发，段长 `RISE_MOTER_SEGMENT_COUNTS`）、`riseMoterSegmentActive/Fault/CountsGet`（当前启用） |
| **`task/modbus`** | **`modbusinit`、`modbusRs485Send`、`Modbus_RxIdleCallback`、`modbusUsart3IdleIsr`、`Modbus_CRC16`、`processBurst`（逐字节解析）、`supportedFunction`（功能码白名单）（当前主用）** |
| **`task/getState`** | **`my_task02_func`（20ms 传感器缓存刷新，强符号覆盖 freertos_app.c 的 __WEAK 版）** |
| `moter28` | `moter28Forward/Backward/Stop/RunStep`、`valveOn/Off`、`getMoter28Fault` |
| `AT8236` | `waterPumpOn/Off/GetSpeed`、`brinePumpOn/Off/GetSpeed`、`hosePumpOn/Off/GetSpeed`、`brinePumpIsIdle/waterPumpIsIdle`、`pumpDecSpdTick`、`pumpSharedInputSync`（speed 为 -100~100 百分比；清水泵/卤水泵 0.5s S 曲线软启/软停/调速，蠕动泵沿用 main 线性斜坡：启动即 100%、每 tick −150 降到目标，停泵立即清零两路并关 CH1/CH2）；**编译期默认速度宏** `PUMP_DEFAULT_WATER_PERCENT`/`PUMP_DEFAULT_BRINE_PERCENT`/`PUMP_DEFAULT_HOSE_PERCENT`（`app/AT8236.h`，线圈启动即用，改宏重编译烧录生效） |
| `FT` | `waterFTtick`、`brineFTtick`、`waterFTPulseGet`、`brineFTPulseGet`、`clearWaterFT`【**清水**：`waterFTNum > waterFtMaxNum`（13270）即 `waterPumpOff()` 停泵，计数在 `waterPumpOff()` 内由 `clearWaterFT()` 清零（与 main 行为一致，无提前量补偿）；**点卤**：时间定量、停泵由主站直接控制，下位机只计数，`brineFTNum > brineFtMaxNum`（200）即清零（`// brinePumpOff()` 保留为注释）】 |
| `usrDebug` | UART5（PC12/PD2，1 Mbps，`USRDEBUG_AT_ENABLE=1`）AT/NB 文本指令口：`usrDebugInit`、`usrDebugUart5IdleIsr`、`usrDebugAtPoll`、`usrDebugGetFrame`、`usrDebugWrite`；泵支持 `NB+pump=water/brine/hose,<速度 -100~100>`、`NB+pump=off`，cbt7 风格短命令 `W/H/B<档位1~10>`、`S`，57 电流表标定支持 `NB+m57=run,<rpm>,auto`、`NB+m57=watch,on|off|<周期ms>`（默认 50ms，打印 `cur/amp/pos/denc/adcA/adcB/zA/zB/ia/ib`）、`NB+m57=sweep,<rpm>,<加速ms>,<保持ms>,<循环次数>`（自动往返扫描；越限方向启动回 `NB+m57=run/sweep,blocked,high|low`，运行中撞限位回 `NB+m57=sweep,abort,high|low` 并停机），42 抬升电流表标定支持 `NB+rise=watch,on|off|<周期ms>`（默认 50ms，打印 `cur/ampL/ampR/pos/dencL/dencR/ia/ib/iaR/ibR/dir/ramp/hi/lo/encL/encR/dev`，与 57 watch 位点独立、可同时打开）（详见 `code/docs/NB仿AT协议.md`）；**空行静默**：`usrDebugAtHandleLine()` 收到空行只返回，不再回 `NB+ERR,empty`（串口工具行尾 `\r\n` 会多产生一条空行） |
| `totolCtl` | `moterEN/moterDIS`、`moterRstSet`、`getTotolRstLevel` |
| `current` | `currentInit/currentUpdateAll/currentReadBus/currentReadLed/...` |
| `tmux1308` | `tmux1308Init/getTMUX1308Data/tmux1308SetAddress`，缓存数组 `tmux1308Data0/1/2Values[]` |
| `getBusVol` | `getBusVoltage()` |
| `enc` | `encInit/encAbzPositionGet/encTmr3AbPositionGet/encTmr1EncoderGet/...`、`encPressMoterREncPositionGet/Reset`、`encRiseMoterLEncPositionGet/REncPositionGet`、`getPressMoterTurns`（压制编码器真实脉冲） |
| `ws2812` | `ws2812Init/ws2812Show/ws2812SetRgb` |
| ~~`hostif`~~ | ~~`hostifInit/hostifPoll/...`~~ —— **未接入，已被链接器移除** |
| ~~`app/modbus`~~ | ~~`modbusInit/modbusPoll/...`~~ —— **已删除**（原 UART5 屏幕口自定义帧协议，命名误导，死代码） |

---

## 9. 关键坑点与约定（必读）

0. **【强制】代码修改后必须同步更新 README**：任何代码变更（文件增删改、引脚映射、定时器/中断配置、API 接口、构建流程、时钟配置等）都必须同步修改本 README 对应章节，保持文档与代码一致。AI Agent 完成代码修改后，必须检查并更新 README。

1. **ARMCC5 = C90**：for 循环内不能声明变量、不支持 `//` 后的某些语法特性，需先声明后使用；`stdint.h` 与 `stdbool.h` 可用。
2. **`wk_*.c` 用户代码必须写在 `/* add user code */` 块内**，否则 WorkBench 重新生成会覆盖。
3. **构建走 Keil UV4，不是 CMake**：`CMakeLists.txt` 存在但工程实际由 `code.uvprojx` 驱动。
4. **`.bat` 脚本里 echo 文本不要用 `(`)`**：会破坏多层 `if(...)` 括号配对，导致输出被截断（此前已踩坑）。
5. **PowerShell 5.1 编码**：含中文的 `.ps1` 脚本要存为 UTF-8 with BOM，否则按 GBK 解析报错。
6. **`keilkill.bat` 会删除 `.map` 文件**：删完需重新编译才会重新生成，`analyze_map.ps1` 依赖它。
7. **TMR4 CH3/CH4 同时驱动 moter57（压制电机）与被 moter42 共用节拍**：moter42 双电机（含 TMR5 上的电机2）的相位更新统一由 TMR4 OVF 中断的 `StepperPulse42_Tick()` 驱动；TMR5 已配置为 TMR4 从机（复位模式 + ITR3），载波与 TMR4 锁相，因此 moter57 的 ISR 耗时变化对左右 42 电机是同步影响。
8. **DMA 重配置前需先 `dma_channel_enable(..., FALSE)`**（AT32 系列 DMA 使能后 `dtcnt` 只读）。Modbus RTU 的接收路径正是利用这一点：IDLE 中断里先关 DMA 使 `dtcnt` 可读，算出本次接收长度后重载 `dtcnt`/`maddr` 再重启 DMA，参见 §6.5。
9. **map 中的 Flash/RAM 用量**：`Total ROM Size`（含代码+只读数据+RW 数据）= Flash 占用；`Total RW Size`（RW+ZI）= RAM 占用。
10. **工程未启用 microlib**（`useUlib=0`），标准 C 库的浮点格式化 `_printf_fp_dec` 已被正确链接，**浮点 `sprintf("%.3f")` 可用**。
11. **`code.uvprojx` 的 `<FileName>` 历史遗留显示名已清理**：`task/modbus.c` → `task/modbusTask.c`，工程条目已同步（目标文件 `modbusTask.o`）。新增/删除源文件后务必核对 `<FilePath>`。
12. **RS485 方向控制是软件 GPIO 翻转**（PB2，高发低收）：AT32F415 无硬件 DE/RE。发送完需等 `USART_TDC_FLAG` 再延时 3 字符时间才能拉低。
13. **RAM 紧张（仅剩 <6KB）**：若需要内存，优先把 `project/src/freertos_app.c` 的 `modbusData_handle = xQueueCreate(1000, sizeof(uint16_t))` 改为 **4**（它实际只传 0~3 的槽位号，可省 ~2KB），并删掉全工程未用的 `modbusCMD_handle`。
14. **USART3 IDLE 只有 1 字符时间，不足 RTU 的 3.5 字符**：Modbus 从站采用**关 DMA → 读 dtcnt → 清 IDLE → 回调 → 重载 → 重启**模式（见 §6.5）。⚠️ **IDLE 标志必须在关 DMA 之后清除**：`usart_flag_clear()` 会读 STS 再读 DT，若 DMA 还在运行，读 DT 会从接收寄存器偷走字节，导致 DMA buffer 内容损坏（每个字节 bit7 被置 1、长度不稳定）。V1 在关 DMA 后清 IDLE，V2 已对齐此顺序。帧拆分在任务上下文 `processBurst()` 用逐字节游标推进完成。修改接收逻辑时切勿照搬“IDLE 就是完整帧”的假设——一次 IDLE 可能收到多帧突发，`processBurst` 的逐字节推进能正确拆出每一帧。
15. **固件版本号映射到 Modbus 输入寄存器 0x000B~0x000C**（snapshot[11..12]）：依 `站号2_AT32F415CBT7_Modbus协议.md`，0x000B 高字节=主版本/低字节=次版本，0x000C 高字节=修订/低字节=构建，当前为 `0.0.0.1`。版本号不再仅靠上电版本帧单次发送，主站可随时通过 FC04 读取。版本定义在 `project/inc/wk_debug.h` 的 `FW_VERSION_MAJOR/MINOR/PATCH/BUILD`。
16. **Modbus t3.5 轮转延时与总线释放确认**（`modbusRs485Send()`）：发送应答前先等 t3.5（2 ms）并确认 DMA 写位置在 1 字符时间内不前进（主站已释放 DE），最多重试 3 次，避免总线竞争。依 Modbus over Serial Line V1.02 §2.2 规范。
17. **Modbus 寄存器布局严格遵循参考协议文档**（`ESP32S3/docs/站号2_AT32F415CBT7_Modbus协议.md`）：线圈 0x0000~0x0009、离散输入 0x0000~0x000F、保持寄存器 0x0000~0x0003、输入寄存器 0x0000~0x0027 必须与文档完全一致；V2 特有功能（多路电流、编码器位置、NTC2/3/Back、流量计、ADC采集状态等）追加在 0x0028 之后。已发布地址永久冻结，新字段只能从末尾追加，禁止中间插入/重排/复用。

18. **【强制】协议调整必须同步更新协议文档**：任何 Modbus 协议变更（寄存器地址增删改、功能码支持范围调整、数据类型/字节序变更、异常码定义、波形窗口映射、寄存器布局重排等）都必须**先更新协议文档**，再修改固件代码。协议文档包括：
    - 本工程内的 `code/通信协议说明.md` 与 `code/docs/站号2_AT32F415CBT7_Modbus协议.md`（V2 主板本地副本）；
    - **跨工程参考文档** `D:\workspace\Code\Keil\NULLVE_V2\ESP32S3\docs\站号2_AT32F415CBT7_Modbus协议.md`（ESP32S3 侧副本，主站/上位机以此为准）。
    
    **同步规则**：协议变更后，必须把更新后的协议文档**复制覆盖**到 `D:\workspace\Code\Keil\NULLVE_V2\ESP32S3\docs\` 目录，保持两侧文档完全一致。**除非该目录不存在**（此时跳过同步，但需在提交信息里注明"ESP32S3/docs 目录不存在，未同步"）。AI Agent 完成协议修改后，必须执行此同步步骤，并在提交说明中记录同步状态。

19. **【触发词】"同步协议"**：当用户说"同步协议"（或"同步协议文档"、"同步到 ESP32S3"等同类表述）时，AI Agent 必须立即执行以下完整动作，无需再次确认：
    1. **更新本地协议文档**：根据当前固件代码（`task/modbusTask.c` 的寄存器布局、`buildInputSnapshot()` 的实际字段、`MODBUS_SLAVE_ADDRESS`、`MB_BAUD_RATE`、`supportedFunction` 白名单、波形窗口映射等）核对并更新 `code/通信协议说明.md` 与 `code/docs/站号2_AT32F415CBT7_Modbus协议.md`，使文档与代码完全一致（寄存器地址、数量、数据类型、字节序、功能码、异常码、测试命令、变更记录等）。
    2. **检查目标目录**：用 `Test-Path "D:\workspace\Code\Keil\NULLVE_V2\ESP32S3\docs"` 确认跨工程目录是否存在。
    3. **复制覆盖**（目录存在时）：把更新后的 `站号2_AT32F415CBT7_Modbus协议.md` 复制覆盖到 `D:\workspace\Code\Keil\NULLVE_V2\ESP32S3\docs\`，保持两侧文档完全一致。命令示例：
       ```powershell
       Copy-Item "d:\workspace\Code\Keil\NULLVE_V2\at32f415rbt7\code\docs\站号2_AT32F415CBT7_Modbus协议.md" "D:\workspace\Code\Keil\NULLVE_V2\ESP32S3\docs\站号2_AT32F415CBT7_Modbus协议.md" -Force
       ```
    4. **目录不存在时**：跳过复制，但在回复中明确提示"ESP32S3/docs 目录不存在，未同步"。
    5. **报告同步状态**：完成后向用户报告更新了哪些字段、复制是否成功、两侧文档是否一致。
    
    此触发词的目的是让用户用一句话即可驱动"代码→文档→跨工程同步"的完整闭环，避免文档与代码脱节。

20. **【强制】Modbus 寄存器地址只能追加，禁止修改已发布地址**：一旦某寄存器地址已在协议文档中发布并被上位机/主站使用，该地址的**语义、数据类型、字节序、字段含义即永久冻结**，后续任何变更只能从当前末尾追加新地址，严禁以下行为：
    - ❌ **中间插入**：在已发布地址区间内塞入新字段（会顶替后续所有地址）；
    - ❌ **重排顺序**：调整已发布字段的先后位置；
    - ❌ **复用地址**：把废弃字段的位置挪作他用；
    - ❌ **改语义/类型**：把原 int16 改成 float、把原"温度"改成"电流"等（即使地址不变）；
    - ❌ **删除地址**：移除已发布字段（会导致后续地址前移）。
    
    ✅ **唯一允许的变更方式**：在当前各区的末尾追加新地址。各区当前末尾：
    - 线圈 Coils：`0x0009`（共 10 个），新字段从 `0x000A` 起；
    - 离散输入：`0x000F`（共 16 个），新字段从 `0x0010` 起；
    - 保持寄存器：`0x0066`（共 103 个），新字段从 `0x0067` 起；
    - 输入寄存器：`0x004F`（共 80 个），新字段从 `0x0050` 起；
    - 波形窗口：`0x3F47`（合法范围 `8000~16199`），新通道从 `0x3F48`（=16200）起，但需注意单帧 125 寄存器上限与未映射间隙规则。
    
    ⚠️ **"已发布"的判定**：只要该地址出现在 `code/通信协议说明.md`、`code/docs/站号2_AT32F415CBT7_Modbus协议.md` 或 `ESP32S3/docs/站号2_AT32F415CBT7_Modbus协议.md` 任一文档中，即视为已发布，不得修改。即便文档与代码不一致（如本次同步发现的波形窗口 B 通道地址偏差），也应以**代码实际行为**为准修正文档，而非反向改代码迁就旧文档——但此类修正必须在变更记录中显式标注"修正历史文档错误"，与"新增功能追加"区分开。
    
    此规则与第 17、18 条配合：第 17 条规定布局遵循参考文档，第 18 条规定协议变更必须先更新文档再改代码并同步到 ESP32S3，本条（第 20 条）规定已发布地址永久冻结、只能末尾追加。三者共同保证上位机/主站与固件的二进制兼容性，避免因地址重排导致现场设备通信失效。

# -*- coding: utf-8 -*-
"""
57 步进电机 + 42 抬升电机 + 泵 / 阀 / LED / 传感器 上位机
========================================================
串口协议：**标准 Modbus RTU 主站**（USART3 RS485，从站地址 2，默认 1000000 8N1）

  波特率下拉框可选 9600 ~ 1000000（含 1M）；**必须与固件一致**，
  改完记得同步 project/src/wk_usart.c、code.ATWP 与 MB_BAUD_RATE。

  功能码（与固件 task/modbusTask.c 完全对应）：
    FC01 读线圈        coils 0..15      （0..2 泵, 3/4/5 57 正/反/停,
                                         6/7/8 42 正/反/停, 9/10 阀开/关, 11/12 灯开/关）
    FC02 读离散输入    discrete 0..21（0..15 保持原映射，16..21 为新增状态）
    FC03 读保持寄存器  holding 0..127（实际使用 0..15）
    FC04 读输入寄存器  input 0..70（浮点为大端 IEEE754 双字：高字在前）
    FC05 写单线圈      values 0x0000 / 0xFF00
    FC06 写单寄存器
    FC0F 写多线圈
    FC10 写多寄存器

  保持寄存器写入语义（FC06 / FC10）：
    0/1/2  泵速度（有符号，0=停）
    3      57 电机转速（rpm×10，0=停）
    4      42 抬升电机转速（rpm×10，0=停）
    5/6/7  WS2812 RGB（写任一即刷新三色）
    8      开关阀（非 0=开，0=关）
    9      LED 总开关（非 0=亮，0=灭）
    10     57 电流幅值 (%)
    11     PC10 RST 电平（非 0=高 moterEN，0=低 moterDIS）
    14     ADC 采集启动（512 或 1024）

  输入寄存器映射（FC04）：
    0..1    温度 (float)        2..3   母线电压 (float)
    4..5    母线电流 (float)    6..7   压制圈数 (float)
    8       离散位掩码          9      线圈位掩码
    10/11/12 清水/卤水/蠕动泵速度
    13/14   holding[3] / holding[4]
    15      母线电流 ADC 原始值
    16..21  诊断计数器 (queue/crc/frame 各 32 位，高字在前)
    32..39  TMUX D1 枚举值      40..47 TMUX D2 枚举值
    48      57 电流幅值         49      RST 电平 (0/1)
    50      采集就绪            51      已采集样本数
    52      当前回放下标        53/54  回放样本 ADC_A / ADC_B
    55..70  8 路绕组电流 (float)

运行环境: conda plotPic (pyserial + matplotlib + tkinter)
  C:\\Users\\29546\\.conda\\envs\\plotPic\\python.exe motor57_gui.py
"""

import re
import time
import queue
import csv
import os
import threading

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.ticker import FuncFormatter

# ---------------- 常量 ----------------
# 链路实际波特率由固件 project/src/wk_usart.c: wk_usart3_init() 决定，
# 当前固件 wk_usart3_init() = **1000000** 8N1。
# 改动波特率时必须三处同步：wk_usart.c 的 usart_init()、code.ATWP 的 BaudRate、
# 以及 task/modbusTask.c 的 MB_BAUD_RATE（RS485 方向脚三字符延时依赖它）。
DEFAULT_BAUD = 1000000
# 候选按升序排列；1000000 是固件当前值（排在下拉框里才不会被切走后回不来），
# 1000000 即 1M；中间几档是历史/备用值。
BAUD_CHOICES = ["9600", "19200", "38400", "57600", "100000",
                "115200", "230400", "460800", "921600", "1000000"]
# 1 Mbaud（1000000）在 APB1=75MHz 下分频值恰为 75，理论误差 0%；
# 921600 实际为 925926（+0.47%）。两者都需要固件同步改到同一值才能通信。
MAX_POINTS = 2000          # 波形滚动窗口点数
PLOT_REFRESH_MS = 50       # 绘图刷新周期
PLOT_MAX_POINTS = 30000     # 仅限制屏幕绘制量；完整采集历史仍保留，可导出 CSV
# Modbus 变量刷新周期（原为硬编码 500ms，明显偏慢）。
# 每轮 = FC04(0,72) 快照 + FC02(0,22)，用于刷新原有与新增离散状态。
# 当前链路 100000 baud（10 bit/字节 = 100µs）下一轮约 250 字节 ≈ 25ms，
# 所以 20ms 下限只对 921600 成立；选 100000 时实际周期会被串口时间撑到约 25ms。
# 下限不可再小，否则串口 I/O 长时间占用 Tk 主线程，界面反而卡顿。
POLL_INTERVAL_MS = 100
POLL_INTERVAL_MIN_MS = 20
POLL_INTERVAL_MAX_MS = 2000
CUR_RE = re.compile(r"^CUR,(\d+),(-?[\d.]+),(-?[\d.]+)(?:,(\d+),(\d+))?\s*$")
INFO_RE = re.compile(r"^INFO,(-?\d+),(\d+),(-?\d+),(\d+)(?:,(-?\d+),(-?\d+),(-?\d+),(\d+))?\s*$")
CAP_BEGIN_RE = re.compile(r"^CAPBEGIN,(\d+),(\d+),(\d+)\s*$")
CAP_RE = re.compile(r"^CAP,(\d+),(\d+),(\d+)\s*$")
SENS_RE = re.compile(r"^SENS,")
TMUX_RE = re.compile(r"^TMUX,")

# 保持寄存器地址（与固件 writeHoldingRegister() 的 switch 分支一一对应）
# 泵 ID 与寄存器地址并不一致：0=软管(hose)→2, 1=清水(water)→0, 2=卤水(brine)→1
HREG_PUMP = {0: 2, 1: 0, 2: 1}     # 泵 ID → 保持寄存器地址
HREG_M57_SPEED = 3                 # 57 电机转速（rpm×10）
HREG_M42_SPEED = 4                 # 42 抬升电机转速（rpm×10）
HREG_M57_ACCEL = 16                # 57 S 曲线加速时间（ms，扩展寄存器）
HREG_M57_START = 17                # 57 S 曲线起步速度（rpm×10，扩展寄存器）
HREG_RGB = 5                       # WS2812 RGB（5=R, 6=G, 7=B，写任一即刷新）
HREG_VALVE = 8                     # 开关阀（0=关，非 0=开）
HREG_LED = 9                       # LED 总开关（0=灭，非 0=亮）
HREG_AMP = 10                      # 57 电流幅值 (%)
HREG_RST = 11                      # PC10 RST 电平
HREG_CAPTURE = 14                  # ADC 采集启动（512 / 1024）
HREG_AMP42 = 18                    # 42 抬升电机电流幅值 (%)，运行中可实时调整

# 57 绕组电流波形（FC04 高地址窗口，映射见固件 task/modbusTask.c 文件头）
WAVE_REG_BASE = 8000               # 窗口基址
WAVE_CH_A_OFFSET = 4               # 绕组 A 首点偏移（8004）
WAVE_CH_B_OFFSET = 4604            # 绕组 B 首点偏移（12604）
WAVE_MAX_SAMPLES = 1024            # 与固件 STEPPER57_ADC_CAPTURE_MAX_SAMPLES 一致
WAVE_CHUNK = 125                   # Modbus 单帧最多 125 个寄存器
WAVE_ADC_PER_AMP = 0.002015        # ADC→安培系数，与固件 getMoter57Curr() 一致
WAVE_SAMPLE_RATE = 40000.0         # 采样率 40 kHz（TMR4 触发 ADC 注入组）
WAVE_REG_MARKER = 1                # 窗口 8001（有效标志）固定为 1，用于探测寄存器段是否已实现

# 线圈地址（与固件 writeCoil() 的 switch 分支一一对应，均为"一次性动作"）
COIL_M57_RUN_POS = 3               # 57 正向运行（用 holding[3]）
COIL_M57_RUN_NEG = 4               # 57 反向运行
COIL_M57_STOP = 5                  # 57 停止
COIL_M42_RUN_POS = 6               # 42 正向运行（用 holding[4]）
COIL_M42_RUN_NEG = 7               # 42 反向运行
COIL_M42_STOP = 8                  # 42 停止
COIL_VALVE_ON = 9                  # 开阀
COIL_VALVE_OFF = 10                # 关阀
COIL_LED_ON = 11                   # LED 亮
COIL_LED_OFF = 12                  # LED 灭
# 新增机构动作；保留既有 0~15 线圈地址不变。
COIL_RISE_UP = 16                  # 抬升电机上行
COIL_RISE_DOWN = 17                # 抬升电机下行
COIL_RISE_STOP = 18                # 抬升电机停止
COIL_PRESS_UP = 19                 # 压制电机上行
COIL_PRESS_DOWN = 20               # 压制电机下行
COIL_PRESS_STOP = 21               # 压制电机停止
COIL_M42_SAME_POS = 22             # 双42机械同向正转
COIL_M42_SAME_NEG = 23             # 双42机械同向反转
COIL_M42_SAME_STOP = 24            # 双42机械同向停止

# 线圈位掩码显示名（下标 = 线圈地址）
COIL_NAMES = ["清水泵", "卤水泵", "蠕动泵", "57正转", "57反转", "57停止",
              "42正转", "42反转", "42停止", "阀开", "阀关", "灯亮",
              "灯灭", "28正转", "28反转", "28停止",
              "抬升上行", "抬升下行", "抬升停止",
              "压制上行", "压制下行", "压制停止",
              "42同向正转", "42同向反转", "42同向停止"]

# 离散输入显示名（下标 = 离散地址，顺序同固件 readDiscrete()）
DISCRETE_NAMES = ["抬升高位R", "抬升低位L", "压制高位", "压制低位",
                  "压制在位", "清水在位", "水位高", "水位>2800",
                  "废料盒在位", "阀动中", "阀关到位", "卤水在位",
                  "溢流", "盖子在位", "滤芯(未实现)", "模式在位",
                  "左抬升低位(新增)", "左抬升高位(预留)",
                  "右抬升低位(预留)", "右抬升高位(新增)",
                  "压制高位状态(新增)", "压制低位状态(新增)"]

# 传感器通道名称（与固件 hostif.c hostifSendSensFrame 输出顺序一致）
SENS_CUR_NAMES = ["母线", "LED", "刹车", "应变", "右升B", "右升A", "左升B", "左升A"]

# TMUX1308 显示名：严格对应 app/tmux1308.h 的枚举顺序与注释（S0~S7）。
SENS_D0_NAMES = ["压制在位 S18", "卤水盒在位 S15", "点卤盒低水位 S6", "黄水盒在位 S8",
                 "阀故障 Fault-M7", "水位高 S14", "盖子在位 S16", "水箱在位 S2"]
SENS_D1_NAMES = ["压制电机故障 Fault-M4", "右抬升线圈B电流 AD-M12",
                 "右抬升线圈A电流 AD-M11", "成型通道在位 S13",
                 "右抬升电机故障 Fault-M1", "左抬升电机故障 Fault-M2",
                 "左抬升线圈B电流 AD-M22", "左抬升线圈A电流 AD-M21"]
SENS_D2_NAMES = ["筒壁NTC2 AIN8", "筒壁NTC3 AIN7", "预留NTC AIN6", "筒壁NTC1 AIN4",
                 "水位 AIN9", "盖子1 AIN2", "溢流 AIN3", "盖子2 AIN5"]

# 泵 ID 与显示名（与固件 hostif.c 中的 HOSTIF_PUMP_* 保持一致）
PUMPS = [(0, "软管泵"), (1, "清水泵"), (2, "卤水泵")]

# ---------------- DMA + 空闲中断压力测试常量 ----------------
# 与固件 hostif.c 保持一致：HOSTIF_RX_BUF_SIZE = 128，UART5 @ 115200 8N1
DMA_RX_BUF = 128
BAUD_UART5 = 115200
BYTE_TIME_MS = 10.0 / BAUD_UART5       # 单字节理论时长（ms），约 0.087ms
# 填充命令统一用无副作用的 STOP，避免压力测试过程中意外驱动电机/泵。
FILLER = "STOP"
FILLER_STEP_BYTES = 5                  # "STOP\n"


def _filler_bytes(total):
    """生成恰好 total 字节的填充数据（多行「STOP」命令，\\n 分隔）。

    末尾可能落在命令中间（不以 \\n 结尾），用于制造「被截断的尾帧」。
    """
    raw = b""
    while len(raw) < total:
        raw += FILLER.encode("ascii") + b"\n"
    return raw[:total]


class DmaCase:
    """一个 DMA/空闲中断极端场景的测试用例。

    纯上位机黑盒测试：固件只回 OK / ERR / 无响应（超时），
    因此期望值用"应该收到的应答列表"来表达，断言实际列表是否匹配。
    """

    def __init__(self, key, name, desc, kind="ok", expect=(), **kw):
        self.key = key              # 用例唯一键
        self.name = name            # 显示名
        self.desc = desc            # 中文说明（说明在测固件的什么行为）
        self.expect = list(expect)  # 期望收到的应答（按顺序）；空表示不做精确比对
        self.kind = kind            # 判读方式
        self.opts = kw              # 用例参数


# 全部用例定义（顺序即界面顺序，覆盖情况 1/2/3 与边界/并发）
# 波形类用例不用 expect 断言，由 kind 决定判读方式：
#   expect 列表中的符号： OK=应答 OK，ERR=应答 ERR，TIMEOUT=预期无应答
#   kind： error_tolerant = 只要不异常复位/死锁即通过（结果仅供参考）
#          single_ack     = 期望只收到 1 条应答
#          throughput     = 统计应答数量与速率，判定是否跟得上
DMA_CASES = [
    DmaCase(
        "normal", "0. 基准：整帧一次发送",
        "对照组。一次写入完整帧「STOP\\n」，应精确收到 1 条 OK。"
        "后续所有异常用例的结果都应与本条对比。",
        expect=["OK"],
        mode="multi",
        payload=b"STOP\n",
    ),
    DmaCase(
        "split_one", "1.1 分段发送：逐字节",
        "情况1。把「STOP\\n」拆成 5 次单字节写入，间隔远超 1 字节时间。"
        "空闲中断提前触发后每字节都被单独锁存，固件无法拼出命令，应逐段回 ERR。",
        kind="error_tolerant",
        mode="split_bytes",
        payload=b"STOP\n",
        gap_ms=0.6,
    ),
    DmaCase(
        "split_two", "1.2 分段发送：前后两半",
        "情况1。把「STOP\\n」拆成「ST」+「OP\\n」，中间停顿 >1 字节时间。"
        "前半段被空闲中断提前锁存并当成未知命令处理。",
        kind="error_tolerant",
        mode="split_two",
        payload=b"STOP\n",
        gap_ms=1.5,
    ),
    DmaCase(
        "split_slow", "1.3 分段发送：慢速逐字节",
        "情况1。每 5ms 发 1 字节，共 5 字节。模拟上位机逐字符发送（如工控 HMI），"
        "验证固件在每次空闲中断都重启 DMA 后不会粘包也不死锁。",
        kind="error_tolerant",
        mode="split_bytes",
        payload=b"STOP\n",
        gap_ms=5.0,
    ),
    DmaCase(
        "split_gap", "1.4 分段发送：长停顿后补发",
        "情况1 边界。发半截「STO」，停顿 200ms，再补「P\\n」。"
        "若固件在两次中断之间正确重启 DMA 且不粘包，验证其仍能给出确定应答。",
        kind="error_tolerant",
        mode="split_two",
        payload=b"STOP\n",
        gap_ms=200.0,
    ),
    DmaCase(
        "two_frames", "1.5 粘包：两帧同一批次",
        "两帧「STOP\\nSTOP\\n」零间隔连发，中间无空闲时间，应只触发一次空闲中断。"
        "固件只处理锁存到的第一行，期望收到 1 条 OK。",
        kind="single_ack",
        mode="multi",
        payload=b"STOP\nSTOP\n",
    ),
    DmaCase(
        "blank_line", "1.6 边界：只发一个换行",
        "只发「\\n」。固件解析后遇空行走「忽略」分支，不应回任何应答。",
        expect=["TIMEOUT"],
        mode="multi",
        payload=b"\n",
    ),
    DmaCase(
        "no_terminator", "1.7 边界：无换行结尾",
        "发「STOP」不带结尾换行。数据停发后空闲中断仍会触发，"
        "固件按锁存内容解析，应正常执行并回 OK。",
        expect=["OK"],
        mode="multi",
        payload=b"STOP",
    ),
    DmaCase(
        "stuffed_bad", "2.1 传输错误：批内掺垃圾字节",
        "情况2。发「STOP\\nXXX」，合法命令后跟乱码，中间无空闲时间。"
        "固件 hostifPoll 只处理锁存批次的首行，故只应答 1 条 OK，乱码行被丢弃（不报错）。",
        expect=["OK"],
        mode="multi",
        payload=b"STOP\nXXX\n",
    ),
    DmaCase(
        "bad_prefix", "2.2 传输错误：命令名被破坏",
        "情况2。发「STP\\n」，命令名少一个字符，不属于任何已知前缀。"
        "固件应回 ERR，绝不误执行任何动作。",
        expect=["ERR"],
        mode="multi",
        payload=b"STP\n",
    ),
    DmaCase(
        "split_number", "2.3 传输错误：行内嵌换行",
        "情况2。发「PERIOD:1\\n00」，命令中间被插入换行。无副作用命令（PERIOD 只改上报周期）。"
        "固件只取首行解析，值仍为 1，应回 OK；后段「00」连同本批一起被丢弃。",
        expect=["OK"],
        mode="multi",
        payload=b"PERIOD:1\n00\n",
    ),
    DmaCase(
        "over_tiny", "3.1 溢出边界：恰好 128 字节",
        "情况3 边界。发送恰好等于缓冲区的 128 字节。DMA 计数归零的临界点，"
        "验证「正好填满」时固件仍能锁存并处理，不丢最后一帧。",
        kind="error_tolerant",
        mode="burst",
        total=DMA_RX_BUF,
    ),
    DmaCase(
        "over_mid", "3.2 溢出：129 字节",
        "情况3。在 128 字节缓冲区基础上多发 1 字节。DMA 计数归零后不再搬运，"
        "验证缓冲区不溢出、软件不越界。",
        kind="error_tolerant",
        mode="burst",
        total=129,
    ),
    DmaCase(
        "over_half", "3.3 溢出：192 字节",
        "情况3。发送 192 字节（1.5 倍缓冲区）。验证 DMA 停止接收后超出部分被丢弃、"
        "固件不崩溃且能给出确定性应答。",
        kind="error_tolerant",
        mode="burst",
        total=192,
    ),
    DmaCase(
        "recover", "3.4 溢出后恢复",
        "先制造 192 字节溢出，再停 50ms 发一条正常「INFO」。"
        "验证溢出后固件能重新使能 DMA 并恢复正常应答。",
        kind="recover",
        mode="burst_recover",
        total=192,
        extra=b"INFO\n",
    ),
    DmaCase(
        "over_double", "3.5 溢出：256 字节整数倍",
        "情况3。发送 256 字节（2 倍缓冲区）。严重超量输入，"
        "验证固件不出现异常复位、能给出确定性应答。",
        kind="error_tolerant",
        mode="burst",
        total=256,
    ),
    DmaCase(
        "over_1k", "3.6 溢出压力：1024 字节",
        "情况3 压力。发送 1024 字节（8 倍缓冲区），长时间满速灌入。"
        "验证固件在 DMA 停止搬运后仍能响应后续命令（不出死锁）。",
        kind="recover",
        mode="burst_recover",
        total=1024,
        extra=b"STOP\n",
    ),
    DmaCase(
        "flood", "4.1 高频连发：100 条",
        "以 1ms 间隔连发 100 条「STOP\\n」，随后补发 1 条基准命令。"
        "统计应答率，验证缓冲区被反复快速占用时固件应答是否跟得上。",
        kind="throughput",
        mode="flood",
        count=100,
        gap_ms=1.0,
    ),
    DmaCase(
        "flood_fast", "4.2 高频连发：500 条零间隔",
        "零间隔连发 500 条命令，最恶劣的粘包/溢流叠加场景。"
        "验证固件不会因接收积压而丢失全部应答或造成死锁。",
        kind="throughput",
        mode="flood",
        count=500,
        gap_ms=0.0,
    ),
    DmaCase(
        "multi_max", "4.3 单批次最大合法负载",
        "构造恰好占满 128 字节、25 条以 \\n 结尾的「STOP」命令（末尾被截断）。"
        "验证单次空闲中断锁存后的行为：固件只处理首行，应回 1 条 OK，其余 24 行被丢弃。",
        kind="single_ack",
        mode="multi",
        payload=_filler_bytes(DMA_RX_BUF),
    ),
]

# 分组筛选：按钮显示名 -> 该组的用例 key 列表（显式列出，避免依赖中文名称前缀）
DMA_GROUPS = [
    ("情况1 分段", ["split_one", "split_two", "split_slow", "split_gap"]),
    ("情况2 传错", ["stuffed_bad", "bad_prefix", "split_number"]),
    ("情况3 溢出", ["over_tiny", "over_mid", "over_half", "over_double", "over_1k",
                    "recover"]),
    ("情况4 高频", ["flood", "flood_fast", "multi_max"]),
]


class SerialWorker(threading.Thread):
    """独立线程读串口，避免阻塞 GUI。"""

    def __init__(self, port, baud, rx_queue):
        super().__init__(daemon=True)
        self.port_name = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.rx_queue = rx_queue
        self.running = True
        self._rx_buf = b""
        # DMA 应力测试用：字节级收发统计
        self.lock = threading.Lock()
        self.bytes_tx = 0
        self.bytes_rx = 0
        self.lines = 0

    def write_raw(self, data):
        """写入原始字节（不追加 \\n，不做 ASCII 编码）。

        用于向固件发送任意畸形/超长/分段的字节流。
        """
        if isinstance(data, str):
            data = data.encode("latin-1", "replace")
        try:
            with self.lock:
                self.ser.write(data)
                self.bytes_tx += len(data)
            return True
        except (serial.SerialException, OSError):
            return False

    def send(self, line):
        """线程安全发送（pyserial 内部有锁）。"""
        try:
            self.ser.write((line + "\n").encode("ascii"))
        except serial.SerialException:
            pass

    def run(self):
        while self.running:
            try:
                data = self.ser.read(256)
            except (serial.SerialException, OSError):
                self.rx_queue.put(("__PORT_LOST__", None))
                return
            if not data:
                continue
            self.bytes_rx += len(data)
            self._rx_buf += data
            while b"\n" in self._rx_buf:
                line, self._rx_buf = self._rx_buf.split(b"\n", 1)
                line = line.strip().decode("ascii", "ignore")
                if line:
                    self.lines += 1
                    self.rx_queue.put(("LINE", line))

    def stop(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


class ModbusRtuMaster:
    """Synchronous Modbus RTU master for the USART3 RS485 slave (address 2)."""

    SLAVE = 2

    def __init__(self, port, baud):
        self.port_name = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0.35, write_timeout=0.35)
        self.lock = threading.Lock()

    @staticmethod
    def _crc(data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return crc

    def _read_exact(self, count):
        data = bytearray()
        while len(data) < count:
            chunk = self.ser.read(count - len(data))
            if not chunk:
                raise TimeoutError("Modbus response timeout")
            data.extend(chunk)
        return bytes(data)

    def request(self, function, payload=b""):
        frame = bytes([self.SLAVE, function]) + payload
        crc = self._crc(frame)
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame + bytes([crc & 0xFF, crc >> 8]))
            self.ser.flush()
            header = self._read_exact(2)
            if header[0] != self.SLAVE:
                raise ValueError("unexpected Modbus slave address")
            if header[1] & 0x80:
                code = self._read_exact(3)
                raise ValueError(f"Modbus exception {code[0]}")
            if header[1] != function:
                raise ValueError("unexpected Modbus function")
            if function in (0x01, 0x02, 0x03, 0x04):
                byte_count = self._read_exact(1)[0]
                body = bytes([byte_count]) + self._read_exact(byte_count + 2)
            else:
                body = self._read_exact(6)
            received = header + body
            got_crc = received[-2] | (received[-1] << 8)
            if self._crc(received[:-2]) != got_crc:
                raise ValueError("Modbus CRC error")
            return received[2:-2]

    def read_input_fragmented(self, address, count, chunks, gap_s=0.003):
        """Issue a legal 0x04 request in pieces, with an IDLE-sized gap.

        This exercises the USART3 IDLE ISR and its DMA assembly path.  The
        response parser deliberately remains identical to ``request()`` so a
        passing result proves that the slave reconstructed a valid RTU frame.
        """
        payload = address.to_bytes(2, "big") + count.to_bytes(2, "big")
        frame = bytes([self.SLAVE, 0x04]) + payload
        crc = self._crc(frame)
        raw = frame + bytes([crc & 0xFF, crc >> 8])
        with self.lock:
            self.ser.reset_input_buffer()
            offset = 0
            for size in chunks:
                if size <= 0 or offset >= len(raw):
                    continue
                self.ser.write(raw[offset:offset + size])
                self.ser.flush()
                offset += size
                if offset < len(raw):
                    time.sleep(gap_s)
            if offset < len(raw):
                self.ser.write(raw[offset:])
                self.ser.flush()
            header = self._read_exact(2)
            if header != bytes([self.SLAVE, 0x04]):
                raise ValueError(f"unexpected fragmented response header: {header!r}")
            byte_count = self._read_exact(1)[0]
            body = bytes([byte_count]) + self._read_exact(byte_count + 2)
            received = header + body
            if self._crc(received[:-2]) != (received[-2] | (received[-1] << 8)):
                raise ValueError("Modbus CRC error")
            if byte_count != count * 2:
                raise ValueError("invalid fragmented input-register response length")
            return [int.from_bytes(body[i:i + 2], "big") for i in range(1, len(body) - 2, 2)]

    def transact_raw(self, raw, function=0x04, expect_response=True, chunks=None, gap_s=0.0):
        """Send a pre-built RTU ADU and return its response, including exceptions."""
        with self.lock:
            self.ser.reset_input_buffer()
            if chunks:
                at = 0
                for size in chunks:
                    self.ser.write(raw[at:at + size]); self.ser.flush(); at += size
                    if at < len(raw) and gap_s:
                        time.sleep(gap_s)
                if at < len(raw): self.ser.write(raw[at:]); self.ser.flush()
            else:
                self.ser.write(raw); self.ser.flush()
            if not expect_response:
                return None
            header = self._read_exact(2)
            if header[0] != self.SLAVE:
                raise ValueError("unexpected slave address")
            if header[1] == (function | 0x80):
                body = self._read_exact(3)
                got = header + body
                if self._crc(got[:-2]) != (got[-2] | got[-1] << 8): raise ValueError("exception CRC")
                # USB-RS485 adapters may report the last response byte before
                # their driver has released the bus.  Keep a conservative
                # RTU inter-frame gap before the next sub-test request.
                time.sleep(0.002)
                return ("exception", body[0])
            if header[1] != function: raise ValueError("unexpected function")
            size = self._read_exact(1)[0] if function in (1, 2, 3, 4) else 4
            body = bytes([size]) + self._read_exact(size + 2) if function in (1, 2, 3, 4) else bytes([size]) + self._read_exact(5)
            time.sleep(0.002)
            return ("ok", body)

    def dma_counters(self):
        r = self.read_input(16, 6)
        return {"queue": (r[0] << 16) | r[1], "crc": (r[2] << 16) | r[3],
                "frame": (r[4] << 16) | r[5]}

    # ---------------- 57 绕组电流波形（FC04 高地址窗口） ----------------
    def _wave_window_implemented(self):
        """探测固件是否已实现 FC04 波形窗口。

        只读探测，不写任何寄存器，因此绝不会驱动泵/电机：
        当前固件把输入寄存器限制在 0~79，读 8000 会回 Modbus 异常码 2
        （非法地址）；实现了波形窗口的固件则返回实际采样数据。
        """
        try:
            head = self.read_input(WAVE_REG_BASE, 2)
            return head[1] in (0, WAVE_REG_MARKER)
        except (serial.SerialException, OSError, TimeoutError, ValueError):
            return False

    def read_wave_capture(self, sample_count):
        """约束采样数并下载整段 57 绕组 A/B 波形，返回 (samples, zero_a, zero_b)。

        samples = [(adc_a, adc_b), ...]，原始 ADC 值（未减零点）。
        耗时 ≈ 两组采样 × ⌈采样数/125⌉ 帧，1024 点约 18 帧 ≈ 60ms。
        """
        count = max(1, min(WAVE_MAX_SAMPLES, int(sample_count)))
        samples = []
        for start in range(0, count, WAVE_CHUNK):
            n = min(WAVE_CHUNK, count - start)
            chunk_a = self.read_input(WAVE_REG_BASE + WAVE_CH_A_OFFSET + start, n)
            chunk_b = self.read_input(WAVE_REG_BASE + WAVE_CH_B_OFFSET + start, n)
            samples.extend(zip(chunk_a, chunk_b))
        # 零点基准在窗口头部（8002/8003），读一次即可
        head = self.read_input(WAVE_REG_BASE, 4)
        return samples, head[2], head[3]

    def read_input(self, address, count):
        body = self.request(0x04, address.to_bytes(2, "big") + count.to_bytes(2, "big"))
        if not body or body[0] != count * 2:
            raise ValueError("invalid input-register response length")
        return [int.from_bytes(body[i:i + 2], "big") for i in range(1, len(body), 2)]

    def read_discrete(self, address, count):
        body = self.request(0x02, address.to_bytes(2, "big") + count.to_bytes(2, "big"))
        if not body or body[0] != (count + 7) // 8:
            raise ValueError("invalid discrete-input response length")
        return [(body[1 + i // 8] >> (i % 8)) & 1 for i in range(count)]

    def read_coils(self, address, count):
        """FC01: read coil on/off states (16 coils implemented by the slave)."""
        body = self.request(0x01, address.to_bytes(2, "big") + count.to_bytes(2, "big"))
        if not body or body[0] != (count + 7) // 8:
            raise ValueError("invalid coil response length")
        return [(body[1 + i // 8] >> (i % 8)) & 1 for i in range(count)]

    def read_holding(self, address, count):
        """FC03: read holding registers (the slave implements 0..15)."""
        body = self.request(0x03, address.to_bytes(2, "big") + count.to_bytes(2, "big"))
        if not body or body[0] != count * 2:
            raise ValueError("invalid holding-register response length")
        return [int.from_bytes(body[i:i + 2], "big") for i in range(1, len(body), 2)]

    def write_register(self, address, value):
        self.request(0x06, address.to_bytes(2, "big") + (value & 0xFFFF).to_bytes(2, "big"))

    def write_coil(self, address, on=True):
        """FC05: force a single coil. Note the slave maps 3..12 to one-shot
        actions (open/close split across two addresses), so writing 0 is a
        no-op for those addresses rather than the inverse action."""
        value = 0xFF00 if on else 0x0000
        self.request(0x05, address.to_bytes(2, "big") + value.to_bytes(2, "big"))

    def write_coils(self, address, bits):
        """FC0F: force multiple coils from an iterable of 0/1 values."""
        bits = [1 if b else 0 for b in bits]
        count = len(bits)
        if not count:
            raise ValueError("write_coils needs at least one coil")
        packed = bytearray((count + 7) // 8)
        for i, bit in enumerate(bits):
            if bit:
                packed[i // 8] |= 1 << (i % 8)
        payload = (address.to_bytes(2, "big") + count.to_bytes(2, "big")
                   + bytes([len(packed)]) + bytes(packed))
        self.request(0x0F, payload)

    def write_registers(self, address, values):
        payload = address.to_bytes(2, "big") + len(values).to_bytes(2, "big")
        payload += bytes([len(values) * 2]) + b"".join((v & 0xFFFF).to_bytes(2, "big") for v in values)
        self.request(0x10, payload)

    def stop(self):
        self.ser.close()

    def join(self, timeout=None):
        return None

    def write_raw(self, data):
        with self.lock:
            self.ser.write(data)
            self.ser.flush()


class ModbusDmaStress(threading.Thread):
    """RTU DMA stress worker; rounds may be configured up to 5,000,000."""
    def __init__(self, master, rounds, out_queue):
        super().__init__(daemon=True)
        self.master, self.rounds, self.out_queue = master, min(max(1, rounds), 5000000), out_queue
        self.stop_flag = threading.Event()
    def stop(self): self.stop_flag.set()
    def run(self):
        ok = fail = 0
        for n in range(1, self.rounds + 1):
            if self.stop_flag.is_set(): break
            try:
                # Normal request plus deliberately fragmented legal RTU request.
                self.master.read_input(0, 2)
                frame = bytes([2, 4, 0, 0, 0, 2]); c = self.master._crc(frame)
                raw = frame + bytes([c & 255, c >> 8])
                for b in raw: self.master.write_raw(bytes([b]))
                ok += 1
            except Exception: fail += 1
            if n % 100 == 0 or n == self.rounds:
                self.out_queue.put(("MODBUS_DMA_PROGRESS", n, ok, fail))
        self.out_queue.put(("MODBUS_DMA_DONE", n, ok, fail))


class DmaTestEngine(threading.Thread):
    """在后台线程里按顺序执行 DMA + 空闲中断极端场景用例。

    设计要点：
      - 发送端：用 worker.write_raw() 直接写原始字节，可精确控制分段、间隔与超量，
        绕开 "命令 + \\n" 的封装，从而制造各种畸形输入。
      - 接收端：GUI 线程收到固件应答后通过 feed_line() 塞进来（加锁），
        本线程只读快照，避免跨线程操作 Tk。
      - 结果通过 out_queue 回传给 GUI 线程（Tk 只能在主线程更新界面）。
    """

    def __init__(self, worker, cases, out_queue):
        super().__init__(daemon=True)
        self.worker = worker
        self.cases = list(cases)
        self.out_queue = out_queue
        self.stop_flag = threading.Event()
        self.resp_lock = threading.Lock()
        self.responses = []
        self.current_key = None

    # ---------- 供 GUI 线程调用 ----------
    def feed_line(self, line):
        """把固件应答交给引擎统计（GUI 线程调用）。"""
        with self.resp_lock:
            self.responses.append(line)

    def clear_responses(self):
        with self.resp_lock:
            self.responses = []

    def snapshot(self):
        with self.resp_lock:
            return list(self.responses)

    def stop(self):
        self.stop_flag.set()

    # ---------- 线程主体 ----------
    def run(self):
        for case in self.cases:
            if self.stop_flag.is_set():
                break
            self.current_key = case.key
            self.out_queue.put(("CASE_START", case.key, None, None, None))
            try:
                verdict, detail, elapsed = self._run_case(case)
            except Exception as exc:                       # 保证界面不会卡在"执行中"
                verdict, detail, elapsed = "error", f"测试脚本异常: {exc}", 0.0
            if self.stop_flag.is_set():
                verdict, detail = "stopped", "已手动停止"
            self.out_queue.put(("CASE_DONE", case.key, verdict, detail, elapsed))
        self.out_queue.put(("RUN_DONE", None, None, None, None))

    # ---------- 工具 ----------
    def _write(self, data):
        return self.worker.write_raw(data)

    def _sleep(self, ms):
        if ms > 0:
            self.stop_flag.wait(ms / 1000.0)

    def _settle(self, total_bytes, extra_ms=250.0):
        """等待固件处理完并应答。

        估算依据：满线速下 total_bytes 的收发往返时间，再留固定余量。
        """
        estimate = total_bytes * BYTE_TIME_MS * 2.0
        self._sleep(max(extra_ms, estimate + extra_ms))

    # ---------- 单个用例 ----------
    def _run_case(self, case):
        self.clear_responses()
        opts = case.opts
        mode = opts.get("mode", "multi")
        t_start = time.monotonic()
        total_tx = 0

        if mode == "multi":
            payload = opts["payload"]
            self._write(payload)
            total_tx += len(payload)
            self._settle(len(payload))

        elif mode == "split_two":
            payload = opts["payload"]
            cut = max(1, len(payload) // 2)
            self._write(payload[:cut])
            self._sleep(opts.get("gap_ms", 1.5))
            self._write(payload[cut:])
            total_tx += len(payload)
            self._settle(len(payload))

        elif mode == "split_bytes":
            payload = opts["payload"]
            gap = opts.get("gap_ms", 0.6)
            for i in range(len(payload)):
                self._write(payload[i:i + 1])
                total_tx += 1
                self._sleep(gap)
            self._settle(len(payload))

        elif mode == "burst":
            raw = _filler_bytes(opts["total"])
            for i in range(0, len(raw), 64):        # 64 字节一块，零间隔连发
                self._write(raw[i:i + 64])
            total_tx += len(raw)
            self._settle(len(raw))

        elif mode == "burst_recover":
            raw = _filler_bytes(opts["total"])
            for i in range(0, len(raw), 64):
                self._write(raw[i:i + 64])
            total_tx += len(raw)
            self._sleep(50.0)                       # 留出恢复时间
            extra = opts["extra"]
            self._write(extra)
            total_tx += len(extra)
            self._settle(len(raw) + len(extra))

        elif mode == "flood":
            cmd = b"STOP\n"
            gap = opts.get("gap_ms", 0.0)
            for _ in range(opts["count"]):
                if self.stop_flag.is_set():
                    break
                self._write(cmd)
                total_tx += len(cmd)
                if gap > 0:
                    self._sleep(gap)
            self._settle(total_tx)

        else:
            raise ValueError(f"未知测试模式: {mode}")

        elapsed = (time.monotonic() - t_start) * 1000.0
        return self._judge(case, self.snapshot(), elapsed, total_tx)

    # ---------- 判读 ----------
    @staticmethod
    def _judge(case, responses, elapsed, total_tx):
        ok = responses.count("OK")
        err = responses.count("ERR")
        other = [r for r in responses if r not in ("OK", "ERR")]
        kind = case.kind

        # 1) 有明确期望序列的用例，直接比对
        if case.expect:
            want = case.expect
            if want == ["TIMEOUT"]:
                if not responses:
                    return "pass", "固件无应答（符合预期）", elapsed
                return "fail", f"预期无应答，实际收到 {len(responses)} 条: {responses[:6]}", elapsed
            actual = [(r if r in ("OK", "ERR") else "OTHER") for r in responses]
            if actual == want:
                return "pass", f"应答序列与预期一致: {actual}", elapsed
            return "fail", f"预期 {want}，实际 {actual}", elapsed

        if other:
            return "fail", f"出现非法应答行: {other[:4]}", elapsed

        if kind == "single_ack":
            if len(responses) == 1:
                return "pass", f"只收到 1 条应答 ({responses[0]})，符合预期", elapsed
            if not responses:
                return "warn", "无任何应答（固件可能未触发或已死锁）", elapsed
            return "fail", f"收到 {len(responses)} 条应答，预期 1 条", elapsed

        if kind == "recover":
            if responses and responses[-1] == "OK":
                return "pass", f"溢出后恢复正常应答（溢出期间 OK={ok} ERR={err}）", elapsed
            if not responses:
                return "fail", "溢出后完全无应答，疑似死锁/未重新使能 DMA", elapsed
            return "fail", f"溢出后末条应答为 {responses[-1]}，未恢复正常", elapsed

        if kind == "throughput":
            if not responses:
                return "fail", f"发送 {total_tx} 字节，固件完全无应答", elapsed
            return "pass", f"发送 {total_tx} 字节，收到 OK={ok} ERR={err}", elapsed

        # kind == "error_tolerant"：只要不崩溃、不漏越、不应答垃圾即算通过
        if not responses:
            return "warn", f"发送 {total_tx} 字节无应答，请人工确认固件是否仍在运行", elapsed
        return "pass", f"未出现非法应答；OK={ok} ERR={err}", elapsed


# These cases intentionally target the firmware selected by main.c:
# USART3 / DMA1_CH1 / circular 256-byte buffer / Modbus RTU.  They do not use
# the retired UART5 ASCII host interface.
MODBUS_DMA_CASES = [
    DmaCase("normal", "单帧正常读 0x04", "发送一帧完整的输入寄存器读取请求，验证正常响应。"),
    DmaCase("byte", "单帧逐字节拆分", "逐字节发送，段间隔为 1 个字符时间，验证 IDLE 后的帧组装。"),
    DmaCase("parts", "两段、三段、随机分段", "覆盖两段、三段和随机分段，使用 1 字符、3.5 字符及长间隔。"),
    DmaCase("sticky", "多帧粘包与读写混合", "连续发送合法读请求与无副作用的写请求，验证粘包解析与恢复。"),
    DmaCase("bad", "坏 CRC、地址、功能码、长度", "发送 CRC 错误、地址/功能码/长度异常帧，再立即发送正常帧验证恢复。"),
    DmaCase("illegal", "非法寄存器地址与非法数量", "发送越界地址、零数量和超范围数量，期望收到 Modbus 异常响应。"),
    DmaCase("ring", "环形 DMA 临界与溢出恢复", "覆盖 255/256/257 字节临界，以及 1.5 倍、2 倍、8 倍缓冲区连续灌入后的恢复。"),
    DmaCase("queue", "连续高频与请求队列压力", "不等待响应连续发送合法读请求，制造队列压力后验证正常帧恢复。"),
    DmaCase("random", "多轮随机分段和随机间隔", "随机拆分合法帧并随机选取段间隔，验证长期稳定性。"),
]
MODBUS_DMA_GROUPS = [
    ("拆帧与随机", ["byte", "parts", "random"]), ("异常与恢复", ["bad", "illegal", "ring"]),
    ("粘包与压力", ["sticky", "queue"]),
]


class ModbusDmaTestEngine(threading.Thread):
    """Repeated black-box suite for USART3 DMA1_CH1 Modbus RTU."""
    def __init__(self, master, cases, out_queue, rounds=1):
        super().__init__(daemon=True)
        self.master, self.cases, self.out_queue = master, list(cases), out_queue
        self.rounds = min(max(1, int(rounds)), 5000000)
        self.stop_flag = threading.Event()
        self.stats = {c.key: {"pass": 0, "fail": 0, "timeout": 0, "crc": 0, "exception": 0, "ms": 0.0} for c in self.cases}

    def stop(self):
        self.stop_flag.set()

    def _adu(self, function=4, address=0, value=2):
        p = bytes([2, function]) + address.to_bytes(2, "big") + value.to_bytes(2, "big")
        c = self.master._crc(p); return p + bytes([c & 255, c >> 8])

    def _ok_read(self, chunks=None, gap=0):
        result = self.master.transact_raw(self._adu(), chunks=chunks, gap_s=gap)
        if result[0] != "ok": raise ValueError(f"unexpected exception {result[1]}")

    def _recover(self):
        time.sleep(0.004); self._ok_read()

    def _run_one(self, case):
        start = time.monotonic()
        k = case.key; raw = self._adu()
        if k == "normal": self._ok_read()
        elif k == "byte": self._ok_read([1] * 8, 10 / self.master.baud)
        elif k == "parts":
            self._ok_read([4, 4], 35 / self.master.baud); self._ok_read([2, 3, 3], 0.003)
            cuts = [1, 1, 2, 1, 3]; self._ok_read(cuts, 35 / self.master.baud)
        elif k == "sticky":
            write = self._adu(6, 12, 0)  # harmless hose-pump speed=0
            self.master.write_raw(raw + write); time.sleep(0.01); self._recover()
        elif k == "bad":
            # Every injected bad request is a complete RTU ADU.  Do not leave
            # a truncated frame in the MCU assembler, otherwise it corrupts
            # the following test rounds and produces misleading timeouts.
            bad = bytearray(raw); bad[-1] ^= 1
            self.master.write_raw(bytes(bad)); self._recover()
            # Function 0x10 with quantity=1 but byte-count=0: complete frame,
            # expected Modbus exception 0x03 (illegal data value).
            malformed = bytes([2, 0x10, 0, 0, 0, 1, 0])
            crc = self.master._crc(malformed)
            result = self.master.transact_raw(malformed + bytes([crc & 255, crc >> 8]), function=0x10)
            if result[0] != "exception": raise ValueError("长度错误帧未返回异常响应")
            self._recover()
            result = self.master.transact_raw(self._adu(0x41), function=0x41)
            if result[0] != "exception": raise ValueError("非法功能码未返回异常响应")
            self._recover()
            self.stats[k]["crc"] += 1
        elif k == "illegal":
            for a, q in ((80, 1), (0, 0), (0, 126)):
                r = self.master.transact_raw(self._adu(4, a, q));
                if r[0] != "exception": raise ValueError("illegal request not rejected")
                self.stats[k]["exception"] += 1
        elif k == "ring":
            for n in (255, 256, 257, 384, 512, 2048): self.master.write_raw(b"\x55" * n); self._recover()
        elif k == "queue":
            self.master.write_raw(raw * 12); self._recover()
        elif k == "random":
            import random
            left = 8; chunks = []
            while left: n = random.randint(1, left); chunks.append(n); left -= n
            self._ok_read(chunks, random.choice((10, 35, 3000)) / self.master.baud)
        return "pass", "OK", (time.monotonic() - start) * 1000

    def run(self):
        before = self.master.dma_counters()
        done = 0
        last_report = 0.0
        for rnd in range(self.rounds):
            for case in self.cases:
                if self.stop_flag.is_set(): break
                try: verdict, detail, elapsed = self._run_one(case)
                except TimeoutError as exc: verdict, detail, elapsed = "timeout", str(exc), 0
                except Exception as exc: verdict, detail, elapsed = "fail", str(exc), 0
                s = self.stats[case.key]; s[verdict if verdict in s else "fail"] += 1; s["ms"] += elapsed
            if self.stop_flag.is_set(): break
            done = rnd + 1
            now = time.monotonic()
            if now - last_report >= 0.2 or done == self.rounds:
                self.out_queue.put(("SUITE_PROGRESS", done, self.rounds, self.stats, None))
                last_report = now
        after = self.master.dma_counters()
        self.out_queue.put(("SUITE_DONE", done, before, after, self.stats))


class Motor57Gui:
    def __init__(self, root):
        self.root = root
        root.title("57 步进电机 / 泵 / RST 上位机")

        # 字体：优先使用含中文字形的等宽字体，避免中文显示为方框。
        # Consolas 不含 CJK 字形，纯中文标签必须用中文字体。
        self.font_mono = self._pick_font([
            "Consolas", "Cascadia Mono", "Courier New", "SimHei", "Microsoft YaHei",
            "DejaVu Sans Mono", "TkFixedFont",
        ])
        self.font_cjk = self._pick_font([
            "Microsoft YaHei UI", "Microsoft YaHei", "SimHei", "SimSun",
            "Noto Sans CJK SC", "DejaVu Sans",
        ])
        # matplotlib 中文字体
        matplotlib.rcParams["font.sans-serif"] = [
            self.font_cjk, "Microsoft YaHei", "SimHei", "DejaVu Sans",
        ]
        matplotlib.rcParams["axes.unicode_minus"] = False

        # 窗口自适应屏幕：先按自然尺寸，若超出屏幕则收缩并允许缩放
        self._fit_window()

        self.worker = None
        self.rx_queue = queue.Queue()
        self.stream_on = False

        # Modbus 变量轮询状态（周期可调，见 POLL_INTERVAL_MS）
        self.sens_auto_on = False
        self.poll_interval_ms = POLL_INTERVAL_MS
        self.poll_after_id = None
        self.poll_busy = False      # 重入保护：上一轮未结束时跳过本轮

        # 波形数据（deque 语义用 list + 裁剪）
        self.t_data = []
        self.ia_data = []
        self.ib_data = []
        self.t0 = None
        self.capture_zero_a = 0
        self.capture_zero_b = 0
        self.auto_capture = False
        self.capture_sample_count = 1024
        self.capture_time_base = 0.0
        self.capture_thread = None  # 波形下载后台线程（见 _capture_worker）
        self.capture_busy = False    # GUI 线程侧的下载互斥标志（比 is_alive 可靠）

        # DMA 压力测试（独立窗口，可重复开关）
        self.dma_win = None
        self.dma_engine = None
        self.dma_queue = queue.Queue()
        self.dma_results = {}       # key -> (verdict, detail, elapsed_ms)
        self.dma_stats = {}
        self.dma_mcu_before = None
        self.dma_mcu_after = None
        self.dma_row_iid = {}       # key -> Treeview iid
        self.dma_lines = []         # 测试期间收到的应答原文
        self.dma_log = None         # 窗口未打开时为 None
        self.dma_detail = None
        self.dma_resume_poll = False

        self._build_ui()
        self._refresh_ports()
        self.poll_queue_after_id = None
        self.poll_queue_after_id = self.root.after(PLOT_REFRESH_MS, self._poll_queue)
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI ----------------
    def _build_ui(self):
        # 顶部：串口连接区（主窗口与 DMA 测试窗口共用同一个串口连接）
        conn = ttk.LabelFrame(self.root, text="连接：串口控制", padding=6)
        conn.pack(fill=tk.X, padx=8, pady=(8, 4))

        ttk.Label(conn, text="串口:").grid(row=0, column=0, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(conn, textvariable=self.port_var, width=10, state="readonly")
        self.port_cb.grid(row=0, column=1, padx=4)

        ttk.Label(conn, text="波特率:").grid(row=0, column=2, sticky=tk.W)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.baud_cb = ttk.Combobox(conn, textvariable=self.baud_var, width=9,
                                    state="readonly",
                                    values=BAUD_CHOICES)
        self.baud_cb.grid(row=0, column=3, padx=4)

        self.btn_refresh = ttk.Button(conn, text="刷新", width=6, command=self._refresh_ports)
        self.btn_refresh.grid(row=0, column=4, padx=4)

        self.btn_connect = ttk.Button(conn, text="连接", width=8, command=self._toggle_connect)
        self.btn_connect.grid(row=0, column=5, padx=4)

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(conn, textvariable=self.status_var, foreground="gray").grid(
            row=0, column=6, padx=8, sticky=tk.W)

        # 右侧：独立打开 DMA 压力测试窗口（与参数获取界面完全分开）
        self.btn_open_dma = ttk.Button(conn, text="DMA 压力测试…",
                                       command=self._open_dma_window)
        self.btn_open_dma.grid(row=0, column=7, padx=(12, 4))

        # 主界面分为两页：波形页保留完整的 57 控制；其他执行器和诊断
        # 集中到另一页，避免它们遮挡或压缩波形。
        self.main_pane = ttk.Notebook(self.root)
        self.main_pane.pack(fill=tk.BOTH, expand=True)
        tab_wave = ttk.Frame(self.main_pane)
        tab_other = ttk.Frame(self.main_pane)
        self.main_pane.add(tab_wave, text="57 电机 / 电流波形")
        self.main_pane.add(tab_other, text="其他控制与诊断")

        wave_controls = ttk.Frame(tab_wave)
        wave_controls.pack(side=tk.LEFT, fill=tk.Y)
        wave_pane = ttk.Frame(tab_wave)
        wave_pane.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 分列容器：把矮面板纵向堆叠到同一列，避免 6 个面板横向铺开
        # 导致自然宽度（1905px）远超窗口（1440px）而被裁切。
        col_ctrl = ttk.Frame(wave_controls)
        col_ctrl.pack(fill=tk.BOTH, expand=True)
        col_pump = ttk.Frame(tab_other)
        col_pump.pack(side=tk.LEFT, fill=tk.Y)
        # 42 与阀/LED 单独成列，避免被 57、28、泵面板占用纵向空间。
        col_lift = ttk.Frame(tab_other)
        col_lift.pack(side=tk.LEFT, fill=tk.Y)
        col_diag = ttk.Frame(tab_other)
        col_diag.pack(side=tk.LEFT, fill=tk.Y)

        # 第 1 列：电机控制区
        ctrl = ttk.LabelFrame(col_ctrl, text="57 电机控制", padding=4)
        # 不固定高度：Windows 的显示缩放不一致时，固定像素高度会裁切
        # 下方的波形/状态控件。fill=X 且 expand=False 已可避免占用多余纵向空间。
        ctrl.pack(side=tk.TOP, fill=tk.X, expand=False, padx=(8, 4), pady=(4, 0))

        # 速度
        ttk.Label(ctrl, text="速度 (rpm×10):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.speed_var = tk.StringVar(value="150")
        sp = ttk.Spinbox(ctrl, from_=-15000, to=15000, increment=10, width=9,
                         textvariable=self.speed_var)
        sp.grid(row=0, column=1, padx=4, pady=2)

        self.speed_scale = ttk.Scale(ctrl, from_=-1500, to=1500, orient=tk.HORIZONTAL,
                                     length=130, command=self._on_speed_scale)
        self.speed_scale.set(150)
        self.speed_scale.grid(row=1, column=0, columnspan=2, pady=(0, 6), sticky=tk.EW)

        self.btn_run = ttk.Button(ctrl, text="启动 RUN", command=self._cmd_run)
        self.btn_run.grid(row=2, column=0, padx=2, pady=2, sticky=tk.EW)
        self.btn_stop = ttk.Button(ctrl, text="停止 STOP", command=self._cmd_stop)
        self.btn_stop.grid(row=2, column=1, padx=2, pady=2, sticky=tk.EW)

        # 幅值
        ttk.Label(ctrl, text="电流幅值 (%):").grid(row=3, column=0, sticky=tk.W, pady=(6, 2))
        self.amp_var = tk.StringVar(value="10")
        ttk.Spinbox(ctrl, from_=0, to=100, increment=1, width=9,
                    textvariable=self.amp_var).grid(row=3, column=1, padx=4)
        self.btn_amp = ttk.Button(ctrl, text="应用幅值 AMP", command=self._cmd_amp)
        self.btn_amp.grid(row=4, column=0, columnspan=2, pady=2, sticky=tk.EW)

        # 波形控制
        sep = ttk.Separator(ctrl, orient=tk.HORIZONTAL)
        sep.grid(row=5, column=0, columnspan=2, sticky=tk.EW, pady=6)

        ttk.Label(ctrl, text="轮询周期 (ms):").grid(row=6, column=0, sticky=tk.W, pady=2)
        self.period_var = tk.StringVar(value=str(POLL_INTERVAL_MS))
        ttk.Spinbox(ctrl, from_=POLL_INTERVAL_MIN_MS, to=POLL_INTERVAL_MAX_MS,
                    increment=10, width=9,
                    textvariable=self.period_var).grid(row=6, column=1, padx=4)
        self.btn_period = ttk.Button(ctrl, text="应用轮询周期", command=self._cmd_period)
        self.btn_period.grid(row=7, column=0, columnspan=2, pady=2, sticky=tk.EW)

        self.btn_stream = ttk.Button(ctrl, text="下载波形 (40kHz)", command=self._toggle_stream)
        self.btn_stream.grid(row=8, column=0, columnspan=2, pady=2, sticky=tk.EW)

        self.btn_capture_512 = ttk.Button(ctrl, text="采集 512",
                                          command=lambda: self._cmd_capture(512))
        self.btn_capture_512.grid(row=9, column=0, pady=2, sticky=tk.EW)
        self.btn_capture_1024 = ttk.Button(ctrl, text="采集 1024",
                                           command=lambda: self._cmd_capture(1024))
        self.btn_capture_1024.grid(row=9, column=1, pady=2, sticky=tk.EW)

        self.btn_auto_capture = ttk.Button(ctrl, text="自动采集: OFF",
                                           command=self._toggle_auto_capture)
        self.btn_auto_capture.grid(row=10, column=0, columnspan=2, pady=2, sticky=tk.EW)

        self.btn_info = ttk.Button(ctrl, text="查询状态 INFO", command=self._cmd_info)
        self.btn_info.grid(row=11, column=0, columnspan=2, pady=2, sticky=tk.EW)

        self.btn_clear = ttk.Button(ctrl, text="清空波形", command=self._clear_wave)
        self.btn_clear.grid(row=12, column=0, columnspan=2, pady=(6, 2), sticky=tk.EW)

        # 状态显示
        self.info_var = tk.StringVar(value="RPM: -  AMP: -  POS: -  EN: -")
        ttk.Label(ctrl, textvariable=self.info_var, foreground="blue").grid(
            row=13, column=0, columnspan=2, pady=(8, 0), sticky=tk.W)

        # ADC 原始值显示
        self.adc_var = tk.StringVar(value="ADC_A: -  ADC_B: -")
        ttk.Label(ctrl, textvariable=self.adc_var, foreground="#a0522d").grid(
            row=14, column=0, columnspan=2, pady=(2, 0), sticky=tk.W)

        self.btn_export_history = ttk.Button(ctrl, text="导出波形 CSV", command=self._export_history_csv)
        self.btn_export_history.grid(row=15, column=0, columnspan=2, pady=(8, 2), sticky=tk.EW)

        motion = ttk.LabelFrame(col_ctrl, text="57 / 28 / 压制电机控制", padding=2)
        motion.pack(side=tk.TOP, fill=tk.X, expand=False, padx=(8, 4), pady=(0, 4))
        self.accel57_var = tk.StringVar(value="0")
        self.start57_var = tk.StringVar(value="0")
        ttk.Label(motion, text="57加速ms / S起步rpm×10:").grid(row=0, column=0, sticky=tk.W, pady=1)
        ttk.Spinbox(motion, from_=0, to=60000, width=8, textvariable=self.accel57_var).grid(row=0, column=1, pady=1)
        ttk.Spinbox(motion, from_=0, to=15000, width=8, textvariable=self.start57_var).grid(row=0, column=2, pady=1)
        ttk.Label(motion, text="28步进:").grid(row=1, column=0, sticky=tk.W, pady=1)
        ttk.Button(motion, text="正转", command=lambda: self._cmd_28(13)).grid(row=1, column=1, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motion, text="反转", command=lambda: self._cmd_28(14)).grid(row=1, column=2, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motion, text="停止", command=lambda: self._cmd_28(15)).grid(row=1, column=3, padx=1, pady=1, sticky=tk.EW)
        ttk.Label(motion, text="压制电机:").grid(row=2, column=0, sticky=tk.W, pady=1)
        ttk.Button(motion, text="上行", command=lambda: self._cmd_press(COIL_PRESS_UP)).grid(row=2, column=1, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motion, text="下行", command=lambda: self._cmd_press(COIL_PRESS_DOWN)).grid(row=2, column=2, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motion, text="停止", command=lambda: self._cmd_press(COIL_PRESS_STOP)).grid(row=2, column=3, padx=1, pady=1, sticky=tk.EW)

        # 复制一份 57 电机控制到本页。所有参数变量和命令与波形页共用，
        # 因此不会出现两套速度/幅值状态不一致的问题。
        motor57_other = ttk.LabelFrame(col_pump, text="57 电机控制", padding=2)
        motor57_other.pack(side=tk.TOP, fill=tk.X, expand=False, padx=(0, 4), pady=(4, 0))
        ttk.Label(motor57_other, text="速度 (rpm×10):").grid(row=0, column=0, sticky=tk.W, pady=1)
        ttk.Spinbox(motor57_other, from_=-15000, to=15000, increment=10, width=8,
                    textvariable=self.speed_var).grid(row=0, column=1, padx=1, pady=1)
        ttk.Button(motor57_other, text="启动 RUN", command=self._cmd_run).grid(
            row=0, column=2, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motor57_other, text="停止 STOP", command=self._cmd_stop).grid(
            row=0, column=3, padx=1, pady=1, sticky=tk.EW)
        ttk.Label(motor57_other, text="电流幅值 (%):").grid(row=1, column=0, sticky=tk.W, pady=1)
        ttk.Spinbox(motor57_other, from_=0, to=100, increment=1, width=8,
                    textvariable=self.amp_var).grid(row=1, column=1, padx=1, pady=1)
        ttk.Button(motor57_other, text="应用 AMP", command=self._cmd_amp).grid(
            row=1, column=2, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motor57_other, text="查询 INFO", command=self._cmd_info).grid(
            row=1, column=3, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motor57_other, text="采集 512", command=lambda: self._cmd_capture(512)).grid(
            row=2, column=0, columnspan=2, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motor57_other, text="采集 1024", command=lambda: self._cmd_capture(1024)).grid(
            row=2, column=2, columnspan=2, padx=1, pady=1, sticky=tk.EW)
        self.encoder_pulse_var = tk.StringVar(value="抬升左: -  抬升右: -  压制: - 脉冲")
        ttk.Label(motor57_other, textvariable=self.encoder_pulse_var,
                  foreground="#8e44ad", font=(self.font_mono, 8)).grid(
            row=3, column=0, columnspan=4, padx=1, pady=(2, 1), sticky=tk.W)

        # “其他控制与诊断”页也保留 28/压制快捷操作；两页按钮调用同一
        # Modbus 线圈，不产生第二套状态或额外协议。
        motion_other = ttk.LabelFrame(col_pump, text="28 / 压制电机快捷控制", padding=2)
        motion_other.pack(side=tk.TOP, fill=tk.X, expand=False, padx=(0, 4), pady=(4, 0))
        ttk.Label(motion_other, text="28步进:").grid(row=0, column=0, sticky=tk.W, pady=1)
        ttk.Button(motion_other, text="正转", command=lambda: self._cmd_28(13)).grid(
            row=0, column=1, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motion_other, text="反转", command=lambda: self._cmd_28(14)).grid(
            row=0, column=2, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motion_other, text="停止", command=lambda: self._cmd_28(15)).grid(
            row=0, column=3, padx=1, pady=1, sticky=tk.EW)
        ttk.Label(motion_other, text="压制电机:").grid(row=1, column=0, sticky=tk.W, pady=1)
        ttk.Button(motion_other, text="上行", command=lambda: self._cmd_press(COIL_PRESS_UP)).grid(
            row=1, column=1, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motion_other, text="下行", command=lambda: self._cmd_press(COIL_PRESS_DOWN)).grid(
            row=1, column=2, padx=1, pady=1, sticky=tk.EW)
        ttk.Button(motion_other, text="停止", command=lambda: self._cmd_press(COIL_PRESS_STOP)).grid(
            row=1, column=3, padx=1, pady=1, sticky=tk.EW)

        # 第 2 列（上）：泵 / RST（公共 PC10）控制区
        pump = ttk.LabelFrame(col_pump, text="泵 / 电机复位 (PC10)", padding=6)
        pump.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=(0, 4), pady=4)

        ttk.Label(pump, text="速度 % ( -100~100, 0=停 )").grid(
            row=0, column=0, columnspan=4, sticky=tk.W, pady=(0, 4))

        # 每个泵一行：名称 / 速度滑块+数值 / 启动 / 停止
        self.pump_rows = {}
        self.pump_scales = {}
        for row, (pid, name) in enumerate(PUMPS, start=1):
            ttk.Label(pump, text=name).grid(row=row, column=0, sticky=tk.W, pady=2)
            var = tk.StringVar(value="50")
            frame = ttk.Frame(pump)
            frame.grid(row=row, column=1, columnspan=2, padx=4, sticky=tk.EW)
            scale = ttk.Scale(frame, from_=-100, to=100, orient=tk.HORIZONTAL,
                              length=95, command=lambda v, vv=var: vv.set(str(int(float(v)))))
            scale.set(50)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ttk.Label(frame, textvariable=var, width=5).pack(side=tk.LEFT, padx=2)
            ttk.Button(pump, text="启动", width=5,
                       command=lambda i=pid, v=var: self._cmd_pump(i, v.get())
                       ).grid(row=row, column=3, padx=1)
            ttk.Button(pump, text="停止", width=5,
                       command=lambda i=pid: self._cmd_pump(i, 0)
                       ).grid(row=row, column=4, padx=1)
            self.pump_rows[pid] = var
            self.pump_scales[pid] = scale

        # 一键停止全部泵
        ttk.Button(pump, text="全部停止", command=self._cmd_pump_all_off).grid(
            row=len(PUMPS) + 1, column=0, columnspan=5, pady=(6, 2), sticky=tk.EW)

        sep2 = ttk.Separator(pump, orient=tk.HORIZONTAL)
        sep2.grid(row=len(PUMPS) + 2, column=0, columnspan=5, sticky=tk.EW, pady=6)

        # RST（PC10）电平
        ttk.Label(pump, text="RST 电平 (moterRST):").grid(
            row=len(PUMPS) + 3, column=0, columnspan=5, sticky=tk.W, pady=2)
        self.btn_rst_high = ttk.Button(pump, text="高电平 RST:1 / moterEN",
                                       command=lambda: self._cmd_rst(1))
        self.btn_rst_high.grid(row=len(PUMPS) + 4, column=0, columnspan=5,
                               pady=2, sticky=tk.EW)
        self.btn_rst_low = ttk.Button(pump, text="低电平 RST:0 / moterDIS",
                                      command=lambda: self._cmd_rst(0))
        self.btn_rst_low.grid(row=len(PUMPS) + 5, column=0, columnspan=5,
                              pady=2, sticky=tk.EW)

        self.pump_status_var = tk.StringVar(value="泵状态: -")
        ttk.Label(pump, textvariable=self.pump_status_var, foreground="#2e8b57").grid(
            row=len(PUMPS) + 6, column=0, columnspan=5, pady=(6, 0), sticky=tk.W)

        self.rst_status_var = tk.StringVar(value="RST: -")
        ttk.Label(pump, textvariable=self.rst_status_var, foreground="#b8860b").grid(
            row=len(PUMPS) + 7, column=0, columnspan=5, pady=(2, 0), sticky=tk.W)

        ttk.Button(pump, text="查询状态 INFO", command=self._cmd_info).grid(
            row=len(PUMPS) + 8, column=0, columnspan=5, pady=(8, 2), sticky=tk.EW)

        # 阀和 LED 独立放在 42 控制区上方，避免被较长的 42 控制区挤出窗口。
        aux = ttk.LabelFrame(col_lift, text="开关阀 / LED 总开关", padding=4)
        aux.pack(side=tk.TOP, fill=tk.X, expand=False, padx=(0, 4), pady=(0, 4))
        ttk.Button(aux, text="开阀", command=lambda: self._cmd_valve(True)).grid(
            row=0, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(aux, text="关阀", command=lambda: self._cmd_valve(False)).grid(
            row=0, column=1, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(aux, text="LED 亮", command=lambda: self._cmd_led_sw(True)).grid(
            row=0, column=2, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(aux, text="LED 灭", command=lambda: self._cmd_led_sw(False)).grid(
            row=0, column=3, padx=2, pady=2, sticky=tk.EW)

        # 第 2 列（下）：42 抬升电机
        lift = ttk.LabelFrame(col_lift, text="42 抬升电机", padding=6)
        lift.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=(0, 4), pady=4)

        ttk.Label(lift, text="抬升电机速度 (rpm×10):").grid(
            row=0, column=0, columnspan=3, sticky=tk.W, pady=2)
        self.m42_speed_var = tk.StringVar(value="50")
        ttk.Spinbox(lift, from_=-15000, to=15000, increment=10, width=9,
                    textvariable=self.m42_speed_var).grid(
            row=0, column=3, padx=4, pady=2)

        self.m42_scale = ttk.Scale(lift, from_=-1500, to=1500, orient=tk.HORIZONTAL,
                                   length=120, command=self._on_m42_speed_scale)
        self.m42_scale.set(50)
        self.m42_scale.grid(row=1, column=0, columnspan=4, pady=(0, 6), sticky=tk.EW)

        ttk.Label(lift, text="双42电流幅值 (%):").grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=2)
        self.amp42_var = tk.StringVar(value="60")
        ttk.Spinbox(lift, from_=1, to=100, increment=1, width=8,
                    textvariable=self.amp42_var).grid(row=2, column=2, padx=2, pady=2)
        ttk.Button(lift, text="应用", command=self._cmd_amp42).grid(row=2, column=3, padx=2, pady=2, sticky=tk.EW)
        ttk.Label(lift, text="42 直接控制（无保护）", font=(self.font_cjk, 9, "bold")).grid(
            row=3, column=0, columnspan=4, sticky=tk.W, pady=(4, 0))
        ttk.Button(lift, text="42 上行", command=self._cmd_m42_pos).grid(
            row=4, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(lift, text="42 下行", command=self._cmd_m42_neg).grid(
            row=4, column=1, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(lift, text="42 停止", command=self._cmd_m42_stop).grid(
            row=4, column=2, columnspan=2, padx=2, pady=2, sticky=tk.EW)
        ttk.Label(lift, text="42 直接控制（两台机械同向，无保护）", font=(self.font_cjk, 9, "bold")).grid(
            row=5, column=0, columnspan=4, sticky=tk.W, pady=(4, 0))
        ttk.Button(lift, text="同向上行", command=lambda: self._cmd_m42(COIL_M42_SAME_POS)).grid(
            row=6, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(lift, text="同向下行", command=lambda: self._cmd_m42(COIL_M42_SAME_NEG)).grid(
            row=6, column=1, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(lift, text="同向停止", command=lambda: self._cmd_m42(COIL_M42_SAME_STOP)).grid(
            row=6, column=2, columnspan=2, padx=2, pady=2, sticky=tk.EW)
        ttk.Label(lift, text="抬升控制（两台反向，限位 / 编码器保护）", font=(self.font_cjk, 9, "bold")).grid(
            row=7, column=0, columnspan=4, sticky=tk.W, pady=(4, 0))
        ttk.Button(lift, text="抬升上行", command=lambda: self._cmd_rise(COIL_RISE_UP)).grid(
            row=8, column=0, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(lift, text="抬升下行", command=lambda: self._cmd_rise(COIL_RISE_DOWN)).grid(
            row=8, column=1, padx=2, pady=2, sticky=tk.EW)
        ttk.Button(lift, text="抬升停止", command=lambda: self._cmd_rise(COIL_RISE_STOP)).grid(
            row=8, column=2, columnspan=2, padx=2, pady=2, sticky=tk.EW)

        self.lift_status_var = tk.StringVar(value="42 目标: -\nRST: -")
        ttk.Label(lift, textvariable=self.lift_status_var, foreground="#8e44ad",
                  font=(self.font_mono, 9)).grid(
            row=9, column=0, columnspan=4, pady=(8, 0), sticky=tk.W)

        # 第 3 列：系统诊断 / 状态（只读，随周期轮询刷新）
        diag = ttk.LabelFrame(col_diag, text="系统诊断 / 状态", padding=6)
        diag.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=(0, 4), pady=4)

        self.diag_temp_var = tk.StringVar(value="温度: - °C")
        self.diag_amp_var = tk.StringVar(value="57 幅值: - %   RST: -")
        self.diag_capture_var = tk.StringVar(value="采集: -  样本 -  回放 -")
        self.diag_adc_var = tk.StringVar(value="ADC_A: -  ADC_B: -")
        self.diag_queue_var = tk.StringVar(value="RX 队列溢出: -")
        self.diag_crc_var = tk.StringVar(value="CRC 错误: -")
        self.diag_frame_var = tk.StringVar(value="帧错误: -")
        self.diag_coilmask_var = tk.StringVar(value="线圈激活: -")
        self.diag_discmask_var = tk.StringVar(value="离散输入 0~21 (FC02): -")

        diag_rows = [
            self.diag_temp_var, self.diag_amp_var, self.diag_capture_var,
            self.diag_adc_var, self.diag_queue_var, self.diag_crc_var,
            self.diag_frame_var,
        ]
        for row, var in enumerate(diag_rows):
            ttk.Label(diag, textvariable=var, font=(self.font_mono, 8),
                      justify=tk.LEFT).grid(row=row, column=0, sticky=tk.W, pady=0)

        ttk.Separator(diag, orient=tk.HORIZONTAL).grid(
            row=len(diag_rows), column=0, sticky=tk.EW, pady=4)

        # 位掩码显示：离散输入逐项列出**全部 16 项**及 0/1 状态；
        # 线圈只列置位项（线圈 3~12 是一次性动作地址，历史位置不代表当前动作）。
        ttk.Label(diag, textvariable=self.diag_discmask_var,
                  font=(self.font_cjk, 8), wraplength=190, justify=tk.LEFT).grid(
            row=len(diag_rows) + 1, column=0, sticky=tk.W, pady=(4, 0))
        ttk.Label(diag, textvariable=self.diag_coilmask_var,
                  font=(self.font_cjk, 8), wraplength=190, justify=tk.LEFT).grid(
            row=len(diag_rows) + 2, column=0, sticky=tk.W, pady=(4, 0))

        ttk.Button(diag, text="刷新诊断", command=self._cmd_diag_refresh).grid(
            row=len(diag_rows) + 3, column=0, columnspan=2, pady=(8, 2), sticky=tk.EW)

        # 第 4 列：WS2812 RGB 控制
        led = ttk.LabelFrame(tab_other, text="WS2812 RGB", padding=6)
        led.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4), pady=4)

        self.led_r_var = tk.StringVar(value="0")
        self.led_g_var = tk.StringVar(value="0")
        self.led_b_var = tk.StringVar(value="0")
        for row, (label, var) in enumerate(
            [("R", self.led_r_var), ("G", self.led_g_var), ("B", self.led_b_var)], start=0):
            ttk.Label(led, text=f"{label}:").grid(row=row, column=0, sticky=tk.W, pady=2)
            scale = ttk.Scale(led, from_=0, to=255, orient=tk.HORIZONTAL, length=95,
                              command=lambda v, vv=var: vv.set(str(int(float(v)))))
            scale.grid(row=row, column=1, padx=4, pady=2)
            ttk.Entry(led, textvariable=var, width=5).grid(row=row, column=2, padx=2)

        ttk.Button(led, text="发送 (LED)", command=self._cmd_led).grid(
            row=3, column=0, columnspan=3, pady=(6, 2), sticky=tk.EW)
        ttk.Button(led, text="关灯 (0,0,0)", command=self._cmd_led_off).grid(
            row=4, column=0, columnspan=3, pady=2, sticky=tk.EW)
        # 颜色预设两列排布，节省纵向空间
        ttk.Button(led, text="白光", width=11,
                   command=lambda: self._cmd_led_set(255, 255, 255)).grid(
            row=5, column=0, columnspan=2, pady=2, sticky=tk.EW)
        ttk.Button(led, text="红", width=8,
                   command=lambda: self._cmd_led_set(255, 0, 0)).grid(
            row=5, column=2, pady=2, sticky=tk.EW)
        ttk.Button(led, text="绿", width=11,
                   command=lambda: self._cmd_led_set(0, 255, 0)).grid(
            row=6, column=0, columnspan=2, pady=2, sticky=tk.EW)
        ttk.Button(led, text="蓝", width=8,
                   command=lambda: self._cmd_led_set(0, 0, 255)).grid(
            row=6, column=2, pady=2, sticky=tk.EW)

        # 传感器数据显示（固定宽度 Text 多行，避免长文本撑爆布局）
        sep_led = ttk.Separator(led, orient=tk.HORIZONTAL)
        sep_led.grid(row=9, column=0, columnspan=3, sticky=tk.EW, pady=6)

        ttk.Button(led, text="查询传感器 (SENS)", command=self._cmd_sens_query).grid(
            row=10, column=0, columnspan=3, pady=2, sticky=tk.EW)
        self.btn_sens_auto = ttk.Button(led, text="轮询: OFF",
                                        command=self._toggle_sens_auto)
        self.btn_sens_auto.grid(row=11, column=0, columnspan=3, pady=2, sticky=tk.EW)

        # 母线电压 / 母线电流单独显示（大字体，实时更新）
        self.bus_vol_var = tk.StringVar(value="母线电压: - V")
        self.bus_cur_var = tk.StringVar(value="母线电流: - A")
        ttk.Label(led, textvariable=self.bus_vol_var,
                  font=(self.font_mono, 11, "bold"),
                  foreground="#c0392b").grid(
            row=12, column=0, columnspan=3, pady=(2, 0), sticky=tk.W)
        ttk.Label(led, textvariable=self.bus_cur_var,
                  font=(self.font_mono, 11, "bold"),
                  foreground="#2980b9").grid(
            row=13, column=0, columnspan=3, pady=(0, 0), sticky=tk.W)

        # 其余通道电流用固定宽度 Text 多行显示（7 行一次显示完）
        self.sens_text = tk.Text(led, width=28, height=7, wrap=tk.WORD,
                                 font=(self.font_mono, 8), state=tk.DISABLED,
                                 background="#f5f5f5")
        self.sens_text.grid(row=14, column=0, columnspan=3, pady=(2, 0),
                            sticky=tk.NSEW)

        # TMUX1308 枚举值（与电流同周期刷新，均来自同一帧 FC04 快照）
        sep_tmux = ttk.Separator(led, orient=tk.HORIZONTAL)
        sep_tmux.grid(row=15, column=0, columnspan=3, sticky=tk.EW, pady=3)

        ttk.Label(led, text="TMUX1308 枚举值", font=(self.font_cjk, 9, "bold")).grid(
            row=16, column=0, columnspan=3, sticky=tk.W)

        # 三组 Label 网格（D0 数字 / D1 模拟 / D2 模拟），每组 8 项，
        # 一次全部显示、不用滚动条。每项 = "名称: 值" 的 StringVar。
        tmux_frame = ttk.Frame(led)
        tmux_frame.grid(row=17, column=0, columnspan=3, pady=(1, 0), sticky=tk.NSEW)

        self.tmux_vars = {"D0": [], "D1": [], "D2": []}
        tmux_groups = [
            ("D0 数字", "D0", SENS_D0_NAMES),
            ("D1 模拟", "D1", SENS_D1_NAMES),
            ("D2 模拟", "D2", SENS_D2_NAMES),
        ]
        for gi, (title, key, names) in enumerate(tmux_groups):
            grp = ttk.LabelFrame(tmux_frame, text=title, padding=2)
            grp.grid(row=0, column=gi, sticky=tk.NSEW, pady=1, padx=(0, 3))
            for i, nm in enumerate(names):
                var = tk.StringVar(value=f"{nm}: -")
                ttk.Label(grp, textvariable=var, font=(self.font_cjk, 8),
                          anchor=tk.W).grid(row=i, column=0, sticky=tk.EW,
                                            padx=2, pady=0)
                self.tmux_vars[key].append(var)

        # D2 的四个 NTC 同时显示原始 ADC 和由 Tempurature.c 得出的温度。
        ntc_frame = ttk.LabelFrame(tmux_frame, text="NTC 温度（°C）", padding=2)
        ntc_frame.grid(row=1, column=0, columnspan=3, sticky=tk.EW, pady=(3, 0))
        self.ntc_temp_vars = []
        for i, name in enumerate(("NTC2", "NTC3", "NTCBack", "NTC1")):
            var = tk.StringVar(value=f"{name}: - °C")
            ttk.Label(ntc_frame, textvariable=var, font=(self.font_cjk, 8),
                      anchor=tk.W).grid(row=0, column=i, sticky=tk.EW, padx=8)
            ntc_frame.columnconfigure(i, weight=1)
            self.ntc_temp_vars.append(var)

        # 第 5 列（右侧，可拉伸）：波形区
        wave = ttk.LabelFrame(wave_pane, text="绕组电流波形 (A)", padding=4)
        wave.pack(fill=tk.BOTH, expand=True, padx=(4, 8), pady=4)

        # 波形是调试步进电机最重要的区域，不能只给窄列的默认尺寸。
        # FigureCanvas 仍会随窗口缩放；这里提高其自然最小尺寸，避免被
        # 控制面板挤成难以观察的细长区域。
        self.fig = Figure(figsize=(6.4, 5.8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        # 内部时间轴仍以秒存储（CSV 导出保持 time_s），显示用毫秒：
        # 40 kHz 采 1024 点只有 25.6 ms，秒刻度全是 0.00x。
        self.ax.set_xlabel("时间 (ms)")
        self.ax.set_ylabel("电流 (A)")
        self.ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _pos: f"{x * 1000.0:.1f}"))
        self.ax.grid(True, alpha=0.3)
        self.line_a, = self.ax.plot([], [], lw=1, color="tab:red", label="绕组 A")
        self.line_b, = self.ax.plot([], [], lw=1, color="tab:blue", label="绕组 B")
        self.ax.legend(loc="upper right", fontsize=9)

        self.canvas = FigureCanvasTkAgg(self.fig, master=wave)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ---------------- DMA 压力测试：独立窗口 ----------------
    def _open_dma_window(self):
        """打开（或前置）DMA + 空闲中断压力测试独立窗口。"""
        if self.dma_win is not None and self.dma_win.winfo_exists():
            self.dma_win.deiconify()
            self.dma_win.lift()
            self.dma_win.focus_force()
            return

        win = tk.Toplevel(self.root)
        self.dma_win = win
        win.title("DMA + 空闲中断 压力测试")
        win.transient(self.root)
        win.protocol("WM_DELETE_WINDOW", self._close_dma_window)

        # 窗口尺寸自适应（略窄于主窗口，避免超出屏幕）
        try:
            sw = win.winfo_screenwidth()
            sh = win.winfo_screenheight()
        except Exception:
            sw, sh = 1440, 900
        w = min(1320, max(900, sw - 120))
        h = min(900, max(600, sh - 100))
        win.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+{max(0, (sh - h) // 4)}")
        win.minsize(880, 520)

        self._build_dma_ui(win)
        self.dma_win.after(60, self._poll_dma_queue)

    def _build_dma_ui(self, win):
        """构建压力测试窗口：连接状态 / 控制按钮 / 用例表 / 详情 / 日志。"""
        # 顶部：串口连接状态提示
        top = ttk.LabelFrame(win, text="测试对象：USART3 + DMA1_CH1 + Modbus RTU IDLE 接收", padding=6)
        top.pack(fill=tk.X, padx=8, pady=(8, 4))

        self.dma_conn_var = tk.StringVar()
        ttk.Label(top, textvariable=self.dma_conn_var,
                  foreground="#c0392b").grid(row=0, column=0, sticky=tk.W)
        ttk.Button(top, text="刷新连接状态", command=self._dma_refresh_conn).grid(
            row=0, column=1, padx=(12, 4))
        ttk.Label(top, text=f"循环 DMA 缓冲区 {DMA_RX_BUF} 字节   当前串口波特率 8N1",
                  foreground="gray").grid(row=0, column=2, padx=12, sticky=tk.W)
        self.dma_rounds_var = tk.StringVar(value="1")
        ttk.Label(top, text="轮数 (1~500万):").grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        ttk.Spinbox(top, from_=1, to=5000000, textvariable=self.dma_rounds_var,
                    width=12).grid(row=1, column=1, sticky=tk.W, pady=(5, 0))

        # 底部：应答原文日志
        # 注意：必须先于 expand=True 的 body 打包（side=BOTTOM），
        # 否则 body 会占满整个窗口，导致日志区完全没有空间而不被显示。
        log = ttk.LabelFrame(win, text="固件应答原文（测试期间捕获）", padding=4)
        log.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(0, 8))
        self.dma_log = tk.Text(log, height=4, wrap=tk.NONE,
                               font=(self.font_mono, 9), state=tk.DISABLED,
                               background="#f5f5f5")
        self.dma_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        lsb = ttk.Scrollbar(log, orient=tk.VERTICAL, command=self.dma_log.yview)
        lsb.pack(side=tk.RIGHT, fill=tk.Y)
        self.dma_log.configure(yscrollcommand=lsb.set)

        # 中部：左侧用例列表 + 右侧控制/详情
        body = ttk.Frame(win)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=4)

        left = ttk.LabelFrame(body, text="测试用例", padding=4)

        # 右侧：控制面板（先占用 side=RIGHT，保证不会被左侧列表挤掉）
        right = ttk.Frame(body)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 用例说明：必须先于 expand=True 的列表区打包（side=BOTTOM），
        # 否则列表区会占满 left，下方详情区高度变为 0 而不显示。
        detail = ttk.LabelFrame(left, text="用例说明", padding=4)
        detail.pack(side=tk.BOTTOM, fill=tk.X, pady=(6, 0))
        self.dma_detail = tk.Text(detail, height=6, wrap=tk.WORD,
                                  font=(self.font_cjk, 9), state=tk.DISABLED,
                                  background="#fbfbfb")
        self.dma_detail.pack(fill=tk.BOTH, expand=True)

        # 列表区：填充剩余空间
        tree_area = ttk.Frame(left)
        tree_area.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        columns = ("no", "verdict", "ms", "detail")
        # 显式指定行高，否则 ttk 默认行高偏小，中文字形会上下重叠
        dma_style = ttk.Style(win)
        dma_style.configure("Dma.Treeview", rowheight=23,
                            font=(self.font_cjk, 9))
        dma_style.configure("Dma.Treeview.Heading", font=(self.font_cjk, 9))
        self.dma_tree = ttk.Treeview(tree_area, columns=columns,
                                     show="tree headings", height=14,
                                     style="Dma.Treeview")
        self.dma_tree.heading("#0", text="用例")
        self.dma_tree.heading("no", text="序")
        self.dma_tree.heading("verdict", text="结果")
        self.dma_tree.heading("ms", text="耗时")
        self.dma_tree.heading("detail", text="判定说明")
        self.dma_tree.column("#0", width=190, minwidth=150, anchor=tk.W)
        self.dma_tree.column("no", width=40, minwidth=40, stretch=False, anchor=tk.CENTER)
        self.dma_tree.column("verdict", width=56, minwidth=52, stretch=False,
                             anchor=tk.CENTER)
        self.dma_tree.column("ms", width=52, minwidth=48, stretch=False, anchor=tk.E)
        self.dma_tree.column("detail", width=300, minwidth=180, anchor=tk.W)
        self.dma_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tsb = ttk.Scrollbar(tree_area, orient=tk.VERTICAL, command=self.dma_tree.yview)
        tsb.pack(side=tk.RIGHT, fill=tk.Y)
        self.dma_tree.configure(yscrollcommand=tsb.set)
        self.dma_tree.bind("<<TreeviewSelect>>", self._on_dma_select)

        # 颜色标记：通过 tag 区分通过/失败/警告
        self.dma_tree.tag_configure("pass", foreground="#1e8449")
        self.dma_tree.tag_configure("fail", foreground="#c0392b")
        self.dma_tree.tag_configure("warn", foreground="#b9770e")
        self.dma_tree.tag_configure("running", background="#fdf2cf")

        ctl = ttk.LabelFrame(right, text="执行控制", padding=6)
        ctl.pack(fill=tk.X)

        self.dma_progress = ttk.Progressbar(ctl, orient=tk.HORIZONTAL, length=280,
                                            mode="determinate")
        self.dma_progress.grid(row=0, column=0, columnspan=2, pady=(0, 6), sticky=tk.EW)

        self.btn_dma_start = ttk.Button(ctl, text="开始全部测试",
                                        command=lambda: self._dma_start(None))
        self.btn_dma_start.grid(row=1, column=0, pady=2, sticky=tk.EW)
        self.btn_dma_stop = ttk.Button(ctl, text="停止", command=self._dma_stop)
        self.btn_dma_stop.grid(row=1, column=1, padx=(4, 0), pady=2, sticky=tk.EW)
        self.btn_dma_stop.state(["disabled"])

        ttk.Label(ctl, text="只跑某个分组：").grid(row=2, column=0, columnspan=2,
                                                   sticky=tk.W, pady=(6, 0))
        for i, (label, keys) in enumerate(MODBUS_DMA_GROUPS):
            btn = ttk.Button(ctl, text=label, width=12,
                             command=lambda ks=keys: self._dma_start(ks))
            btn.grid(row=3 + i // 2, column=i % 2, padx=2, pady=2, sticky=tk.EW)

        ttk.Separator(ctl, orient=tk.HORIZONTAL).grid(
            row=3 + (len(MODBUS_DMA_GROUPS) + 1) // 2, column=0, columnspan=2,
            sticky=tk.EW, pady=6)
        row_clear = 4 + (len(MODBUS_DMA_GROUPS) + 1) // 2
        ttk.Button(ctl, text="清空结果", command=self._dma_clear_results).grid(
            row=row_clear, column=0, columnspan=2, pady=2, sticky=tk.EW)
        self.btn_dma_export = ttk.Button(ctl, text="导出测试报告 CSV",
                                         command=self._dma_export_csv)
        self.btn_dma_export.grid(row=row_clear + 1, column=0, columnspan=2,
                                 pady=2, sticky=tk.EW)

        self.dma_summary_var = tk.StringVar(value="尚未执行")
        ttk.Label(ctl, textvariable=self.dma_summary_var,
                  font=(self.font_cjk, 9, "bold"), foreground="#2c3e50",
                  wraplength=280, justify=tk.LEFT).grid(
            row=row_clear + 2, column=0, columnspan=2, pady=(8, 0), sticky=tk.W)

        # 初始化用例列表
        self._dma_populate_cases()
        self._dma_refresh_conn()

    def _dma_populate_cases(self):
        """把 DMA_CASES 填充到 Treeview（每次重开窗口重建）。"""
        if not self._dma_ui_ready():
            return
        for item in self.dma_tree.get_children():
            self.dma_tree.delete(item)
        self.dma_row_iid = {}
        for i, case in enumerate(MODBUS_DMA_CASES, start=1):
            iid = self.dma_tree.insert("", tk.END, text=case.name,
                                       values=(i, "-", "-", ""))
            self.dma_row_iid[case.key] = iid

    def _dma_ui_ready(self):
        """测试窗口是否已打开（未打开时所有界面更新必须跳过）。"""
        return self.dma_win is not None and self.dma_win.winfo_exists()

    def _dma_refresh_conn(self):
        if not self._dma_ui_ready():
            return
        if self.worker:
            self.dma_conn_var.set(f"已连接：{self.worker.port_name} @ {self.worker.baud}，"
                                  f"可以开始测试")
        else:
            self.dma_conn_var.set("未连接串口：请先在主窗口连接，再运行测试")

    def _close_dma_window(self):
        """关闭独立窗口：先停掉正在跑的测试。"""
        self._dma_stop()
        if self.dma_win is not None:
            try:
                self.dma_win.destroy()
            except Exception:
                pass
        self.dma_win = None

    # ---------- 执行控制 ----------
    def _dma_start(self, group_keys=None):
        """开始测试。group_keys=None 表示全部；否则只跑 key 在该列表中的用例。"""
        if not self.worker:
            messagebox.showwarning("提示", "请先在主窗口连接串口")
            return
        if self.dma_engine and self.dma_engine.is_alive():
            messagebox.showinfo("提示", "测试正在运行中")
            return
        try:
            rounds = int(self.dma_rounds_var.get())
            if not 1 <= rounds <= 5000000: raise ValueError
        except ValueError:
            messagebox.showwarning("提示", "轮数必须是 1~5000000 的整数")
            return

        # The regular UI also performs periodic Modbus reads.  Suspend it so
        # every transaction observed during this run belongs to this test.
        self.dma_resume_poll = bool(getattr(self, "sens_auto_on", False))
        if self.dma_resume_poll:
            self.sens_auto_on = False
            self._cancel_poll()
            self.btn_sens_auto.config(text="轮询: OFF")

        cases = MODBUS_DMA_CASES
        if group_keys:
            wanted = set(group_keys)
            cases = [c for c in MODBUS_DMA_CASES if c.key in wanted]
            if not cases:
                return

        self._dma_populate_cases()
        self.dma_results.clear()
        self.dma_lines.clear()
        self._dma_log_clear()
        self.dma_progress.configure(maximum=len(cases), value=0)
        self.dma_summary_var.set(f"执行中… 共 {len(cases)} 项")
        self.btn_dma_start.state(["disabled"])
        self.btn_dma_stop.state(["!disabled"])

        self.dma_engine = ModbusDmaTestEngine(self.worker, cases, self.dma_queue, rounds)
        self.dma_engine.start()

    def _dma_stop(self):
        if self.dma_engine and self.dma_engine.is_alive():
            self.dma_engine.stop()
        if self._dma_ui_ready():
            self.btn_dma_start.state(["!disabled"])
            self.btn_dma_stop.state(["disabled"])

    def _dma_clear_results(self):
        self.dma_results.clear()
        self.dma_lines.clear()
        # 表格恢复为「未运行」显示
        for i, case in enumerate(MODBUS_DMA_CASES, start=1):
            iid = self.dma_row_iid.get(case.key)
            if iid and self._dma_ui_ready():
                self.dma_tree.item(iid, values=(i, "-", "-", ""), tags=())
        self.dma_progress.configure(value=0)
        self.dma_summary_var.set("尚未执行")
        self._dma_log_clear()

    def _poll_dma_queue(self):
        """在 Tk 主线程消费测试引擎的消息。"""
        try:
            while True:
                kind, key, verdict, detail, elapsed = self.dma_queue.get_nowait()
                if kind == "CASE_START":
                    iid = self.dma_row_iid.get(key)
                    if iid and self.dma_win and self.dma_win.winfo_exists():
                        self.dma_tree.item(iid, values=(self.dma_tree.set(iid, "no"),
                                                        "运行", "-", ""),
                                           tags=("running",))
                        self.dma_tree.see(iid)
                elif kind == "CASE_DONE":
                    self.dma_results[key] = (verdict, detail, elapsed)
                    iid = self.dma_row_iid.get(key)
                    if iid and self.dma_win and self.dma_win.winfo_exists():
                        label = {"pass": "通过", "fail": "失败",
                                 "warn": "需人工", "error": "异常",
                                 "stopped": "中断"}.get(verdict, verdict)
                        self.dma_tree.item(
                            iid,
                            values=(self.dma_tree.set(iid, "no"), label,
                                    f"{elapsed:.0f}", detail),
                            tags=(verdict,))
                        self.dma_tree.see(iid)
                    self.dma_progress.step(1)
                elif kind == "RUN_DONE":
                    self.btn_dma_start.state(["!disabled"])
                    self.btn_dma_stop.state(["disabled"])
                    self._dma_update_summary()
                    if self.dma_resume_poll and self.worker:
                        self.dma_resume_poll = False
                        self.sens_auto_on = True
                        self.btn_sens_auto.config(text="轮询: ON")
                        self._schedule_poll(0)
                elif kind == "SUITE_PROGRESS":
                    done, total, stats = key, verdict, detail
                    passed = failed = timed = 0
                    for c in self.dma_engine.cases:
                        s = stats[c.key]
                        attempts = s["pass"] + s["fail"] + s["timeout"]
                        passed += s["pass"]; failed += s["fail"]; timed += s["timeout"]
                        avg = s["ms"] / attempts if attempts else 0.0
                        iid = self.dma_row_iid.get(c.key)
                        if iid:
                            detail_text = (f"通过 {s['pass']}  失败 {s['fail']}  超时 {s['timeout']}  "
                                           f"CRC {s['crc']}  异常响应 {s['exception']}  平均 {avg:.1f}ms")
                            rate = 100.0 * s["pass"] / attempts if attempts else 0.0
                            self.dma_tree.item(iid, values=(self.dma_tree.set(iid, "no"),
                                                            f"{rate:.1f}%", f"{avg:.1f}", detail_text))
                    attempts = passed + failed + timed
                    rate = 100.0 * passed / attempts if attempts else 0.0
                    self.dma_summary_var.set(
                        f"执行中：第 {done}/{total} 轮  |  实时通过率 {rate:.2f}%  "
                        f"(通过 {passed} / 总计 {attempts}，失败 {failed}，超时 {timed})")
                elif kind == "SUITE_DONE":
                    done, before, after, stats = key, verdict, detail, elapsed
                    self.dma_stats = stats
                    self.dma_mcu_before, self.dma_mcu_after = before, after
                    for c in self.dma_engine.cases:
                        s = stats[c.key]; attempts = s["pass"] + s["fail"] + s["timeout"]
                        avg = s["ms"] / attempts if attempts else 0
                        text = (f"通过 {s['pass']}  失败 {s['fail']}  超时 {s['timeout']}  "
                                f"CRC {s['crc']}  异常响应 {s['exception']}  平均 {avg:.1f}ms")
                        self.dma_results[c.key] = ("pass" if s["fail"] == s["timeout"] == 0 else "fail", text, avg)
                        iid = self.dma_row_iid.get(c.key)
                        if iid: self.dma_tree.item(iid, values=(self.dma_tree.set(iid, "no"), "完成", f"{avg:.1f}", text))
                    delta = {k: after[k] - before[k] for k in before}
                    self.dma_summary_var.set(f"结束：{done} 轮；MCU增量 queue={delta['queue']} crc={delta['crc']} frame={delta['frame']}")
                    self.btn_dma_start.state(["!disabled"]); self.btn_dma_stop.state(["disabled"])
                    if self.dma_resume_poll and self.worker:
                        self.dma_resume_poll = False; self.sens_auto_on = True
                        self.btn_sens_auto.config(text="轮询: ON"); self._schedule_poll(0)
        except queue.Empty:
            pass

        if self.dma_win is not None and self.dma_win.winfo_exists():
            self.dma_win.after(60, self._poll_dma_queue)

    def _dma_update_summary(self):
        if not self._dma_ui_ready():
            return
        counts = {"pass": 0, "fail": 0, "warn": 0, "error": 0, "stopped": 0}
        for verdict, _, _ in self.dma_results.values():
            counts[verdict] = counts.get(verdict, 0) + 1
        done = sum(counts.values())
        text = (f"完成 {done}/{len(MODBUS_DMA_CASES)}    "
                f"通过 {counts['pass']}  失败 {counts['fail']}  "
                f"需人工 {counts['warn']}  异常 {counts['error']}")
        if counts["fail"] == 0 and counts["error"] == 0 and done > 0:
            text += "\n结论：未发现确定性缺陷"
        elif counts["fail"] or counts["error"]:
            text += "\n结论：存在不符合预期的应答，见上表"
        self.dma_summary_var.set(text)

    # ---------- 日志 / 详情 ----------
    def _dma_log_append(self, line):
        self.dma_lines.append(line)
        if len(self.dma_lines) > 2000:
            del self.dma_lines[:1000]
        if self.dma_log is None or not self.dma_log.winfo_exists():
            return
        self.dma_log.configure(state=tk.NORMAL)
        self.dma_log.insert(tk.END, line + "\n")
        self.dma_log.see(tk.END)
        self.dma_log.configure(state=tk.DISABLED)

    def _dma_log_clear(self):
        if self.dma_log is not None and self.dma_log.winfo_exists():
            self.dma_log.configure(state=tk.NORMAL)
            self.dma_log.delete("1.0", tk.END)
            self.dma_log.configure(state=tk.DISABLED)

    def _on_dma_select(self, _event=None):
        if not self._dma_ui_ready():
            return
        sel = self.dma_tree.selection()
        if not sel:
            return
        key = next((k for k, v in self.dma_row_iid.items() if v == sel[0]), None)
        case = next((c for c in MODBUS_DMA_CASES if c.key == key), None)
        if case is None:
            return
        current = self.dma_tree.set(sel[0], "verdict")
        verdict, detail, elapsed = self.dma_results.get(key, ("-", "", 0.0))
        label = {"pass": "通过", "fail": "失败", "warn": "需人工确认",
                 "error": "脚本异常", "stopped": "已中断"}.get(verdict, "未运行")
        text = (f"{case.name}\n\n"
                f"【测什么】\n{case.desc}\n\n"
                f"【当前状态】{label}   (界面显示: {current})\n"
                f"【判定说明】{detail or '—'}\n"
                f"【耗时】{elapsed:.1f} ms\n")
        if case.expect:
            text += f"\n【预期应答序列】{case.expect}\n"
        self.dma_detail.configure(state=tk.NORMAL)
        self.dma_detail.delete("1.0", tk.END)
        self.dma_detail.insert(tk.END, text)
        self.dma_detail.configure(state=tk.DISABLED)

    def _dma_export_csv(self):
        if not self.dma_results:
            messagebox.showwarning("提示", "还没有测试结果")
            return
        filename = filedialog.asksaveasfilename(
            title="导出 DMA 压力测试报告",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")])
        if not filename:
            return
        try:
            with open(filename, "w", newline="", encoding="utf-8-sig") as fh:
                writer = csv.writer(fh)
                writer.writerow(["序号", "用例", "结果", "耗时ms", "判定说明", "用例说明"])
                for i, case in enumerate(MODBUS_DMA_CASES, start=1):
                    verdict, detail, elapsed = self.dma_results.get(
                        case.key, ("not_run", "", 0.0))
                    writer.writerow([i, case.name, verdict, f"{elapsed:.1f}",
                                     detail, case.desc])
                if self.dma_mcu_before and self.dma_mcu_after:
                    writer.writerow([])
                    writer.writerow(["MCU counters before", self.dma_mcu_before])
                    writer.writerow(["MCU counters after", self.dma_mcu_after])
                    writer.writerow(["MCU counter delta", {k: self.dma_mcu_after[k] - self.dma_mcu_before[k] for k in self.dma_mcu_before}])
            self.status_var.set(f"测试报告已导出: {filename}")
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc))

    # ---------------- 字体 / 窗口辅助 ----------------
    @staticmethod
    def _pick_font(candidates):
        """从候选列表中挑选本机实际安装的第一个字体。"""
        try:
            import tkinter.font as tkfont
            available = set(tkfont.families())
        except Exception:
            return candidates[0]
        for name in candidates:
            if name in available:
                return name
        return candidates[0]

    def _fit_window(self):
        """将窗口尺寸限制在屏幕可用区域内，避免超出屏幕被裁切。"""
        try:
            screen_w = self.root.winfo_screenwidth()
            screen_h = self.root.winfo_screenheight()
        except Exception:
            self.root.geometry("1280x680")
            return
        w = min(1440, max(1000, screen_w - 40))
        h = min(830, max(600, screen_h - 90))
        x = max(0, (screen_w - w) // 2)
        y = max(0, (screen_h - h) // 3)
        self.root.geometry(f"{w}x{h}+{x}+{y}")
        self.root.minsize(960, 560)

    # ---------------- 串口 ----------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.worker:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("提示", "请先选择串口")
            return
        try:
            baud = int(self.baud_var.get())
            self.worker = ModbusRtuMaster(port, baud)
        except serial.SerialException as e:
            messagebox.showerror("连接失败", str(e))
            self.worker = None
            return
        self.btn_connect.config(text="断开")
        self.status_var.set(f"{port} @ {baud} 已连接")
        self._clear_wave()
        # 连接后自动开启周期轮询（默认 100ms，可用「轮询周期」按钮调整）
        self.root.after(100, self._auto_enable_sens_report)
        self._dma_refresh_conn()

    def _disconnect(self):
        # 串口要断了，先停掉正在跑的 DMA 压力测试，避免半途写已关闭的句柄
        self._dma_stop()
        self.stream_on = False
        self.auto_capture = False
        # 断开前先停掉周期轮询并取消已排队回调
        self.sens_auto_on = False
        self._cancel_poll()
        self.btn_sens_auto.config(text="轮询: OFF")
        if self.worker:
            self.worker.stop()
            self.worker.join(timeout=1)
            self.worker = None
        # 波形下载线程持有旧 worker，等待它收尾（pyserial 读超时 0.35s，1s 足够）
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=1)
            self.capture_thread = None
        self.capture_busy = False
        self.btn_connect.config(text="连接")
        self.status_var.set("未连接")
        self._dma_refresh_conn()

    def _on_close(self):
        """关闭窗口前先断开串口，避免串口句柄泄漏。"""
        try:
            self._close_dma_window()
        except Exception:
            pass
        try:
            self._disconnect()
        finally:
            # 窗口销毁后已排队的绘图回调会报 "invalid command name"，先撤掉
            if self.poll_queue_after_id is not None:
                try:
                    self.root.after_cancel(self.poll_queue_after_id)
                except tk.TclError:
                    pass
                self.poll_queue_after_id = None
            self.root.destroy()

    def _send(self, line):
        self.status_var.set("旧 ASCII 命令已禁用；请使用 Modbus 控件")

    def _mb_write(self, address, value):
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return False
        try:
            self.worker.write_register(address, value)
            return True
        except (serial.SerialException, OSError, TimeoutError, ValueError) as exc:
            self.status_var.set(f"Modbus 通信失败: {exc}")
            return False

    # ---------------- 命令 ----------------
    def _cmd_run(self):
        try:
            v = int(self.speed_var.get())
        except ValueError:
            messagebox.showwarning("提示", "速度必须为整数 (rpm×10)")
            return
        v = max(-15000, min(15000, v))
        try:
            amp = max(0, min(100, int(self.amp_var.get())))
            accel = max(0, min(60000, int(self.accel57_var.get())))
            start = max(0, min(15000, int(self.start57_var.get())))
        except ValueError:
            messagebox.showwarning("提示", "57 的电流、加速时间和起步速度必须为整数")
            return
        if (self._mb_write(HREG_AMP, amp) and self._mb_write(HREG_M57_ACCEL, accel)
                and self._mb_write(HREG_M57_START, start)):
            self._mb_write(HREG_M57_SPEED, v & 0xFFFF)

    def _cmd_stop(self):
        self._mb_write(HREG_M57_SPEED, 0)

    def _amp42_value(self):
        """读取并限幅 42 幅值输入框（1~100）；无效时返回 None。"""
        try:
            v = int(self.amp42_var.get())
        except ValueError:
            messagebox.showwarning("提示", "42 幅值必须为 1~100 整数")
            return None
        return max(1, min(100, v))

    def _cmd_amp42(self):
        """仅下发 42 抬升电机电流幅值（保持寄存器 18）。
        固件里幅值是独立生效的，不会影响当前转速，可在运行中实时调整。"""
        v = self._amp42_value()
        if v is None:
            return
        self.amp42_var.set(str(v))
        self._mb_write(HREG_AMP42, v)

    def _cmd_42(self):
        """同时设置 42 速度与幅值。先写幅值，避免启动瞬间用旧幅值。"""
        try: value = max(-15000, min(15000, int(self.speed42_var.get())))
        except ValueError:
            messagebox.showwarning("提示", "42 速度必须为整数")
            return
        amp = self._amp42_value()
        if amp is None:
            return
        if self._mb_write(HREG_AMP42, amp):
            self._mb_write(HREG_M42_SPEED, value & 0xFFFF)

    def _cmd_28(self, coil):
        self._mb_write_coil(coil)

    def _cmd_amp(self):
        try:
            v = int(self.amp_var.get())
        except ValueError:
            messagebox.showwarning("提示", "幅值必须为 0~100 整数")
            return
        v = max(0, min(100, v))
        self._mb_write(HREG_AMP, v)

    def _cmd_period(self):
        """设置上位机轮询周期（ms）。

        固件没有上报周期寄存器（Modbus 严格主从），所以该值只决定上位机
        自己多久读一次 FC04 快照，取 20~2000ms；不连接也能先设好。
        """
        try:
            v = int(self.period_var.get())
        except ValueError:
            messagebox.showwarning("提示", "轮询周期必须为 20~2000 的整数 (ms)")
            return
        v = max(POLL_INTERVAL_MIN_MS, min(POLL_INTERVAL_MAX_MS, v))
        self.period_var.set(str(v))
        self.poll_interval_ms = v
        if self.sens_auto_on:
            # 立即用新周期重排，不必等旧周期走完
            self._schedule_poll(0)
        self.status_var.set(f"轮询周期已设为 {v} ms（约 {1000 / v:.1f} Hz）")

    def _cmd_info(self):
        self._cmd_sens_query()

    # ---------------- 42 抬升电机 / 阀 / LED ----------------
    def _cmd_m42_pos(self):
        self._cmd_m42(COIL_M42_RUN_POS)

    def _cmd_m42_neg(self):
        self._cmd_m42(COIL_M42_RUN_NEG)

    def _cmd_m42_stop(self):
        self._cmd_m42(COIL_M42_STOP)

    def _cmd_m42(self, coil):
        """42 抬升电机：先写 speed(holding[4])，再触发对应线圈。
        线圈 6/7 在固件里是一次性动作（用当前 holding[4] 启动），
        因此必须先写速度，否则会按上一次的速度运行。"""
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return
        try:
            v = int(self.m42_speed_var.get())
        except ValueError:
            messagebox.showwarning("提示", "抬升电机速度必须为整数 (rpm×10)")
            return
        v = max(-15000, min(15000, v))
        if coil in (COIL_M42_RUN_POS, COIL_M42_RUN_NEG,
                    COIL_M42_SAME_POS, COIL_M42_SAME_NEG):
            # 幅值在固件里是启动时取用，因此必须在触发方向线圈前先写进 holding[18]。
            # S 曲线参数（holding 19/20）保持默认 0 = 立即启动，可用 FC06 直接设置。
            amp = self._amp42_value()
            if amp is None:
                return
            if not self._mb_write(HREG_AMP42, amp):
                return
            if not self._mb_write(HREG_M42_SPEED, v & 0xFFFF):
                return
        self._mb_write_coil(coil)

    def _cmd_rise(self, coil):
        """抬升机构只通过新动作线圈调用 riseMoterUp/Down/Stop。"""
        self._mb_write_coil(coil)

    def _cmd_press(self, coil):
        """压制机构只通过新动作线圈调用 pressMoterUp/Down/Stop。"""
        self._mb_write_coil(coil)

    def _cmd_valve(self, on):
        """开关阀：holding[8] 非 0=开、0=关（固件同时提供 9/10 一次性线圈）。"""
        self._mb_write(HREG_VALVE, 1 if on else 0)

    def _cmd_led_sw(self, on):
        """LED 总开关：holding[9] 非 0=亮、0=灭（固件同时提供 11/12 一次性线圈）。"""
        self._mb_write(HREG_LED, 1 if on else 0)

    def _on_m42_speed_scale(self, val):
        self.m42_speed_var.set(str(int(float(val))))

    # ---------------- 诊断 / 状态 ----------------
    def _cmd_diag_refresh(self):
        """一次性读取诊断区（温度、幅值、RST、采集状态、计数器、位掩码）。"""
        self._cmd_sens_query()

    @staticmethod
    def _float_at(regs, index):
        """按大端 IEEE754 双字（高字在前）解析一个 float 寄存器对。"""
        import struct
        return struct.unpack(">f", regs[index].to_bytes(2, "big")
                             + regs[index + 1].to_bytes(2, "big"))[0]

    @staticmethod
    def _int32_at(regs, index):
        """读取高字在前的 Modbus int32 编码器位置。"""
        raw = (regs[index] << 16) | regs[index + 1]
        return raw - 0x100000000 if raw & 0x80000000 else raw

    @staticmethod
    def _uint32_at(regs, index):
        """读取高字在前的 Modbus uint32 脉冲累计值。"""
        return (regs[index] << 16) | regs[index + 1]

    def _apply_snapshot(self, regs, discrete_states=None):
        """用一次 FC04(0..71) 快照刷新全部状态显示，避免多次串口往返。"""
        f = self._float_at
        self.diag_temp_var.set(f"温度: {f(regs, 0):7.2f} °C")
        self.bus_vol_var.set(f"母线电压: {f(regs, 2):7.3f} V")
        self.bus_cur_var.set(f"母线电流: {f(regs, 4):7.3f} A")
        self.info_var.set(
            f"压制圈数: {f(regs, 6):.3f}  清水:{regs[10]} 卤水:{regs[11]} 蠕动:{regs[12]}")
        self.pump_status_var.set(
            f"泵状态(%): 软管 {regs[12]}  清水 {regs[10]}  卤水 {regs[11]}")
        self.lift_status_var.set(
            f"57 目标: {regs[13]}\n42 目标: {regs[14]}\n"
            f"42 幅值: {regs[71]} %   RST: {'高 (moterEN)' if regs[49] else '低 (moterDIS)'}")
        if len(regs) >= 92:
            self.encoder_pulse_var.set(
                f"抬升左: {self._int32_at(regs, 74)}  "
                f"抬升右: {self._int32_at(regs, 76)}  "
                f"压制: {self._int32_at(regs, 78)} 脉冲\n"
                f"清水 FT: {self._uint32_at(regs, 88)}  "
                f"卤水 FT: {self._uint32_at(regs, 90)} 脉冲")
        if discrete_states is None:
            self._show_masks(regs[8], regs[9])
        else:
            self._show_discrete_states(discrete_states, regs[9])

        self.diag_amp_var.set(f"57 幅值: {regs[48]} %   RST: {'高' if regs[49] else '低'}")
        self.diag_capture_var.set(
            f"采集: {'就绪' if regs[50] else '空闲'}  样本 {regs[51]}  回放 {regs[52]}")
        self.diag_adc_var.set(f"ADC_A: {regs[53]}  ADC_B: {regs[54]}")

        self.diag_queue_var.set(f"RX 队列溢出: {(regs[16] << 16) | regs[17]}")
        self.diag_crc_var.set(f"CRC 错误: {(regs[18] << 16) | regs[19]}")
        self.diag_frame_var.set(f"帧错误: {(regs[20] << 16) | regs[21]}")

        # TMUX D0/D1/D2 三组原始值（D0=22~29 数字电平，D1=32~39，D2=40~47）。
        # 三组都从这同一帧快照里取，不再额外发 FC02/FC04，一次往返刷新全部。
        if self.tmux_vars.get("D0"):
            for i in range(8):
                self.tmux_vars["D0"][i].set(f"{SENS_D0_NAMES[i]}: {regs[22 + i]}")
                self.tmux_vars["D1"][i].set(f"{SENS_D1_NAMES[i]}: {regs[32 + i]}")
                self.tmux_vars["D2"][i].set(f"{SENS_D2_NAMES[i]}: {regs[40 + i]}")
        if len(regs) >= 88 and getattr(self, "ntc_temp_vars", None):
            for var, name, index in zip(self.ntc_temp_vars,
                                        ("NTC2", "NTC3", "NTCBack", "NTC1"),
                                        (80, 82, 84, 86)):
                var.set(f"{name}: {f(regs, index):6.2f} °C")

        # 母线电流已在上方用大字显示；这里仅列出其余七路电流。
        lines = []
        for index, name in enumerate(SENS_CUR_NAMES[1:], start=1):
            lines.append(f"{name:>6}: {f(regs, 55 + index * 2):7.3f} A")
        self.sens_text.config(state=tk.NORMAL)
        self.sens_text.delete("1.0", tk.END)
        self.sens_text.insert("1.0", "\n".join(lines))
        self.sens_text.config(state=tk.DISABLED)

    def _update_diagnostics(self):
        regs = self._read_input_snapshot()
        discrete_states, _ = self._read_discrete_states()
        self._apply_snapshot(regs, discrete_states)

    def _read_input_snapshot(self):
        """分帧读取 0..91 输入寄存器并合并。

        一次 FC04 读取 72 个寄存器会产生 149 字节应答，超过当前 MCU
        UART DMA 接收缓冲的可靠范围，且该请求已被实测为不应答。每帧 36
        个寄存器的应答为 77 字节，能安全通过现有 DMA 环形缓冲。
        """
        first = self.worker.read_input(0, 36)
        second = self.worker.read_input(36, 36)
        third = self.worker.read_input(72, 20)
        return first + second + third

    def _read_discrete_states(self):
        """Read FC02 including the added position states.

        Boards that have not yet been flashed with the 22-input firmware only
        implement addresses 0..15.  They correctly return exception 02 for a
        22-input request; that must not be presented as a broken RS485 link.
        """
        try:
            return self.worker.read_discrete(0, len(DISCRETE_NAMES)), False
        except ValueError as exc:
            if "Modbus exception 2" not in str(exc):
                raise
            return self.worker.read_discrete(0, 16), True

    @staticmethod
    def _fmt_bits(names, mask, per_line=2):
        """把 16 位掩码渲染为「名称:0/1」的多行文本，每行 per_line 项。"""
        items = [f"{names[i]}:{(mask >> i) & 1}"
                 for i in range(min(16, len(names)))]
        return "\n".join("   ".join(items[i:i + per_line])
                         for i in range(0, len(items), per_line))

    def _show_masks(self, discrete_mask, coil_mask):
        """把位掩码翻译成中文名称并标注状态。

        离散输入：列出**全部 16 项**及 0/1 状态（0=无效 1=有效），不再只列置位项。
        线圈：只列当前置位项（线圈 3~12 是「一次性动作」地址，历史置位不代表
              电机仍在运动，列全部反而误导）。
        """
        active_coil = [COIL_NAMES[i] for i in range(min(16, len(COIL_NAMES)))
                       if coil_mask & (1 << i)]
        self.diag_discmask_var.set(
            f"离散输入 0~15 (FC02, 1=有效)  0x{discrete_mask:04X}:\n"
            + self._fmt_bits(DISCRETE_NAMES, discrete_mask))
        self.diag_coilmask_var.set(
            f"线圈激活 0x{coil_mask:04X}: "
            + ("、".join(active_coil) if active_coil else "无"))

    def _show_discrete_states(self, states, coil_mask):
        """显示 FC02 实时读取的全部离散输入，含新增的 16~21 地址。"""
        items = [f"{name}:{states[i] if i < len(states) else '-'}"
                 for i, name in enumerate(DISCRETE_NAMES)]
        self.diag_discmask_var.set(
            "离散输入 0~21 (FC02, 1=有效):\n"
            + "\n".join("   ".join(items[i:i + 2])
                        for i in range(0, len(items), 2)))
        active_coil = [COIL_NAMES[i] for i in range(min(16, len(COIL_NAMES)))
                       if coil_mask & (1 << i)]
        self.diag_coilmask_var.set(
            f"线圈激活 0x{coil_mask:04X}: "
            + ("、".join(active_coil) if active_coil else "无"))

    def _mb_write_coil(self, address, on=True):
        """FC05 写单个线圈（用于固件里的一次性动作地址）。"""
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return False
        try:
            self.worker.write_coil(address, on)
            self.status_var.set(f"写线圈 {address} = {'ON' if on else 'OFF'}")
            return True
        except (serial.SerialException, OSError, TimeoutError, ValueError) as exc:
            self.status_var.set(f"Modbus 通信失败: {exc}")
            return False

    # ---------------- 泵 / RST ----------------
    def _cmd_pump(self, pump_id, speed):
        """设置指定泵速度：PUMP:<id>,<speed_pct>（0=停止）。"""
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return
        try:
            v = int(float(speed))
        except (TypeError, ValueError):
            messagebox.showwarning("提示", "泵速度必须为 -100~100 整数")
            return
        v = max(-100, min(100, v))
        self._mb_write(HREG_PUMP[pump_id], v & 0xFFFF)

    def _cmd_pump_all_off(self):
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return
        for pid, _ in PUMPS:
            self._mb_write(HREG_PUMP[pid], 0)

    def _cmd_rst(self, level):
        """设置 PC10 电平：1=高电平(moterEN)，0=低电平(moterDIS)。"""
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return
        self._mb_write(HREG_RST, 1 if level else 0)

    # ---------------- WS2812 / 传感器 ----------------
    def _cmd_led(self):
        """发送 LED:<r>,<g>,<b> 命令控制 WS2812 灯珠颜色。"""
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return
        try:
            r = int(self.led_r_var.get())
            g = int(self.led_g_var.get())
            b = int(self.led_b_var.get())
        except ValueError:
            messagebox.showwarning("提示", "RGB 值必须为 0~255 整数")
            return
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))
        try:
            self.worker.write_registers(HREG_RGB, [r, g, b])
        except (serial.SerialException, OSError, TimeoutError, ValueError) as exc:
            self.status_var.set(f"Modbus 通信失败: {exc}")

    def _cmd_led_off(self):
        self._cmd_led_set(0, 0, 0)

    def _cmd_led_set(self, r, g, b):
        """快捷设置 RGB 并立即发送（FC10 写 holding 5/6/7）。"""
        self.led_r_var.set(str(r))
        self.led_g_var.set(str(g))
        self.led_b_var.set(str(b))
        if self.worker:
            try:
                self.worker.write_registers(HREG_RGB, [r, g, b])
            except (serial.SerialException, OSError, TimeoutError, ValueError) as exc:
                self.status_var.set(f"Modbus 通信失败: {exc}")

    def _cmd_sens_query(self):
        """读取一次完整输入寄存器快照（FC04 0..70）并刷新所有显示。"""
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return
        try:
            regs = self._read_input_snapshot()
            discrete_states, legacy_discrete = self._read_discrete_states()
            self._apply_snapshot(regs, discrete_states)
            if legacy_discrete:
                self.status_var.set("输入寄存器读取成功；MCU 仍为旧固件，新增 FC02 地址 16~21 未启用")
            else:
                self.status_var.set("Modbus 输入寄存器读取成功")
        except (serial.SerialException, OSError, TimeoutError, ValueError) as exc:
            self.status_var.set(f"Modbus 通信失败: {exc}")

    def _toggle_sens_auto(self):
        """切换周期轮询（Modbus 无主动上报，由上位机定时读一帧 FC04 快照）。"""
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return
        if self.sens_auto_on:
            self.sens_auto_on = False
            self._cancel_poll()
            self.btn_sens_auto.config(text="轮询: OFF")
            self.status_var.set("周期轮询已停止")
        else:
            self.sens_auto_on = True
            self.btn_sens_auto.config(text="轮询: ON")
            self._schedule_poll(0)

    def _auto_enable_sens_report(self):
        """连接串口后自动开启周期轮询（默认 100ms）。"""
        if not self.worker:
            return
        self.sens_auto_on = True
        self.btn_sens_auto.config(text="轮询: ON")
        self._schedule_poll(0)

    def _cancel_poll(self):
        """取消已排队的轮询回调（避免关窗/断开后仍重排）。"""
        if self.poll_after_id is not None:
            try:
                self.root.after_cancel(self.poll_after_id)
            except tk.TclError:
                pass
            self.poll_after_id = None

    def _schedule_poll(self, delay_ms):
        """重新排队下一次轮询（先取消旧回调，保证全局只有一个在途）。"""
        self._cancel_poll()
        if self.worker and self.sens_auto_on:
            self.poll_after_id = self.root.after(delay_ms, self._poll_modbus_data)

    def _poll_modbus_data(self):
        """周期读取变量并刷新界面。

        只发 **1 帧**：FC04(0,72) 一次拿全温度/电压/7 路电流/离散与线圈掩码/
        诊断计数器/TMUX D0·D1·D2/57·42 幅值，_apply_snapshot() 负责全部刷新。

        三个关键点：
          * 重入保护：Tk 的 after 回调在串口超时时会堆积，用 poll_busy 直接跳过。
          * 异常也不断链：失败也照常重排下一次，否则界面彻底停更。
          * 不再额外发 FC02/FC04：TMUX D0 已并入固件快照（输入寄存器 22~29），
            省掉两帧往返。单帧 72 寄存器 = 145 字节，在 921600 下往返约 3~4ms，
            当前 100000 下约 15ms（故默认轮询周期 100ms 足够宽裕）。
        """
        if self.poll_busy:
            # 上一轮还没跑完（串口卡住），跳过本轮，不排队也不重排。
            return
        if not self.worker or not self.sens_auto_on:
            return
        self.poll_busy = True
        try:
            self._cmd_sens_query()
        except (serial.SerialException, OSError, TimeoutError, ValueError) as exc:
            # 单帧失败不应中断刷新链，记一次状态后继续排下一轮
            self.status_var.set(f"Modbus 轮询失败: {exc}")
        finally:
            self.poll_busy = False
            self._schedule_poll(self.poll_interval_ms)

    def _cmd_capture(self, sample_count):
        """启动一次 40 kHz 采集，并在采集完成后从波形窗口下载整段数据。

        Modbus 是严格主从，固件的采集结果不会主动上报，因此流程为：
          1. 写 holding[14] 启动采集；
          2. 轮询输入寄存器 50（采集就绪）直到为 1；
          3. 从 FC04 高地址窗口分批下载两份各 N 点的原始 ADC 并绘图。
        下载要发十几帧，跑在 Tk 主线程会卡界面，所以放到后台线程。
        """
        if not self.worker:
            messagebox.showwarning("提示", "请先连接串口")
            return
        # 用 GUI 线程侧标志而不是 thread.is_alive()：线程把结果丢进队列后
        # 还要过几毫秒才真正退出，靠 is_alive() 会把自动采集的下一轮误判为"忙"。
        if self.capture_busy:
            self.status_var.set("上一次采集还在进行中")
            return
        self.capture_busy = True
        self.capture_sample_count = sample_count
        self.status_var.set(f"已启动 {sample_count} 点采集，等待完成后下载波形...")
        if not self._mb_write(HREG_CAPTURE, sample_count):
            self.capture_busy = False
            return
        self.capture_thread = threading.Thread(
            target=self._capture_worker, args=(sample_count,), daemon=True)
        self.capture_thread.start()

    def _capture_worker(self, sample_count):
        """后台线程：等采集就绪 → 下载波形 → 回传结果给 GUI 线程。"""
        try:
            # 40 kHz 下 1024 点只需 25.6ms，但采集在 ADC 中断里推进，
            # 且固件可能正忙，所以给足 600ms 的等待窗口。
            deadline = time.monotonic() + 0.6
            ready = 0
            while time.monotonic() < deadline:
                regs = self.worker.read_input(50, 1)
                if regs[0] != 0:
                    ready = 1
                    break
                time.sleep(0.005)
            if not ready:
                self.rx_queue.put(("WAVE_ERROR", "采集超时：固件未置位采集就绪标志"))
                return
            samples, zero_a, zero_b = self.worker.read_wave_capture(sample_count)
            self.rx_queue.put(("WAVE_DATA", samples, zero_a, zero_b))
        except Exception as exc:
            # 后台线程的异常不会传到 Tk 主循环，必须自己兜住并回传状态栏，
            # 否则线程静默死掉、界面永远停在"等待下载"。
            self.rx_queue.put(("WAVE_ERROR", f"{type(exc).__name__}: {exc}"))

    def _apply_wave_capture(self, samples, zero_a, zero_b):
        """把下载到的一整段波形替换到绘图缓冲（在 GUI 线程调用）。"""
        self.capture_zero_a = zero_a
        self.capture_zero_b = zero_b
        # 新一段采集从 t=0 重新开始，不叠加在旧波形后面
        self.t_data.clear()
        self.ia_data.clear()
        self.ib_data.clear()
        if not samples:
            self.status_var.set("波形下载为空，请确认固件已启动采集")
            return
        # 偏移 1/4 个采样周期，避免 40 kHz 采样点重叠在 0/1 刻度上
        base = 0.5 / WAVE_SAMPLE_RATE
        for index, (adc_a, adc_b) in enumerate(samples):
            self.t_data.append(base + index / WAVE_SAMPLE_RATE)
            self.ia_data.append((adc_a - zero_a) * WAVE_ADC_PER_AMP)
            self.ib_data.append((adc_b - zero_b) * WAVE_ADC_PER_AMP)
        self.t0 = 0.0
        self.adc_var.set(f"ADC_A: {samples[-1][0]}  ADC_B: {samples[-1][1]}")
        self._draw_wave()
        self.status_var.set(
            f"57 绕组电流波形：{len(samples)} 点 @40kHz（{len(samples) / WAVE_SAMPLE_RATE * 1000:.1f} ms）"
            f"  零点 A={zero_a} B={zero_b}")

    def _draw_wave(self):
        """按 PLOT_MAX_POINTS 抽稀后重绘波形。"""
        if not self.t_data:
            return
        stride = max(1, len(self.t_data) // PLOT_MAX_POINTS)
        self.line_a.set_data(self.t_data[::stride], self.ia_data[::stride])
        self.line_b.set_data(self.t_data[::stride], self.ib_data[::stride])
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    def _toggle_auto_capture(self):
        self.auto_capture = not self.auto_capture
        state = "ON" if self.auto_capture else "OFF"
        self.btn_auto_capture.config(text=f"自动采集: {state}")
        if self.auto_capture and self.worker:
            self._cmd_capture(self.capture_sample_count)

    def _toggle_stream(self):
        """Modbus RTU 无 STREAM 命令：波形来源改为 40 kHz ADC 采集回放。"""
        self._cmd_capture(self.capture_sample_count)

    def _on_speed_scale(self, val):
        # 滑块拖动实时更新输入框（取整到 rpm×10）
        self.speed_var.set(str(int(float(val))))

    def _clear_wave(self):
        self.t_data.clear()
        self.ia_data.clear()
        self.ib_data.clear()
        self.t0 = None
        self.line_a.set_data([], [])
        self.line_b.set_data([], [])
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()
        self.status_var.set("波形已清空")

    def _export_history_csv(self):
        if not self.t_data:
            messagebox.showwarning("提示", "还没有波形数据，请先采集")
            return
        filename = filedialog.asksaveasfilename(
            title="导出 40 kHz 采集波形",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")])
        if not filename:
            return
        try:
            with open(filename, "w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(["sample", "time_s", "current_a_A", "current_b_A"])
                for sample_index, (time_s, curr_a, curr_b) in enumerate(zip(self.t_data, self.ia_data, self.ib_data)):
                    writer.writerow([sample_index, time_s, curr_a, curr_b])
            self.status_var.set(f"已导出 {len(self.t_data)} 个波形点到 CSV")
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc))

    # ---------------- 数据处理 ----------------
    def _poll_queue(self):
        try:
            while True:
                message = self.rx_queue.get_nowait()
                kind = message[0]
                # 后台线程回传的 40 kHz 波形下载结果（在 GUI 线程落到画布）
                if kind == "WAVE_DATA":
                    self.capture_busy = False
                    self._apply_wave_capture(message[1], message[2], message[3])
                    if self.auto_capture and self.worker:
                        self.root.after(1, lambda: self._cmd_capture(self.capture_sample_count))
                    continue
                if kind == "WAVE_ERROR":
                    self.capture_busy = False
                    self.status_var.set(f"波形采集失败: {message[1]}")
                    continue
                payload = message[1]
                if kind == "__PORT_LOST__":
                    self.status_var.set("串口已断开")
                    if self.worker:
                        self.worker = None
                        self.btn_connect.config(text="连接")
                    self._dma_refresh_conn()
                    messagebox.showerror("错误", "串口连接已断开")
                    return
                # DMA 压力测试期间：把固件应答同时喂给测试引擎并记录到日志窗口
                if self.dma_engine is not None:
                    self.dma_engine.feed_line(payload)
                    self._dma_log_append(payload)
                self._handle_line(payload)
        except queue.Empty:
            pass

        # 绘图只在收到新波形（或清空）时重绘，不再周期性重算，省下主线程时间
        self.poll_queue_after_id = self.root.after(PLOT_REFRESH_MS, self._poll_queue)

    def _handle_line(self, line):
        """处理固件主动上报的文本帧。

        Modbus RTU 是无主动上报的严格主从协议，这个 UART5 文本通道已停用；
        函数保留作为兜底（若将来恢复文本上报，这里仍能解析）。
        """
        m = CUR_RE.match(line)
        if m:
            # 波形缓冲现在完全由 Modbus 下载的 40 kHz 采集填充，
            # 这里只同步一下 ADC 原始值显示，不往波形里插点。
            if m.group(4) is not None:
                self.adc_var.set(f"ADC_A: {m.group(4)}  ADC_B: {m.group(5)}")
            return

        m = INFO_RE.match(line)
        if m:
            rpm, amp, pos, en, hose, water, brine, rst = m.groups()
            self.info_var.set(f"RPM: {int(rpm)/10:.1f}  AMP: {amp}%  POS: {pos}  EN: {en}")
            if hose is not None:
                # 新增字段：泵目标速度（%）与 PC10 电平
                self.pump_status_var.set(
                    f"泵状态(%): 软管 {hose}  清水 {water}  卤水 {brine}")
                self.rst_status_var.set(
                    f"RST(PC10): {'高 (moterEN)' if int(rst) else '低 (moterDIS)'}")
            return

        m = SENS_RE.match(line)
        if m:
            self._handle_sens(line)
            return

        m = TMUX_RE.match(line)
        if m:
            self._handle_tmux(line)
            return

        # OK / ERR 忽略（可扩展日志区）
        # 调试：打印未识别的行（帮助排查协议问题）
        if line not in ("OK", "ERR"):
            print(f"[DEBUG] 未识别行: {line[:80]}")

    def _handle_sens(self, line):
        """解析 SENS 帧并更新传感器显示。

        帧格式（与固件 hostifSendSensFrame 一致）：
          SENS,<busA>,<busRaw>,<ledA>,<ledRaw>,<brakeA>,<brakeRaw>,
               <strainA>,<strainRaw>,<rrB>,<rrB_raw>,<rrA>,<rrA_raw>,
               <rlB>,<rlB_raw>,<rlA>,<rlA_raw>,<busVol>,<busVolADC>
        共 16 个电流字段（8 通道 × 安培+原始值）+ 2 个母线电压字段 = 18 字段
        """
        parts = line.split(",")
        if len(parts) < 19:
            return
        try:
            # 母线电流（第 1 通道）和母线电压单独显示
            bus_cur = float(parts[1])
            bus_cur_raw = int(parts[2])
            bus_vol = float(parts[17])
            bus_adc = int(parts[18])
            self.bus_cur_var.set(f"母线电流: {bus_cur:7.3f} A  (raw={bus_cur_raw})")
            self.bus_vol_var.set(f"母线电压: {bus_vol:7.3f} V  (ADC={bus_adc})")

            # 其余 7 通道电流写入 Text
            lines = []
            for i in range(1, 8):
                amp_val = float(parts[1 + i * 2])
                raw_val = int(parts[2 + i * 2])
                lines.append(f"{SENS_CUR_NAMES[i]:>6}: {amp_val:7.3f} A  (raw={raw_val})")

            self.sens_text.config(state=tk.NORMAL)
            self.sens_text.delete("1.0", tk.END)
            self.sens_text.insert("1.0", "\n".join(lines))
            self.sens_text.config(state=tk.DISABLED)
        except (ValueError, IndexError):
            pass

    def _handle_tmux(self, line):
        """解析 TMUX 帧并更新 TMUX1308 枚举值显示。

        帧格式（与固件 hostifSendTmuxFrame 一致）：
          TMUX,<d0_0..d0_7>,<d1_0..d1_7>,<d2_0..d2_7>
        共 24 个数据字段（D0 数字电平 + D1/D2 ADC 原始值）
        """
        parts = line.split(",")
        if len(parts) < 25:
            return
        try:
            # D0 数字电平 parts[1..8]
            for i in range(8):
                val = int(parts[1 + i])
                self.tmux_vars["D0"][i].set(f"{SENS_D0_NAMES[i]}: {val}")
            # D1 ADC 原始值 parts[9..16]
            for i in range(8):
                val = int(parts[9 + i])
                self.tmux_vars["D1"][i].set(f"{SENS_D1_NAMES[i]}: {val}")
            # D2 ADC 原始值 parts[17..24]
            for i in range(8):
                val = int(parts[17 + i])
                self.tmux_vars["D2"][i].set(f"{SENS_D2_NAMES[i]}: {val}")
        except (ValueError, IndexError):
            pass


def main():
    root = tk.Tk()
    Motor57Gui(root)
    root.mainloop()


if __name__ == "__main__":
    main()

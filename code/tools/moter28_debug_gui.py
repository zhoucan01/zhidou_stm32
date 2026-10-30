# -*- coding: utf-8 -*-
"""
28 步进 / 阀门 AT 指令调试上位机（极简版）
==========================================
串口：UART5 调试口（固件 project/src/wk_usart.c: wk_uart5_init()）
      波特率 1000000 8N1（默认，下拉框可改）
      MCU 侧引脚 TX=PC12 / RX=PD2，PC 侧需 USB-TTL 转接板，注意交叉接线和 GND 共地。

固件侧开关宏：app/usrDebug.h -> USRDEBUG_AT_ENABLE (1=开, 0=关)

指令（'=' 前后空格可有可无，大小写不敏感，行以 \\n 结束）：
    NB+moter28 = up      正转
    NB+moter28 = down    反转
    NB+moter28 = stop    停止
    NB+valve   = on      开阀（等效 NB+moter28=up + 开到位自动停）
    NB+valve   = off     关阀
    NB+moter28 = turn    查询当前步数

应答：
    NB+moter28=up,OK
    NB+moter28=turn,<数值>     <- 本工具把这行的数值显示出来
    NB+ERR,<原因>

运行： python moter28_debug_gui.py
依赖： pyserial（tkinter 为标准库）
"""

import re
import time
import threading

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk

# 与 app/usrDebug.h 的 AT 指令保持一致
BAUD_CHOICES = ["115200", "230400", "460800", "921600", "1000000"]
DEFAULT_BAUD = "1000000"

# 查询类指令的自动轮询周期（ms）。0 = 关闭自动轮询。
# 一次只发送一条命令；不能在一个周期内突发发送全部查询，否则 UART5 的
# 4 帧接收队列会在调试任务（10 ms）取走之前溢出，造成状态字段长期为空。
POLL_INTERVAL_MS = 100

# 应答解析：NB+<key>=<sub>[,<数值字段>...]
RE_RESP = re.compile(r"^NB\+(\w+)=(\w+)(?:,(.*))?$", re.IGNORECASE)


class Motor28DebugApp:
    def __init__(self, root):
        self.root = root
        self.root.title("NB+ AT 调试上位机")
        # 中文控件统一使用微软雅黑粗体；数值和收发日志仍单独使用等宽字体，
        # 以免串口数据、地址和脉冲数的对齐受到影响。
        self.ui_font = ("Microsoft YaHei UI", 10, "bold")
        self.ui_small_font = ("Microsoft YaHei UI", 8, "bold")
        self._configure_fonts()
        self.ser = None
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.tx_lock = threading.Lock()

        self.var_port = tk.StringVar()
        self.var_baud = tk.StringVar(value=DEFAULT_BAUD)
        self.var_status = tk.StringVar(value="未连接")
        self.var_auto = tk.BooleanVar(value=True)
        self.poll_index = 0
        self.poll_commands = (
            "NB+moter28=turn", "NB+moter28=pos", "NB+m57=info", "NB+m42=info",
            "NB+diag=tmux", "NB+diag=curr", "NB+rise=pos", "NB+press=pos",
            "NB+diag=bus", "NB+diag=enc", "NB+diag=temp", "NB+diag=flow",
        )

        # 各查询显示变量（每个字段独立一格）
        # 28 步数
        self.var_m28_turn = tk.StringVar(value="--")
        # 阀门软件到位标志：开位/关位
        self.var_valve_on_pos = tk.StringVar(value="--")
        self.var_valve_off_pos = tk.StringVar(value="--")
        # 57 信息：pos/ang/rpm/amp
        self.var_m57_pos = tk.StringVar(value="--")
        self.var_m57_ang = tk.StringVar(value="--")
        self.var_m57_rpm = tk.StringVar(value="--")
        self.var_m57_amp = tk.StringVar(value="--")
        # 42 信息：pos/ang
        self.var_m42_pos = tk.StringVar(value="--")
        self.var_m42_ang = tk.StringVar(value="--")
        self.var_m42_amp_left = tk.StringVar(value="--")
        self.var_m42_amp_right = tk.StringVar(value="--")
        # 抬升限位：high/low
        self.var_rise_high = tk.StringVar(value="--")
        self.var_rise_low = tk.StringVar(value="--")
        # 压制限位：high/low
        self.var_press_high = tk.StringVar(value="--")
        self.var_press_low = tk.StringVar(value="--")
        # 母线：vol/amp
        self.var_bus_vol = tk.StringVar(value="--")
        self.var_bus_amp = tk.StringVar(value="--")
        # 编码器：riseL/riseR/pressR
        self.var_enc_riseL = tk.StringVar(value="--")
        self.var_enc_riseR = tk.StringVar(value="--")
        self.var_enc_pressR = tk.StringVar(value="--")
        # 温度：ntc1/2/3/B
        self.var_temp_ntc1 = tk.StringVar(value="--")
        self.var_temp_ntc2 = tk.StringVar(value="--")
        self.var_temp_ntc3 = tk.StringVar(value="--")
        self.var_temp_ntcB = tk.StringVar(value="--")
        # 流量：water/brine
        self.var_flow_water = tk.StringVar(value="--")
        self.var_flow_brine = tk.StringVar(value="--")
        # 电流 8 路：bus/led/brake/strain/rR_A/rR_B/rL_A/rL_B
        self.var_curr_bus = tk.StringVar(value="--")
        self.var_curr_led = tk.StringVar(value="--")
        self.var_curr_brake = tk.StringVar(value="--")
        self.var_curr_strain = tk.StringVar(value="--")
        self.var_curr_rR_A = tk.StringVar(value="--")
        self.var_curr_rR_B = tk.StringVar(value="--")
        self.var_curr_rL_A = tk.StringVar(value="--")
        self.var_curr_rL_B = tk.StringVar(value="--")
        # TMUX1308 D0 数字量 8 通道
        self.var_d0 = [tk.StringVar(value="--") for _ in range(8)]
        # TMUX1308 D1 模拟量 8 通道
        self.var_d1 = [tk.StringVar(value="--") for _ in range(8)]
        # TMUX1308 D2 模拟量 8 通道
        self.var_d2 = [tk.StringVar(value="--") for _ in range(8)]

        # 输入框变量
        # 控制输入与查询结果必须分开；自动轮询不能覆盖用户准备发送的速度。
        self.var_m57_rpm_cmd = tk.IntVar(value=1100)
        self.var_m57_amp_cmd = tk.StringVar(value="50")
        self.var_m42_rpm = tk.StringVar(value="500")
        self.var_m42_amp = tk.StringVar(value="50")
        self.var_m42_amp_left_cmd = tk.StringVar(value="50")
        self.var_m42_amp_right_cmd = tk.StringVar(value="50")
        self.var_rise_rpm_cmd = tk.StringVar(value="1000")
        self.var_rise_amp_left_cmd = tk.StringVar(value="30")
        self.var_rise_amp_right_cmd = tk.StringVar(value="25")
        self.var_press_rpm_cmd = tk.StringVar(value="2000")
        self.var_pump_water = tk.StringVar(value="80")
        self.var_pump_brine = tk.StringVar(value="80")
        self.var_pump_hose = tk.StringVar(value="80")
        self.var_ws_r = tk.StringVar(value="255")
        self.var_ws_g = tk.StringVar(value="255")
        self.var_ws_b = tk.StringVar(value="255")

        self._build_ui()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_fonts(self):
        """设置 ttk 中文界面的默认粗体字体。"""
        style = ttk.Style(self.root)
        style.configure(".", font=self.ui_font)
        style.configure("TLabel", font=self.ui_font)
        style.configure("TButton", font=self.ui_font)
        style.configure("TCheckbutton", font=self.ui_font)
        style.configure("TRadiobutton", font=self.ui_font)
        style.configure("TLabelframe.Label", font=self.ui_font)
        style.configure("TCombobox", font=self.ui_font)

    # ------------------------------ UI ------------------------------
    def _build_ui(self):
        pad = {"padx": 4, "pady": 3}

        top = ttk.LabelFrame(self.root, text="串口")
        top.pack(fill="x", **pad)

        ttk.Label(top, text="端口").grid(row=0, column=0, sticky="w", **pad)
        self.cmb_port = ttk.Combobox(top, textvariable=self.var_port,
                                     width=26, state="readonly")
        self.cmb_port.grid(row=0, column=1, **pad)
        ttk.Button(top, text="刷新", command=self._refresh_ports,
                   width=6).grid(row=0, column=2, **pad)

        ttk.Label(top, text="波特率").grid(row=0, column=3, sticky="w", **pad)
        ttk.Combobox(top, textvariable=self.var_baud, values=BAUD_CHOICES,
                     width=9, state="readonly").grid(row=0, column=4, **pad)

        self.btn_conn = ttk.Button(top, text="打开", command=self._toggle_conn,
                                   width=8)
        self.btn_conn.grid(row=0, column=5, **pad)

        ttk.Label(top, textvariable=self.var_status).grid(
            row=1, column=0, columnspan=6, sticky="w", **pad)

        # ---- 主区域：左控制 / 右查询 ----
        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, **pad)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 2))
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew", padx=(2, 0))

        self._build_ctrl_left(left, pad)
        self._build_query_right(right, pad)

        # 日志
        logf = ttk.LabelFrame(self.root, text="收发记录")
        logf.pack(fill="both", expand=True, **pad)
        self.txt = tk.Text(logf, height=8, width=80, font=("Consolas", 9),
                           state="disabled", wrap="none")
        sb = ttk.Scrollbar(logf, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        self.txt.pack(side="left", fill="both", expand=True, padx=(4, 0), pady=4)
        sb.pack(side="right", fill="y", padx=(0, 4), pady=4)

        self.root.update_idletasks()
        w = max(820, self.root.winfo_reqwidth())
        h = max(720, self.root.winfo_reqheight())
        self.root.geometry("%dx%d" % (w, h))
        self.root.minsize(780, 680)

    def _build_ctrl_left(self, parent, pad):
        """左侧：所有控制按钮（按电机/外设分组）"""

        # 28 步进 / 阀门
        f = ttk.LabelFrame(parent, text="28 步进 / 阀门", padding=4)
        f.pack(fill="x", **pad)
        row = 0
        for text, cmd in [
            ("28 正转", "NB+moter28 = up"),
            ("28 反转", "NB+moter28 = down"),
            ("28 停止", "NB+moter28 = stop"),
            ("开阀", "NB+valve = on"),
            ("关阀", "NB+valve = off"),
        ]:
            ttk.Button(f, text=text, width=10,
                       command=lambda c=cmd: self.send_cmd(c)).grid(
                row=row // 3, column=row % 3, padx=2, pady=2, sticky="ew")
            row += 1

        # 57 步进电机
        f = ttk.LabelFrame(parent, text="57 步进电机", padding=4)
        f.pack(fill="x", **pad)
        ttk.Label(f, text="rpm×10:").grid(row=0, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_m57_rpm_cmd, width=6).grid(row=0, column=1, padx=2)
        ttk.Label(f, text="幅值%:").grid(row=0, column=2, sticky="w", padx=2)
        ttk.Entry(f, textvariable=self.var_m57_amp_cmd, width=6).grid(row=0, column=3, padx=2)
        ttk.Button(f, text="启动", width=6,
                   command=self._m57_run).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(f, text="停止", width=6,
                   command=lambda: self.send_cmd("NB+m57 = stop")).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(f, text="应用幅值", width=8,
                   command=self._m57_amp).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(f, text="查询 INFO", width=8,
                   command=lambda: self.send_cmd("NB+m57 = info")).grid(row=1, column=3, padx=2, pady=2)
        # 滑条与数值输入框共用同一变量，可拖动也可精确输入。
        ttk.Label(f, text="速度拖动:").grid(row=2, column=0, sticky="w", padx=2, pady=(4, 0))
        tk.Scale(f, from_=-3000, to=3000, resolution=10, orient="horizontal",
                 variable=self.var_m57_rpm_cmd, showvalue=False, length=250,
                 highlightthickness=0).grid(row=2, column=1, columnspan=3,
                                             sticky="ew", padx=2, pady=(4, 0))

        # 42 抬升电机
        f = ttk.LabelFrame(parent, text="42 抬升电机", padding=4)
        f.pack(fill="x", **pad)
        ttk.Label(f, text="rpm×10:").grid(row=0, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_m42_rpm, width=6).grid(row=0, column=1, padx=2)
        ttk.Label(f, text="幅值%:").grid(row=0, column=2, sticky="w", padx=2)
        ttk.Entry(f, textvariable=self.var_m42_amp, width=6).grid(row=0, column=3, padx=2)
        ttk.Button(f, text="反向启动", width=8,
                   command=self._m42_run).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(f, text="同向启动", width=8,
                   command=self._m42_same).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(f, text="停止", width=6,
                   command=lambda: self.send_cmd("NB+m42 = stop")).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(f, text="应用幅值", width=8,
                   command=self._m42_amp).grid(row=1, column=3, padx=2, pady=2)
        ttk.Button(f, text="查询 INFO", width=8,
                   command=lambda: self.send_cmd("NB+m42 = info")).grid(row=2, column=0, padx=2, pady=2)
        ttk.Label(f, text="左幅值%:").grid(row=3, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_m42_amp_left_cmd, width=6).grid(row=3, column=1, padx=2)
        ttk.Label(f, text="右幅值%:").grid(row=3, column=2, sticky="w", padx=2)
        ttk.Entry(f, textvariable=self.var_m42_amp_right_cmd, width=6).grid(row=3, column=3, padx=2)
        ttk.Button(f, text="应用左右电流", width=12,
                   command=self._m42_amp_left_right).grid(row=4, column=0, columnspan=4, padx=2, pady=2, sticky="ew")

        # 抬升 / 压制机构
        f = ttk.LabelFrame(parent, text="抬升 / 压制机构", padding=4)
        f.pack(fill="x", **pad)
        for text, cmd in [
            ("抬升上行", self._rise_up), ("抬升下行", self._rise_down), ("抬升停止", lambda: self.send_cmd("NB+rise = stop")),
            ("压制上行", self._press_up), ("压制下行", self._press_down), ("压制停止", lambda: self.send_cmd("NB+press = stop")),
        ]:
            ttk.Button(f, text=text, width=10,
                       command=cmd).grid(
                row=0 if "抬升" in text else 1,
                column=["上行", "下行", "停止"].index(text[2:]),
                padx=2, pady=2, sticky="ew")
        ttk.Label(f, text="抬升速度 rpm×10:").grid(row=2, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_rise_rpm_cmd, width=8).grid(row=2, column=1, padx=2)
        ttk.Label(f, text="范围 1~6000；上/下决定方向").grid(row=2, column=2, columnspan=2, sticky="w", padx=2)
        ttk.Button(f, text="抬升两机反向", width=12,
                   command=lambda: self.send_cmd("NB+rise=mode,opposite")).grid(row=3, column=0, columnspan=2, padx=2, pady=2, sticky="ew")
        ttk.Button(f, text="抬升两机同向", width=12,
                   command=lambda: self.send_cmd("NB+rise=mode,same")).grid(row=3, column=2, columnspan=2, padx=2, pady=2, sticky="ew")
        ttk.Label(f, text="抬升左电流%:").grid(row=4, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_rise_amp_left_cmd, width=6).grid(row=4, column=1, padx=2)
        ttk.Label(f, text="抬升右电流%:").grid(row=4, column=2, sticky="w", padx=2)
        ttk.Entry(f, textvariable=self.var_rise_amp_right_cmd, width=6).grid(row=4, column=3, padx=2)
        ttk.Button(f, text="应用抬升左右电流", width=16,
                   command=self._rise_amp_left_right).grid(row=5, column=0, columnspan=4, padx=2, pady=2, sticky="ew")
        ttk.Label(f, text="压制速度 rpm×10:").grid(row=6, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_press_rpm_cmd, width=8).grid(row=6, column=1, padx=2)
        ttk.Label(f, text="范围 1~6000").grid(row=6, column=2, columnspan=2, sticky="w", padx=2)

        # 泵
        f = ttk.LabelFrame(parent, text="泵 (speed -100~100, 0=停)", padding=4)
        f.pack(fill="x", **pad)
        ttk.Label(f, text="清水:").grid(row=0, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_pump_water, width=5).grid(row=0, column=1, padx=2)
        ttk.Button(f, text="设", width=3, command=lambda: self._pump("water")).grid(row=0, column=2, padx=2)
        ttk.Label(f, text="卤水:").grid(row=0, column=3, sticky="w", padx=2)
        ttk.Entry(f, textvariable=self.var_pump_brine, width=5).grid(row=0, column=4, padx=2)
        ttk.Button(f, text="设", width=3, command=lambda: self._pump("brine")).grid(row=0, column=5, padx=2)
        ttk.Label(f, text="软管:").grid(row=1, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_pump_hose, width=5).grid(row=1, column=1, padx=2)
        ttk.Button(f, text="设", width=3, command=lambda: self._pump("hose")).grid(row=1, column=2, padx=2)
        ttk.Button(f, text="全部停止", width=8,
                   command=lambda: self.send_cmd("NB+pump = off")).grid(row=1, column=3, columnspan=3, padx=2, pady=2)

        # RST / LED / WS2812
        f = ttk.LabelFrame(parent, text="RST / LED / WS2812", padding=4)
        f.pack(fill="x", **pad)
        ttk.Button(f, text="RST 高", width=6,
                   command=lambda: self.send_cmd("NB+rst = on")).grid(row=0, column=0, padx=2, pady=2)
        ttk.Button(f, text="RST 低", width=6,
                   command=lambda: self.send_cmd("NB+rst = off")).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(f, text="LED 亮", width=6,
                   command=lambda: self.send_cmd("NB+led = on")).grid(row=0, column=2, padx=2, pady=2)
        ttk.Button(f, text="LED 灭", width=6,
                   command=lambda: self.send_cmd("NB+led = off")).grid(row=0, column=3, padx=2, pady=2)
        ttk.Label(f, text="R:").grid(row=1, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_ws_r, width=4).grid(row=1, column=1, padx=2)
        ttk.Label(f, text="G:").grid(row=1, column=2, sticky="w", padx=2)
        ttk.Entry(f, textvariable=self.var_ws_g, width=4).grid(row=1, column=3, padx=2)
        ttk.Label(f, text="B:").grid(row=2, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(f, textvariable=self.var_ws_b, width=4).grid(row=2, column=1, padx=2)
        ttk.Button(f, text="发送 WS2812", width=10,
                   command=self._ws2812).grid(row=2, column=2, columnspan=2, padx=2, pady=2)

    def _build_query_right(self, parent, pad):
        """右侧：查询显示区（每个字段独立一格，可滚动）"""
        f = ttk.LabelFrame(parent, text="状态查询显示", padding=4)
        f.pack(fill="both", expand=True, **pad)

        # 可滚动容器
        canvas = tk.Canvas(f, highlightthickness=0, height=420)
        sb = ttk.Scrollbar(f, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        inner = ttk.Frame(canvas)
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_inner(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
        inner.bind("<Configure>", _on_inner)
        def _on_canvas(event):
            canvas.itemconfigure(win_id, width=event.width)
        canvas.bind("<Configure>", _on_canvas)
        # 鼠标滚轮滚动
        def _on_wheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind("<MouseWheel>", _on_wheel)
        inner.bind("<MouseWheel>", _on_wheel)

        # 查询按钮（每行一个，发对应指令）
        def add_query_btn(row, cmd):
            ttk.Button(inner, text="查询", width=5,
                       command=lambda c=cmd: self.send_cmd(c)).grid(
                row=row, column=0, padx=2, pady=2, sticky="w")

        # 单格：标签 + 值，放在同一行的后续列
        def add_cell(row, col, label, var):
            ttk.Label(inner, text=label, anchor="w", font=self.ui_small_font
                      ).grid(row=row, column=col, sticky="w", padx=2)
            ttk.Label(inner, textvariable=var, font=("Consolas", 9),
                      anchor="w", width=10, relief="sunken"
                      ).grid(row=row, column=col + 1, sticky="w", padx=2, pady=2)

        r = 0
        # 28 阀门位置：currTurns 是 28BYJ-48 的累计驱动步数，不是圈数。
        add_query_btn(r, "NB+moter28=turn"); add_cell(r, 1, "28阀门位置(步)", self.var_m28_turn); r += 1
        # 阀门开位/关位软件标志
        add_query_btn(r, "NB+moter28=pos")
        add_cell(r, 1, "阀门开位", self.var_valve_on_pos)
        add_cell(r, 3, "阀门关位", self.var_valve_off_pos); r += 1
        # 57 电机信息：位置是微步累计值，角度为累计机械角度（可超过一圈）。
        add_query_btn(r, "NB+m57=info")
        add_cell(r, 1, "57位置(微步)", self.var_m57_pos); add_cell(r, 3, "57角度(°)", self.var_m57_ang); r += 1
        add_cell(r, 1, "RPM×10", self.var_m57_rpm); add_cell(r, 3, "幅值%", self.var_m57_amp); r += 1
        # 42 信息：固件上报电机1；双电机反向模式下电机2的位置/角度为该值的相反数。
        add_query_btn(r, "NB+m42=info")
        add_cell(r, 1, "42电机1位置(微步)", self.var_m42_pos); add_cell(r, 3, "42电机1角度(°)", self.var_m42_ang); r += 1
        add_cell(r, 1, "42左幅值%", self.var_m42_amp_left); add_cell(r, 3, "42右幅值%", self.var_m42_amp_right); r += 1
        # 抬升限位
        add_query_btn(r, "NB+rise=pos")
        add_cell(r, 1, "抬升高位(1=到位)", self.var_rise_high); add_cell(r, 3, "抬升低位(1=到位)", self.var_rise_low); r += 1
        # 压制限位
        add_query_btn(r, "NB+press=pos")
        add_cell(r, 1, "压制高位(1=到位)", self.var_press_high); add_cell(r, 3, "压制低位(1=到位)", self.var_press_low); r += 1
        # 母线
        add_query_btn(r, "NB+diag=bus")
        add_cell(r, 1, "电压V", self.var_bus_vol); add_cell(r, 3, "电流A", self.var_bus_amp); r += 1
        # 编码器
        add_query_btn(r, "NB+diag=enc")
        add_cell(r, 1, "左抬升编码器(脉冲)", self.var_enc_riseL)
        add_cell(r, 3, "右抬升编码器(脉冲)", self.var_enc_riseR); r += 1
        add_cell(r, 1, "压制编码器(脉冲)", self.var_enc_pressR); r += 1
        # 温度
        add_query_btn(r, "NB+diag=temp")
        add_cell(r, 1, "NTC1℃", self.var_temp_ntc1); add_cell(r, 3, "NTC2℃", self.var_temp_ntc2); r += 1
        add_cell(r, 1, "NTC3℃", self.var_temp_ntc3); add_cell(r, 3, "NTCB℃", self.var_temp_ntcB); r += 1
        # 流量
        add_query_btn(r, "NB+diag=flow")
        add_cell(r, 1, "清水脉冲", self.var_flow_water); add_cell(r, 3, "卤水脉冲", self.var_flow_brine); r += 1

        # 电流 8 路（每路一格）
        add_query_btn(r, "NB+diag=curr")
        add_cell(r, 1, "母线A", self.var_curr_bus); add_cell(r, 3, "LED A", self.var_curr_led); r += 1
        add_cell(r, 1, "抱闸A", self.var_curr_brake); add_cell(r, 3, "应变A", self.var_curr_strain); r += 1
        add_cell(r, 1, "右抬A", self.var_curr_rR_A); add_cell(r, 3, "右抬B", self.var_curr_rR_B); r += 1
        add_cell(r, 1, "左抬A", self.var_curr_rL_A); add_cell(r, 3, "左抬B", self.var_curr_rL_B); r += 1

        # TMUX1308 D0 数字量 8 通道（各占一格，带中文注释）
        d0_names = ["压制在位", "卤水低水位", "卤水盒在位", "黄水盒在位",
                    "阀故障", "水位高", "盖子在位", "水箱在位"]
        add_query_btn(r, "NB+diag=tmux")
        for i, nm in enumerate(d0_names):
            add_cell(r, 1 + (i % 2) * 2, nm, self.var_d0[i])
            if i % 2 == 1:
                r += 1
        # TMUX1308 D1 模拟量 8 通道
        d1_names = ["压制故障", "右抬B电流", "右抬A电流", "成型在位",
                    "右抬故障R", "右抬故障L", "左抬B电流", "左抬A电流"]
        for i, nm in enumerate(d1_names):
            add_cell(r, 1 + (i % 2) * 2, nm, self.var_d1[i])
            if i % 2 == 1:
                r += 1
        # TMUX1308 D2 模拟量 8 通道
        d2_names = ["NTC2", "NTC3", "NTCB", "NTC1",
                    "水位", "盖子1", "溢流", "盖子2"]
        for i, nm in enumerate(d2_names):
            add_cell(r, 1 + (i % 2) * 2, nm, self.var_d2[i])
            if i % 2 == 1:
                r += 1

        ttk.Checkbutton(inner, text="自动轮询 (100ms)", variable=self.var_auto).grid(
            row=r, column=0, columnspan=5, sticky="w", padx=2, pady=6)

    # --------------------------- 串口管理 ---------------------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb_port["values"] = ports
        if ports and self.var_port.get() not in ports:
            self.var_port.set(ports[0])

    def _toggle_conn(self):
        if self.ser is None:
            self._open_port()
        else:
            self._close_port()

    def _open_port(self):
        port = self.var_port.get()
        if not port:
            self.var_status.set("没有可用串口")
            return
        try:
            baud = int(self.var_baud.get())
        except ValueError:
            self.var_status.set("波特率非法")
            return
        try:
            self.ser = serial.Serial(port=port, baudrate=baud,
                                     bytesize=8, parity="N", stopbits=1,
                                     timeout=0.2)
        except Exception as exc:                        # noqa: BLE001
            self.ser = None
            self.var_status.set("打开失败: %s" % exc)
            return

        self.rx_stop.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.btn_conn.configure(text="关闭")
        self.var_status.set("已连接 %s @ %d 8N1" % (port, baud))
        self._log("[打开] %s @ %d" % (port, baud))

    def _close_port(self):
        self.rx_stop.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:                           # noqa: BLE001
                pass
            self.ser = None
        self.rx_thread = None
        self.btn_conn.configure(text="打开")
        self.var_status.set("未连接")
        self._log("[关闭]")

    def _on_close(self):
        self._close_port()
        self.root.destroy()

    # ----------------------------- 收发 -----------------------------
    def _send_raw(self, text, quiet=False):
        """发送一行（自动补 \n）。返回是否成功。"""
        if self.ser is None:
            self.var_status.set("未连接，无法发送")
            return False
        data = (text + "\n").encode("ascii", errors="ignore")
        try:
            with self.tx_lock:
                self.ser.write(data)
        except Exception as exc:                        # noqa: BLE001
            self.var_status.set("发送失败: %s" % exc)
            self._close_port()
            return False
        if not quiet:
            self._log("TX > " + text)
        return True

    def send_cmd(self, cmd):
        self._send_raw(cmd)

    def _schedule_poll(self):
        """单链轮询：每个周期只发送一条查询，避免 UART5 DMA 帧队列溢出。"""
        if self.var_auto.get() and self.ser is not None:
            self._send_raw(self.poll_commands[self.poll_index], quiet=True)
            self.poll_index = (self.poll_index + 1) % len(self.poll_commands)
        if POLL_INTERVAL_MS > 0:
            self.root.after(POLL_INTERVAL_MS, self._schedule_poll)

    def _rx_loop(self):
        buf = b""
        while not self.rx_stop.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception:                           # noqa: BLE001
                break
            if not chunk:
                continue

            # 调试口也可能收到非 NB 文本或二进制帧（例如 DMA/协议测试数据）。
            # 这类数据通常没有换行，不能等待行解析，否则日志看起来像“没有接收”。
            if any(byte < 0x20 or byte > 0x7E for byte in chunk
                   if byte not in (0x0A, 0x0D, 0x09)):
                self._log("RX HEX < " + " ".join("%02X" % byte for byte in chunk))
                continue

            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_line(line.decode("ascii", errors="replace"))
        # 线程退出时把残余输出一次
        if buf:
            self._handle_line(buf.decode("ascii", errors="replace"))

    def _handle_line(self, line):
        line = line.strip("\r\n \t")
        if not line:
            return
        self._log("RX < " + line)

        m = RE_RESP.match(line)
        if not m:
            return
        key = m.group(1).lower()
        sub = m.group(2).lower()
        rest = m.group(3) or ""

        # 把应答分发到对应显示变量
        def setv(var, text):
            try:
                self.root.after(0, var.set, text)
            except tk.TclError:
                pass

        # 解析 "k1=v1,k2=v2,..." 为字典
        def parse_kv(s):
            d = {}
            for part in s.split(","):
                if "=" in part:
                    k, v = part.split("=", 1)
                    d[k.strip().lower()] = v.strip()
            return d

        # 解析 "v1,v2,..." 为列表
        def parse_list(s):
            return [x.strip() for x in s.split(",")]

        if key == "moter28" and sub == "turn":
            setv(self.var_m28_turn, rest)
        elif key == "moter28" and sub == "pos":
            d = parse_kv(rest)
            setv(self.var_valve_on_pos, d.get("on", "--"))
            setv(self.var_valve_off_pos, d.get("off", "--"))
        elif key == "m57" and sub == "info":
            d = parse_kv(rest)
            setv(self.var_m57_pos, d.get("pos", "--"))
            setv(self.var_m57_ang, d.get("ang", "--"))
            setv(self.var_m57_rpm, d.get("rpm", "--"))
            setv(self.var_m57_amp, d.get("amp", "--"))
        elif key == "m42" and sub == "info":
            d = parse_kv(rest)
            setv(self.var_m42_pos, d.get("pos", "--"))
            setv(self.var_m42_ang, d.get("ang", "--"))
            setv(self.var_m42_amp_left, d.get("ampl", "--"))
            setv(self.var_m42_amp_right, d.get("ampr", "--"))
        elif key == "rise" and sub == "pos":
            d = parse_kv(rest)
            setv(self.var_rise_high, d.get("high", "--"))
            setv(self.var_rise_low, d.get("low", "--"))
        elif key == "press" and sub == "pos":
            d = parse_kv(rest)
            setv(self.var_press_high, d.get("high", "--"))
            setv(self.var_press_low, d.get("low", "--"))
        elif key == "diag" and sub == "bus":
            d = parse_kv(rest)
            setv(self.var_bus_vol, d.get("vol", "--"))
            setv(self.var_bus_amp, d.get("amp", "--"))
        elif key == "diag" and sub == "enc":
            d = parse_kv(rest)
            setv(self.var_enc_riseL, d.get("risel", "--"))
            setv(self.var_enc_riseR, d.get("riser", "--"))
            setv(self.var_enc_pressR, d.get("pressr", "--"))
        elif key == "diag" and sub == "temp":
            d = parse_kv(rest)
            setv(self.var_temp_ntc1, d.get("ntc1", "--"))
            setv(self.var_temp_ntc2, d.get("ntc2", "--"))
            setv(self.var_temp_ntc3, d.get("ntc3", "--"))
            setv(self.var_temp_ntcB, d.get("ntcb", "--"))
        elif key == "diag" and sub == "flow":
            d = parse_kv(rest)
            setv(self.var_flow_water, d.get("water", "--"))
            setv(self.var_flow_brine, d.get("brine", "--"))
        elif key == "diag" and sub == "curr":
            d = parse_kv(rest)
            setv(self.var_curr_bus, d.get("bus", "--"))
            setv(self.var_curr_led, d.get("led", "--"))
            setv(self.var_curr_brake, d.get("brake", "--"))
            setv(self.var_curr_strain, d.get("strain", "--"))
            setv(self.var_curr_rR_A, d.get("rr_a", "--"))
            setv(self.var_curr_rR_B, d.get("rr_b", "--"))
            setv(self.var_curr_rL_A, d.get("rl_a", "--"))
            setv(self.var_curr_rL_B, d.get("rl_b", "--"))
        elif key == "diag" and sub == "tmux":
            # 三行应答：d0=v,v,...  d1=v,v,...  d2=v,v,...
            # 注意 d0 后跟逗号列表，不能用通用 parse_kv（会把列表拆散）
            low = rest.lower()
            for tag, varlist in (("d0=", self.var_d0), ("d1=", self.var_d1), ("d2=", self.var_d2)):
                idx = low.find(tag)
                if idx == 0:
                    vals = parse_list(rest[len(tag):])
                    for i in range(min(8, len(vals))):
                        setv(varlist[i], vals[i])
                    break

    # --------------------------- 带参指令 ---------------------------
    def _m57_run(self):
        try:
            rpm = int(self.var_m57_rpm_cmd.get())
            amp = int(self.var_m57_amp_cmd.get())
        except (ValueError, tk.TclError):
            self.var_status.set("57 电机速度或幅值必须是整数")
            return
        if not -3000 <= rpm <= 3000 or not 0 <= amp <= 100:
            self.var_status.set("57 速度范围 -3000~3000，幅值范围 0~100")
            return
        self.send_cmd("NB+m57=run,%d,%d" % (rpm, amp))

    def _m57_amp(self):
        self.send_cmd("NB+m57=amp,%s" % self.var_m57_amp_cmd.get())

    def _m42_run(self):
        self.send_cmd("NB+m42 = run,%s,%s" % (self.var_m42_rpm.get(), self.var_m42_amp.get()))

    def _m42_same(self):
        self.send_cmd("NB+m42 = same,%s,%s" % (self.var_m42_rpm.get(), self.var_m42_amp.get()))

    def _m42_amp(self):
        self.send_cmd("NB+m42 = amp,%s" % self.var_m42_amp.get())

    def _m42_amp_left_right(self):
        try:
            left = int(self.var_m42_amp_left_cmd.get())
            right = int(self.var_m42_amp_right_cmd.get())
        except ValueError:
            self.var_status.set("42 左右电流幅值必须是整数")
            return
        if not 0 <= left <= 100 or not 0 <= right <= 100:
            self.var_status.set("42 左右电流幅值范围为 0~100")
            return
        self.send_cmd("NB+m42=amp,%d,%d" % (left, right))

    def _rise_speed(self):
        try:
            speed = int(self.var_rise_rpm_cmd.get())
        except ValueError:
            self.var_status.set("抬升速度必须是整数")
            return None
        if not 1 <= speed <= 6000:
            self.var_status.set("抬升速度范围为 1~6000 (rpm×10)")
            return None
        return speed

    def _rise_up(self):
        speed = self._rise_speed()
        if speed is not None:
            self.send_cmd("NB+rise=up,%d" % speed)

    def _rise_down(self):
        speed = self._rise_speed()
        if speed is not None:
            self.send_cmd("NB+rise=down,%d" % speed)

    def _press_speed(self):
        try:
            speed = int(self.var_press_rpm_cmd.get())
        except ValueError:
            self.var_status.set("压制速度必须是整数")
            return None
        if not 1 <= speed <= 6000:
            self.var_status.set("压制速度范围为 1~6000")
            return None
        return speed

    def _press_up(self):
        speed = self._press_speed()
        if speed is not None:
            self.send_cmd("NB+press=up,%d" % speed)

    def _press_down(self):
        speed = self._press_speed()
        if speed is not None:
            self.send_cmd("NB+press=down,%d" % speed)

    def _rise_amp_left_right(self):
        try:
            left = int(self.var_rise_amp_left_cmd.get())
            right = int(self.var_rise_amp_right_cmd.get())
        except ValueError:
            self.var_status.set("抬升左右电流幅值必须是整数")
            return
        if not 0 <= left <= 100 or not 0 <= right <= 100:
            self.var_status.set("抬升左右电流幅值范围为 0~100")
            return
        self.send_cmd("NB+rise=amp,%d,%d" % (left, right))

    def _pump(self, name):
        var = {"water": self.var_pump_water, "brine": self.var_pump_brine,
               "hose": self.var_pump_hose}[name]
        self.send_cmd("NB+pump = %s,%s" % (name, var.get()))

    def _ws2812(self):
        self.send_cmd("NB+ws2812 = %s,%s,%s" % (
            self.var_ws_r.get(), self.var_ws_g.get(), self.var_ws_b.get()))

    # ----------------------------- 日志 -----------------------------
    def _log(self, text):
        stamp = time.strftime("%H:%M:%S")

        def append():
            self.txt.configure(state="normal")
            self.txt.insert("end", "[%s] %s\n" % (stamp, text))
            self.txt.see("end")
            self.txt.configure(state="disabled")

        try:
            self.root.after(0, append)
        except tk.TclError:
            pass


def main():
    root = tk.Tk()
    app = Motor28DebugApp(root)
    # 启动自动轮询
    app._schedule_poll()
    root.mainloop()


if __name__ == "__main__":
    main()

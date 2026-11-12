import time,queue
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog,messagebox
from protocol import Config,query,control,data_frame,crc_ok,split_firmware,RxBuffer
from firmware_pack import prepare_ota_image, inspect_image, OTA_FILENAME, RAW_APP_FILENAME, FACTORY_FILENAME
from factory_pack import build_factory_image, build_app_flash_image, to_intel_hex
from serial_ota import SerialOTA, validate_timeout
from stress_test import StressTest, version_parts

class App(tk.Tk):
    def __init__(self):
        super().__init__();self.title('AT32F415 OTA 上位机 · 自定义Modbus');self.geometry('1060x900');self.minsize(950,800)
        self.events=queue.Queue();self.ota=SerialOTA(self._on_event)
        self.bin_path=tk.StringVar();self.version=tk.StringVar(value='1.0.1');self.port=tk.StringVar();self.baud=tk.StringVar(value='1000000');self.parity=tk.StringVar(value='N');self.stopbits=tk.StringVar(value='1')
        self.session_key=tk.StringVar(value=str(int(time.time())&0xffffffff));self.packet_size=tk.StringVar(value='1024');self.timeout=tk.StringVar(value='8')
        self.first_packet=tk.StringVar(value='0');self.fields_le=tk.BooleanVar(value=False);self.crc16_le=tk.BooleanVar(value=True);self.crc32_le=tk.BooleanVar(value=False)
        self.stress=None;self.closing=False
        self.startup_timeout=tk.StringVar(value='30')
        self.log_directory=tk.StringVar(value=str(Path(__file__).resolve().parent/'logs'))
        self.raw=None;self.image=None;self.metadata=None;self._build();self.after(80,self._poll);self.refresh_ports();self.protocol('WM_DELETE_WINDOW',self._close)
    def _build(self):
        top=ttk.Frame(self,padding=10);top.pack(fill='both',expand=True)
        p=ttk.LabelFrame(top,text='1. 串口连接',padding=8);p.pack(fill='x')
        ttk.Label(p,text='串口').grid(row=0,column=0);self.ports=ttk.Combobox(p,textvariable=self.port,width=19);self.ports.grid(row=0,column=1,padx=4)
        ttk.Button(p,text='刷新',command=self.refresh_ports).grid(row=0,column=2)
        ttk.Label(p,text='波特率').grid(row=0,column=3,padx=(15,2))
        ttk.Combobox(p,textvariable=self.baud,values=['9600','19200','38400','57600','115200','230400','460800','921600','1000000'],width=12).grid(row=0,column=4)
        ttk.Label(p,text='校验').grid(row=0,column=5,padx=(12,0));ttk.Combobox(p,textvariable=self.parity,values=['N','E','O'],width=4,state='readonly').grid(row=0,column=6)
        ttk.Label(p,text='停止位').grid(row=0,column=7);ttk.Combobox(p,textvariable=self.stopbits,values=['1','2'],width=4,state='readonly').grid(row=0,column=8)
        ttk.Button(p,text='连接',command=self.connect).grid(row=0,column=9,padx=5);ttk.Button(p,text='断开',command=self.disconnect).grid(row=0,column=10)
        p=ttk.LabelFrame(top,text='2. OTA 固件（支持原始 APP / Keil OTA BIN）',padding=8);p.pack(fill='x',pady=8)
        ttk.Label(p,text='APP / OTA BIN').grid(row=0,column=0,sticky='w');ttk.Entry(p,textvariable=self.bin_path,width=75).grid(row=0,column=1,columnspan=5,sticky='ew');ttk.Button(p,text='导入',command=self.open_bin).grid(row=0,column=6)
        ttk.Label(p,text='版本号').grid(row=1,column=0,sticky='w');self.version_entry=ttk.Entry(p,textvariable=self.version,width=24);self.version_entry.grid(row=1,column=1,sticky='w');self.bind_version_source(self.bin_path,self.version,self.version_entry)
        ttk.Label(p,text='codeBin.bin：可填写版本并生成摘要；OTA.bin：内置版本只读，普通升级保持原文件内容。',foreground='#9b4510').grid(row=2,column=0,columnspan=7,sticky='w',pady=4)
        ttk.Button(p,text='合并 Bootloader + APP 烧录文件…',command=self.open_factory_pack).grid(row=3,column=0,columnspan=3,sticky='w',pady=3)
        ttk.Button(p,text='单独导出 APP 烧录文件…',command=self.open_app_flash_pack).grid(row=3,column=3,columnspan=4,sticky='w',pady=3)
        ttk.Button(p,text='导出 OTA BIN…',command=self.export_ota_bin).grid(row=4,column=0,columnspan=3,sticky='w',pady=3)
        ttk.Button(p,text='导出纯 APP codeBin.bin…',command=self.export_raw_app).grid(row=4,column=3,columnspan=4,sticky='w',pady=3)
        p=ttk.LabelFrame(top,text='3. 协议设置 · 当前C代码默认普通字段大端 / CRC16小端 / CRC32大端',padding=8);p.pack(fill='x')
        ttk.Checkbutton(p,text='普通字段小端',variable=self.fields_le).grid(row=0,column=0,sticky='w');ttk.Checkbutton(p,text='CRC16小端',variable=self.crc16_le).grid(row=0,column=1)
        ttk.Checkbutton(p,text='CRC32小端',variable=self.crc32_le).grid(row=0,column=2)
        ttk.Label(p,text='首包号').grid(row=0,column=3);ttk.Combobox(p,textvariable=self.first_packet,values=['0','1'],width=4,state='readonly').grid(row=0,column=4)
        ttk.Label(p,text='每包数据字节').grid(row=0,column=5);ttk.Entry(p,textvariable=self.packet_size,width=7).grid(row=0,column=6)
        ttk.Label(p,text='会话Key (十进制/0x十六进制)').grid(row=1,column=0,sticky='w');ttk.Entry(p,textvariable=self.session_key,width=20).grid(row=1,column=1,sticky='w')
        ttk.Button(p,text='当前时间戳',command=lambda:self.session_key.set(str(int(time.time())&0xffffffff))).grid(row=1,column=2)
        ttk.Label(p,text='查询/握手/数据应答超时(秒)').grid(row=1,column=3);ttk.Entry(p,textvariable=self.timeout,width=8).grid(row=1,column=4)
        p=ttk.LabelFrame(top,text='4. 操作与测试',padding=8);p.pack(fill='x',pady=8)
        for idx,(name,callback) in enumerate([('查询OTA状态',self.query),('开始OTA',self.start_upgrade),('查询/无操作测试',self.safe_tests),('离线协议自检',self.offline_tests),('手动回滚',self.rollback_device),('重启设备',self.reset_device),('重启并查询OTA',self.reset_query_device)]):
            ttk.Button(p,text=name,command=callback).grid(row=0,column=idx,padx=4,sticky='ew');p.columnconfigure(idx,weight=1)
        ttk.Button(p,text='查询版本号',command=self.query_version).grid(row=6,column=0,pady=4)
        ttk.Button(p,text='开始压力测试',command=self.start_stress).grid(row=6,column=1)
        ttk.Button(p,text='停止压力测试',command=self.stop_stress).grid(row=6,column=2)
        ttk.Label(p,text='APP启动超时(秒)').grid(row=6,column=3)
        ttk.Entry(p,textvariable=self.startup_timeout,width=7).grid(row=6,column=4)
        ttk.Label(p,text='日志目录').grid(row=7,column=0)
        ttk.Entry(p,textvariable=self.log_directory).grid(row=7,column=1,columnspan=5,sticky='ew')
        ttk.Button(p,text='选择',command=self.choose_log_directory).grid(row=7,column=6)
        self.version_label=ttk.Label(p,text='设备版本：尚未查询')
        self.version_label.grid(row=8,column=0,columnspan=7,sticky='w')
        self.stress_label=ttk.Label(p,text='压测：未运行；日志循环保留，本次最多64MiB；目录上限512MiB；停止时完成当前轮。',wraplength=950)
        self.stress_label.grid(row=9,column=0,columnspan=7,sticky='w')
        self.progress=ttk.Progressbar(p,maximum=100);self.progress.grid(row=1,column=0,columnspan=5,sticky='ew',pady=7)
        self.progress_label=ttk.Label(p,text='0%  |  已确认 0 / 0 包');self.progress_label.grid(row=2,column=0,columnspan=5)
        self.progress_detail=ttk.Label(p,text='已确认 0 / 0 B | 耗时 0.0s | 平均速度 0.00 KiB/s',wraplength=950)
        self.progress_detail.grid(row=3,column=0,columnspan=5,sticky='w')
        self.activity_label=ttk.Label(p,text='当前阶段：待开始',wraplength=950)
        self.activity_label.grid(row=4,column=0,columnspan=5,sticky='w',pady=(3,0))
        self.rx_preview=ttk.Label(p,text='最近接收：尚未收到数据',wraplength=950)
        self.rx_preview.grid(row=5,column=0,columnspan=5,sticky='w',pady=(3,0))
        p=ttk.LabelFrame(top,text='5. 收发日志（HEX原始字节 + 协议字段解释）',padding=6);p.pack(fill='both',expand=True)
        self.log_text=tk.Text(p,height=8,font=('Consolas',10),wrap='word');self.log_text.pack(side='left',fill='both',expand=True)
        scroll=ttk.Scrollbar(p,command=self.log_text.yview);scroll.pack(side='right',fill='y');self.log_text.configure(yscrollcommand=scroll.set)
        ttk.Button(top,text='清空日志',command=lambda:self.log_text.delete('1.0','end')).pack(anchor='e',pady=4)
        ttk.Button(top,text='保存当前界面日志',command=self.save_log).pack(anchor='e')
        self.write('提示：可导入原始 APP codeBin.bin 或 OTA.bin；原始 APP 自动生成 SHA-256，封装镜像保留原摘要。')
    @staticmethod
    def bind_version_source(source, version, entry):
        state = {'raw_version': version.get(), 'kind': 'raw_app'}
        def refresh(*_):
            if state['kind'] == 'raw_app':
                state['raw_version'] = version.get()
            try:
                metadata = inspect_image(Path(source.get()).read_bytes())
            except (OSError, ValueError):
                state['kind'] = 'raw_app'
                version.set(state['raw_version'])
                entry.configure(state='normal')
            else:
                state['kind'] = 'ota'
                version.set(metadata['version'])
                entry.configure(state='readonly')
        source.trace_add('write', refresh)
        refresh()

    def export_raw_app(self):
        try:
            self.load_image()
            path=filedialog.asksaveasfilename(parent=self, initialfile=RAW_APP_FILENAME,
                defaultextension='.bin', filetypes=[('Raw APP BIN','*.bin')])
            if not path:return
            if Path(path).resolve()==Path(self.bin_path.get()).resolve():
                raise ValueError('输出文件不能覆盖输入固件，请选择其他目录或文件名')
            Path(path).write_bytes(self.image[2048:])
            self.write('纯 APP 已导出：'+path+'；不含版本头、摘要或有效记录，链接地址 0x08004000')
        except Exception as exc:messagebox.showerror('纯 APP 导出失败',str(exc),parent=self)

    def export_ota_bin(self):
        try:
            if not self.bin_path.get():
                self.open_bin()
                if not self.bin_path.get():return
            self.load_image()
            path=filedialog.asksaveasfilename(parent=self,title='保存串口 OTA 固件',
                initialfile=OTA_FILENAME,defaultextension='.bin',filetypes=[('OTA BIN','*.bin')])
            if not path:return
            if Path(path).resolve()==Path(self.bin_path.get()).resolve():
                raise ValueError('输出文件不能覆盖输入固件，请选择其他文件名')
            Path(path).write_bytes(self.image)
            self.write('OTA BIN 已导出：%s；%d字节，版本=%s；包含2KB头和实际 APP，可用于串口 OTA。'%(
                path,len(self.image),self.metadata['version']))
            messagebox.showinfo('OTA 导出完成','已生成串口 OTA BIN：%d字节。\n包含1KB版本区、1KB摘要区和实际 APP，不含记录页。'%len(self.image),parent=self)
        except Exception as exc:messagebox.showerror('OTA 导出失败',str(exc),parent=self)
    def open_app_flash_pack(self):
        window=tk.Toplevel(self);window.title('单独导出 APP 烧录文件 · BIN / HEX');window.transient(self)
        panel=ttk.Frame(window,padding=12);panel.pack(fill='both',expand=True)
        source=tk.StringVar(window,value=self.bin_path.get());version=tk.StringVar(window,value=self.version.get())
        ttk.Label(panel,text='APP / OTA BIN').grid(row=0,column=0,sticky='w')
        ttk.Entry(panel,textvariable=source,width=65).grid(row=0,column=1,padx=6,sticky='ew')
        def choose():
            path=filedialog.askopenfilename(parent=window,title='选择原始 codeBin.bin 或 OTA.bin',filetypes=[('APP / OTA BIN','*.bin')])
            if path:source.set(path)
        ttk.Button(panel,text='选择',command=choose).grid(row=0,column=2)
        ttk.Label(panel,text='APP 版本号').grid(row=1,column=0,sticky='w',pady=6)
        version_entry=ttk.Entry(panel,textvariable=version,width=24);version_entry.grid(row=1,column=1,sticky='w',padx=6)
        self.bind_version_source(source,version,version_entry)
        ttk.Label(panel,text='可选择原始 APP codeBin.bin 或 Keil OTA.bin，自动计算 APP 的 SHA-256。\n'
                  '包含版本区、摘要、APP、有效标记和长度记录；APP 必须链接到 0x08004000。\n'
                  'BIN 烧录起点：0x08003800；HEX 已包含地址（推荐）。\n'
                  '设备需已有 Bootloader；烧录选择按扇区擦除，不能全片擦除。\n'
                  '此文件用于下载器直烧 Flash，不能作为串口 OTA 输入。',wraplength=660).grid(row=2,column=0,columnspan=3,sticky='w',pady=10)
        def export():
            try:
                if not source.get():raise ValueError('请选择原始 APP BIN 或 OTA.bin')
                image,meta=build_app_flash_image(Path(source.get()).read_bytes(),version.get())
                path=filedialog.asksaveasfilename(parent=window,title='保存单独 APP 烧录文件',initialfile='appFlash.hex',
                    defaultextension='.hex',filetypes=[('Intel HEX（推荐）','*.hex'),('APP 烧录 BIN','*.bin')])
                if not path:return
                if Path(path).resolve()==Path(source.get()).resolve():raise ValueError('输出文件不能覆盖输入固件')
                suffix=Path(path).suffix.lower()
                if suffix=='.hex':Path(path).write_text(to_intel_hex(image,meta['base_address']),encoding='ascii')
                elif suffix=='.bin':Path(path).write_bytes(image)
                else:raise ValueError('请选择 .bin 或 .hex 文件格式')
                self.write('单独 APP 烧录文件已导出：%s；版本=%s，APP=%dB，SHA-256=%s；起点0x%08X，包含有效标记和长度。'%(
                    path,meta['version'],meta['app_size'],meta['digest'],meta['base_address']))
                messagebox.showinfo('APP 导出完成','已生成含版本、SHA-256、有效标记和长度的 APP 烧录文件。\n'
                    'BIN 起点：0x08003800；HEX 已包含地址。\n设备需已有 Bootloader，烧录请选择按扇区擦除。',parent=window)
            except Exception as exc:messagebox.showerror('APP 导出失败',str(exc),parent=window)
        ttk.Button(panel,text='导出 APP BIN / HEX',command=export).grid(row=3,column=0,columnspan=3,sticky='e')
        panel.columnconfigure(1,weight=1)

    def open_factory_pack(self):
        window=tk.Toplevel(self);window.title('合并 Bootloader + APP · BIN / HEX');window.transient(self)
        panel=ttk.Frame(window,padding=12);panel.pack(fill='both',expand=True)
        boot_path=tk.StringVar(window);app_path=tk.StringVar(window)
        version=tk.StringVar(window,value=self.version.get())
        def choose(target,title):
            path=filedialog.askopenfilename(parent=window,title=title,filetypes=[('原始 BIN','*.bin')])
            if path:target.set(path)
        for row,(label,target) in enumerate([('Bootloader BIN',boot_path),('原始 APP BIN',app_path)]):
            ttk.Label(panel,text=label).grid(row=row,column=0,sticky='w',pady=4)
            ttk.Entry(panel,textvariable=target,width=65).grid(row=row,column=1,sticky='ew',padx=6)
            ttk.Button(panel,text='选择',command=lambda v=target,t=label:choose(v,t)).grid(row=row,column=2)
        ttk.Label(panel,text='APP 版本号').grid(row=2,column=0,sticky='w',pady=4)
        version_entry=ttk.Entry(panel,textvariable=version,width=24);version_entry.grid(row=2,column=1,sticky='w',padx=6)
        self.bind_version_source(app_path,version,version_entry)
        ttk.Label(panel,text='Bootloader 链接到 0x08000000（最多13KB）；APP 链接到 0x08004000（最多55296B）。\n'
                  '选择原始 APP BIN，不是 OTA.bin。自动生成 SHA-256、长度和有效标记。\n'
                  '导出完整128KB镜像，B区为空。烧录时需擦除全部Flash，将清除原备份。\n'
                  'BIN 烧录起点为 0x08000000；HEX 已包含地址。',wraplength=640).grid(
                      row=3,column=0,columnspan=3,sticky='w',pady=10)
        def export():
            try:
                if not boot_path.get() or not app_path.get():raise ValueError('请分别选择 Bootloader 和原始 APP BIN')
                image,meta=build_factory_image(Path(boot_path.get()).read_bytes(),Path(app_path.get()).read_bytes(),version.get())
                path=filedialog.asksaveasfilename(parent=window,title='保存合并烧录文件',initialfile=FACTORY_FILENAME,
                    defaultextension='.bin',filetypes=[('合并 BIN','*.bin'),('Intel HEX','*.hex')])
                if not path:return
                if Path(path).resolve() in (Path(boot_path.get()).resolve(),Path(app_path.get()).resolve()):
                    raise ValueError('输出文件不能覆盖所选 Bootloader 或 APP 源文件')
                suffix=Path(path).suffix.lower()
                if suffix=='.hex':Path(path).write_text(to_intel_hex(image),encoding='ascii')
                elif suffix=='.bin':Path(path).write_bytes(image)
                else:raise ValueError('请选择 .bin 或 .hex 文件格式')
                self.write('合并烧录文件已保存：%s；Bootloader=%dB，APP=%dB，镜像=%dB，版本=%s，SHA-256=%s；BIN基址0x08000000，烧录需全片擦除。'%(
                    path,meta['boot_size'],meta['app_size'],meta['size'],meta['version'],meta['digest']))
                messagebox.showinfo('导出完成','已生成合并烧录文件。\nBIN起点：0x08000000；HEX包含地址。\n烧录时全片擦除，将清除原B区备份。',parent=window)
            except Exception as exc:messagebox.showerror('合并失败',str(exc),parent=window)
        ttk.Button(panel,text='生成并导出 BIN / HEX',command=export).grid(row=4,column=0,columnspan=3,sticky='e')
        panel.columnconfigure(1,weight=1)

    def _on_event(self,kind,value):
        self.events.put((kind,value,time.time()))
    def write(self,s):
        self.log_text.insert('end',datetime.now().strftime('%H:%M:%S.%f')[:-3]+' '+str(s)+'\n');self.log_text.see('end')
    def save_log(self):
        path=filedialog.asksaveasfilename(defaultextension='.log',initialfile='ota_'+time.strftime('%Y%m%d_%H%M%S')+'.log',filetypes=[('日志','*.log'),('文本','*.txt')])
        if path:
            try:Path(path).write_text(self.log_text.get('1.0','end-1c'),encoding='utf-8');self.write('当前界面日志已保存：'+path)
            except Exception as exc:messagebox.showerror('保存日志失败',str(exc))
    def _poll(self):
        logs=[]
        # 分批插入日志，避免大量HEX数据让界面一直停留在队列处理循环中。
        for _ in range(200):
            try:k,v,stamp=self.events.get_nowait()
            except queue.Empty:break
            if k=='log':logs.append(datetime.fromtimestamp(stamp).strftime('%H:%M:%S.%f')[:-3]+' '+str(v)+'\n')
            elif k=='received':self.rx_preview['text']=f'最近接收 [{len(v)}字节]：'+v.hex(' ').upper()
            elif k=='activity':self.activity_label['text']='当前阶段：'+v
            elif k=='version':self.version_label['text']='设备版本：%s | 当前运行：%s'%(v['version'],v['mode'])
            elif k=='stress':
                self.stress_label['text']='压测：%d轮，成功%d，失败%d | 目标版本 %s | 日志 %s'%(v['rounds'],v['success'],v['failed'],v['version'],v['log'])
            elif k=='done' and self.stress and not self.stress.running and '已停止' not in self.stress_label['text']:
                self.stress_label['text']+=' | 已停止'
            elif k=='progress':
                self.progress['value']=v['percent']
                packet='—' if v.get('packet') is None else str(v['packet'])
                self.progress_label['text']=f"{v['percent']}%  |  已确认 {v['completed']} / {v['total']} 包  |  当前协议包号 {packet}"
                eta='—' if v.get('eta') is None else f"{v['eta']:.1f}s"
                rtt='—' if v.get('rtt_ms') is None else f"{v['rtt_ms']:.2f}ms"
                self.progress_detail['text']=(f"已确认 {v.get('bytes',0)} / {v.get('total_bytes',0)} B | 耗时 {v.get('elapsed',0):.1f}s | "
                    f"平均速度 {v.get('rate',0)/1024:.2f} KiB/s | 预计剩余 {eta} | 最近包确认耗时 {rtt} | "
                    f"串口 TX {v.get('tx_bytes',0)} B / RX {v.get('rx_bytes',0)} B")
                self.activity_label['text']='当前阶段：'+v.get('stage','')
            elif k=='error':messagebox.showerror('任务失败',v)
        if logs:
            self.log_text.insert('end',''.join(logs));self.log_text.see('end')
            lines=int(self.log_text.index('end-1c').split('.')[0])
            if lines>5000:self.log_text.delete('1.0',f'{lines-5000}.0')
        self.after(80,self._poll)
    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            names=[p.device for p in list_ports.comports()];self.ports['values']=names
            if names and not self.port.get():self.port.set(names[0])
        except Exception as e:self.write('无法枚举串口：'+str(e))
    def config(self):return Config(little_fields=self.fields_le.get(),little_crc16=self.crc16_le.get(),little_crc32=self.crc32_le.get(),first_packet=int(self.first_packet.get()),payload=int(self.packet_size.get()))
    def key(self):
        k=int(self.session_key.get(),0) if self.session_key.get().lower().startswith('0x') else int(self.session_key.get())
        if not 0<=k<=0xffffffff:raise ValueError('会话key必须为uint32')
        return k
    def connect(self):
        try:
            if self.ota.busy:raise RuntimeError('当前任务执行中，请先停止任务再连接')
            self.ota.open(self.port.get(),int(self.baud.get()),self.config(),self.parity.get(),int(self.stopbits.get()))
        except Exception as e:messagebox.showerror('连接失败',str(e))
    def disconnect(self):self.ota.close();self.write('串口已断开')
    def load_image(self):
        self.image=None;self.metadata=None
        if not self.bin_path.get():raise ValueError('请导入原始 APP codeBin.bin 或 OTA.bin')
        image=Path(self.bin_path.get()).read_bytes()
        image,metadata=prepare_ota_image(image,self.version.get())
        self.image=image;self.metadata=metadata
        self.version.set(metadata['version'])
        self.version_entry.configure(state='readonly' if metadata['source_format']=='ota' else 'normal')
        action='已为原始 APP 添加2KB版本/摘要头并计算 SHA-256' if metadata['source_format']=='raw_app' else '使用文件内置版本，完整保留 OTA 镜像'
        self.write('OTA 固件：%s，%d字节，版本=%s；%s；SHA-256=%s'%(self.bin_path.get(),len(image),metadata['version'],action,metadata['digest']))
    def open_bin(self):
        s=filedialog.askopenfilename(title='导入原始 APP codeBin.bin 或 OTA.bin',filetypes=[('原始 APP / OTA BIN','*.bin')])
        if s:
            self.bin_path.set(s)
            try:self.load_image()
            except Exception as exc:messagebox.showerror('导入失败',str(exc))
    def task(self,fn,*args):
        try:self.ota.run_async(fn,*args)
        except Exception as e:messagebox.showerror('操作失败',str(e))
    def query(self):
        try:self.task(self.ota.query_status,validate_timeout(self.timeout.get()))
        except Exception as e:messagebox.showerror('参数错误',str(e))
    def query_version(self):
        try:self.task(self.ota.query_version,validate_timeout(self.timeout.get()))
        except Exception as e:messagebox.showerror('参数错误',str(e))
    def choose_log_directory(self):
        path=filedialog.askdirectory(title='选择压力测试日志目录')
        if path:self.log_directory.set(path)
    def start_stress(self):
        try:
            if self.ota.busy:raise RuntimeError('当前有任务正在执行')
            if not self.ota.ser or not self.ota.ser.is_open:raise RuntimeError('请先连接串口')
            self.load_image();version_parts(self.version.get())
            timeout=validate_timeout(self.timeout.get());startup=validate_timeout(self.startup_timeout.get())
            if not self.log_directory.get().strip():raise ValueError('请选择日志目录')
            self.stress=StressTest(self.ota)
            self.stress_label['text']='压测运行中：自动递增版本、重启、升级、核对APP版本，直到手动停止。'
            self.ota.run_async(self.stress.run,self.image,self.metadata['version'],timeout,startup,self.log_directory.get(),self.bin_path.get())
        except Exception as e:messagebox.showerror('压力测试启动失败',str(e))
    def stop_stress(self):
        if self.stress and self.stress.running:
            self.stress.stop();self.stress_label['text']+=' | 正在完成当前轮后停止'
        elif self.stress and self.ota.busy:self.stress.stop()
    def safe_tests(self):
        try:self.task(self.ota.tests,self.key(),validate_timeout(self.timeout.get()))
        except Exception as e:messagebox.showerror('参数错误',str(e))
    def start_upgrade(self):
        try:
            self.load_image()
            if not messagebox.askyesno('执行升级','即将向真实设备写入Flash。是否已经核实应用分区、串口配置和镜像大小？'):return
            self.task(self.ota.upgrade,self.image,self.key(),validate_timeout(self.timeout.get()))
        except Exception as e:messagebox.showerror('操作失败',str(e))
    def rollback_device(self):
        try:
            if messagebox.askyesno('确认回滚','将用B区备份覆盖A区固件，完成后启动备份版本。继续？'):
                self.task(self.ota.rollback,self.key(),validate_timeout(self.timeout.get()))
        except Exception as e:messagebox.showerror('参数错误',str(e))
    def reset_query_device(self):
        try:self.task(self.ota.reset_and_query,self.key(),validate_timeout(self.timeout.get()))
        except Exception as e:messagebox.showerror('参数错误',str(e))
    def reset_device(self):
        try:
            if messagebox.askyesno('确认重启','向设备发送重启操作码2？'):self.task(self.ota.reset,self.key(),validate_timeout(self.timeout.get()))
        except Exception as e:messagebox.showerror('参数错误',str(e))
    def offline_tests(self):
        try:
            cfg=self.config();k=self.key()
            assert len(query(cfg))==14 and crc_ok(query(cfg),cfg)
            assert len(control(cfg,1,k))==20 and crc_ok(control(cfg,1,k),cfg)
            frame=data_frame(cfg,b'\x01\x02\x03',1,cfg.first_packet,k)
            assert len(frame)==25 and crc_ok(frame,cfg)
            assert split_firmware(b'A'*1025,cfg)
            buf=RxBuffer(cfg)
            from protocol import with_crc,num
            from protocol import ADDRESS
            reply=with_crc(bytes([ADDRESS,0xAA,1])+num(cfg.first_packet,2,cfg.little_fields),cfg)
            assert not buf.feed(reply[:2]);assert buf.feed(reply[2:])[0]['ack']==1
            self.write('离线测试通过：查询/控制帧长度、CRC16、CRC32封装、分包、接收分段拼接')
        except Exception as e:messagebox.showerror('离线测试失败',str(e))
    def _close(self):
        if self.stress and self.ota.busy:
            if not self.closing:
                self.closing=True;self.stress.stop()
                self.stress_label['text']+=' | 当前轮结束后关闭窗口'
            if self.ota.busy:
                self.after(200,self._close);return
        self.ota.close();self.destroy()

if __name__=='__main__':App().mainloop()

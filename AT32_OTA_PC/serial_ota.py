"""串口升级线程；不对已写入但应答丢失的数据包做盲重传。"""
import queue, threading, time
import math
from protocol import Config,RxBuffer,split_firmware,query,control,data_frame,version_query,VERSION_CHUNK,ACK_NAMES,STATUS_NAMES,val,crc_ok
from firmware_pack import HEADER_SIZE, MIN_RAW_SIZE, MAX_IMAGE_SIZE

def validate_timeout(value):
    try:
        seconds=float(value)
    except (TypeError,ValueError):
        raise ValueError('应答超时必须是大于0的秒数，例如8或0.5') from None
    if not math.isfinite(seconds) or seconds<=0:
        raise ValueError('应答超时必须是有限且大于0的秒数')
    return seconds

class SerialOTA:
    def __init__(self, callback):
        self.callback=callback
        self.ser=None;self.reader=None;self.stop_event=threading.Event();self.frames=queue.Queue()
        self.cfg=Config();self.busy=False;self.tx_bytes=0;self.rx_bytes=0;self.log_sink=None
        self.command_sink=None;self._control_operation=None
    def command(self,text):
        sink=self.command_sink
        if sink:sink(text)
    def log(self,text):
        if self.log_sink:self.log_sink(text)
        self.callback('log',text)
    def activity(self,text):
        self.callback('activity',text)
        self.log('[阶段] '+text)
    def open(self,port,baud,cfg,parity='N',stopbits=1):
        import serial
        self.close();self.stop_event.clear();self.cfg=cfg;self.frames=queue.Queue()
        self.ser=serial.Serial(port=port,baudrate=int(baud),bytesize=8,parity=parity,stopbits=stopbits,timeout=0.1,write_timeout=3)
        self.tx_bytes=self.rx_bytes=0
        self.reader=threading.Thread(target=self._reader,daemon=True);self.reader.start()
        self.log(f'串口已打开：{port} {baud} 8{parity}{stopbits}')
        self.log('协议配置：普通字段%s端，CRC16%s端，CRC32%s端，单包上限%dB，首包号%d'%(
            '小' if cfg.little_fields else '大','小' if cfg.little_crc16 else '大',
            '小' if cfg.little_crc32 else '大',cfg.payload,cfg.first_packet))
        self.activity('串口已连接，待操作')
    def close(self):
        self.stop_event.set()
        if self.ser:
            self.log('关闭串口：本次连接累计TX=%dB RX=%dB'%(self.tx_bytes,self.rx_bytes))
            try:self.ser.close()
            except Exception:pass
        self.ser=None
    def _reader(self):
        buf=RxBuffer(self.cfg,self.log)
        while not self.stop_event.is_set() and self.ser:
            try:
                raw=self.ser.read(self.ser.in_waiting or 1)
                if raw:
                    self.rx_bytes+=len(raw)
                    # 原始字节在协议解析前显示，半帧、噪声和CRC错误数据也不遗漏。
                    self.callback('received',raw)
                    self.log(f'RX [{len(raw)}B / 累计{self.rx_bytes}B] HEX '+raw.hex(' ').upper())
                    self.log('RX 原始字节 '+repr(raw))
                    parsed=buf.feed(raw)
                    self.log(f'RX 流解析：本次提取 {len(parsed)} 帧，残留 {len(buf.buf)}B')
                    for f in parsed:
                        self.log(f'RX 完整帧 [{len(f["raw"])}B] CRC16通过 HEX '+f['raw'].hex(' ').upper())
                        if f['type']=='data':self.log('RX 解析：数据应答 %02X %s，包号=%d'%(f['ack'],ACK_NAMES.get(f['ack'],'未知'),f['packet']))
                        elif f['type']=='query':self.log('RX 解析：状态=%s(%d)，key=%08X'%(STATUS_NAMES.get(f['status'],'未知'),f['status'],f['key']))
                        elif f['type']=='version':self.log('RX 版本：模式=%s，偏移=%d，内容=%r'%(f['mode'],f['offset'],f['data']))
                        else:self.log('RX 解析：控制确认')
                        self.frames.put(f)
            except Exception as exc:
                if not self.stop_event.is_set():self.activity('串口接收异常：'+str(exc))
                break
    def send(self,frame):
        if not self.ser or not self.ser.is_open:raise RuntimeError('尚未连接串口')
        if len(frame)>=4:
            self.log('TX 帧头：地址=0x%02X 功能码=0x%02X 长度=%dB CRC16=0x%04X 本地校验=%s'%(
                frame[0],frame[1],len(frame),val(frame[-2:],self.cfg.little_crc16),'通过' if crc_ok(frame,self.cfg) else '失败'))
            read=lambda a,b:val(frame[a:b],self.cfg.little_fields)
            if frame[1]==0xAA and len(frame)>=23:
                self.log('TX 数据字段：L=%d 协议=%d key=0x%08X L1=%d total末包号=%d 当前包号=%d 固件CRC32=0x%08X'%(
                    read(2,4),read(4,6),read(6,10),read(10,12),read(12,14),read(14,16),val(frame[-6:-2],self.cfg.little_crc32)))
            elif frame[1]==0x10 and len(frame) in (14,20):
                query_frame=len(frame)==14
                self.log('TX %s：寄存器=%d 数量=%d 状态=%d key=0x%08X%s'%(
                    '查询' if query_frame else '控制',read(2,4),read(4,6),read(6,8) if query_frame else read(10,12),
                    read(8,12) if query_frame else read(12,16),'' if query_frame else ' 操作码=%d'%read(16,18)))
        self.log('TX HEX '+frame.hex(' ').upper())
        if frame[1]==0x10 and len(frame)==14:
            self.command('发送：查询 OTA 状态（站号2）')
        elif frame[1]==0x10 and len(frame)==20:
            self._control_operation=val(frame[16:18],self.cfg.little_fields)
            name={0:'无操作测试',1:'开始 OTA 升级',2:'重启设备',3:'手动回滚'}.get(self._control_operation,'未知控制')
            self.command('发送：%s（操作码%d，站号2）'%(name,self._control_operation))
        elif frame[1]==0x11 and len(frame)==8 and val(frame[2:4],self.cfg.little_fields)==0:
            self.command('发送：查询运行模式和版本号（站号2）')
        started=time.monotonic()
        written=self.ser.write(frame)
        self.tx_bytes+=written
        self.ser.flush()
        self.log('TX 串口提交并flush完成：%d/%dB，耗时 %.2fms，累计发送 %dB'%(written,len(frame),(time.monotonic()-started)*1000,self.tx_bytes))
        if written!=len(frame):raise IOError('串口发送不完整：%d/%dB'%(written,len(frame)))
    def await_reply(self,kind,timeout=4,packet=None,offset=None):
        timeout=validate_timeout(timeout)
        started=time.monotonic();deadline=started+timeout;heartbeat=started+1
        target=kind+(' 包号=%d'%packet if packet is not None else '')
        self.activity('等待 '+target+' 应答，超时阈值 %.2fs'%timeout)
        while time.monotonic()<deadline:
            if self.stop_event.is_set():raise RuntimeError('串口已关闭/任务停止')
            now=time.monotonic()
            if now>=heartbeat:
                text='等待 %s：已等 %.1fs，剩余 %.1fs'%(target,now-started,max(0,deadline-now))
                self.callback('activity',text);self.log('[等待] '+text);heartbeat=now+1
            try:r=self.frames.get(timeout=min(0.1,max(.001,deadline-time.monotonic())))
            except queue.Empty:continue
            if r['type']!=kind:
                self.log('忽略非预期应答：期待=%s，实际=%s，内容=%s'%(kind,r['type'],r));continue
            if packet is not None and r.get('packet')!=packet:
                self.log('忽略包号不一致应答：期待=%d，实际=%s'%(packet,r.get('packet')));continue
            if offset is not None and r.get('offset')!=offset:
                self.log('忽略版本偏移不一致应答：期待=%d，实际=%s'%(offset,r.get('offset')));continue
            self.log('应答匹配：%s，等待 %.2fms%s'%(target,(time.monotonic()-started)*1000,
                '，ACK=0x%02X %s'%(r['ack'],ACK_NAMES.get(r['ack'],'未知错误')) if kind=='data' else ''))
            if kind=='query':
                self.command('收到回复：OTA 状态=%s（%d）'%(STATUS_NAMES.get(r['status'],'未知'),r['status']))
            elif kind=='control':
                name={0:'无操作',1:'开始升级',2:'重启',3:'回滚'}.get(self._control_operation,'控制')
                self.command('收到回复：%s指令已确认'%name)
            elif kind=='data' and r['ack']!=1:
                self.command('收到回复：固件包%d写入失败，%s'%(r['packet'],ACK_NAMES.get(r['ack'],'未知错误')))
            return r
        elapsed=time.monotonic()-started
        self.log('[超时] %s，实际等待 %.3fs，累计TX=%dB RX=%dB'%(target,elapsed,self.tx_bytes,self.rx_bytes))
        name={'query':'查询 OTA 状态','version':'查询运行模式和版本号','control':'控制指令','data':'固件数据包'}.get(kind,kind)
        self.command('等待回复超时：%s%s，%.3f秒'%(name,'（包号%d）'%packet if packet is not None else '',elapsed))
        raise TimeoutError('等待 %s 应答超时 %.3fs（不自动重传，避免重复Flash写入）'%(target,elapsed))
    def clear_replies(self):
        discarded=0
        while not self.frames.empty():
            try:
                old=self.frames.get_nowait();discarded+=1
                self.log('清理残留应答：'+str(old))
            except queue.Empty:break
        if discarded:self.log(f'清理完成：丢弃 {discarded} 条旧应答')
    def run_async(self,fn,*args):
        if self.busy:raise RuntimeError('当前有任务正在执行')
        self.busy=True
        def worker():
            try:fn(*args)
            except Exception as e:self.log('任务失败：'+str(e));self.callback('error',str(e))
            finally:self.busy=False;self.callback('done',None)
        threading.Thread(target=worker,daemon=True).start()
    def query_status(self,timeout=4):
        timeout=validate_timeout(timeout)
        self.clear_replies();self.send(query(self.cfg));return self.await_reply('query',timeout)
    def query_version(self,timeout=4):
        deadline=time.monotonic()+validate_timeout(timeout)
        data=bytearray();mode=None
        for offset in range(0,1024,VERSION_CHUNK):
            remaining=deadline-time.monotonic()
            if remaining<=0:raise TimeoutError('版本查询超时')
            self.clear_replies();self.send(version_query(self.cfg,offset))
            reply=self.await_reply('version',remaining,offset=offset)
            if len(reply['data'])!=VERSION_CHUNK:raise ValueError('版本分块长度不匹配')
            if mode is not None and mode!=reply['mode']:raise ValueError('查询过程中设备运行模式变化')
            mode=reply['mode'];data.extend(reply['data'])
            if 0 in reply['data']:break
        raw=bytes(data).split(b'\0',1)[0]
        try:version=raw.decode('utf-8')
        except UnicodeDecodeError:raise ValueError('设备版本区不是有效 UTF-8') from None
        if not version or not version.isprintable():raise ValueError('设备版本区无有效版本号')
        result={'mode':mode,'version':version}
        self.log('设备版本：%s，运行模式：%s'%(version,mode))
        self.command('收到回复：%s，版本=%s'%('APP 模式，禁止升级' if mode=='app' else 'Bootloader 模式',version))
        self.callback('version',result)
        return result
    def start(self,key,timeout=15):
        timeout=validate_timeout(timeout)
        self.clear_replies();self.send(control(self.cfg,1,key));return self.await_reply('control',timeout)
    def rollback(self,key=0,timeout=8):
        """操作码3：校验备份并恢复至A，成功控制应答发完后设备启动。"""
        timeout=validate_timeout(timeout)
        self.clear_replies();self.send(control(self.cfg,3,key))
        result=self.await_reply('control',timeout)
        self.log('手动回滚已完成，设备将从A运行备份固件。')
        return result
    def reset(self,key=0,timeout=8):
        """操作码2：等待设备发完控制确认，设备随后复位。"""
        timeout=validate_timeout(timeout)
        self.clear_replies();self.send(control(self.cfg,2,key))
        result=self.await_reply('control',timeout)
        self.log('已收到重启确认，设备随后复位，可再次查询OTA状态。')
        return result
    def reset_and_query(self,key=0,timeout=8):
        """复位后短间隔查询，捕获 Bootloader 的约2秒握手窗口。"""
        timeout=validate_timeout(timeout)
        self.reset(key,timeout)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.stop_event.wait(0.05):
                raise RuntimeError('串口已关闭，任务停止')
            try:
                state=self.query_status(timeout=min(0.2,max(0.001,deadline-time.monotonic())))
            except TimeoutError:
                continue
            if state['status'] in (1,2,0x55):
                self.activity('Bootloader 已就绪，请立即开始 OTA（启动窗口约2秒）')
                return state
        raise TimeoutError('复位后未查询到允许 OTA 的 Bootloader 状态')

    def upgrade(self,image,key,timeout=8):
        timeout=validate_timeout(timeout)
        if not HEADER_SIZE + MIN_RAW_SIZE <= len(image) <= MAX_IMAGE_SIZE:
            raise ValueError('封装镜像长度必须在2056..57344字节内')
        chunks=split_firmware(image,self.cfg)
        started=time.monotonic();tx_start=self.tx_bytes;rx_start=self.rx_bytes
        completed=confirmed=0;packet=None;rtt_ms=None;phase='准备升级';rtts=[]
        def report(stage):
            elapsed=time.monotonic()-started
            rate=confirmed/elapsed if elapsed>0 else 0
            self.callback('progress',{'percent':int(completed*100/len(chunks)),'completed':completed,'total':len(chunks),
                'stage':stage,'packet':packet,'bytes':confirmed,'total_bytes':len(image),'elapsed':elapsed,
                'rate':rate,'eta':(len(image)-confirmed)/rate if rate>0 else None,'rtt_ms':rtt_ms,
                'tx_bytes':self.tx_bytes-tx_start,'rx_bytes':self.rx_bytes-rx_start})
        self.log('========== OTA开始 ==========')
        self.log('配置：镜像=%dB 分包=%dB 总包数=%d 首包号=%d 末包字段=%d 协议=%d 会话key=0x%08X 应答超时=%.2fs'%(
            len(image),self.cfg.payload,len(chunks),self.cfg.first_packet,len(chunks)-1,self.cfg.protocol,key,timeout))
        report(phase)
        try:
            phase='查询设备状态';self.activity(phase);report(phase)
            state=self.query_status(timeout=timeout)
            self.log('设备状态：%s(%d)，设备当前key=0x%08X'%(STATUS_NAMES.get(state['status'],'未知'),state['status'],state.get('key',0)))
            if state['status'] not in (1,2,0x55):raise RuntimeError('设备当前不允许开始升级（状态 %s）'%state['status'])
            if state['status']==2:self.log('设备保留未完成会话；本次开始OTA会保留有效备份并重新擦除运行区A，从包号0开始。')
            phase='开始握手，等待设备擦除完成';self.activity(phase);report(phase)
            handshake_started=time.monotonic()
            self.start(key,timeout=timeout)
            self.log('开始握手确认已收到，耗时 %.3fs；开始逐包传输'%(time.monotonic()-handshake_started))
            self.command('开始发送固件：%d字节，共%d包（省略逐包收发）'%(len(image),len(chunks)))
            for i,ch in enumerate(chunks):
                packet=i+self.cfg.first_packet;offset=confirmed
                phase='发送第%d/%d包（协议包号%d）'%(i+1,len(chunks),packet)
                self.activity(phase);report(phase)
                self.log('分包：序号=%d/%d 包号=%d 镜像偏移=0x%08X..0x%08X 长度=%dB 剩余=%dB'%(
                    i+1,len(chunks),packet,offset,offset+len(ch)-1,len(ch),len(image)-offset-len(ch)))
                for lo,hi,name in [(0,1024,'版本区'),(1024,2048,'SHA-256校验区'),(2048,len(image),'原始BIN')]:
                    a=max(offset,lo);b=min(offset+len(ch),hi)
                    if b>a:self.log('本包内容：%s %dB，区域内偏移0x%X'%(name,b-a,a-lo))
                self.clear_replies();frame=data_frame(self.cfg,ch,len(chunks)-1,packet,key)
                packet_started=time.monotonic();self.send(frame)
                phase='等待包号%d写入应答'%packet;report(phase)
                result=self.await_reply('data',timeout,packet)
                rtt_ms=(time.monotonic()-packet_started)*1000
                if result['ack']!=1:raise RuntimeError('第%d包设备拒绝：ACK=0x%02X %s'%(packet,result['ack'],ACK_NAMES.get(result['ack'],'未知错误')))
                completed=i+1;confirmed+=len(ch);rtts.append(rtt_ms)
                elapsed=time.monotonic()-started;rate=confirmed/max(elapsed,.001)
                self.log('包号%d已确认：ACK=0x01，发送至确认耗时 %.2fms，已确认 %d/%d 包、%d/%dB，平均 %.2fKiB/s，预计剩余 %.2fs'%(
                    packet,rtt_ms,completed,len(chunks),confirmed,len(image),rate/1024,(len(image)-confirmed)/rate))
                report('包号%d已确认'%packet)
            phase='传输完成，设备SHA-256校验和跳转结果待确认'
            report(phase);self.activity(phase)
            self.log('OTA传输汇总：已确认%d包/%dB，总耗时%.3fs，TX=%dB RX=%dB，包确认耗时最小/平均/最大=%.2f/%.2f/%.2fms'%(
                completed,confirmed,time.monotonic()-started,self.tx_bytes-tx_start,self.rx_bytes-rx_start,min(rtts),sum(rtts)/len(rtts),max(rtts)))
            self.log('当前协议没有整镜像SHA-256校验或应用启动结果应答；100%仅表示全部数据包已获0x01，需查看设备实际启动结果。')
            self.command('收到回复：全部%d个固件包写入确认；等待 APP 启动并核对版本'%len(chunks))
        except Exception as exc:
            self.log('OTA停止：阶段=%s，包号=%s，已确认%d/%d包、%d/%dB，耗时%.3fs，原因=%s'%(
                phase,packet,completed,len(chunks),confirmed,len(image),time.monotonic()-started,exc))
            report('失败：'+phase);self.activity('失败：'+str(exc))
            raise
    def tests(self,key,timeout=6):
        """真实设备只执行查询和操作码0；不发送开始OTA、重启或AA数据。"""
        timeout=validate_timeout(timeout)
        self.log('测试1：查询OTA状态')
        before=self.query_status(timeout=timeout)
        assert before['status'] in (0,1,2,3,0x55)
        self.log('状态=%s(%d)，设备当前会话key=0x%08X'%(STATUS_NAMES.get(before['status'],'未知'),before['status'],before['key']))
        self.log('测试2：使用设备当前key发送无操作控制（操作码0）')
        self.clear_replies();self.send(control(self.cfg,0,before['key']))
        self.await_reply('control',timeout=timeout)
        self.log('测试3：再次查询，确认状态和会话key未变化')
        after=self.query_status(timeout=timeout)
        assert after['status']==before['status'] and after['key']==before['key'],'无操作前后的状态或key不同'
        self.log('查询/无操作测试完成；未发送操作码1/2或AA数据，不擦写Flash。')

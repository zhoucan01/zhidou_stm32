#include "modbus_port.h"
#include "boot_log.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define USART_RDBF_FLAG 32u
#define USART_IDLEF_FLAG 16u
#define USART_ROERR_FLAG 8u
#define USART_NERR_FLAG 4u
#define USART_FERR_FLAG 2u
#define USART_PERR_FLAG 1u
#define USART_TDBE_FLAG 128u
#define USART_TDC_FLAG 64u
#define USART_IDLE_INT 1u
#define USART_ERR_INT 2u
#define RESET 0
#define GPIOB 0
#define GPIO_PINS_2 4
#define __DMB() ((void)0)
#define __disable_irq() ((void)0)
#define __get_PRIMASK() 0u
#define __set_PRIMASK(x) ((void)(x))
#define TRUE 1
#define FALSE 0
#define DMA1_GL1_FLAG 1u
#define DMA1_DTERR1_FLAG 8u
#define DMA1_FDT1_FLAG 2u
#define DMA_FDT_INT 2u
#define DMA_DTERR_INT 8u
static struct { uint32_t sts,dt; } uart;
static struct { uint32_t dtcnt,paddr,maddr; } dma_regs;
#define USART3 (&uart)
#define UART5 ((void *)5)
static char debug_text[8192];
static unsigned debug_size;
#define DMA1_CHANNEL1 (&dma_regs)
static unsigned dma_full,dma_error,dma_enabled,dma_interrupts,uart_interrupts,receiver;
static unsigned direction,tdc_seen,hold_seen,sent,resets;
static uint8_t tx[128];
volatile uint32_t ticks_count_us=1;
static uint32_t now=0xffffff;
static uint32_t fake_tick(void) { return now; }
static void dma_channel_enable(void *d,int e) { (void)d;dma_enabled=(unsigned)e; }
static void dma_flag_clear(unsigned f) { (void)f;dma_error=dma_full=0; }
static int dma_flag_get(unsigned f) { return f==DMA1_FDT1_FLAG?(int)dma_full:(int)dma_error; }
static void wk_dma_channel_config(void *d,uint32_t p,uint32_t m,uint16_t n)
{ (void)d;dma_regs.dtcnt=n;dma_regs.paddr=p;dma_regs.maddr=m; }
static void usart_interrupt_enable(void *u,unsigned f,int e)
{ (void)u;if(e)uart_interrupts|=f;else uart_interrupts&=~f; }
static void dma_interrupt_enable(void *u,unsigned f,int e)
{ (void)u;if(e)dma_interrupts|=f;else dma_interrupts&=~f; }
static void usart_receiver_enable(void *u,int e) { (void)u;receiver=(unsigned)e; }
static void gpio_bits_set(int p,int b)
{ (void)p;(void)b;assert(!direction && !dma_enabled);direction=1;tdc_seen=hold_seen=0; }
static void gpio_bits_reset(int p,int b)
{ (void)p;(void)b;if(direction)assert(tdc_seen && hold_seen && dma_enabled);direction=0; }
static void wk_delay_us(unsigned t) { assert(t==30 && direction && tdc_seen);hold_seen=1; }
static int usart_flag_get(void *u,unsigned f)
{ if(u==UART5)return 1;assert(direction);if(f==USART_TDC_FLAG)tdc_seen=1;return 1; }
static void usart_flag_clear(void *u,unsigned f) { (void)u;(void)f;uart.sts=0; }
static void usart_data_transmit(void *u,unsigned v)
{ if(u==UART5) { assert(debug_size+1<sizeof(debug_text));debug_text[debug_size++]=(char)v;debug_text[debug_size]=0;return; }assert(direction && sent<sizeof(tx));tx[sent++]=(uint8_t)v; }
static void nvic_system_reset(void) { assert(tdc_seen && hold_seen && !direction);++resets; }
static uint8_t version_header[1024]="1.2.345";
#define MODBUS_VERSION_BASE version_header

#include "modbus_port_under_test.inc"

static void put(uint8_t *p,unsigned n) { p[0]=(uint8_t)(n>>8);p[1]=(uint8_t)n; }
static void advance(unsigned us) { now=(now-us)&0xffffff; }
static void feed(const uint8_t *p,unsigned n)
{
    assert(dma_enabled);memcpy(mb_dma_target,p,n);DMA1_CHANNEL1->dtcnt=MODBUS_PORT_DMA_SIZE-n;
    uart.sts=USART_IDLEF_FLAG;modbus_port_irq();uart.sts=0;
}
int main(void)
{
    uint8_t frame[25]={MODBUS_ADDRESS,0xaa},fw[3];modbus_aa_data_t out;unsigned crc;
    boot_log_start();boot_notice("TEST",1234u,5678u);boot_log_flush();
    assert(strstr(debug_text,"TEST 1234/5678\r\n"));
    assert(modbus_port_init()==0);mb_dma_restart();
    put(frame+2,25);put(frame+4,1);put(frame+10,3);put(frame+12,2);put(frame+14,1);
    frame[16]=1;frame[17]=2;frame[18]=3;
    crc=modbus_crc16(frame,23);frame[23]=(uint8_t)crc;frame[24]=(uint8_t)(crc>>8);
    feed(frame,16);assert(modbus_port_get_aa(&out,fw,3)==0);
    feed(frame+16,9);assert(modbus_port_get_aa(&out,fw,3)==1 && memcmp(fw,frame+16,3)==0);
    feed(frame,25);assert(mb_head!=mb_tail);
    frame[24]^=1;feed(frame,25);assert(mb_error_head!=mb_error_tail);
    modbus_port_pause_rx();
    assert(mb_rx_paused && !dma_enabled && !receiver && uart_interrupts==0 && dma_interrupts==0);
    assert(mb_head==mb_tail && mb_error_head==mb_error_tail && mb_assembly_size==0);
    dma_full=1;uart.sts=USART_IDLEF_FLAG;modbus_port_irq();assert(!dma_enabled);
    modbus_port_resume_rx();
    assert(!mb_rx_paused && dma_enabled && receiver && uart_interrupts==3 && dma_interrupts==10);
    assert(modbus_port_get_aa(&out,fw,3)==0 && modbus_port_reply_errors()==0);
    frame[24]^=1;feed(frame,25);assert(modbus_port_get_aa(&out,fw,3)==1);
    feed(frame,16);advance(3499);modbusdeal();assert(sent==0);
    advance(1);modbusdeal();assert(sent==7 && tx[2]==3 && modbus_crc16(tx,7)==0);
    frame[24]^=1;feed(frame,25);advance(2000);modbusdeal();assert(sent==14 && tx[9]==7);
    put(frame+2,1047);put(frame+10,1025);feed(frame,16);advance(2000);modbusdeal();
    assert(sent==21 && tx[16]==8 && resets==0);
    /* 完整控制帧操作码2：必须发完8字节确认、释放总线后才复位。 */
    modbus_port_pause_rx();modbus_port_resume_rx();
    memset(frame,0,20);frame[0]=MODBUS_ADDRESS;frame[1]=0x10;put(frame+2,0xe6);
    put(frame+4,4);put(frame+8,8);put(frame+10,1);put(frame+16,2);
    crc=modbus_crc16(frame,18);frame[18]=(uint8_t)crc;frame[19]=(uint8_t)(crc>>8);
    feed(frame,20);
    mb_ready=0; /* 模拟发送层暂不可用，重启请求不能丢失。 */
    modbusdeal();assert(resets==0 && modbus_reply_pending());
    mb_ready=1;advance(2000);modbusdeal();
    assert(resets==1 && sent==29 && tx[21]==MODBUS_ADDRESS && tx[22]==0x10 && tx[26]==4);
    assert(modbus_crc16(tx+21,8)==0);
    boot_log_poll();assert(strstr(debug_text,"CRC16") && strstr(debug_text,"RX: reboot") && strstr(debug_text,"Reboot ACK sent; resetting") && strstr(debug_text,"1/2"));
    /* Version requests survive DMA fragmentation, do not enter OTA queues,
       report Bootloader mode, and remain pending when TX is unavailable. */
    modbus_port_pause_rx();modbus_port_resume_rx();sent=0;
    memset(frame,0,8);frame[0]=MODBUS_ADDRESS;frame[1]=0x11;put(frame+4,32);
    crc=modbus_crc16(frame,6);frame[6]=(uint8_t)crc;frame[7]=(uint8_t)(crc>>8);
    feed(frame,3);feed(frame+3,5);assert(mb_version_pending);
    mb_ready=0;assert(modbus_port_reply_version()==0 && mb_version_pending);
    mb_ready=1;advance(2000);modbusdeal();
    assert(sent==40 && tx[0]==MODBUS_ADDRESS && tx[1]==0x11 && tx[2]==0);
    assert(tx[5]==32 && memcmp(tx+6,version_header,32)==0 && modbus_crc16(tx,40)==0);
    assert(!mb_version_pending && modbus_port_get_10(&((modbus_10_data_t){0}))==0);
    sent=0;frame[7]^=1;feed(frame,8);assert(!mb_version_pending);
    put(frame+2,1020);crc=modbus_crc16(frame,6);frame[6]=(uint8_t)crc;frame[7]=(uint8_t)(crc>>8);
    feed(frame,8);assert(!mb_version_pending && sent==0);
    puts("DMA pause/resume and receive scenarios passed");return 0;
}

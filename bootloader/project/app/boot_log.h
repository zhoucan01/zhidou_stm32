#ifndef BOOT_LOG_H
#define BOOT_LOG_H
#include <stdint.h>
#include <stddef.h>

/* 关闭后不产生串口日志；载荷默认只打印前16字节，可改为1024。 */
#ifndef BOOT_LOG_ENABLED
#define BOOT_LOG_ENABLED 1
#endif
#define BOOT_LOG_PAYLOAD_BYTES 16u
#ifndef BOOT_LOG_VERBOSE
#define BOOT_LOG_VERBOSE 0 /* 1额外显示原始HEX、地址和详细校验值 */
#endif
#define BOOT_LOG_NONE 0xFFFFFFFFu

#if BOOT_LOG_ENABLED
/* 普通上下文调用：默认英文说明+十进制包号；详细模式附加HEX，不引入printf。
 * 所有日志字符串均为ASCII，不依赖终端中文编码。 */
void boot_log_start(void);
void boot_notice(const char *text, uint32_t first, uint32_t second);
/* 普通上下文使用；自动记录调用位置，数值规则与 boot_notice 相同。 */
void boot_trace_at(const char *function, uint32_t line, const char *text,
                   uint32_t first, uint32_t second);
#define boot_trace(text, first, second) \
    boot_trace_at(__func__, __LINE__, (text), (first), (second))
#if BOOT_LOG_VERBOSE
void boot_log(const char *tag, uint32_t first, uint32_t second);
void boot_log_hex(const char *tag, const uint8_t *data, size_t size);
#else
#define boot_log(tag, a, b) ((void)0)
#define boot_log_hex(tag, data, size) ((void)0)
#endif
void boot_log_poll(void);
void boot_log_flush(void);
#else
#define boot_notice(text, a, b) ((void)0)
#define boot_trace(text, a, b) ((void)0)
#define boot_log_start() ((void)0)
#define boot_log(tag, a, b) ((void)0)
#define boot_log_hex(tag, data, size) ((void)0)
#define boot_log_poll() ((void)0)
#define boot_log_flush() ((void)0)
#endif
#endif

#ifndef __LOG_H
#define __LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void log_write(char level, const char *function, uint32_t line,
               const char *format, ...);

#ifdef __cplusplus
}
#endif

#define log_d(...) log_write('D', __FUNCTION__, (uint32_t)__LINE__, __VA_ARGS__)
#define log_e(...) log_write('E', __FUNCTION__, (uint32_t)__LINE__, __VA_ARGS__)

#endif

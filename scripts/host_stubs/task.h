#ifndef HOST_TASK_H
#define HOST_TASK_H
#include <stdint.h>
static inline void vTaskDelay(uint32_t ms) { (void)ms; }
static inline uint32_t ulTaskNotifyTake(int clear, uint32_t ms) { (void)clear; (void)ms; return 0; }
#endif

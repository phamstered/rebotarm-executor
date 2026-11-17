#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
typedef struct { int unused; } FDCAN_HandleTypeDef;
typedef struct { uint32_t CR, SR, DR, HTCR; } HostRng;
extern HostRng host_rng;
#define RNG (&host_rng)
#define RNG_CR_RNGEN 1U
#define RNG_CR_CED 2U
#define RNG_CR_CONDRST 4U
#define RNG_SR_DRDY 1U
#define RNG_SR_SECS 2U
#define RNG_SR_CECS 4U
#define RNG_SR_SEIS 8U
#define RNG_SR_CEIS 16U
#define RCC_RNGCLKSOURCE_HSI48 0
#define __HAL_RCC_RNG_CONFIG(x) ((void)(x))
#define __HAL_RCC_RNG_CLK_ENABLE() ((void)0)
uint32_t HAL_GetTick(void);
#endif

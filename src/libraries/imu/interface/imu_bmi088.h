#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t debug_value;

void bmi088_initialize();

bool bmi088_acce_ready();

bool bmi088_gyro_ready();

uint64_t bmi088_acce_read(float* data);

uint64_t bmi088_gyro_read(float* data);

void bmi088_deinitialize();

#ifdef __cplusplus
}
#endif
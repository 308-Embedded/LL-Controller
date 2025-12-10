#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t debug_value;

void bmi088_initialize();

void bmi088_deinitialize();

#ifdef __cplusplus
}
#endif
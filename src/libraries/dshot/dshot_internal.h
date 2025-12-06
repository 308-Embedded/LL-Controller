#pragma once
#include <stdint.h>
// implementation of DSHOT600
#ifdef __cplusplus
extern "C" {
#endif

void bdshot_initialize();

void bdshot_deinitialize();

void bdshot_start();

void bdshot_write(int channel, uint32_t throttle);

uint32_t bdshot_read(int channel);

#ifdef __cplusplus
}
#endif
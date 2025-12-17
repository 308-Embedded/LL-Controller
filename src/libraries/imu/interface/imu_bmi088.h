#pragma once
#include <stdint.h>
/*
    TODO: switch to synchronized driver.
    gyro sampling time  : 500us
    acce sampling time  : 625us
    gyro - acce latency : 25us
*/
#ifdef __cplusplus
extern "C" {
#endif

void bmi088_initialize();

bool bmi088_acce_ready();

bool bmi088_gyro_ready();

uint64_t bmi088_acce_read(float* data);

uint64_t bmi088_gyro_read(float* data);

void bmi088_deinitialize();

// this will block the current thread until the internal semaphore is notified.
void bmi088_wait();

#ifdef __cplusplus
}
#endif
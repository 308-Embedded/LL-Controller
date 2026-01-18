#pragma once

#include <ringbuf.h>

#define LOG_BUFFER_SIZE 2048

struct LogBuffer {
    ringbuf_t rb;
    uint8_t buffer[LOG_BUFFER_SIZE];
};

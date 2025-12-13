#pragma once

#define MAX_TABLE_SIZE 32

struct SharedTableEntry {
    char name[24];      // Name 
    uint32_t tag;       // type tag
    void* address;     // Address
};

struct Registry {
    char name[24]; 
    uint32_t tag;
    uint32_t size;
};

typedef struct SharedTableEntry SharedTable;


#pragma once

#include "../SharedTableDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SharedTable* gSharedTable;

int st_initialize();

int st_find(const char* var_name);

int st_write(const int entry, const void* data);

int st_read(const int entry, void* data);

int st_list();

int st_status();

const char* tag_to_string(int tag);

#ifdef __cplusplus
}
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arch/barriers.h>
#include <stm32_dtcm.h>
#include <ll_types/motor.h>
#include "SharedTable.h"


/* ---------------------------------------- */
/* ----------Edit to Add new type---------- */
/* ---------------------------------------- */
typedef enum {
    t_QuadrotorCmd = 0
} TagType;

const char* tag_to_string(int tag) {
    switch (tag) {
        case t_QuadrotorCmd:
            return "QuadrotorCmd";
        default:
            return "Unknown Tag";
    }
}


const struct Registry gRegistedMembers[] = {
    {"motor_cmd", t_QuadrotorCmd, sizeof(struct QuadrotorCmd)}
};

/* ---------------------------------------- */

SharedTable* gSharedTable;

int st_initialize()
{   
    dtcm_initialize();
    
    int table_size = sizeof(gRegistedMembers) / sizeof(gRegistedMembers[0]);
    if(table_size > MAX_TABLE_SIZE)
    {
        return -1;
    }

    gSharedTable = (SharedTable*)dtcm_malloc(MAX_TABLE_SIZE * sizeof(SharedTable));

    if(gSharedTable == NULL)
    {
        return -1;
    }

    for(int i = 0; i < table_size; i++)
    {
        strcpy(gSharedTable[i].name, gRegistedMembers[i].name);
        gSharedTable[i].tag = gRegistedMembers[i].tag;
        gSharedTable[i].address = dtcm_memalign(32, gRegistedMembers[i].size); // 32byte alignment
    }
    return 0;
} 

int st_list()
{
    int table_size = sizeof(gRegistedMembers) / sizeof(gRegistedMembers[0]);
    return table_size;
}

int st_find(const char* var_name)
{
    int table_size = sizeof(gRegistedMembers) / sizeof(gRegistedMembers[0]);
    for(int i = 0; i < table_size; i++)
    {
        if(0 == strcmp(var_name, gSharedTable[i].name))
        {
            return i;
        }
    }
    return -1;
}

int st_write(const int entry, const void* data)
{
    memcpy(gSharedTable[entry].address, data, gRegistedMembers[entry].size);
    UP_DMB();
    return 0;
}

int st_read(const int entry, void* data)
{
    UP_DMB();
    memcpy(data, gSharedTable[entry].address, gRegistedMembers[entry].size);
    return 0;
}
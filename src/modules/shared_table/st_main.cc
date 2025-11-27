#include <stdio.h>
#include <stdlib.h>
#include <SharedTable.h>

extern "C" {
int st_main(int argc, char** argv) {

    uint8_t flag_list = 0;
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "-l") == 0)
        {
            flag_list = 1;
            i++;
        }
    }

    st_initialize();

    if(flag_list)
    {
        int num = st_list();
        for(int i = 0; i < num; i++)
        {
            const char* type = tag_to_string(gSharedTable[i].tag);
            printf("name: %s, type: %s, address: %p\n", gSharedTable[i].name, type, gSharedTable[i].address);
        }
    }
    return 0;
}
}

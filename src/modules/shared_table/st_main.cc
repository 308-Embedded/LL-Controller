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
        else if (strcmp(argv[i], "-h") == 0)
        {
            printf("st:\n  [-l] : list all shared variables \n  [-i] : initialize shared table\n  [-t] : test ring buffer\n");
            return 0;
        }
        else if (strcmp(argv[i], "-i") == 0)
        {
            printf("shared table initialized \n");
            st_initialize();
        }
        else if (strcmp(argv[i], "-t") == 0)
        {
            printf("testing ring buffer\n");
            int entry = st_find("log_message");
            if(entry < 0)
            {
                printf("could not find log_message\n");
                return -1;
            }

            char* test_data = "hello world";
            st_write_block(entry, test_data, strlen(test_data));

            char read_buf[128];
            st_read_block(entry, read_buf, strlen(test_data));
            
            if(strcmp(test_data, read_buf) == 0)
            {
                printf("ring buffer test passed\n");
            }
            else
            {
                printf("ring buffer test failed\n");
            }
        }
    }

    if(0!=st_status())
    {
        printf("shared table not initialized \n");
        return 0;
    }

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

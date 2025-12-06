#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>
#include <dshot_internal.h>

extern "C"
{
    int dshot_main(int argc, FAR char *argv[])
    {
        float throttle = 0;
        for (int i = 1; i < argc; ++i)
        {
            if (strcmp(argv[i], "-t") == 0 && i + 1 < argc)
            {
                throttle = static_cast<float>(atof(argv[i + 1]));
                i++;
            }
        }

        bdshot_initialize();

        bdshot_write(1, 1000);
        bdshot_write(2, 1000);
        bdshot_write(3, 1000);
        bdshot_write(4, 1000);

        int cnt =0;
        while(cnt < 5000)
        {
            usleep(1000);
            cnt++;
            printf("rpm %d \n",bdshot_read(1));
            if(cnt>=4000)
            {
                bdshot_write(1, 1600);
                bdshot_write(2, 1600);
                bdshot_write(3, 1600);
                bdshot_write(4, 1600);
            }
            usleep(1000);
 
            bdshot_start();
        }
        bdshot_deinitialize();
        return 0;
    }
}
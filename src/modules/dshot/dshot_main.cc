#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>
#include <dshot.h>

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
        DShot::DShot mDshot{};
        mDshot.register_motor_channel_map(1, 2, 3, 4);

        mDshot.set_motor_throttle(0,0,0,0);

        int cnt =0;
        while(cnt < 5000)
        {
            usleep(1000);
            cnt++;
            auto rpms = mDshot.get_motor_rpms();
            if(cnt % 20 ==0)
            {
                printf("rpm  %d  %d  %d  %d\n", rpms[0], rpms[1], rpms[2], rpms[3]);
            }
            if(cnt>=3000)
            {
                mDshot.set_motor_throttle(0.2, 0.2, 0.2, 0.2);
            }
            else
            {
                mDshot.set_motor_throttle(0.0, 0.0, 0.0, 0.0);
            }
            usleep(1000);
        }

        return 0;
    }
}
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/time.h>
#include <dshot.h>

extern "C"
{
    int dshot_main(int argc, FAR char *argv[])
    {
        float throttle = 0.12f;
        for (int i = 1; i < argc; ++i)
        {
            if (strcmp(argv[i], "-t") == 0 && i + 1 < argc)
            {
                throttle = static_cast<float>(atof(argv[i + 1]));
                i++;
            }
        }

        if (throttle < 0.0f)
        {
            throttle = 0.0f;
        }
        else if (throttle > 0.25f)
        {
            throttle = 0.25f;
        }

        printf("[dshot-test] REMOVE PROPS before running this test.\n");
        printf("[dshot-test] throttle %.3f, motors start after 3 seconds.\n",
               throttle);

        DShot::DShot mDshot{};
        mDshot.register_motor_channel_map(1, 2, 3, 4);

        mDshot.set_motor_throttle(0,0,0,0);
        usleep(3000000);

        int cnt =0;
        while(cnt < 5000)
        {
            usleep(1000);
            cnt++;
            auto rpms = mDshot.get_motor_rpms();
            if(cnt % 20 ==0)
            {
                printf("[dshot-test] rpm  %lu  %lu  %lu  %lu\n",
                       static_cast<unsigned long>(rpms[0]),
                       static_cast<unsigned long>(rpms[1]),
                       static_cast<unsigned long>(rpms[2]),
                       static_cast<unsigned long>(rpms[3]));
            }
            if(cnt>=3000)
            {
                mDshot.set_motor_throttle(throttle, throttle, throttle, throttle);
            }
            else
            {
                mDshot.set_motor_throttle(0.0, 0.0, 0.0, 0.0);
            }
            usleep(1000);
        }

        mDshot.set_motor_throttle(0.0, 0.0, 0.0, 0.0);
        printf("[dshot-test] done, motors stopped.\n");

        return 0;
    }
}

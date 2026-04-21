#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dshot.h>

namespace
{
float parse_throttle(int argc, char *argv[])
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
        return 0.0f;
    }

    if (throttle > 0.25f)
    {
        return 0.25f;
    }

    return throttle;
}

void print_rpms(DShot::DShot &dshot)
{
    auto rpms = dshot.get_motor_rpms();
    printf("[motor-test] rpm %lu %lu %lu %lu\n",
           static_cast<unsigned long>(rpms[0]),
           static_cast<unsigned long>(rpms[1]),
           static_cast<unsigned long>(rpms[2]),
           static_cast<unsigned long>(rpms[3]));
}

void stop_motors(DShot::DShot &dshot)
{
    dshot.set_motor_throttle(0.0f, 0.0f, 0.0f, 0.0f);
}
}

extern "C"
{
    int motor_test_main(int argc, char *argv[])
    {
        const float throttle = parse_throttle(argc, argv);

        printf("[motor-test] REMOVE PROPS before running this test.\n");
        printf("[motor-test] throttle %.3f, sequence starts after 3 seconds.\n",
               static_cast<double>(throttle));

        DShot::DShot dshot{};
        dshot.register_motor_channel_map(1, 2, 3, 4);
        stop_motors(dshot);

        usleep(3000000);

        const float motor_cmds[4][4] = {
            {throttle, 0.0f,     0.0f,     0.0f},
            {0.0f,     throttle, 0.0f,     0.0f},
            {0.0f,     0.0f,     throttle, 0.0f},
            {0.0f,     0.0f,     0.0f,     throttle},
        };

        for (int motor = 0; motor < 4; ++motor)
        {
            printf("[motor-test] M%d on\n", motor + 1);

            for (int tick = 0; tick < 1000; ++tick)
            {
                dshot.set_motor_throttle(motor_cmds[motor][0],
                                          motor_cmds[motor][1],
                                          motor_cmds[motor][2],
                                          motor_cmds[motor][3]);

                if (tick % 100 == 0)
                {
                    print_rpms(dshot);
                }

                usleep(1000);
            }

            stop_motors(dshot);
            usleep(1000000);
        }

        printf("[motor-test] all motors low throttle\n");
        for (int tick = 0; tick < 1500; ++tick)
        {
            dshot.set_motor_throttle(throttle, throttle, throttle, throttle);

            if (tick % 100 == 0)
            {
                print_rpms(dshot);
            }

            usleep(1000);
        }

        stop_motors(dshot);
        usleep(500000);
        stop_motors(dshot);
        printf("[motor-test] done, motors stopped.\n");

        return 0;
    }
}

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <dshot.h>
#include <imu_bmi088.h>

namespace
{
volatile int g_test_running = 0;
float g_test_throttle = 0.12f;

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

void stop_motors(DShot::DShot &dshot)
{
    dshot.set_motor_throttle(0.0f, 0.0f, 0.0f, 0.0f);
}

void print_rpms(DShot::DShot &dshot)
{
    auto rpms = dshot.get_motor_rpms();
    printf("[microair-test] rpm %lu %lu %lu %lu\n",
           static_cast<unsigned long>(rpms[0]),
           static_cast<unsigned long>(rpms[1]),
           static_cast<unsigned long>(rpms[2]),
           static_cast<unsigned long>(rpms[3]));
}

int microair_motor_task(int argc, char *argv[])
{
    DShot::DShot dshot{};
    dshot.register_motor_channel_map(1, 2, 3, 4);
    stop_motors(dshot);

    usleep(3000000);

    const float motor_cmds[4][4] = {
        {g_test_throttle, 0.0f,            0.0f,            0.0f},
        {0.0f,            g_test_throttle, 0.0f,            0.0f},
        {0.0f,            0.0f,            g_test_throttle, 0.0f},
        {0.0f,            0.0f,            0.0f,            g_test_throttle},
    };

    for (int motor = 0; motor < 4; ++motor)
    {
        printf("[microair-test] M%d on\n", motor + 1);

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

    printf("[microair-test] all motors low throttle\n");
    for (int tick = 0; tick < 1500; ++tick)
    {
        dshot.set_motor_throttle(g_test_throttle,
                                 g_test_throttle,
                                 g_test_throttle,
                                 g_test_throttle);

        if (tick % 100 == 0)
        {
            print_rpms(dshot);
        }

        usleep(1000);
    }

    stop_motors(dshot);
    usleep(500000);
    stop_motors(dshot);

    g_test_running = 0;
    printf("[microair-test] motor sequence done, motors stopped.\n");
    return 0;
}

int microair_imu_task(int argc, char *argv[])
{
    float gyro[3] = {};
    float accel[3] = {};
    int sample = 0;

    bmi088_initialize();
    printf("[microair-test] BMI088 test started on board SPI bus.\n");

    while (g_test_running)
    {
        bmi088_gyro_read(gyro);
        bmi088_acce_read(accel);

        if ((sample % 20) == 0)
        {
            printf("[microair-test] gyro(rad/s) %.4f %.4f %.4f | accel(m/s2) %.4f %.4f %.4f\n",
                   static_cast<double>(gyro[0]),
                   static_cast<double>(gyro[1]),
                   static_cast<double>(gyro[2]),
                   static_cast<double>(accel[0]),
                   static_cast<double>(accel[1]),
                   static_cast<double>(accel[2]));
        }

        sample++;
        usleep(10000);
    }

    bmi088_deinitialize();
    printf("[microair-test] BMI088 test stopped.\n");
    return 0;
}
}

extern "C"
{
    int microair_test_main(int argc, char *argv[])
    {
        g_test_throttle = parse_throttle(argc, argv);
        g_test_running = 1;

        printf("[microair-test] REMOVE PROPS before running motor output.\n");
        printf("[microair-test] BMI088 + motor test, throttle %.3f.\n",
               static_cast<double>(g_test_throttle));
        printf("[microair-test] MicoAir743v2: BMI088 on SPI2, motors on M1-M4 DShot pins.\n");

        int ret = task_create("microair_imu", 120, 40960, microair_imu_task, NULL);
        if (ret < 0)
        {
            printf("[microair-test] failed to start IMU task: %d\n", ret);
            g_test_running = 0;
            return ret;
        }

        ret = task_create("microair_motor", 120, 40960, microair_motor_task, NULL);
        if (ret < 0)
        {
            printf("[microair-test] failed to start motor task: %d\n", ret);
            g_test_running = 0;
            return ret;
        }

        return 0;
    }
}

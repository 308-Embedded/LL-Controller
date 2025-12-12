#include <imu_bmi088.h>
#include <stdio.h>
extern "C"
{
    int vibration_main(int argc, FAR char *argv[])
    {
        bmi088_initialize();

        uint64_t time;
        float data[4];
        while(1)
        {
            if(bmi088_gyro_ready())
            {
                time = bmi088_gyro_read(data);
                printf("%llu  - %f %f %f  \n", time, data[0], data[1], data[2]);
            }
            usleep(1000);
        }
    }
}
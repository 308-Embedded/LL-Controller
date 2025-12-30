#include "common.h"
#include <imu_bmi088.h>
#include <stdio.h>
struct imuData g_imu;
sem_t ctrl_rdy;

int imu_task(int argc, char *argv[])
{
    float raw_data[6];
    uint64_t gyro_time, acce_time;
    int cnt = 0;
    sem_init(&ctrl_rdy, 0, 0);
    bmi088_initialize();
    
    while(1)
    {
        bmi088_wait();

        if(bmi088_acce_ready())
        {
            acce_time = bmi088_acce_read(raw_data+3);
        }

        if(bmi088_gyro_ready())
        {
            cnt++;
            //  read out gyro data
            gyro_time = bmi088_gyro_read(raw_data);

            //  integration

            /*  
                filtering
            */

            //  dynamic notch
            
            //  low pass

            // control frequency 500hz
            if(cnt % 2 == 1)
            {
                sem_post(&ctrl_rdy);
                
            }
        }
        
    }
    bmi088_deinitialize();
    
    return 0;
}
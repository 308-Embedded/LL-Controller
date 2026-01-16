#include "common.h"
#include <imu_bmi088.h>
#include <stdio.h>
#include <EmbeddedLie.hpp>
#include <AttiFilter.hpp>
#include <cmath>
struct imuData g_imu;
sem_t ctrl_rdy;
uint8_t reset = 0;

int imu_task(int argc, char *argv[])
{
    using namespace Filter;
    using namespace EmbeddedMath;
    float raw_data[6];
    uint64_t gyro_time, acce_time;
    uint64_t prev_gyro_time, prev_acce_time;
    static int cnt = 0;

    Vector3f prev_omega = Vector3f::Zero();
    Vector3f prev_prev_omega = Vector3f::Zero();
    Vector3f prev_prev_prev_omega = Vector3f::Zero();
    Vector3f gyro_noise(1e-6, 1e-6, 1e-6);
    Vector3f meas_noise(1, 1, 1);

    AttiFilter<float> att_filter{};

    sem_init(&ctrl_rdy, 0, 0);
    bmi088_initialize();
    
    while(1)
    {
        bmi088_wait();

        if(bmi088_acce_ready())
        {
            acce_time = bmi088_acce_read(raw_data+3);

            g_imu.linear_acce = Vector3f(-raw_data[4], raw_data[3], raw_data[5]) - g_imu.acce_bias;

            auto z = g_imu.linear_acce;
            z = z / 9.81;

            if(abs(z.norm() - 1)< 0.2 && g_imu.angular_vel.norm()< M_PI/6)
            {
                att_filter.update(z);
            }

        }

        if(bmi088_gyro_ready())
        {
            cnt++;
            //  read out gyro data
            gyro_time = bmi088_gyro_read(raw_data);
            g_imu.angular_vel = Vector3f(-raw_data[1], raw_data[0], raw_data[2]) - g_imu.gyro_bias;

            
            // handle controrl logic
            if(reset)
            {
                reset = 0;
                g_imu.quat = Quaternionf::Identity();
            }

            //  integration
            // g_imu.quat = g_imu.quat * EmbeddedLie::quat_Exp(g_imu.angular_vel * 0.001);

            g_imu.quat = att_filter.predict(g_imu.angular_vel);

            // printf("R: %f P: %f Y: %f \n", euler(0) * 180 / M_PI, euler(1)* 180 / M_PI, euler(2)* 180 / M_PI);
            /*  
                filtering
            */

            //  dynamic notch
            
            //  low pass


            g_imu.angular_acce = (g_imu.angular_vel - prev_omega) * 1000;
            // control frequency 500hz
            prev_gyro_time = gyro_time;
            prev_omega = g_imu.angular_vel;
            prev_prev_omega = prev_omega;
            prev_prev_prev_omega = prev_prev_omega;
            if(cnt % 2 == 1)
            {
                sem_post(&ctrl_rdy);
            }
        }
        
    }
    bmi088_deinitialize();
    
    return 0;
}
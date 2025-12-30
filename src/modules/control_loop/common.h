#pragma once
#include <semaphore.h>
#include <EmbeddedMath.hpp>
using namespace EmbeddedMath;
struct imuData
{
    Vector3f angular_vel = Vector3f::Zero(); // gyro 
    Vector3f linear_acce = Vector3f::Zero(); // acce
    Vector3f gyro_bias = Vector3f::Zero(); // gyro bias
    Vector3f acce_bias = Vector3f::Zero(); // acce bias
    Quaternionf quat = Quaternionf::Identity(); // current quaternion
};

extern struct imuData g_imu;
extern sem_t ctrl_rdy;

int imu_task(int argc, char *argv[]);
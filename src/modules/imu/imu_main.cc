#include "bmi08_port.h"
#include "bmi08x.h"
#include <cstdio>
#include <sys/time.h>
#include <mqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include "filter/ConsistentOrientationFilter.hpp"
#include "dshot.hpp"
#include <syslog.h>
static float acc_x, acc_y, acc_z;
static float gyro_x, gyro_y, gyro_z;

#define GRAVITY_EARTH (9.80665f)
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

/*********************************************************************/
/*                        Global variables                           */
/*********************************************************************/
/*! @brief This structure containing relevant bmi08 info */
struct bmi08_dev bmi08dev;

/*! @brief variable to hold the bmi08 accel data */
struct bmi08_sensor_data bmi08_accel;

/*! @brief variable to hold the bmi08 gyro data */
struct bmi08_sensor_data bmi08_gyro;

/*! bmi08 accel int config */
struct bmi08_accel_int_channel_cfg accel_int_config;

/*! bmi08 gyro int config */
struct bmi08_gyro_int_channel_cfg gyro_int_config;

static int8_t init_bmi08(void)
{
    int8_t rslt;

    rslt = bmi08a_init(&bmi08dev);

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_init(&bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_400_HZ;

        if (bmi08dev.variant == BMI085_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
        }
        else if (bmi08dev.variant == BMI088_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        }

        bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
        bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;    /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&bmi08dev);

        rslt = bmi08xa_set_meas_conf(&bmi08dev);

        bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_47_ODR_400_HZ;
        bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
        bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_47_ODR_400_HZ;
        bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08dev);

        rslt = bmi08g_set_meas_conf(&bmi08dev);
    }

    return rslt;
}

static int8_t enable_bmi08_interrupt()
{
    int8_t rslt;
    uint8_t data = 0;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg *)&accel_int_config, &bmi08dev);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

        /* Enable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);

        rslt = bmi08g_get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &bmi08dev);
    }

    return rslt;
}

static int8_t disable_bmi08_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    /* Disable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg *)&accel_int_config, &bmi08dev);
    // bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

        /* Disable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);
        // bmi08_error_codes_print_result("bmi08g_set_int_config", rslt);
    }

    return rslt;
}


struct MotorCmd
{
    uint64_t time;
    float u1;
    float u2;
    float u3;
    float u4;
};

DShot::DShot<4> dshotDrv;
extern "C"
{
    int imu_main(int argc, char **argv)
    {
        int seconds = 10;
        float k_value = 10;
        float i_value = 10;
        for (int i = 1; i < argc; ++i)
        {
            if (strcmp(argv[i], "-t") == 0 && i + 1 < argc)
            {
                seconds = atoi(argv[i + 1]);
                i++;
            }
            else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc)
            {
                i_value = atof(argv[i + 1]);
                i++;
            }
            else if (strcmp(argv[i], "-k") == 0 && i + 1 < argc)
            {
                k_value = atof(argv[i + 1]);
                i++; 

                dshotDrv.init();
                dshotDrv.register_motor_channel_map(4, 2, 1, 3);
                dshotDrv.start();
                sleep(4);
                dshotDrv.set_motor_throttle(0.1, 0.0, 0.0, 0.0);
                sleep(1);
                dshotDrv.set_motor_throttle(0.1, 0.1, 0.0, 0.0);
                sleep(1);
                dshotDrv.set_motor_throttle(0.1, 0.1, 0.1, 0.0);
                sleep(1);
                dshotDrv.set_motor_throttle(0.1, 0.1, 0.1, 0.1);
                sleep(1);
                dshotDrv.set_motor_throttle(0.0, 0.0, 0.0, 0.0);
                
            }
            else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc)
            {
                dshotDrv.init();

                dshotDrv.register_motor_channel_map(4, 2, 1, 3);
                dshotDrv.start();
                sleep(3);
                dshotDrv.set_motor_throttle(0.1, 0.1, 0.1, 0.1);
                sleep(1);
                dshotDrv.set_motor_throttle(0.0, 0.0, 0.0, 0.0);
                dshotDrv.deinit();
                printf("init dshot \n");
                return 0;
                
            }
        }
        if (seconds < 1)
        {
            printf("Invalid time value. Using default value of 10 seconds.\n");
            seconds = 10;
        }
        printf("k set to %f \n", k_value);
        printf("i set to %f \n", i_value);
        const float gyro_noise = 1e-5;
        const float gyro_random_walk = 1e-5;
        double gravity_noise = 0.01f;
        using namespace EmbeddedMath;
        Filter::ConsistentOrientation::ConsistentOrientationFilter consistent_filter{};
        // consistent_filter.setImuParam(Vector3f(gyro_noise, gyro_noise, gyro_noise), 400.0f);
        // consistent_filter.setMeasurementParam(Vector3f(gravity_noise, gravity_noise, gravity_noise));
        // consistent_filter.setInitialState(Quaternionf::Identity());
        Vector3f gyro_bias = Vector3f::Zero();
        int calibation_count = 400;
        // sqrt(1.0f);
        int8_t rslt;
        uint32_t times_to_read = 0;
        float x = 0.0, y = 0.0, z = 0.0;
        uint8_t status = 0;

        struct timeval t;
        struct mq_attr attr;

        attr.mq_maxmsg = 20;
        attr.mq_msgsize = sizeof(MotorCmd);
        attr.mq_flags = 0;
        mqd_t motor_mq;
        motor_mq = mq_open("motor", O_WRONLY | O_CREAT | O_NONBLOCK, 0666, &attr);
        if (motor_mq == (mqd_t)-1)
        {
            printf("sender_thread: ERROR mq_open failed, errno=%d\n", errno);
            return -1;
        }
        uint64_t currentTime;
        // Parameters
        const float b = 2.56379;
        const float c = 2.12746;
        const float mass = 0.242;
        const float g = 9.8;
        const float arm_length = 0.05;
        const float torque_ratio = 1.0;
        Vector3f Centroid = Vector3f(0.004462809917, 0.0, 0.03087603306);
        Matrix3f J = Matrix3f::Zero();
        J(0,0) = 0.00063507735;
        J(1,1) = 0.00076497735;
        J(2,2) = 0.0003916547;
        J(2,0) = 0.0000044;
        J(0,2) = 0.0000044;
        Matrix4f Mixer;
        Mixer(0,0) = 0.35355339059 / arm_length;
        Mixer(1,0) = -0.35355339059 / arm_length;
        Mixer(2,0) = -0.35355339059 / arm_length;
        Mixer(3,0) = 0.35355339059 / arm_length;

        Mixer(0,1) = 0.35355339059 / arm_length;
        Mixer(1,1) = 0.35355339059 / arm_length;
        Mixer(2,1) = -0.35355339059 / arm_length;
        Mixer(3,1) = -0.35355339059 / arm_length;

        Mixer(0,2) = -1/(torque_ratio * 4);
        Mixer(1,2) = 1/(torque_ratio * 4);
        Mixer(2,2) = -1/(torque_ratio * 4);
        Mixer(3,2) = 1/(torque_ratio * 4);

        Mixer(0,3) = 0.25;
        Mixer(1,3) = 0.25;
        Mixer(2,3) = 0.25;
        Mixer(3,3) = 0.25;

        Vector3f accum_omega = Vector3f::Zero();
        Quaternionf Rot = Quaternionf::Identity();
        rslt = bmi08_interface_init(&bmi08dev, BMI08_SPI_INTF, BMI088_VARIANT);
        printf("bmi interface init mark %d \n", rslt);
        if (rslt == BMI08_OK)
        {
            rslt = init_bmi08();
            printf("init bmi088 %d \n", rslt);
            if (rslt < 0)
                return -1;

            rslt = enable_bmi08_interrupt();

            if (rslt == BMI08_OK)
            {
                for (;;)
                {
                    uint8_t last_acce_flag = 0;
                    uint8_t last_gyro_flag = 0;
                    /* Read accel data ready interrupt status */
                    rslt = bmi08a_get_data_int_status(&status, &bmi08dev);

                    if ((status & BMI08_ACCEL_DATA_READY_INT) && (last_acce_flag == 0))
                    {
                        gettimeofday(&t, 0);
                        
                        rslt = bmi08a_get_data(&bmi08_accel, &bmi08dev);

                        acc_x = static_cast<float>(bmi08_accel.x) / 1365.0f * GRAVITY_EARTH;
                        acc_y = static_cast<float>(bmi08_accel.y) / 1365.0f * GRAVITY_EARTH;
                        acc_z = static_cast<float>(bmi08_accel.z) / 1365.0f * GRAVITY_EARTH;

                        rslt = bmi08g_get_data(&bmi08_gyro, &bmi08dev);
                        gyro_x = static_cast<float>(bmi08_gyro.x) / 16.384f * DEG_TO_RAD;
                        gyro_y = static_cast<float>(bmi08_gyro.y) / 16.384f * DEG_TO_RAD;
                        gyro_z = static_cast<float>(bmi08_gyro.z) / 16.384f * DEG_TO_RAD;

                        double current_time = static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_usec) / 1000000.0;
                        if (times_to_read < calibation_count)
                        {
                            gyro_bias += Vector3f(gyro_x, gyro_y, gyro_z);
                        }
                        else
                        {
                            if (times_to_read == calibation_count)
                            {
                                gyro_bias /= static_cast<float>(calibation_count);
                                printf("gyro bias %f %f %f \n", gyro_bias.x(), gyro_bias.y(), gyro_bias.z());
                            }

                            // Vector4f input(0.0025, gyro_x - gyro_bias.x(), gyro_y - gyro_bias.y(), gyro_z - gyro_bias.z());
                            // auto consistent_prediction = consistent_filter.predict(input);
                            // Vector3f acce(acc_x, acc_y, acc_z);

                            // imu placement
                            Vector3f gyro(-gyro_y + gyro_bias.y() , gyro_x - gyro_bias.x(), -gyro_z + gyro_bias.z());

                            Rot = Rot * EmbeddedLie::quat_Exp(gyro*0.0025);
                            
                            
                            Quaternionf diffRot = Rot.inverse();
                            Vector3f e_omega;
                            if(diffRot.w()>=0)
                            {
                                e_omega = 2 * diffRot.head<3>();
                            }
                            else
                            {
                                e_omega = -2 * diffRot.head<3>();
                            }
                            Vector3f gravity_global = Vector3f(0.0, 0.0, -g) * mass;
                            Vector3f gravity_local = Rot.toRotationMatrix() * gravity_global;
                            Vector3f torque_gravity =  Centroid.cross(gravity_local);

                            Vector3f tmp = i_value * J * (e_omega - gyro);

                            Vector3f torque_des = k_value * J * e_omega + tmp + gyro.cross(J * gyro) + torque_gravity;

                            Vector4f full_des = Vector4f(torque_des.x(), torque_des.y(), torque_des.z(), 3.5);

                            Vector4f force_des = Mixer * full_des;
                            currentTime = gethrtime();
                            // syslog(LOG_INFO,"----------%llu --------- \n", currentTime);
                            // syslog(LOG_INFO,"tmp        | %f %f %f  \n", tmp.x(), tmp.y(), tmp.z());
                            // syslog(LOG_INFO,"raw gyro   | %f %f %f  \n", gyro.x(), gyro.y(), gyro.z());
                            // syslog(LOG_INFO,"quaternion | %f %f %f %f \n", Rot.w(), Rot.x(), Rot.y(), Rot.z());
                            // syslog(LOG_INFO,"e_omega    | %f %f %f \n", e_omega.x(), e_omega.y(), e_omega.z());
                            // syslog(LOG_INFO,"acc_omega  | %f %f %f \n", accum_omega.x(), accum_omega.y(), accum_omega.z());
                            // syslog(LOG_INFO,"tor_grav   | %f %f %f \n", torque_gravity.x(), torque_gravity.y(), torque_gravity.z());
                            // syslog(LOG_INFO,"force_des  | %f %f %f %f\n", force_des(0), force_des(1), force_des(2), force_des(3));
                            printf("tmp        | %f %f %f  \n", tmp.x(), tmp.y(), tmp.z());
                            printf("raw gyro   | %f %f %f  \n", gyro.x(), gyro.y(), gyro.z());
                            printf("quaternion | %f %f %f %f \n", Rot.w(), Rot.x(), Rot.y(), Rot.z());
                            printf("e_omega    | %f %f %f \n", e_omega.x(), e_omega.y(), e_omega.z());
                            printf("acc_omega  | %f %f %f \n", accum_omega.x(), accum_omega.y(), accum_omega.z());
                            printf("tor_grav   | %f %f %f \n", torque_gravity.x(), torque_gravity.y(), torque_gravity.z());
                            printf("torque_des | %f %f %f \n", torque_des.x(), torque_des.y(), torque_des.z());
                            printf("force_des  | %f %f %f %f\n", force_des(0), force_des(1), force_des(2), force_des(3));
                            struct MotorCmd cmd;
                            if(force_des(0)<=0)
                            {
                                cmd.u1 = 0.05;
                            }
                            else
                            {
                                cmd.u1 = (-b + sqrt(b*b+4*c*force_des(0)))/(2*c);
                            }

                            if(force_des(1)<=0)
                            {
                                cmd.u2 = 0.05;
                            }
                            else
                            {
                                cmd.u2 = (-b + sqrt(b*b+4*c*force_des(1)))/(2*c);
                            }

                            if(force_des(2)<=0)
                            {
                                cmd.u3 = 0.05;
                            }
                            else
                            {
                                cmd.u3 = (-b + sqrt(b*b+4*c*force_des(2)))/(2*c);
                            }

                            if(force_des(3)<=0)
                            {
                                cmd.u4 = 0.05;
                            }
                            else
                            {
                                cmd.u4 = (-b + sqrt(b*b+4*c*force_des(3)))/(2*c);
                            }
                            
                            cmd.time = currentTime;
                            dshotDrv.set_motor_throttle(cmd.u1, cmd.u2, cmd.u3, cmd.u4);
                            printf("motor_des  | %f %f %f %f\n", cmd.u1, cmd.u2, cmd.u3, cmd.u4);
                            // syslog(LOG_INFO,"motor_des  | %f %f %f %f\n", cmd.u1, cmd.u2, cmd.u3, cmd.u4);
                            // mq_send(motor_mq, reinterpret_cast<const char *>(&cmd), sizeof(MotorCmd), 42);
                            // consistent_filter.update(acce.normalized());
                            // printf("current time %f \n acce %4.2f %4.2f %4.2f\n gyro %4.2f %4.2f %4.2f\n", current_time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
                            // printf("%f %f %f %f \n", consistent_prediction.w(), consistent_prediction.x(), consistent_prediction.y(), consistent_prediction.z());
                            
                        }
                        times_to_read++;
                    }
                    last_acce_flag = status & BMI08_ACCEL_DATA_READY_INT;
                    // last_gyro_flag = status & BMI08_GYRO_DATA_READY_INT;

                    if (times_to_read > seconds * 400)
                    {
                        struct MotorCmd cmd = {currentTime, 0.0, 0.0, 0.0, 0.0};
                        printf("motor_des  | %f %f %f %f\n", cmd.u1, cmd.u2, cmd.u3, cmd.u4);
                        dshotDrv.set_motor_throttle(0.0, 0.0, 0.0, 0.0); 
                        // mq_send(motor_mq, reinterpret_cast<const char *>(&cmd), sizeof(MotorCmd), 42);
                        break;
                    }

                }
                rslt = disable_bmi08_interrupt();
            }

            return 1;
        }
    }
}
#include "common.h"
#include <stdio.h>
#include <nuttx/arch.h>
#include <SharedTable.h>
#include <ll_types/acro.h>
#include <dshot.h>

const float mass = 0.185;
const float g = 9.8;
const float arm_length = 0.062;
const float torque_ratio = 0.04;

float K_att = 10;
float K_omega = 45;



const static float a = 1.2180;
const static float b = 0.7700;
const static float c = -0.1002;

float force2throttle(const float in)
{
    if(in <= 0)
        return 0.2;


    float out = (-b + sqrt(b*b - 4 * a * (c-in)))/(2*a);
    if(out < 0.8)
        return out;
    else
    {
        printf("output to big %f\n", out);
        return 0.2;
    }
}

float throttle2force(const float in)
{
    return a*in*in + b*in + c;
}

uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

#pragma pack(push, 1)
struct log_msg{
    uint16_t header;
    int cnt;
    float omega[3];
    float motor_cmd[4];
    uint32_t motor_fb[4];
    uint8_t crc;
};
#pragma pack(pop)

int control_task(int argc, char *argv[])
{
    int acro_fd = st_find("acro_cmd");
    int log_fd = st_find("log_message");
    struct AcroCmd rc_cmd;
    int last_rc_id;
    int valid_rc_count = 0;
    uint8_t rc_valid = 0;


    static int cnt = 0;
    
    DShot::DShot mDshot{};
    mDshot.register_motor_channel_map(1, 2, 3, 4);

    Matrix3f J = Matrix3f::Zero();
    J(0,0) = 0.0002407245045;
    J(1,1) = 0.0003232245045;
    J(2,2) = 0.0003523156757;
    J(2,0) = 0.0000402;
    J(0,2) = 0.0000402;

    Matrix4f Mixer;
    Mixer(0,0) = -0.35355339059 / arm_length;
    Mixer(1,0) = 0.35355339059 / arm_length;
    Mixer(2,0) = 0.35355339059 / arm_length;
    Mixer(3,0) = -0.35355339059 / arm_length;

    Mixer(0,1) = -0.35355339059 / arm_length;
    Mixer(1,1) = -0.35355339059 / arm_length;
    Mixer(2,1) = 0.35355339059 / arm_length;
    Mixer(3,1) = 0.35355339059 / arm_length;

    Mixer(0,2) = -1/(torque_ratio * 4);
    Mixer(1,2) = 1/(torque_ratio * 4);
    Mixer(2,2) = -1/(torque_ratio * 4);
    Mixer(3,2) = 1/(torque_ratio * 4);

    Mixer(0,3) = 0.25;
    Mixer(1,3) = 0.25;
    Mixer(2,3) = 0.25;
    Mixer(3,3) = 0.25;

    const Vector3f x_axis = Vector3f(1, 0, 0);
    const Vector3f y_axis = Vector3f(0, 1, 0);
    const Vector3f z_axis = Vector3f(0, 0, 1);

    struct log_msg mlog;
    mlog.header = 0xABEF;
    reset = 1;
    while(1)
    {   
        sem_wait(&ctrl_rdy);
        st_read(acro_fd, &rc_cmd);

        // Update last_rc_id
        last_rc_id = rc_cmd.id;

        cnt++;
        // Vector3f omega_des = Vector3f(rc_cmd.roll_vel, rc_cmd.pitch_vel, rc_cmd.yaw_vel);
        // Quaternionf quat_target = Quaternionf::Identity();

        Vector3f omega_des = Vector3f::Zero();
        auto quat_z = AngleAxisf(rc_cmd.yaw_vel, z_axis);
        auto quat_y = AngleAxisf(rc_cmd.pitch_vel, y_axis);
        auto quat_x = AngleAxisf(rc_cmd.roll_vel, x_axis);
        Quaternionf quat_target = quat_z * quat_y * quat_x;
        Quaternionf error_quat = g_imu.quat.inverse() * quat_target;
        
        Vector3f e_omega;
        if(error_quat.w()>=0)
            e_omega = 2 * error_quat.head<3>();
        else  
            e_omega = -2 * error_quat.head<3>();

        Vector3f torque_des = K_omega * J * (e_omega + omega_des - g_imu.angular_vel ) + g_imu.angular_vel.cross(J * g_imu.angular_vel);
        
        float collab_thrust = 4 * throttle2force(rc_cmd.throttle);

        Vector4f full_des = Vector4f(torque_des.x(), torque_des.y(), torque_des.z(), collab_thrust);

        Vector4f force_des = Mixer * full_des;

        Vector4f motor_cmd = Vector4f(force2throttle(force_des(0)),
                                        force2throttle(force_des(1)),
                                        force2throttle(force_des(2)),
                                        force2throttle(force_des(3)));
        // printf("e_omega    | %f %f %f \n", e_omega.x(), e_omega.y(), e_omega.z());
        // printf("raw        | %f %f %f \n", g_imu.angular_vel.x(), g_imu.angular_vel.y(), g_imu.angular_vel.z());
        // printf("d_omega    | %f %f %f \n", g_imu.angular_acce.x(), g_imu.angular_acce.y(), g_imu.angular_acce.z());
        // printf("full_des   | %f %f %f %f\n", full_des(0), full_des(1), full_des(2), full_des(3));
        // printf("force_des  | %f %f %f %f\n", force_des(0), force_des(1), force_des(2), force_des(3));

        if((rc_cmd.status & 1) == 0)
        {
            motor_cmd = Vector4f::Zero();
        }
        else
        {
            
        }
        
        mDshot.set_motor_throttle(motor_cmd(0), motor_cmd(1), motor_cmd(2)*1.15, motor_cmd(3));

                
        if((rc_cmd.status & 2) != 0 && cnt % 4 ==0)
        {
            auto rpms = mDshot.get_motor_rpms();
            mlog.cnt = cnt;
            mlog.omega[0] = g_imu.angular_vel(0);
            mlog.omega[1] = g_imu.angular_vel(1);
            mlog.omega[2] = g_imu.angular_vel(2);
            mlog.motor_cmd[0] = motor_cmd(0);
            mlog.motor_cmd[1] = motor_cmd(1);
            mlog.motor_cmd[2] = motor_cmd(2);
            mlog.motor_cmd[3] = motor_cmd(3);
            mlog.motor_fb[0] = rpms[0];
            mlog.motor_fb[1] = rpms[1];
            mlog.motor_fb[2] = rpms[2];
            mlog.motor_fb[3] = rpms[3];
            mlog.crc = crc8((uint8_t *)(&mlog), 50);
            st_write_block(log_fd, &mlog, sizeof(log_msg));
        }

        if(cnt %4 ==0)
        {
            
            // printf("\r\033[K[OK] ID:%-6u | ST:%d | THR:%4.2f | Y:%5.2f P:%5.2f R:%5.2f | force %3.4f %3.4f %3.4f %3.4f", 
            //        rc_cmd.id, rc_cmd.status, rc_cmd.throttle, 
            //        rc_cmd.yaw_vel, rc_cmd.pitch_vel, rc_cmd.roll_vel,
            //        motor_cmd(0), motor_cmd(1), motor_cmd(2), motor_cmd(3));
            // fflush(stdout);
        }
           

    }
    return 0;
}

extern "C"
{
    int control_loop_main(int argc, FAR char *argv[])
    {
        
        task_create("imu_sample_task", 150, 40960, imu_task, NULL);
        // imu_task(1, NULL);
        task_create("control_task", 200, 40960, control_task, NULL);
        return 0;
    }
}
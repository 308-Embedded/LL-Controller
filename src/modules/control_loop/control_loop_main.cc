#include "common.h"
#include <stdio.h>
#include <nuttx/arch.h>
#include <SharedTable.h>
#include <ll_types/acro.h>
#include <dshot.h>

int control_task(int argc, char *argv[])
{
    int acro_fd = st_find("acro_cmd");
    struct AcroCmd rc_cmd;
    DShot::DShot mDshot{};
    mDshot.register_motor_channel_map(1, 2, 3, 4);
    mDshot.set_motor_throttle(0,0,0,0);
    while(1)
    {   
        sem_wait(&ctrl_rdy);
        st_read(acro_fd, &rc_cmd);
        mDshot.set_motor_throttle(rc_cmd.throttle, rc_cmd.throttle, rc_cmd.throttle, rc_cmd.throttle);
        printf("\r\033[K[OK] ID:%-6u | ST:%d | THR:%4.2f | Y:%5.2f P:%5.2f R:%5.2f", 
                   rc_cmd.id, rc_cmd.status, rc_cmd.throttle, 
                   rc_cmd.yaw_vel, rc_cmd.pitch_vel, rc_cmd.roll_vel);
        fflush(stdout);

    }
    return 0;
}

extern "C"
{
    int control_loop_main(int argc, FAR char *argv[])
    {
        
        task_create("imu_sample_task", 200, 40960, imu_task, NULL);
        // imu_task(1, NULL);
        task_create("control_task", 150, 40960, control_task, NULL);
        return 0;
    }
}
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>
#include <mqueue.h>
#include <nuttx/timers/timer.h>
#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <sys/time.h>
// #include "stm32_gpio.h"

#include "dshot.hpp"

struct MotorCmd
{
    uint64_t time;
    float u1;
    float u2;
    float u3;
    float u4;
};

uint8_t running = 0;

int motor_task(int argc, char *argv[])
{
    struct mq_attr attr;
    struct MotorCmd cmd;
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

    DShot::DShot<4> dshotDrv;
    dshotDrv.init();

    dshotDrv.register_motor_channel_map(4, 2, 1, 3);
    dshotDrv.start();
    sleep(3);
    dshotDrv.set_motor_throttle(0.1, 0.1, 0.1, 0.1);
    sleep(1);
    dshotDrv.set_motor_throttle(0.0, 0.0, 0.0, 0.0);
    dshotDrv.deinit();
    // need to init twice for some reason
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
    printf("motor task created \n");
    while(running)
    {
        int state = mq_receive(motor_mq, reinterpret_cast<char *>(&cmd), sizeof(MotorCmd), NULL);
        if(state > 0)
        {
            dshotDrv.set_motor_throttle(cmd.u1, cmd.u2, cmd.u3, cmd.u4);
            uint64_t ref_time = gethrtime();
            printf("%llu get motor command %llu %f %f %f %f\n",ref_time, cmd.time, cmd.u1, cmd.u2, cmd.u3, cmd.u4); 
            fflush(stdout);
        }

        usleep(10);
        
    }

    dshotDrv.set_motor_throttle(0.0, 0.0, 0.0, 0.0);
    dshotDrv.deinit();
    mq_close(motor_mq);
    printf("motor task quit \n");
    return 0;
}

extern "C"
{
    int dshot_main(int argc, FAR char *argv[])
    {
        running = 1;
        int pid = task_create("motor_task", 42, 4096, motor_task, NULL);
        return 0;
    }
}
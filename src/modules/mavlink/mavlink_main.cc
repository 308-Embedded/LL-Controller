#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include "./include/LL-message/mavlink.h"

#define SERIAL_PORT "/dev/ttyS0"
#define BAUDRATE B115200

#define SYSTEM_ID 2
#define COMPONENT_ID MAV_COMP_ID_AUTOPILOT1 // 飞控组件ID
#define TYPE 0
#define AUTOPILOT 0

#define HEARTBEAT_INTERVAL 1.0
#define TIMEOUT_TIME 3.0
#define MAX_MSG_LEN 300

// 打开并配置串口
int open_serial(const char *port, speed_t baudrate)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1)
    {
        perror("无法打开串口");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("获取串口属性失败");
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag &= ~PARENB; // 无校验
    tty.c_cflag &= ~CSTOPB; // 1位停止位
    tty.c_cflag &= ~CSIZE;  // 清除数据位设置
    tty.c_cflag |= CS8;     // 8位数据位

    tty.c_cflag |= CREAD | CLOCAL;                  // 启用接收，忽略调制解调器控制线
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // 禁用软件流控
    tty.c_oflag &= ~OPOST;                          // 原始输出

    // 设置读取超时
    tty.c_cc[VTIME] = 10; // 读取超时（10*0.1秒）
    tty.c_cc[VMIN] = 0;   // 最小读取字节数

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("设置串口属性失败");
        close(fd);
        return -1;
    }

    return fd;
}

extern "C"
{
    int mavlink_main(int argc, char **argv)
    {
        printf("MAVLink_uva 测试程序启动\n");

        int serial_fd = open_serial(SERIAL_PORT, BAUDRATE);
        if (serial_fd == -1)
        {
            return 1;
        }
        printf("串口已打开!\n");

        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t send_buf[MAX_MSG_LEN];

        time_t last_recv_heartbeat = 0;
        time_t last_send_heartbeat = 0;
        time_t last_send_attitude = 0;

        memset(&msg, 0, sizeof(msg));
        memset(&status, 0, sizeof(status));

        while (1)
        {
            time_t current_time = time(NULL);
            if (difftime(current_time, last_send_heartbeat) >= HEARTBEAT_INTERVAL)
            {
                mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, TYPE, AUTOPILOT, 0, 0, MAV_STATE_ACTIVE);
                uint16_t len = mavlink_msg_to_send_buffer(send_buf, &msg);
                write(serial_fd, send_buf, len);
                last_send_heartbeat = current_time;
            }

            uint8_t c;
            ssize_t n = read(serial_fd, &c, 1);
            if (n > 0)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
                {
                    // 根据消息ID判断类型
                    switch (msg.msgid)
                    {
                    // 心跳包处理
                    case MAVLINK_MSG_ID_HEARTBEAT:
                    {
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                        last_recv_heartbeat = current_time;
                        printf("[心跳消息] 时间：%ld, 系统状态: %d \n", current_time, heartbeat.system_status);
                        break;
                    }
                    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
                    {
                        mavlink_set_attitude_target_t attitude_target;
                        mavlink_msg_set_attitude_target_decode(&msg, &attitude_target);
                        printf("[控制姿态] 目标四元数: [%.2f, %.2f, %.2f, %.2f], 推力: %.2f\n",
                               attitude_target.q[0],
                               attitude_target.q[1],
                               attitude_target.q[2],
                               attitude_target.q[3],
                               attitude_target.thrust);
                        break;
                    }
                    case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
                    {
                        mavlink_set_actuator_control_target_t actuator_control;
                        mavlink_msg_set_actuator_control_target_decode(&msg, &actuator_control);
                        printf("[控制电机] 目标时间: %llu, 控制值: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
                               actuator_control.time_usec,
                               actuator_control.controls[0],
                               actuator_control.controls[1],
                               actuator_control.controls[2],
                               actuator_control.controls[3],
                               actuator_control.controls[4],
                               actuator_control.controls[5],
                               actuator_control.controls[6],
                               actuator_control.controls[7]);
                        break;
                    }
                    default:
                        printf("[未知消息] ID: %d\n", msg.msgid);
                        break;
                    }
                }
            }
            // 检查心跳超时
            if (last_recv_heartbeat > 0 && difftime(current_time, last_recv_heartbeat) > TIMEOUT_TIME)
            {
                printf(" Lost connection to GCS!!! \n");
                last_recv_heartbeat = 0; // 防止重复输出
            }
            usleep(1000);
        }
        close(serial_fd);
        return 0;
    }
}
#include <nuttx/config.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <mqueue.h>
#include <nuttx/clock.h>
#include <stdlib.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include "static_queue.hpp"
#include <SharedTable.h>
#include <ll_types/acro.h>

struct remoteCmd
{
    float throttle;
    float q_w;
    float q_x;
    float q_y;
    float q_z;
};

#pragma pack(push, 1)
struct serial_packet {
    uint16_t magic;      // Target: 0xE0FD (Memory: FD E0)
    uint16_t status;
    uint32_t sec;
    uint32_t nsec;
    uint32_t id;
    float throttle;
    float yaw_speed;
    float pitch_speed;
    float roll_speed;
    uint8_t  crc;
};
#pragma pack(pop)

uint8_t calculate_crc8(const uint8_t *data, size_t len) {
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

static int setup_serial()
{
    int fd;
    fd = open("/dev/ttyS0", O_RDWR);

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 1; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 921600
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }
    return fd;
}

int serial_task(int argc, char *argv[])
{
   int fd = setup_serial();


    struct mq_attr attr;

    attr.mq_maxmsg = 20;
    attr.mq_msgsize = sizeof(remoteCmd);
    attr.mq_flags = 0;
    mqd_t g_send_mqfd;
    g_send_mqfd = mq_open("mqueue", O_WRONLY | O_CREAT, 0666, &attr);
    if (g_send_mqfd == (mqd_t)-1)
    {
        printf("sender_thread: ERROR mq_open failed, errno=%d\n", errno);
        return -1;
    }

    printf("creating circular buffer \n");
    StaticQueue<uint8_t, 100> msgBuffer{};
    remoteCmd cmd;
    int cnt = 0;
    uint8_t read_buf[25];
    uint8_t msg[25];
    if (fd < 0)
    {
        fprintf(stderr, "ERROR: open failed: %d\n", errno);
        return -1;
    }
    int byte_cnt = 0;
    while (1)
    {
        ssize_t read_count = read(fd, read_buf, 25);
        if (read_count > 0)
        {
            for (size_t i = 0; i < read_count; i++)
            {
                msgBuffer.push(read_buf[i]);
            }
            while (msgBuffer.front() != 0x0F && !msgBuffer.empty())
            {
                msgBuffer.pop();
            }
            if (msgBuffer.size() >= 25)
            {
                // printf("front data %02x \n", msgBuffer.front());

                for (size_t i = 0; i < 25; i++)
                {
                    msg[i] = msgBuffer.front();
                    msgBuffer.pop();
                }

                if (msg[24] == 0xEE)
                {
                    uint32_t tmp = msg[4] << 24 | msg[3] << 16 | msg[2] << 8 | msg[1];
                    memcpy(&(cmd.throttle), &tmp, 4);
                    mq_send(g_send_mqfd, reinterpret_cast<const char *>(&cmd), sizeof(remoteCmd), 42);
                }
            }
        }

        usleep(10000); // 10 ms
        cnt++;
    }
    mq_close(g_send_mqfd);
    close(fd);
    printf("closing serial port ...\n");
    usleep(10000);
    return 1;
}

int receive_task(int argc, char **argv)
{
    st_initialize();
    int fd = setup_serial();
    int acro_fd = st_find("acro_cmd");
    struct serial_packet pkt;
    uint8_t *window = (uint8_t*)&pkt;
    const size_t PKT_SIZE = sizeof(struct serial_packet);

    struct AcroCmd acro_pkg;

    printf("Receiver Active. Synchronizing on 0xE0FD...\n");

    while (1) {
        // 1. Shift existing data left by 1 byte
        memmove(window, window + 1, PKT_SIZE - 1);

        // 2. Read 1 new byte into the very last position
        if (read(fd, &window[PKT_SIZE - 1], 1) <= 0) continue;

        // 3. Check for the Magic Header [FD E0] at the start of our window
        if (window[0] == 0xFD && window[1] == 0xE0) {
            
            // 4. Validate the CRC (calculated on all bytes except the last one)
            uint8_t computed_crc = calculate_crc8(window, PKT_SIZE - 1);
            
            if (computed_crc == pkt.crc) {
                // SUCCESS: Frame synchronized and data validated
                // printf("\r\033[K[OK] ID:%-6u | ST:%d | THR:%4.2f | Y:%5.2f P:%5.2f R:%5.2f", 
                //        pkt.id, pkt.status, pkt.throttle, 
                //        pkt.yaw_speed, pkt.pitch_speed, pkt.roll_speed);
                // fflush(stdout);

                //update shared table here
                acro_pkg.id = pkt.id;
                acro_pkg.status = pkt.status;
                acro_pkg.throttle = pkt.throttle;
                acro_pkg.yaw_vel = pkt.yaw_speed;
                acro_pkg.pitch_vel  = pkt.pitch_speed;
                acro_pkg.roll_vel = pkt.roll_speed;
                st_write(acro_fd, &acro_pkg);
                usleep(5000);
            }
        }
    }
    return 0;
}

extern "C"
{
    int serial_main(int argc, char **argv)
    {
        
        // task_create("remote_task", 50, 16384, serial_task, NULL);
        task_create("receive_task", 120, 4096, receive_task, NULL);
        // serial_task();
        // int fd = setup_serial();
        
        // static const char s[] = "abcdefghijklmnopqrstuvwxyz";
        // const int slength = sizeof(s)-1;
        // int printed = write(fd, s, slength);
        // printf("serial sent %d bytes \n", printed);
        // up_udelay(5000000);
        // close(fd);
        return 0;
    }
}
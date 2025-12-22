#include <imu_bmi088.h>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <dshot.h>
#include <time.h>

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

extern "C"
{
    int vibration_main(int argc, FAR char *argv[])
    {
        struct timespec ts_0, ts_1;
        DShot::DShot mDshot{};
        mDshot.register_motor_channel_map(1, 2, 3, 4);
        mDshot.set_motor_throttle(0,0,0,0);
        
        bmi088_initialize();
        int fd = setup_serial();
        uint64_t time;
        float data[6];
        int cnt =0;
        char buffer[128];
        int valid_data = 0;
        while(cnt < 30 * 400)
        {
            cnt++;
            bmi088_wait();
            if(bmi088_gyro_ready())
            {
                time = bmi088_gyro_read(data);
            }
            if(bmi088_acce_ready())
            {
                
                time = bmi088_acce_read(&(data[3]));
                valid_data++;
                auto rpms = mDshot.get_motor_rpms();
                int j = snprintf(buffer, 128, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f\n", data[0], data[1], data[2], data[3], data[4], data[5],\
                (float)rpms[0]*0.01, (float)rpms[1]*0.01, (float)rpms[2]*0.01, (float)rpms[3]*0.01);
                // int j = snprintf(buffer, 128, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", data[0], data[1], data[2], data[3], data[4], data[5],\
                // (float)rpms[0]*0.01, (float)rpms[1]*0.01, (float)rpms[2]*0.01, (float)rpms[3]*0.01);
                ssize_t send_count = write(fd, buffer, j);
                if(cnt <= 4000)
                {
                    mDshot.set_motor_throttle(0, 0, 0, 0);
                } 
                else if(cnt > 4000 && cnt <= 8000)
                {
                    mDshot.set_motor_throttle(0.2, 0.2, 0.2, 0.2);
                }
                else if(cnt > 8000 && cnt <= 12000)
                {
                    mDshot.set_motor_throttle(0.3, 0.3, 0.3, 0.3);
                }
                printf("cnt %d time %lld send %d\n",valid_data, time, j);
                
            }

        }
        mDshot.set_motor_throttle(0,0,0,0);
        close(fd);
        bmi088_deinitialize();
        return 0;
    }
}
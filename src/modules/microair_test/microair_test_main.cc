#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <dshot.h>
#include <imu_bmi088.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace
{
constexpr const char *kBluetoothCandidates[] = {
    "/dev/ttyS1",
    "/dev/ttyS0",
    "/dev/ttyS7",
    "/dev/ttyS2",
};

constexpr int kDefaultBluetoothBaud = 115200;
constexpr int kDefaultBluetoothSeconds = 15;
constexpr int kDefaultImuSamples = 200;

bool path_exists(const char *path)
{
    return access(path, F_OK) == 0;
}

void print_usage()
{
    printf("usage:\n");
    printf("  microair_test status\n");
    printf("  microair_test imu [samples]\n");
    printf("  microair_test bluetooth [device] [baud] [seconds]\n");
    printf("  microair_test demo [seconds]\n");
    printf("  microair_test motor [-t throttle]\n");
    printf("\n");
    printf("examples:\n");
    printf("  microair_test status\n");
    printf("  microair_test imu 100\n");
    printf("  microair_test bluetooth /dev/ttyS0 115200 30\n");
    printf("  microair_test demo 20\n");
    printf("  microair_test motor -t 0.10\n");
}

void print_board_status()
{
    printf("[microair-test] board: MicoAir743V2 / prototype_microair\n");
    printf("[microair-test] console: %s\n", path_exists("/dev/ttyACM0") ? "ttyACM0 present" : "ttyACM0 missing");

    for (const char *path : kBluetoothCandidates)
    {
        printf("[microair-test] serial %-10s %s\n", path, path_exists(path) ? "present" : "missing");
    }

    printf("[microair-test] test entrypoints:\n");
    printf("[microair-test]   status     : show visible device nodes and board summary\n");
    printf("[microair-test]   imu        : sample BMI088 gyro/accel data\n");
    printf("[microair-test]   bluetooth  : heartbeat + echo over onboard bluetooth uart\n");
    printf("[microair-test]   motor      : run low-throttle DShot motor sequence\n");
    printf("[microair-test]   demo       : status + IMU + bluetooth in one flow\n");
}

int parse_int_or_default(const char *text, int fallback)
{
    if (text == nullptr)
    {
        return fallback;
    }

    int value = atoi(text);
    return value > 0 ? value : fallback;
}

float parse_motor_throttle(int argc, char *argv[])
{
    float throttle = 0.12f;

    for (int i = 2; i < argc; ++i)
    {
        if (strcmp(argv[i], "-t") == 0 && i + 1 < argc)
        {
            throttle = static_cast<float>(atof(argv[i + 1]));
            ++i;
        }
    }

    if (throttle < 0.0f)
    {
        return 0.0f;
    }

    if (throttle > 0.25f)
    {
        return 0.25f;
    }

    return throttle;
}

void stop_motors(DShot::DShot &dshot)
{
    dshot.set_motor_throttle(0.0f, 0.0f, 0.0f, 0.0f);
}

void print_rpms(DShot::DShot &dshot)
{
    auto rpms = dshot.get_motor_rpms();
    printf("[microair-test] rpm %lu %lu %lu %lu\n",
           static_cast<unsigned long>(rpms[0]),
           static_cast<unsigned long>(rpms[1]),
           static_cast<unsigned long>(rpms[2]),
           static_cast<unsigned long>(rpms[3]));
}

int run_motor_sequence(float throttle)
{
    DShot::DShot dshot {};
    dshot.register_motor_channel_map(1, 2, 3, 4);
    stop_motors(dshot);

    printf("[microair-test] REMOVE PROPS before motor test.\n");
    printf("[microair-test] starting low-throttle motor sequence at %.3f\n",
           static_cast<double>(throttle));

    usleep(3000000);

    const float motor_cmds[4][4] = {
        {throttle, 0.0f,     0.0f,     0.0f},
        {0.0f,     throttle, 0.0f,     0.0f},
        {0.0f,     0.0f,     throttle, 0.0f},
        {0.0f,     0.0f,     0.0f,     throttle},
    };

    for (int motor = 0; motor < 4; ++motor)
    {
        printf("[microair-test] M%d on\n", motor + 1);

        for (int tick = 0; tick < 1000; ++tick)
        {
            dshot.set_motor_throttle(motor_cmds[motor][0],
                                     motor_cmds[motor][1],
                                     motor_cmds[motor][2],
                                     motor_cmds[motor][3]);

            if ((tick % 100) == 0)
            {
                print_rpms(dshot);
            }

            usleep(1000);
        }

        stop_motors(dshot);
        usleep(1000000);
    }

    printf("[microair-test] all motors low throttle\n");
    for (int tick = 0; tick < 1500; ++tick)
    {
        dshot.set_motor_throttle(throttle, throttle, throttle, throttle);

        if ((tick % 100) == 0)
        {
            print_rpms(dshot);
        }

        usleep(1000);
    }

    stop_motors(dshot);
    usleep(500000);
    stop_motors(dshot);
    printf("[microair-test] motor sequence done, motors stopped.\n");
    return 0;
}

int run_imu_test(int sample_count)
{
    float gyro[3] = {};
    float accel[3] = {};

    bmi088_initialize();
    printf("[microair-test] BMI088 test started.\n");

    for (int sample = 0; sample < sample_count; ++sample)
    {
        bmi088_gyro_read(gyro);
        bmi088_acce_read(accel);

        if ((sample % 10) == 0 || sample == sample_count - 1)
        {
            printf("[microair-test] imu[%03d/%03d] gyro(rad/s) %.4f %.4f %.4f | accel(m/s2) %.4f %.4f %.4f\n",
                   sample + 1,
                   sample_count,
                   static_cast<double>(gyro[0]),
                   static_cast<double>(gyro[1]),
                   static_cast<double>(gyro[2]),
                   static_cast<double>(accel[0]),
                   static_cast<double>(accel[1]),
                   static_cast<double>(accel[2]));
        }

        usleep(10000);
    }

    bmi088_deinitialize();
    printf("[microair-test] BMI088 test finished.\n");
    return 0;
}

speed_t baud_to_termios(int baud)
{
    switch (baud)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        return 0;
    }
}

int configure_serial(int fd, int baud)
{
    struct termios tty {};
    speed_t baud_rate = baud_to_termios(baud);

    if (baud_rate == 0)
    {
        printf("[microair-test] unsupported baud %d\n", baud);
        return -1;
    }

    if (tcgetattr(fd, &tty) != 0)
    {
        printf("[microair-test] tcgetattr failed: %d\n", errno);
        return -1;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("[microair-test] tcsetattr failed: %d\n", errno);
        return -1;
    }

    return 0;
}

int open_bluetooth_serial(const char *preferred, int baud, const char **selected_path)
{
    if (preferred != nullptr)
    {
        int fd = open(preferred, O_RDWR);
        if (fd >= 0 && configure_serial(fd, baud) == 0)
        {
            *selected_path = preferred;
            return fd;
        }

        if (fd >= 0)
        {
            close(fd);
        }

        return -1;
    }

    for (const char *path : kBluetoothCandidates)
    {
        int fd = open(path, O_RDWR);
        if (fd < 0)
        {
            continue;
        }

        if (configure_serial(fd, baud) == 0)
        {
            *selected_path = path;
            return fd;
        }

        close(fd);
    }

    return -1;
}

void print_rx_line(const uint8_t *buffer, ssize_t count)
{
    printf("[microair-test] bt rx %d bytes | hex:", static_cast<int>(count));
    for (ssize_t i = 0; i < count; ++i)
    {
        printf(" %02X", buffer[i]);
    }

    printf(" | ascii: ");
    for (ssize_t i = 0; i < count; ++i)
    {
        char ch = (buffer[i] >= 32 && buffer[i] <= 126) ? static_cast<char>(buffer[i]) : '.';
        putchar(ch);
    }

    printf("\n");
}

int run_bluetooth_test(const char *device, int baud, int seconds)
{
    const char *selected = nullptr;
    int fd = open_bluetooth_serial(device, baud, &selected);
    if (fd < 0)
    {
        printf("[microair-test] no usable bluetooth serial device found\n");
        return -1;
    }

    printf("[microair-test] bluetooth connected to %s @ %d\n", selected, baud);
    printf("[microair-test] bluetooth test running for %d second(s)\n", seconds);
    printf("[microair-test] send data from phone/peer to verify the link\n");

    uint8_t rxbuf[64];
    time_t start_time = time(nullptr);
    time_t last_heartbeat = 0;
    unsigned heartbeat_seq = 0;

    while ((time(nullptr) - start_time) < seconds)
    {
        ssize_t nread = read(fd, rxbuf, sizeof(rxbuf));
        if (nread > 0)
        {
            print_rx_line(rxbuf, nread);
            write(fd, rxbuf, nread);
        }

        time_t now = time(nullptr);
        if (now != last_heartbeat)
        {
            last_heartbeat = now;
            char heartbeat[96];
            int written = snprintf(heartbeat, sizeof(heartbeat),
                                   "MICOAIR_BT_READY seq=%u baud=%d\n",
                                   heartbeat_seq++, baud);
            if (written > 0)
            {
                write(fd, heartbeat, written);
            }
        }

        usleep(10000);
    }

    close(fd);
    printf("[microair-test] bluetooth test finished.\n");
    return 0;
}

int run_demo(int seconds)
{
    print_board_status();
    printf("[microair-test] demo step 1/2: IMU\n");
    run_imu_test(80);
    printf("[microair-test] demo step 2/2: bluetooth\n");
    return run_bluetooth_test(nullptr, kDefaultBluetoothBaud, seconds);
}
}

extern "C"
{
    int microair_test_main(int argc, char *argv[])
    {
        if (argc < 2)
        {
            print_usage();
            return 0;
        }

        if (strcmp(argv[1], "status") == 0)
        {
            print_board_status();
            return 0;
        }

        if (strcmp(argv[1], "imu") == 0)
        {
            int sample_count = (argc > 2) ? parse_int_or_default(argv[2], kDefaultImuSamples) : kDefaultImuSamples;
            return run_imu_test(sample_count);
        }

        if (strcmp(argv[1], "bluetooth") == 0)
        {
            const char *device = (argc > 2) ? argv[2] : nullptr;
            int baud = (argc > 3) ? parse_int_or_default(argv[3], kDefaultBluetoothBaud) : kDefaultBluetoothBaud;
            int seconds = (argc > 4) ? parse_int_or_default(argv[4], kDefaultBluetoothSeconds) : kDefaultBluetoothSeconds;
            return run_bluetooth_test(device, baud, seconds);
        }

        if (strcmp(argv[1], "demo") == 0)
        {
            int seconds = (argc > 2) ? parse_int_or_default(argv[2], kDefaultBluetoothSeconds) : kDefaultBluetoothSeconds;
            return run_demo(seconds);
        }

        if (strcmp(argv[1], "motor") == 0)
        {
            return run_motor_sequence(parse_motor_throttle(argc, argv));
        }

        print_usage();
        return -1;
    }
}

#pragma once

#include "../dshot_internal.h"

namespace DShot
{
#define DSHOT_CHANNELS 4
    class DShot
    {
    public:
        DShot() {init(); };
        ~DShot() { deinit(); }

        /// @brief initialize the dshot driver
        void init()
        {
            if (init_flag == 0)
            {
                bdshot_initialize();

                init_flag = 1;
            }
        }

        /// @brief deinitialize the dshot driver
        void deinit()
        {
            if (init_flag == 1)
            {
                bdshot_deinitialize();
                init_flag = 0;
            }
        }

        /// @brief register the corresponding dshot channel of each motor
        /// @param motor1 the corresponding dshot channel of motor1
        /// @param motor2 the corresponding dshot channel of motor2
        /// @param motor3 the corresponding dshot channel of motor3
        /// @param motor4 the corresponding dshot channel of motor4
        /// @return true if input values are valid
        bool register_motor_channel_map(const uint8_t motor1, const uint8_t motor2, const uint8_t motor3, const uint8_t motor4)
        {
            if (motor1 + motor2 + motor3 + motor4 != (1 + 2 + 3 + 4))
                return false;
            motor_channel_map[0] = motor1 - 1;
            motor_channel_map[1] = motor2 - 1;
            motor_channel_map[2] = motor3 - 1;
            motor_channel_map[3] = motor4 - 1;
            motor_channel_mapped = 1;
            return true;
        }

        /// @brief set the throttle of each motor
        /// @param motor1 motor1 throttle from [0.0f,1.0f]
        /// @param motor2
        /// @param motor3
        /// @param motor4
        void set_motor_throttle(const float motor1, const float motor2, const float motor3, const float motor4)
        {
            if (motor_channel_mapped == 0)
                return;
            uint16_t motor1_throttle = motor1 > min_throttle ? uint16_t(motor1 * 2000.0f) + 1048 : 1000;
            uint16_t motor2_throttle = motor2 > min_throttle ? uint16_t(motor2 * 2000.0f) + 1048 : 1000;
            uint16_t motor3_throttle = motor3 > min_throttle ? uint16_t(motor3 * 2000.0f) + 1048 : 1000;
            uint16_t motor4_throttle = motor4 > min_throttle ? uint16_t(motor4 * 2000.0f) + 1048 : 1000;

            uint32_t tmp = bdshot_read(motor_channel_map[0]);
            rpm_values[motor_channel_map[0]] = (tmp == 0xFFFFFFFF) ? rpm_values[motor_channel_map[0]] : tmp;
            tmp = bdshot_read(motor_channel_map[1]);
            rpm_values[motor_channel_map[1]] = (tmp == 0xFFFFFFFF) ? rpm_values[motor_channel_map[1]] : tmp;
            tmp = bdshot_read(motor_channel_map[2]);
            rpm_values[motor_channel_map[2]] = (tmp == 0xFFFFFFFF) ? rpm_values[motor_channel_map[2]] : tmp;
            tmp = bdshot_read(motor_channel_map[3]);
            rpm_values[motor_channel_map[3]] = (tmp == 0xFFFFFFFF) ? rpm_values[motor_channel_map[3]] : tmp;

            bdshot_write(motor_channel_map[0], motor1_throttle);
            bdshot_write(motor_channel_map[1], motor2_throttle);
            bdshot_write(motor_channel_map[2], motor3_throttle);
            bdshot_write(motor_channel_map[3], motor4_throttle);

            bdshot_start();
            return;
        }

        uint32_t* get_motor_rpms(void)
        {
            return this->rpm_values;
        }

    private:
        uint8_t init_flag = 0;
        uint8_t motor_channel_mapped = 0;
        uint8_t motor_channel_map[DSHOT_CHANNELS];
        uint32_t rpm_values[DSHOT_CHANNELS]={0};
        float min_throttle = 0.1f;
        float max_throttle = 0.9f;
    };
}

#include "bmi08_port.h"
#include "bmi08x.h"
#include <stm32_tim.h>
#include <time.h>
#include <stm32_gpio.h>
#include <arch/board/board.h>
#include <stdio.h>
#include <math.h>

#include "interface/imu_bmi088.h"

#define GRAVITY_EARTH (9.80665f)
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

/*********************************************************************/
/*                        Global variables                           */
/*********************************************************************/
/*! @brief This structure containing relevant bmi08 info */
static struct bmi08_dev bmi08dev;

/*! @brief variable to hold the bmi08 accel data */
static struct bmi08_sensor_data bmi08_accel;

/*! @brief variable to hold the bmi08 gyro data */
static struct bmi08_sensor_data bmi08_gyro;

/*! bmi08 accel int config */
static struct bmi08_accel_int_channel_cfg accel_int_config;

/*! bmi08 gyro int config */
static struct bmi08_gyro_int_channel_cfg gyro_int_config;

static uint8_t gyro_ready = 0;
struct timespec gyro_ts;
static uint8_t acce_ready = 0;
struct timespec acce_ts;


int bmi088_powerup()
{
    int8_t rslt;
    rslt = bmi08_interface_init(&bmi08dev, BMI08_SPI_INTF, BMI088_VARIANT);

    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_init(&bmi08dev);
    }
    
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

static inline int acce_ready_irq(int id, xcpt_t irqhandler, void *arg)
{
    clock_gettime(CLOCK_REALTIME, &acce_ts);
    int ret = -EINVAL;
    acce_ready = 1;
    ret = stm32_gpiosetevent(GPIO_ACCE0_INT, false, true, true, (xcpt_t)acce_ready_irq, arg);
    return ret;
}

static inline int gyro_ready_irq(int id, xcpt_t irqhandler, void *arg)
{
    clock_gettime(CLOCK_REALTIME, &gyro_ts);
    int ret = -EINVAL;
    gyro_ready = 1;
    ret = stm32_gpiosetevent(GPIO_GYRO0_INT, false, true, true, (xcpt_t)gyro_ready_irq, arg);
    return ret;
}

static inline int config_data_ready()
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

    if (rslt == BMI08_OK)
    {
        stm32_configgpio(GPIO_ACCE0_INT);
        stm32_configgpio(GPIO_GYRO0_INT);
        stm32_gpiosetevent(GPIO_ACCE0_INT, false, true, true, (xcpt_t)acce_ready_irq, NULL);
        stm32_gpiosetevent(GPIO_GYRO0_INT, false, true, true, (xcpt_t)gyro_ready_irq, NULL);
    }

    return rslt;
}


void bmi088_initialize()
{
    if(bmi088_powerup()<0)
    {
        printf("power up failed \n");
    }

    if(config_data_ready()<0)
    {
        printf("config failed \n");
    }
}

bool bmi088_acce_ready()
{
    return acce_ready;
}

bool bmi088_gyro_ready()
{
    return gyro_ready;
}

uint64_t bmi088_acce_read(float* data)
{
    int8_t rslt;
    rslt = bmi08a_get_data(&bmi08_accel, &bmi08dev);
    data[0] = (float)(bmi08_accel.x) / 1365.0f * GRAVITY_EARTH;
    data[1] = (float)(bmi08_accel.y) / 1365.0f * GRAVITY_EARTH;
    data[2] = (float)(bmi08_accel.z) / 1365.0f * GRAVITY_EARTH;
    acce_ready = 0;
    return acce_ts.tv_sec * 1000000000 + (acce_ts.tv_nsec);
}

uint64_t bmi088_gyro_read(float* data)
{
    int8_t rslt;
    rslt = bmi08g_get_data(&bmi08_gyro, &bmi08dev);
    data[0] = (float)(bmi08_gyro.x) / 16.384f * DEG_TO_RAD;
    data[1] = (float)(bmi08_gyro.y) / 16.384f * DEG_TO_RAD;
    data[2] = (float)(bmi08_gyro.z) / 16.384f * DEG_TO_RAD;
    gyro_ready = 0;
    return gyro_ts.tv_sec * 1000000000 + (gyro_ts.tv_nsec);
}

void bmi088_deinitialize()
{
    
}

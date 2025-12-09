#include "dshot_internal.h"

#include <arch/board/board.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <stm32_tim.h>
#include <stm32_gpio.h>
#include <stm32_dma.h>
#include <math.h>
#include <sys/time.h>
#include "arm_internal.h"  
/* send part */

#define BDSHOT_TIM_CH 1

#define BDSHOT_DMA_CH   DMAMAP_DMA12_TIM1CH1_0
#define BDSHOT_DMA_IRQ  STM32_IRQ_DMA1S0

#define BDSHOT_OVERSAMPLE 8
#define MOTOR_POLES_NUMBER 12
#define DSHOT_CHANNEL_NUM 4

#define DSHOT_TXBUF_LENGTH 17 * BDSHOT_OVERSAMPLE
#define DSHOT_RXBUF_LENGTH 30 * BDSHOT_OVERSAMPLE

#ifndef GPIO_BDSHOT_CH1
#error "GPIO_BDSHOT_CH1 is not defined!"
#endif

#ifndef GPIO_BDSHOT_CH2
#error "GPIO_BDSHOT_CH2 is not defined!"
#endif

#ifndef GPIO_BDSHOT_CH3
#error "GPIO_BDSHOT_CH3 is not defined!"
#endif

#ifndef GPIO_BDSHOT_CH4
#error "GPIO_BDSHOT_CH4 is not defined!"
#endif

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// #define GPIO_BDSHOT_CH1_OUT GPIO_BDSHOT_CH1|GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_SET
// #define GPIO_BDSHOT_CH2_OUT GPIO_BDSHOT_CH2|GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_SET
// #define GPIO_BDSHOT_CH3_OUT GPIO_BDSHOT_CH3|GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_SET
// #define GPIO_BDSHOT_CH4_OUT GPIO_BDSHOT_CH4|GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_SET
#define GPIO_BDSHOT_CH1_OUT GPIO_BDSHOT_CH1|GPIO_OUTPUT
#define GPIO_BDSHOT_CH2_OUT GPIO_BDSHOT_CH2|GPIO_OUTPUT
#define GPIO_BDSHOT_CH3_OUT GPIO_BDSHOT_CH3|GPIO_OUTPUT
#define GPIO_BDSHOT_CH4_OUT GPIO_BDSHOT_CH4|GPIO_OUTPUT

#define GPIO_BDSHOT_CH1_IN GPIO_BDSHOT_CH1|GPIO_INPUT|GPIO_FLOAT
#define GPIO_BDSHOT_CH2_IN GPIO_BDSHOT_CH2|GPIO_INPUT|GPIO_FLOAT
#define GPIO_BDSHOT_CH3_IN GPIO_BDSHOT_CH3|GPIO_INPUT|GPIO_FLOAT
#define GPIO_BDSHOT_CH4_IN GPIO_BDSHOT_CH4|GPIO_INPUT|GPIO_FLOAT

#define iv 0xFFFFFFFF
static const uint32_t GCR_table[32] = {
        iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
        iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv};

// using TIM1 and DMA1 


static struct stm32_tim_dev_s *tim_dev;
static DMA_HANDLE dma_handles;
static struct stm32_dma_config_s dma_tx_configs;
static struct stm32_dma_config_s dma_rx_configs;

static volatile uint32_t tx_buffer[DSHOT_TXBUF_LENGTH] __attribute__((aligned(32)));
static volatile uint32_t rx_buffer[DSHOT_RXBUF_LENGTH] __attribute__((aligned(32)));

static inline uint16_t bdshot_read_gpio()
{
    uint32_t  * volatile portinput = (uint32_t *) STM32_GPIOE_ODR;
    uint32_t idr_value = *portinput;
    uint16_t low_16_bits = (uint16_t)(idr_value & 0xFFFF);
    return low_16_bits;
}

static inline void bdshot_write_gpio(uint16_t var)
{
    uint32_t * volatile portoutput = (uint32_t *) STM32_GPIOE_ODR;
    *portoutput = var;
    return;
}

const int BDSHOT_GPIO_PINS[DSHOT_CHANNEL_NUM] = {
    (GPIO_BDSHOT_CH1 & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT,
    (GPIO_BDSHOT_CH2 & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT,
    (GPIO_BDSHOT_CH3 & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT,
    (GPIO_BDSHOT_CH4 & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT,
};

void bdshot_config_gpio(int mode)
{
    uint32_t regval;
    uint32_t pull_regval;
    if(mode == 0) //output
    {
        regval  = getreg32(STM32_GPIOE_BASE + STM32_GPIO_MODER_OFFSET);
        regval &= ~GPIO_MODER_MASK(BDSHOT_GPIO_PINS[0]);
        regval |= ((uint32_t)GPIO_MODER_OUTPUT << GPIO_MODER_SHIFT(BDSHOT_GPIO_PINS[0]));
        regval &= ~GPIO_MODER_MASK(BDSHOT_GPIO_PINS[1]);
        regval |= ((uint32_t)GPIO_MODER_OUTPUT << GPIO_MODER_SHIFT(BDSHOT_GPIO_PINS[1]));
        regval &= ~GPIO_MODER_MASK(BDSHOT_GPIO_PINS[2]);
        regval |= ((uint32_t)GPIO_MODER_OUTPUT << GPIO_MODER_SHIFT(BDSHOT_GPIO_PINS[2]));
        regval &= ~GPIO_MODER_MASK(BDSHOT_GPIO_PINS[3]);
        regval |= ((uint32_t)GPIO_MODER_OUTPUT << GPIO_MODER_SHIFT(BDSHOT_GPIO_PINS[3]));
        putreg32(regval, STM32_GPIOE_BASE + STM32_GPIO_MODER_OFFSET);
    }
    else // input
    {   
        //set to pull up
        pull_regval  = getreg32(STM32_GPIOE_BASE + STM32_GPIO_PUPDR_OFFSET);
        pull_regval &= ~GPIO_PUPDR_MASK(BDSHOT_GPIO_PINS[0]);
        pull_regval &= ~GPIO_PUPDR_MASK(BDSHOT_GPIO_PINS[1]);
        pull_regval &= ~GPIO_PUPDR_MASK(BDSHOT_GPIO_PINS[2]);
        pull_regval &= ~GPIO_PUPDR_MASK(BDSHOT_GPIO_PINS[3]);
        pull_regval |= (GPIO_PUPDR_PULLUP << GPIO_PUPDR_SHIFT(BDSHOT_GPIO_PINS[0]));
        pull_regval |= (GPIO_PUPDR_PULLUP << GPIO_PUPDR_SHIFT(BDSHOT_GPIO_PINS[1])); 
        pull_regval |= (GPIO_PUPDR_PULLUP << GPIO_PUPDR_SHIFT(BDSHOT_GPIO_PINS[2])); 
        pull_regval |= (GPIO_PUPDR_PULLUP << GPIO_PUPDR_SHIFT(BDSHOT_GPIO_PINS[3])); 
        //set to input
        regval  = getreg32(STM32_GPIOE_BASE + STM32_GPIO_MODER_OFFSET);
        regval &= ~GPIO_MODER_MASK(BDSHOT_GPIO_PINS[0]);
        regval |= ((uint32_t)GPIO_MODER_INPUT << GPIO_MODER_SHIFT(BDSHOT_GPIO_PINS[0]));
        regval &= ~GPIO_MODER_MASK(BDSHOT_GPIO_PINS[1]);
        regval |= ((uint32_t)GPIO_MODER_INPUT << GPIO_MODER_SHIFT(BDSHOT_GPIO_PINS[1]));
        regval &= ~GPIO_MODER_MASK(BDSHOT_GPIO_PINS[2]);
        regval |= ((uint32_t)GPIO_MODER_INPUT << GPIO_MODER_SHIFT(BDSHOT_GPIO_PINS[2]));
        regval &= ~GPIO_MODER_MASK(BDSHOT_GPIO_PINS[3]);
        regval |= ((uint32_t)GPIO_MODER_INPUT << GPIO_MODER_SHIFT(BDSHOT_GPIO_PINS[3]));

        putreg32(regval, STM32_GPIOE_BASE + STM32_GPIO_MODER_OFFSET);
        putreg32(pull_regval, STM32_GPIOE_BASE + STM32_GPIO_PUPDR_OFFSET);


    }
}

void bdshot_initialize()
{
    up_invalidate_dcache((uintptr_t)tx_buffer, (uintptr_t)(tx_buffer + DSHOT_TXBUF_LENGTH));
    up_invalidate_dcache((uintptr_t)rx_buffer, (uintptr_t)(rx_buffer + DSHOT_RXBUF_LENGTH));
    tim_dev = stm32_tim_init(BDSHOT_TIM_CH);
    STM32_TIM_SETCLOCK(tim_dev, 120000000); // 120MHZ 
    STM32_TIM_SETMODE(tim_dev, STM32_TIM_MODE_UP);
    STM32_TIM_SETCOUNTER(tim_dev, 0); // initial value for counter
    STM32_TIM_SETPERIOD(tim_dev, 49); // DSHOT 300
    // STM32_TIM_SETPERIOD(tim_dev, 24); // DSHOT 600
    STM32_TIM_ENABLEINT(tim_dev, ATIM_DIER_UDE |
                                     ATIM_DIER_CC1DE); // enable dma update interrupt

    stm32_unconfiggpio(GPIO_BDSHOT_CH1);
    stm32_unconfiggpio(GPIO_BDSHOT_CH2);
    stm32_unconfiggpio(GPIO_BDSHOT_CH3);
    stm32_unconfiggpio(GPIO_BDSHOT_CH4);
    
    // only need one channel 

    dma_handles = stm32_dmachannel(BDSHOT_DMA_CH);

    dma_tx_configs.paddr = (uint32_t)STM32_GPIOE_ODR; // gpio
    dma_tx_configs.maddr = (uint32_t)(tx_buffer);
    dma_tx_configs.ndata = DSHOT_TXBUF_LENGTH;
    dma_tx_configs.cfg1 = DMA_SCR_MINC |   // memory inc
                            DMA_SCR_DIR_M2P |
                            DMA_SCR_PSIZE_32BITS |
                            DMA_SCR_MSIZE_32BITS;
    dma_tx_configs.cfg2 = 0;

    dma_rx_configs.paddr = (uint32_t)STM32_GPIOE_IDR; // gpio
    dma_rx_configs.maddr = (uint32_t)(rx_buffer);
    dma_rx_configs.ndata = DSHOT_RXBUF_LENGTH;
    dma_rx_configs.cfg1 = DMA_SCR_MINC |   // memory inc
                            DMA_SCR_DIR_P2M |
                            DMA_SCR_PSIZE_32BITS |
                            DMA_SCR_MSIZE_32BITS;
    dma_rx_configs.cfg2 = 0;

    stm32_configgpio(GPIO_BDSHOT_CH1_OUT);
    stm32_configgpio(GPIO_BDSHOT_CH2_OUT);
    stm32_configgpio(GPIO_BDSHOT_CH3_OUT);
    stm32_configgpio(GPIO_BDSHOT_CH4_OUT);

    stm32_gpiowrite(GPIO_BDSHOT_CH1_OUT, 1);
    stm32_gpiowrite(GPIO_BDSHOT_CH2_OUT, 1);
    stm32_gpiowrite(GPIO_BDSHOT_CH3_OUT, 1);
    stm32_gpiowrite(GPIO_BDSHOT_CH4_OUT, 1);
    return;
}

void bdshot_deinitialize()
{
    stm32_dmastop(dma_handles);
    stm32_dmafree(dma_handles);
    STM32_TIM_DISABLE(tim_dev);
    stm32_tim_deinit(tim_dev);

    stm32_unconfiggpio(GPIO_BDSHOT_CH1);
    stm32_unconfiggpio(GPIO_BDSHOT_CH2);
    stm32_unconfiggpio(GPIO_BDSHOT_CH3);
    stm32_unconfiggpio(GPIO_BDSHOT_CH4);
}

static inline void send_complete_interrupt(DMA_HANDLE handle, uint8_t status, void *arg)
{
    bdshot_config_gpio(1);
    STM32_TIM_DISABLE(tim_dev);
    STM32_TIM_SETCOUNTER(tim_dev, 0); // initial value for counter
    STM32_TIM_SETPERIOD(tim_dev, 99); // 400k bitrate
    stm32_dmasetup(dma_handles, &dma_rx_configs);
    up_invalidate_dcache((uintptr_t)rx_buffer, (uintptr_t)(rx_buffer + DSHOT_RXBUF_LENGTH));
    STM32_TIM_ENABLE(tim_dev);
    stm32_dmastart(dma_handles, NULL, NULL, false);
    return;
}
void bdshot_start()
{
    // printf("bdshot start %llu \n", gethrtime());
    
    STM32_TIM_DISABLE(tim_dev);
    bdshot_config_gpio(0);
    STM32_TIM_SETCOUNTER(tim_dev, 0); // initial value for counter
    STM32_TIM_SETPERIOD(tim_dev, 49); // 400k bitrate
    stm32_dmasetup(dma_handles, &dma_tx_configs);
    up_invalidate_dcache((uintptr_t)tx_buffer, (uintptr_t)(tx_buffer + DSHOT_TXBUF_LENGTH));
    STM32_TIM_ENABLE(tim_dev);
    stm32_dmastart(dma_handles, send_complete_interrupt, NULL, false);
    // printf("debug %d \n", debug);
    return;
}

// throttle 1000-2000
// channel 1 2 3 4
void bdshot_write(int channel, uint32_t throttle)
{
    uint16_t throttle_int = (throttle - 1000);
    throttle_int <<= 1; // skip telemetry bit
    uint16_t csum_data = throttle_int;
    uint16_t csum = (~(csum_data ^ (csum_data >> 4) ^ (csum_data >> 8))) & 0xf; //bidirectional
    uint16_t packet = (throttle_int << 4) | csum;
    uint16_t orig_odr = bdshot_read_gpio();
    uint16_t high_bits = (1 << BDSHOT_GPIO_PINS[channel]);
    uint16_t low_bits = ~(1 << BDSHOT_GPIO_PINS[channel]);
    for(int i = 0; i < 16; i++)
    {
        if((packet & 0x8000))
        {
            tx_buffer[i * 8 + 0] &=  low_bits;
            tx_buffer[i * 8 + 1] &=  low_bits;
            tx_buffer[i * 8 + 2] &=  low_bits;
            tx_buffer[i * 8 + 3] &=  low_bits;
            tx_buffer[i * 8 + 4] &=  low_bits;
            tx_buffer[i * 8 + 5] |=  high_bits;
            tx_buffer[i * 8 + 6] |=  high_bits;
            tx_buffer[i * 8 + 7] |=  high_bits;

        }
        else
        {
            tx_buffer[i * 8 + 0] &=  low_bits;
            tx_buffer[i * 8 + 1] &=  low_bits;
            tx_buffer[i * 8 + 2] |=  high_bits;
            tx_buffer[i * 8 + 3] |=  high_bits;
            tx_buffer[i * 8 + 4] |=  high_bits;
            tx_buffer[i * 8 + 5] |=  high_bits;
            tx_buffer[i * 8 + 6] |=  high_bits;
            tx_buffer[i * 8 + 7] |=  high_bits;
        }
        packet <<= 1;
    }
    tx_buffer[DSHOT_TXBUF_LENGTH -8] |= high_bits;
    tx_buffer[DSHOT_TXBUF_LENGTH -7] |= high_bits;
    tx_buffer[DSHOT_TXBUF_LENGTH -6] |= high_bits;
    tx_buffer[DSHOT_TXBUF_LENGTH -5] |= high_bits;
    tx_buffer[DSHOT_TXBUF_LENGTH -4] |= high_bits;
    tx_buffer[DSHOT_TXBUF_LENGTH -3] |= high_bits;
    tx_buffer[DSHOT_TXBUF_LENGTH -2] |= high_bits;
    tx_buffer[DSHOT_TXBUF_LENGTH -1] |= high_bits;
    return;
}

void print_binary(uint16_t n) {
    for (int i = sizeof(n) * 8 - 1; i >= 0; i--) {
        printf("%d", (n >> i) & 1);
    }
    printf("\n");
}

static bool bdshot_check_checksum(uint16_t value)
{
    // BDshot frame has 4 last bits CRC:
    if (((value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12)) & 0x0F) == 0x0F)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void sample_io()
{
    printf("sample \n");
    for(int i=50;i<200;i++)
    {
        printf("%d th:  ",i);
        print_binary(rx_buffer[i]);
    }
}


uint32_t bdshot_read(int channel)
{
    // printf("bdshot read %llu \n", gethrtime());
    int motor_shift = BDSHOT_GPIO_PINS[channel];
    uint16_t i = 0;
    uint16_t previous_i = 0;
    uint16_t end_i = 0;
    uint32_t previous_value = 1;
    uint32_t motor_response = 0;
    uint8_t bits = 0;
    int oversample_rate = 3;

    while (i < (int)(DSHOT_RXBUF_LENGTH))
    {
        if (!(rx_buffer[i] & (1 << motor_shift)))
        {
            previous_value = 0;
            previous_i = i;
            end_i = i + oversample_rate * 24;
            // printf("current i %d \n", i);
            break;
        }
        i++;
    }
    // if LOW edge was detected:
    if (previous_value == 0)
    {
        int sum =0;
        while (i < end_i)
        {
            // then look for changes in bits values and compute BDSHOT bits:
            if (((rx_buffer[i] & (1 << motor_shift)) != previous_value))
            {
                int diff = i - previous_i;
                int len = (diff + 0.5 * oversample_rate)/oversample_rate; // how many bits had the same value
                // const uint8_t len = round((i - previous_i) / oversample_rate);
                sum += len;
                bits += len;
                motor_response <<= len;
                if (previous_value != 0)
                {
                    motor_response |= (0x1FFFFF >> (21 - len)); // 21 ones right-shifted by 20 or less
                }
                previous_value = rx_buffer[i] & (1 << motor_shift);
                previous_i = i;
                // printf("current i %d   diff %d   len %d  level: %d    sum %d \n", i, diff, len, (rx_buffer[i] & (1 << motor_shift)), sum);
            }
            i++;
        }
        // if last bits were 1 they were not added so far
        motor_response <<= (21 - bits);
        motor_response |= 0x1FFFFF >> bits; // 21 ones right-shifted
        // printf("motor response %x \n", motor_response);
        motor_response = (motor_response ^ (motor_response >> 1)); // now we have GCR value

        uint32_t decoded_value = GCR_table[(motor_response & 0x1F)];
        decoded_value |= GCR_table[((motor_response >> 5) & 0x1F)] << 4;
        decoded_value |= GCR_table[((motor_response >> 10) & 0x1F)] << 8;
        decoded_value |= GCR_table[((motor_response >> 15) & 0x1F)] << 12;
        if (decoded_value < 0xFFFF && bdshot_check_checksum(decoded_value))
        {
            // if checksum is correct real save real RPM.
            // value sent by ESC is a period between each pole changes [us].
            // to achive eRPM we need to find out how many of these changes are in one minute.
            // eRPM = (60*1000 000)/T_us next RPM can be achived -> RPM = eRPM/(poles/2):
            uint32_t rpm;
            rpm = ((decoded_value & 0x1FF0) >> 4) << (decoded_value >> 13);      // cut off CRC and add shifting - this is period in [us]
            rpm = 60 * 1000000 / rpm * 2 / MOTOR_POLES_NUMBER; // convert to RPM
            // printf("bdshot end %llu \n", gethrtime());
            return rpm;
        }
        else
        {
            return 0xFFFFFFFF;
        }
        
    }
    else
    { // if LOW edge was not found return incorrect motor response:
        return 0xFFFFFFFF;
    }
}
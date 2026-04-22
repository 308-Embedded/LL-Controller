Nuttx configuration for MicoAir743v2 (STM32H743VI)
  HSE: 8MHz external crystal
  LEDs:
    PE3: red
    PE4: blue
    PE2: green
  USB_OTG_FS:
    DM: PA11
    DP: PA12
  SDMMC1:
    CK:   PC12
    CMD:  PD2
    D0:   PC8
    D1:   PC9
    D2:   PC10
    D3:   PC11
  SPI1:
    MISO: PA6
    MOSI: PA7
    SCK:  PA5
    CS:   PB12 (AT7456E OSD)
  SPI2:
    MISO: PC2
    MOSI: PC3
    SCK:  PD3
    CS:   PD4 (BMI088 accel), PD5 (BMI088 gyro)
  SPI3:
    MISO: PB4
    MOSI: PD6
    SCK:  PB3
    CS:   PA15 (BMI270)
  I2C2:
    SCL:  PB10
    SDA:  PB11
  Motors:
    M1: PE14 TIM1_CH4
    M2: PE13 TIM1_CH3
    M3: PE11 TIM1_CH2
    M4: PE9  TIM1_CH1
    M5: PB1  TIM3_CH4
    M6: PB0  TIM3_CH3
    M7: PD12 TIM4_CH1
    M8: PD13 TIM4_CH2
    M9: PE5  TIM15_CH1
    M10: PE6 TIM15_CH2

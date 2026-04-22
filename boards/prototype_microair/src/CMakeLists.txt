# Ref: PX4

set(SRC_FILES
  stm32_boot.c
  stm32_bringup.c
  stm32_userleds.c
  stm32_usb.c
  stm32_pwm.c
  stm32_appinitialize.c
  stm32_reset.c
  stm32_autoleds.c
  stm32_spi.c
  stm32_timer.c
  etc_romfs.c
)

add_library(drivers_board ${SRC_FILES})
target_link_libraries(drivers_board nuttx_arch nuttx_drivers)

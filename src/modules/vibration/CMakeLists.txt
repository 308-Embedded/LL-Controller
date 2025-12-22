add_module(
  MODULE modules__vibration
  MAIN vibration
  SRCS
  vibration_main.cc
  STACK_MAIN 16384
  DEPENDS nuttx_arch nuttx_m nuttx_c library__imu library__dshot
)

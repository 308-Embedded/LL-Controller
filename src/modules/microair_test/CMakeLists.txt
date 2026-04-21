add_module(
  MODULE modules__microair_test
  MAIN microair_test
  SRCS
  microair_test_main.cc
  STACK_MAIN 163840
  DEPENDS nuttx_arch nuttx_m nuttx_c library__dshot library__imu
)

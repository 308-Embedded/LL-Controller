add_module(
  MODULE modules__motor_test
  MAIN motor_test
  SRCS
  motor_test_main.cc
  STACK_MAIN 163840
  DEPENDS nuttx_arch nuttx_m nuttx_c library__dshot
)

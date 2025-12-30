add_module(
  MODULE modules__control_loop
  MAIN control_loop
  SRCS
  control_loop_main.cc
  imu_task.cc
  STACK_MAIN 163840
  DEPENDS nuttx_arch nuttx_m nuttx_c library__imu library__dshot library__embedded_math library__shared_table
)

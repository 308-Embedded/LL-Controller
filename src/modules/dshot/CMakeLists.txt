add_module(
  MODULE modules__dshot
  MAIN dshot
  SRCS
  dshot_main.cc
  STACK_MAIN 163840
  DEPENDS nuttx_arch nuttx_m nuttx_c library__dshot
)

add_module(
  MODULE modules__mavlink
  MAIN mavlink
  SRCS
    mavlink_main.cc
  INCLUDES
    .
  STACK_MAIN 16384
  DEPENDS nuttx_arch nuttx_m
)

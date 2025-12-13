add_module(
  MODULE modules__shared_table
  MAIN st
  SRCS
    st_main.cc
  DEPENDS nuttx_arch library__shared_table
)

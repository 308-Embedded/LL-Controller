ll_add_library(
  LIB_NAME library__ringbuffer
  SRCS
  ringbuf.c
  INTERFACES
  interface
  DEPENDS nuttx_arch nuttx_mm
)

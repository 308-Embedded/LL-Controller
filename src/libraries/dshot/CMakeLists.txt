ll_add_library(
  LIB_NAME library__dshot
  SRCS
  dshot_internal.c
  INTERFACES
  interface
  DEPENDS  nuttx_arch nuttx_mm 
)
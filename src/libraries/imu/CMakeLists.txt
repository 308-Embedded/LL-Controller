ll_add_library(
  LIB_NAME library__imu
  SRCS
  imu_bmi088.c
  bmi08_port.c 
  bmi08a.c 
  bmi08g.c 
  bmi08xa.c
  bmi08_defs.h
  bmi08_port.h
  bmi08.h
  bmi08x.h
  INTERFACES
  interface
  DEPENDS nuttx_arch nuttx_m nuttx_c
)
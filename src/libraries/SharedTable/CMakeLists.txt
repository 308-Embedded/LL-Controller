ll_add_library(
  LIB_NAME library__shared_table
  SRCS
  SharedTable.c
  SharedTableDefs.h
  DEPENDS
  nuttx_arch nuttx_mm library__ringbuffer
  INTERFACES
  interface
)
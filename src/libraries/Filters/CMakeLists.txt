ll_add_library(
  LIB_NAME library__filters
  SRCS
  empty.cc
  INTERFACES
  interface
  DEPENDS library__embedded_math
)
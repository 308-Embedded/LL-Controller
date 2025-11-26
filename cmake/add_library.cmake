include(misc)

function(ll_add_library)

  parse_function_args(
    NAME ll_add_library
    ONE_VALUE LIB_NAME
    MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES INTERFACES
    REQUIRED LIB_NAME
    ARGN ${ARGN}
  )
	add_library(${LIB_NAME} STATIC EXCLUDE_FROM_ALL
		${SRCS}
	)

	target_compile_definitions(${LIB_NAME} PRIVATE MODULE_NAME="${target}")

    target_include_directories(${LIB_NAME} INTERFACE ${INTERFACES})

	set_property(GLOBAL APPEND PROPERTY CUSTOM_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})
	list_make_absolute(ABS_SRCS ${CMAKE_CURRENT_SOURCE_DIR} ${ARGN})
	set_property(GLOBAL APPEND PROPERTY CUSTOM_MODULE_PATHS ${ABS_SRCS})
endfunction()
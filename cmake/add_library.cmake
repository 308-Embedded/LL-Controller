include(misc)

function(ll_add_library)

  parse_function_args(
    NAME ll_add_library
    ONE_VALUE LIB_NAME
    MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS INTERFACES
    REQUIRED LIB_NAME
    ARGN ${ARGN}
  )
	add_library(${LIB_NAME} STATIC EXCLUDE_FROM_ALL
		${SRCS}
	)

	target_compile_definitions(${LIB_NAME} PRIVATE MODULE_NAME="${target}")
  target_include_directories(${LIB_NAME} PUBLIC ${INTERFACES})

  if(DEPENDS)
    # using target_link_libraries for dependencies provides linking
    # as well as interface include and libraries
    foreach(dep ${DEPENDS})
      get_target_property(dep_type ${dep} TYPE)
      if((${dep_type} STREQUAL "STATIC_LIBRARY") OR (${dep_type} STREQUAL "INTERFACE_LIBRARY"))
        target_link_libraries(${LIB_NAME} PRIVATE ${dep})
        message("${dep} ${dep_type}")
      else()
        add_dependencies(${LIB_NAME} ${dep})
      endif()
    endforeach()
  endif()

	set_property(GLOBAL APPEND PROPERTY CUSTOM_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})
	list_make_absolute(ABS_SRCS ${CMAKE_CURRENT_SOURCE_DIR} ${ARGN})
	set_property(GLOBAL APPEND PROPERTY CUSTOM_MODULE_PATHS ${ABS_SRCS})
endfunction()
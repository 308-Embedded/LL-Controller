file(GLOB_RECURSE subdirs LIST_DIRECTORIES true ${CMAKE_CURRENT_SOURCE_DIR}/*)

foreach(subdir ${subdirs})
    if(EXISTS ${subdir}/CMakeLists.txt)
        add_subdirectory(${subdir})
    endif()
endforeach()
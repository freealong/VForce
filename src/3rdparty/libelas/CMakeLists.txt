cmake_minimum_required(VERSION 3.5)
project(elas)

# check if I am a top-level project
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    # using c++11
    set(CMAKE_CXX_STANDARD 11)
    # Define DEBUG in Debug mode
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
    endif()
    # enable openmp
    find_package(OpenMP)
    if (OPENMP_FOUND)
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
endif()

if (NOT OPENMP_FOUND)
    message(FATAL_ERROR "OpenMP not found. This is an OpenMP version library")
endif()

set(ELAS_SOURCES descriptor.cpp elas.cpp filter.cpp matrix.cpp triangle.cpp)
set(ELAS_HEADERS descriptor.h elas.h filter.h image.h matrix.h timer.h triangle.h)

add_library(elas ${ELAS_SOURCES})

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    add_executable(main main.cpp)
    target_link_libraries(main elas)
endif()
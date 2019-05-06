cmake_minimum_required(VERSION 3.5)
project(VForce LANGUAGES CXX VERSION 1.0)

# Set options
option(ENABLE_TESTS "Enable build tests" TRUE)
option(ENABLE_APPS "Enable build apps" TRUE)
option(ENABLE_TOOLS "Enable build tools" FALSE)
option(ENABLE_TIMING "Enable computation time recording" TRUE)
option(BUILD_DOC "Create and install the HTML based API documentation (requires Doxygen)" TRUE)

# Variables configuration
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Guard
## Guard against in-source builds
if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed.
    You should create a separate directory for build files.")
endif ()

## Guard against bad build-type strings
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug"
   AND NOT CMAKE_BUILD_TYPE STREQUAL "Release")
   message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\".
   Allowed values are Debug, Release")
endif ()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
# Compiler configuration
include(cmake/ConfigureCompiler.cmake)

# Dependence
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Project subdirectories
include_directories(${SRC_DIR})

if (ENABLE_TIMING)
    add_definitions(-DENABLE_TIMING)
endif ()

add_subdirectory(${SRC_DIR}/3rdparty/libelas)

add_subdirectory(${SRC_DIR}/utils)
add_subdirectory(${SRC_DIR}/camera)
add_subdirectory(${SRC_DIR}/vision)
add_subdirectory(${SRC_DIR}/robot)

# Setting by manual options
if (ENABLE_TEST)
    # @TODO: add tests
#    add_subdirectory(tests)
endif ()

if (ENABLE_APPS)
    add_subdirectory(apps)
endif ()

if (ENABLE_TOOLS)
    add_subdirectory(tools)
endif ()

# doc
if (BUILD_DOC)
    add_subdirectory(doc)
endif()

# Installation
# @TODO: Set installation

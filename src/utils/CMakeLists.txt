project(VForce-Utils)

find_package(glfw3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIR})

set(utils_ROOT "${SRC_DIR}/utils")
file(GLOB utils_HEADERS ${utils_ROOT}/*.hpp ${utils_ROOT}/*.h)
file(GLOB utils_SOURCES ${utils_ROOT}/*.cpp)

add_library(vforce_utils ${utils_HEADERS} ${utils_SOURCES})
target_link_libraries(vforce_utils ${GLFW3_LIBRARY} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
project(VForce-Utils)

find_package(glfw3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIR})

file(GLOB utils_HEADERS ${CMAKE_CURRENT_LIST_DIR}/*.hpp ${CMAKE_CURRENT_LIST_DIR}/*.h)
file(GLOB utils_SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

add_library(vforce_utils ${utils_HEADERS} ${utils_SOURCES})
target_link_libraries(vforce_utils ${GLFW3_LIBRARY} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
project(VForce-Camera)

find_package(librealsense REQUIRED)
include_directories(${librealsense_INCLUDE_DIRS})

set(camera_ROOT "${SRC_DIR}/camera")
file(GLOB_RECURSE camera_HEADERS ${camera_ROOT}/*.hpp ${camera_ROOT}/*.h)
file(GLOB_RECURSE camera_SOURCES ${camera_ROOT}/*.cpp)

add_library(vforce_camera ${camera_SOURCES} ${camera_HEADERS})
target_link_libraries(vforce_camera glog vforce_utils elas ${librealsense_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

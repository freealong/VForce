project(VForce-Robot)

set(robot_ROOT "${SRC_DIR}/robot")
file(GLOB_RECURSE robot_HEADERS ${robot_ROOT}/*.hpp ${robot_ROOT}/*.h)
file(GLOB_RECURSE robot_SOURCES ${robot_ROOT}/*.cpp)

add_library(vforce_robot ${robot_SOURCES} ${robot_HEADERS})
target_link_libraries(vforce_robot glog ${OpenCV_LIBRARIES})

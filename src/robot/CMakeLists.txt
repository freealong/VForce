project(VForce-Robot)

file(GLOB_RECURSE robot_HEADERS ${CMAKE_CURRENT_LIST_DIR}/*.hpp ${CMAKE_CURRENT_LIST_DIR}/*.h)
file(GLOB_RECURSE robot_SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

add_library(vforce_robot ${robot_SOURCES} ${robot_HEADERS})
target_link_libraries(vforce_robot glog ${OpenCV_LIBRARIES})

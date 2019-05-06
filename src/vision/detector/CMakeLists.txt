project(VForce-Detector)


file(GLOB_RECURSE detector_HEADERS ${CMAKE_CURRENT_LIST_DIR}/*.hpp ${CMAKE_CURRENT_LIST_DIR}/*.h)
file(GLOB_RECURSE detector_SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

add_library(vforce_detector ${detector_SOURCES} ${detector_HEADERS})
target_link_libraries(vforce_detector glog ${OpenCV_LIBRARIES} ${PYTHON_LIBRARIES})

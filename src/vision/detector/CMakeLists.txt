project(VForce-Detector)

find_package(PythonLibs 3.5 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

set(detector_ROOT "${vision_ROOT}/detector")
file(GLOB_RECURSE detector_HEADERS ${detector_ROOT}/*.hpp ${detector_ROOT}/*.h)
file(GLOB_RECURSE detector_SOURCES ${detector_ROOT}/*.cpp)

add_library(vforce_detector ${detector_SOURCES} ${detector_HEADERS})
target_link_libraries(vforce_detector glog ${OpenCV_LIBRARIES} ${PYTHON_LIBRARIES})

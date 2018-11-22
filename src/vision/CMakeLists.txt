project(VForce-Vision)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
find_package(NumPy REQUIRED)
include_directories(${PYTHON_NUMPY_INCLUDE_DIR})

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/detector)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/matcher)

file(GLOB vision_HEADERS ${CMAKE_CURRENT_LIST_DIR}/*.hpp ${CMAKE_CURRENT_LIST_DIR}/*.h)
file(GLOB vision_SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

add_library(vforce_vision ${vision_SOURCES} ${vision_HEADERS})
target_link_libraries(vforce_vision vforce_detector vforce_matcher)

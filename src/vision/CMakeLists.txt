project(VForce-Vision)

set(vision_ROOT "${SRC_DIR}/vision")

add_subdirectory(${vision_ROOT}/detector)
add_subdirectory(${vision_ROOT}/matcher)

file(GLOB vision_HEADERS ${vision_ROOT}/*.hpp ${vision_ROOT}/*.h)
file(GLOB vision_SOURCES ${vision_ROOT}/*.cpp)

add_library(vforce_vision ${vision_SOURCES} ${vision_HEADERS})
target_link_libraries(vforce_vision vforce_detector vforce_matcher)

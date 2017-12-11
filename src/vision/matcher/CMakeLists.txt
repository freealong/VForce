project(VForce-Matcher)

set(matcher_ROOT "${vision_ROOT}/matcher")
file(GLOB_RECURSE matcher_HEADERS ${matcher_ROOT}/*.hpp ${matcher_ROOT}/*.h)
file(GLOB_RECURSE matcher_SOURCES ${matcher_ROOT}/*.cpp)

add_library(vforce_matcher ${matcher_SOURCES} ${matcher_HEADERS})
target_link_libraries(vforce_matcher glog ${PCL_LIBRARIES})

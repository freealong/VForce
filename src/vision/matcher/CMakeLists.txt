project(VForce-Matcher)

file(GLOB_RECURSE matcher_HEADERS ${CMAKE_CURRENT_LIST_DIR}/*.hpp ${CMAKE_CURRENT_LIST_DIR}/*.h)
file(GLOB_RECURSE matcher_SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

add_library(vforce_matcher ${matcher_SOURCES} ${matcher_HEADERS})
target_link_libraries(vforce_matcher glog ${PCL_LIBRARIES})

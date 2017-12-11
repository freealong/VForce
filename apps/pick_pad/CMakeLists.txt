project(VForce-Apps-PickPad)

set(pick_pad_ROOT "${apps_ROOT}/pick_pad")
file(GLOB_RECURSE pick_pad_HEADERS ${pick_pad_ROOT}/*.hpp ${pick_pad_ROOT}/*.h)
file(GLOB_RECURSE pick_pad_SOURCES ${pick_pad_ROOT}/*.cpp)

add_executable(vforce_pick_pad ${pick_pad_SOURCES} ${pick_pad_HEADERS})
target_link_libraries(vforce_pick_pad pthread gflags vforce_utils vforce_camera vforce_vision vforce_robot)

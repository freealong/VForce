project(VForce-Apps-PickBottle)

set(pick_bottle_ROOT "${apps_ROOT}/pick_bottle")
file(GLOB_RECURSE pick_bottle_HEADERS ${pick_bottle_ROOT}/*.hpp ${pick_bottle_ROOT}/*.h)
file(GLOB_RECURSE pick_bottle_SOURCES ${pick_bottle_ROOT}/*.cpp)

add_executable(vforce_pick_bottle ${pick_bottle_SOURCES} ${pick_bottle_HEADERS})
target_link_libraries(vforce_pick_bottle pthread gflags vforce_utils vforce_camera vforce_vision vforce_robot)

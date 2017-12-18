project(VForce-apps)

option(BUILD_PICK_PAD "Build pick_pad app" TRUE)
option(BUILD_PICK_BOTTLE "Build pick_bottle app" TRUE)

set(apps_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

if (BUILD_PICK_PAD)
    add_subdirectory(${apps_ROOT}/pick_pad)
endif ()

if (BUILD_PICK_BOTTLE)
    add_subdirectory(${apps_ROOT}/pick_bottle)
endif ()

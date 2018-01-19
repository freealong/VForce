project(VForce-tools)

option(ENABLE_ROBOCAMCAL "Enable build robocamcal" TRUE)

if (ENABLE_ROBOCAMCAL)
    add_subdirectory(robocamcal)
endif ()

add_executable(data_collector ${CMAKE_CURRENT_SOURCE_DIR}/data_collector.cpp)
target_link_libraries(data_collector vforce_camera vforce_utils)

add_executable(view_rgbd ${CMAKE_CURRENT_SOURCE_DIR}/view_rgbd.cpp)
target_link_libraries(view_rgbd vforce_camera vforce_utils)

add_executable(pose2matrix ${CMAKE_CURRENT_SOURCE_DIR}/pose2matrix.cpp)
target_link_libraries(pose2matrix vforce_utils)

add_executable(process_model ${CMAKE_CURRENT_SOURCE_DIR}/process_model.cpp)
target_link_libraries(process_model vforce_utils)

add_executable(tcpclient ${CMAKE_CURRENT_SOURCE_DIR}/tcpclient.cpp)
target_link_libraries(tcpclient)

add_executable(get_camera_params ${CMAKE_CURRENT_SOURCE_DIR}/get_camera_params.cpp)
target_link_libraries(get_camera_params vforce_camera)

add_executable(cloud2hha ${CMAKE_CURRENT_SOURCE_DIR}/cloud2hha.cpp)
target_link_libraries(cloud2hha vforce_camera)

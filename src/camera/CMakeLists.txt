project(VForce-Camera)

set(camera_HEADERS ${CMAKE_CURRENT_LIST_DIR}/Camera.hpp ${CMAKE_CURRENT_LIST_DIR}/VirtualCamera.hpp)
set(camera_SOURCES ${CMAKE_CURRENT_LIST_DIR}/Camera.cpp ${CMAKE_CURRENT_LIST_DIR}/VirtualCamera.cpp)
set(camera_LIBRARIES glog vforce_utils elas ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

find_package(librealsense QUIET)
if(librealsense_FOUND)
    message("Found realsense")
    include_directories(${librealsense_INCLUDE_DIRS})
    list(APPEND camera_HEADERS ${CMAKE_CURRENT_LIST_DIR}/RealsenseCamera.hpp
            ${CMAKE_CURRENT_LIST_DIR}/StereoRealsenseCamera.hpp)
    list(APPEND camera_SOURCES ${CMAKE_CURRENT_LIST_DIR}/RealsenseCamera.cpp
            ${CMAKE_CURRENT_LIST_DIR}/StereoRealsenseCamera.cpp)
    list(APPEND camera_LIBRARIES ${librealsense_LIBRARIES})
endif()

if(OPENNI2_LIBRARY)
    message("Found OpenNI2")
    list(APPEND camera_HEADERS ${CMAKE_CURRENT_LIST_DIR}/OpenNI2Camera.hpp)
    list(APPEND camera_SOURCES ${CMAKE_CURRENT_LIST_DIR}/OpenNI2Camera.cpp)
endif()

add_library(vforce_camera ${camera_SOURCES} ${camera_HEADERS})
target_link_libraries(vforce_camera glog vforce_utils elas ${librealsense_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

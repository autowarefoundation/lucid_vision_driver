cmake_minimum_required(VERSION 3.5)
project(lucid_vision_driver)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_CXX_FLAGS "-g3 -O3")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindARENA.cmake")

link_directories(${arena_sdk_LIBRARIES})

ament_auto_find_build_dependencies()

ament_auto_add_library(arena_camera_node SHARED
        "include/arena_camera/arena_camera.h"
        "include/arena_camera/arena_camera_node.h"
        "include/arena_camera/arena_cameras_handler.h"
        "include/arena_camera/camera_settings.h"
        "src/arena_camera_node.cpp"
        "src/arena_camera.cpp"
        "src/arena_cameras_handler.cpp")

## ArenaSDK is not handled by ament, so we need the manual steps below.
target_include_directories(arena_camera_node
        SYSTEM
        PUBLIC
        ${arena_sdk_INCLUDES}
        ${OpenCV_INCLUDE_DIRS}
        )
target_link_libraries(arena_camera_node
        ${OpenCV_LIBRARIES}
        ${arena_sdk_LIBRARIES}
        )

# These are external libraries and include folders that we want to be visible to the packages
# that depend on this one.
ament_export_include_directories(${arena_sdk_INCLUDES})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

rclcpp_components_register_node(arena_camera_node
        PLUGIN "ArenaCameraNode"
        EXECUTABLE arena_camera_node_exe)

ament_auto_package(INSTALL_TO_SHARE
        launch
        config
        param
        )
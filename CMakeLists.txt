cmake_minimum_required(VERSION 3.8)
project(sleepy_lio)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(TBB REQUIRED)
find_package(fmt REQUIRED)
find_package(GTSAM 4.3 REQUIRED)
find_package(gtsam_points 1.2.0 REQUIRED)
find_package(livox_ros_driver2 QUIET)

if(livox_ros_driver2_FOUND)
    message(STATUS "Found livox_ros_driver2, enabling Livox lidar support.")
    add_definitions(-DHAVE_LIVOX_DRIVER)
else()
    message(WARNING "livox_ros_driver2 not found. Building without Livox lidar support.")
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/param_deliver.h.in  
    ${CMAKE_BINARY_DIR}/src/param_deliver.h            
    @ONLY
)
file(GLOB_RECURSE SOURCE CONFIGURE_DEPENDS src/*.cpp 3rdparty/*.c 3rdparty/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE})

target_include_directories(${PROJECT_NAME} PUBLIC
    3rdparty
    src
    ${CMAKE_BINARY_DIR}/src
)
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    yaml-cpp
    TBB::tbb
    fmt::fmt
    gtsam
    gtsam_points::gtsam_points
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_components
    rclpy
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
    std_srvs
    nav2_msgs
)
if(livox_ros_driver2_FOUND)
    ament_target_dependencies(${PROJECT_NAME}
        livox_ros_driver2
    )
endif()
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "sleepy::SleepyLioNode"
    EXECUTABLE ${PROJECT_NAME}_node
)
ament_auto_package(
    INSTALL_TO_SHARE
    config
    launch
    rviz2
)

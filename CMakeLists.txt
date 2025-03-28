cmake_minimum_required(VERSION 3.8)
project(romea_path_following)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(romea_mobile_base_utils REQUIRED)
find_package(romea_mobile_base_msgs REQUIRED)
find_package(romea_joystick_utils REQUIRED)
find_package(romea_path_utils REQUIRED)
find_package(romea_path_msgs REQUIRED)
find_package(romea_core_control REQUIRED)
find_package(romea_core_path_following REQUIRED)
# find_package(romea_following_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/path_following/component.cpp
  src/path_following/path_following.cpp
  src/external_control/platoon/component.cpp
  src/external_control/platoon/platoon.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  romea_mobile_base_utils
  romea_mobile_base_msgs
  romea_joystick_utils
  romea_path_utils
  romea_path_msgs
  romea_core_control
  romea_core_path_following
  # romea_following_utils
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# if(${ROS_DISTRO} STREQUAL "ROS_GALACTIC")

#   rclcpp_components_register_nodes(${PROJECT_NAME} "romea::ros2::PathFollowingComponent")

#   add_executable(path_following_node
#     src/path_following.cpp
#     src/path_following_component.cpp
#     src/path_following_node.cpp
#     src/path_platoon.cpp
#     src/path_platoon_component.cpp
#     )

#   target_include_directories(path_following_node PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)


#   ament_target_dependencies(path_following_node
#     class_loader
#     rclcpp
#     rclcpp_components
#     rclcpp_lifecycle
#     sensor_msgs
#     romea_mobile_base_utils
#     romea_mobile_base_msgs
#     romea_joystick_utils
#     romea_path_utils
#     romea_path_msgs
#     romea_core_control
#     romea_core_path_following)

#   install(TARGETS
#     path_following_node
#     DESTINATION lib/${PROJECT_NAME})

# else()

#   rclcpp_components_register_node(${PROJECT_NAME}
#     PLUGIN "romea::ros2::PathFollowingComponent"
#     EXECUTABLE path_following_node
#     EXECUTOR "MultiThreadedExecutor"
#   )
#endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::ros2::path_following::PathFollowingComponent"
  EXECUTABLE path_following_node
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::ros2::path_following::PlatoonComponent"
  EXECUTABLE path_platoon_node
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/uturn_manager
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)
endif()

ament_package()

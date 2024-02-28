cmake_minimum_required(VERSION 3.8)
project(romea_path_following2)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
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
  src/path_following_component.cpp
  src/path_following.cpp
  # src/path_following_algo.cpp
  # src/path_following_mode.cpp
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

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::ros2::PathFollowingComponent"
  EXECUTABLE path_following_node
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)
endif()

ament_package()

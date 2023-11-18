cmake_minimum_required(VERSION 3.8)
project(srs_feetech_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(sample1
  src/test_node.cpp)
target_include_directories(sample1
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sample1
  "rclcpp"
  "geometry_msgs"
)
install(
  TARGETS sample1
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pan_tilt_node
  src/pan_tilt/pan_tilt_node.cpp)
target_include_directories(pan_tilt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pan_tilt_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)
install(
  TARGETS pan_tilt_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(joy_to_cmd_rate
  src/joy_to_cmd_rate/joy_to_cmd_rate_node.cpp)
ament_target_dependencies(joy_to_cmd_rate
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
)
install(
  TARGETS joy_to_cmd_rate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

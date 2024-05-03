cmake_minimum_required(VERSION 3.8)
project(abu_core)


if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

add_library(RobotControl
  src/ManualControl.cpp
)

add_library(tb3_autonomy
  src/autonomy_node.cpp
  src/navigation_behaviors.cpp
)

# ament_python_install_package(
#   tb3_behaviors
#   PACKAGE_DIR python/tb3_behaviors
# )

ament_target_dependencies(RobotControl rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(tb3_autonomy rclcpp behaviortree_cpp_v3 rclcpp_action nav2_msgs tf2 tf2_geometry_msgs ament_index_cpp yaml-cpp)

add_executable(pub src/pub.cpp)
add_executable(joy_drive src/joy_drive.cpp)
# add_executable(detection src/detection.cpp)
add_executable(autonomy_node src/autonomy_node.cpp)
target_link_libraries(joy_drive RobotControl)
target_link_libraries(autonomy_node tb3_autonomy ${YAML_CPP_LIBRARIES})
ament_target_dependencies(pub rclcpp std_msgs)
ament_target_dependencies(autonomy_node)
ament_target_dependencies(joy_drive rclcpp sensor_msgs geometry_msgs)
# ament_target_dependencies(detection rclcpp sensor_msgs geometry_msgs OpenCV cv_bridge)

install(TARGETS
  pub
  joy_drive
  # detection
  autonomy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/imu.py
  scripts/robot_state.py
  scripts/lidar_filter.py
  scripts/nav2_control.py
  scripts/ball_detection.py
  scripts/robot_main_state.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

# install(
#   DIRECTORY python/tb3_behaviors/
#   DESTINATION lib/${PROJECT_NAME}/tb3_behaviors
# )

install(
  DIRECTORY config launch bt_xml rviz
  DESTINATION share/${PROJECT_NAME}
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

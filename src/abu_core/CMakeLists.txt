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
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

add_library(RobotControl
  src/ManualControl.cpp
)

ament_target_dependencies(RobotControl rclcpp sensor_msgs geometry_msgs)

add_executable(pub src/pub.cpp)
add_executable(joy_drive src/joy_drive.cpp)
add_executable(detection src/detection.cpp)
target_link_libraries(joy_drive RobotControl)
ament_target_dependencies(pub rclcpp std_msgs)
ament_target_dependencies(joy_drive rclcpp sensor_msgs geometry_msgs)
ament_target_dependencies(detection rclcpp sensor_msgs geometry_msgs OpenCV cv_bridge)

install(TARGETS
  pub
  joy_drive
  detection
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
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

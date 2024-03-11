cmake_minimum_required(VERSION 3.8)
project(abu_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
add_executable(mtc_tutorial src/mtc_tutorial.cpp)
add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(mtc_tutorial moveit_task_constructor_core rclcpp)
ament_target_dependencies(hello_moveit moveit_task_constructor_core rclcpp)

install(TARGETS
  talker
  mtc_tutorial
  hello_moveit
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME})

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

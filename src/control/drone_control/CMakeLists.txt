cmake_minimum_required(VERSION 3.8)
project(drone_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPositionTarget.srv"
  "srv/SetControlMode.srv"
)

add_executable(drone_controller
  src/drone_controller.cpp
  src/utils.cpp
  src/rate_pid.cpp
  src/mixer.cpp
)

ament_target_dependencies(drone_controller
  actuator_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

rosidl_target_interfaces(drone_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(drone_controller PRIVATE
  include
)

install(PROGRAMS
  scripts/joy_gui_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joy_gui_publisher
)

install(TARGETS drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

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

cmake_minimum_required(VERSION 3.8)
project(group_a_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

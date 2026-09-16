cmake_minimum_required(VERSION 3.22)
project(mini_servo_model)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/shutdown_wrapper.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()

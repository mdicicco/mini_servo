cmake_minimum_required(VERSION 3.22)
project(mini_servo_firmware)

find_package(ament_cmake REQUIRED)

install(FILES mini_servo.ino DESTINATION share/${PROJECT_NAME}/arduino)

ament_package()

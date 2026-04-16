cmake_minimum_required(VERSION 3.8)
project(beach_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

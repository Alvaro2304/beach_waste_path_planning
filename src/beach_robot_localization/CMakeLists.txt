cmake_minimum_required(VERSION 3.8)
project(beach_robot_localization)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Install configuration files and launch files into the share directory
# so they can be located at runtime via get_package_share_directory().
# ---------------------------------------------------------------------------
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

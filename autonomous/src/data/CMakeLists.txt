cmake_minimum_required(VERSION 3.8)
project(data)

find_package(ament_cmake REQUIRED)

# Data-only package: nothing is compiled, the directories are just installed
# into share/data so they can be found by get_package_share_directory('data').
install(DIRECTORY maps waypoints DESTINATION share/${PROJECT_NAME})

ament_package()

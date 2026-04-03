cmake_minimum_required(VERSION 3.8)
project(rfidbot_tags_localization)

# Default to C++14 (safe even if you only use Python)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

################################################
## Find dependencies (ROS 2 / ament)
################################################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

################################################
## Python package setup
################################################

# This installs the Python module:
# rfidbot_tags_localization/
#   __init__.py
#   rfh_tags_localization_main.py
ament_python_install_package(${PROJECT_NAME})

################################################
## Install Python executables (nodes)
################################################

#install(PROGRAMS
#  rfidbot_tags_localization/rfh_tags_localization_main_ros2.py
#  DESTINATION lib/${PROJECT_NAME}
#)

################################################
## Install launch files (XML)
################################################

#install(
#  DIRECTORY rfidbot_tags_localization
#  DESTINATION lib/${PROJECT_NAME}
#)

#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

# Optionally install packaged data files (antenna models, etc.) if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
  install(
    DIRECTORY data
    DESTINATION share/${PROJECT_NAME}
  )
endif()

################################################
## Export dependencies
################################################

ament_export_dependencies(
  rclcpp
  rclpy
  std_msgs
)

ament_package()

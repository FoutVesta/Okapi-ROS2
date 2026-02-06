cmake_minimum_required(VERSION 3.8)
project(rviz_panel)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------
# ROS2: ament instead of catkin
# ---------------------------
find_package(ament_cmake REQUIRED)

# Keep your dependency block conceptually similar, but ROS2 packages differ.
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
# Other dependecies
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rfidbot_tags_reader REQUIRED)
find_package(rfidbot_tags_interfaces REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
# ROS2 RViz uses Qt5
message(STATUS "Using Qt5 (ROS2 RViz2)")
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)

# Avoid keyword definition to avaid conflicts with boost or xapian etc
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
add_definitions(-DQT_NO_KEYWORDS)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/rviz_panel.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/rviz_panel.hpp
  ../rfidbot_tags_reader/include/rfidbot_tags_reader/rfidbot_tags_reader_main.h
)

# Define ui file
set(${PROJECT_NAME}_UIS
  resource/simple_panel.ui
)

# Create header from ui file (uic)
message(STATUS "Generate header for ui with Qt5")
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})

## Add library is needed in order to generate the header file from ui file.
add_library(simple_panel SHARED
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS}
)

# ROS2: Use ament_target_dependencies instead of catkin_LIBRARIES
ament_target_dependencies(simple_panel
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  std_msgs
  ament_index_cpp
  rfidbot_tags_reader
  rfidbot_tags_interfaces
)

target_link_libraries(simple_panel
  ${QT_LIBRARIES}
)

set_target_properties(simple_panel PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(simple_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ROS2 pluginlib export (your tree shows rviz_plugin.xml)
pluginlib_export_plugin_description_file(rviz_common rviz_plugin.xml)

# Install library + headers + plugin xml + resources
install(TARGETS simple_panel
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  rviz_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  std_msgs
)

ament_package()


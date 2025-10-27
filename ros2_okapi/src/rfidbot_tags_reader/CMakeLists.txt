cmake_minimum_required(VERSION 3.5)
project(rfidbot_tags_reader)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_library(EXT_LIBRARY libltkcpp.a
  /home/rfid/okapi_realsense_ws/src/rfidbot_tags_reader/Library)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${rclpy_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${rclpy_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
set(LIBS ${ament_cmake_LIBRARIES} ${rclpy_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
rosidl_generate_interfaces(${PROJECT_NAME} "msg/tagReader.msg"
  DEPENDENCIES builtin_interfaces std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder


## Generate added messages and services with any dependencies listed here

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library

#FIND_LIBRARY(EXT_LIBRARY libltkcpp.a /home/rfid/catkin_ws/src/rfidbot_tags_reader/Library)

#add_library(rfidbot_rfid_tags_reader src/rfidbot_rfid_tags_reader.cpp)
#target_link_libraries(rfidbot_rfid_tags_reader ${EXT_LIBRARY} ${catkin_LIBRARIES})

## Declare a cpp executable
#add_executable(rfidbot_tags_reader_node src/rfidbot_tags_reader_main.cpp)
#target_link_libraries(rfidbot_tags_reader_node ${EXT_LIBRARY} ${catkin_LIBRARIES})
#set_target_properties(rfidbot_tags_reader_node PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(rfidbot_tags_reader_node rfidbot_tags_reader_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(rfidbot_tags_reader_node
#  ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION bin
# )

## Mark executables and/or libraries for installation
# install(TARGETS rfidbot_tags_reader rfidbot_tags_reader_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rfidbot_tags_reader.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclpy)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()

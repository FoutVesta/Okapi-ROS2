cmake_minimum_required(VERSION 3.8)
project(rfidbot_tags_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Find dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ------------------------------------------------------------------------------
# Generate service interfaces
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TagReader.msg"
  "srv/TagLocalizing.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# ------------------------------------------------------------------------------
# Testing (optional)
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Export and package
# ------------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()



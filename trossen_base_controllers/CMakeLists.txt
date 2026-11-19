cmake_minimum_required(VERSION 3.10.0)
project(trossen_base_controllers)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

set(ROS_DEPENDENCIES
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/holonomic.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${ROS_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}.xml)

install(
  DIRECTORY
    include/
  DESTINATION
    include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

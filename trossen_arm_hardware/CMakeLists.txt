cmake_minimum_required(VERSION 3.5)
project(trossen_arm_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)

find_package(trossen_arm_driver REQUIRED)

set(ROS_DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle

  trossen_arm_driver
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${ROS_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}.xml)

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

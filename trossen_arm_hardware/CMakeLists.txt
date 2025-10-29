cmake_minimum_required(VERSION 3.10.0)
project(trossen_arm_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(libtrossen_arm REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(ROS_DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
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

target_link_libraries(${PROJECT_NAME}
  libtrossen_arm
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
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS_DEPENDENCIES})
ament_export_dependencies(libtrossen_arm)
ament_package()

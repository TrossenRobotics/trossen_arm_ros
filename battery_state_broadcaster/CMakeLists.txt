cmake_minimum_required(VERSION 3.10)
project(battery_state_broadcaster)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(battery_state_broadcaster_parameters
  src/battery_state_broadcaster_parameters.yaml
)

add_library(
  battery_state_broadcaster
  SHARED
  src/battery_state_broadcaster.cpp
)

target_compile_features(battery_state_broadcaster PUBLIC cxx_std_17)
target_include_directories(battery_state_broadcaster
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/battery_state_broadcaster>
)
target_link_libraries(battery_state_broadcaster PUBLIC
                      battery_state_broadcaster_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      rcpputils::rcpputils
                      realtime_tools::realtime_tools
                      ${control_msgs_TARGETS}
                      ${sensor_msgs_TARGETS}
                      ${builtin_interfaces_TARGETS})


pluginlib_export_plugin_description_file(
  controller_interface battery_state_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include/battery_state_broadcaster
)
install(
  TARGETS
    battery_state_broadcaster
    battery_state_broadcaster_parameters
  EXPORT export_battery_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_battery_state_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

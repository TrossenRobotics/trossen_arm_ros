cmake_minimum_required(VERSION 3.10.0)
project(trossen_arm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    demos/controllers.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

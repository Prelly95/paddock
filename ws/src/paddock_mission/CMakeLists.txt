cmake_minimum_required(VERSION 3.8)
project(paddock_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mission_node src/mission_node.cpp)
target_compile_features(mission_node PUBLIC cxx_std_17)
ament_target_dependencies(mission_node rclcpp px4_msgs std_msgs)

install(TARGETS mission_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No copyright headers or license file in this portfolio repo yet.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

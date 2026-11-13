cmake_minimum_required(VERSION 3.8)
project(nav_homework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

add_executable(nav_node src/nav_node_rviz.cpp)
ament_target_dependencies(nav_node
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_costmap_2d
  nav2_util
)
install(TARGETS
  nav_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
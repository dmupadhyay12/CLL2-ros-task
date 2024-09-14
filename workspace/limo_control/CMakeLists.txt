cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(limo_control_node src/limo_control_node.cpp)
ament_target_dependencies(limo_control_node rclcpp nav_msgs tf2_geometry_msgs)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS limo_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
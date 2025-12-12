cmake_minimum_required(VERSION 3.8)

project(figure8_path_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(figure8_path_publisher_node
  src/figure8_path_publisher_node.cpp
)

ament_target_dependencies(figure8_path_publisher_node
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS figure8_path_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


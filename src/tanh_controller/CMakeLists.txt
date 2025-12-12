cmake_minimum_required(VERSION 3.8)

project(tanh_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(tanh_controller_core
  src/tanh_controller.cpp
)

ament_target_dependencies(tanh_controller_core
  rclcpp
  px4_msgs
  tf2
  tf2_eigen
)
target_link_libraries(tanh_controller_core Eigen3::Eigen)

add_executable(tanh_controller_node
  src/tanh_controller_node.cpp
)

target_link_libraries(tanh_controller_node tanh_controller_core)
ament_target_dependencies(tanh_controller_node
  rclcpp
  px4_msgs
  tf2
  tf2_eigen
)

install(TARGETS tanh_controller_core tanh_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


cmake_minimum_required(VERSION 3.8)
project(am_jparse_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(jparse_velocity_controller src/jparse_velocity_controller.cpp)
ament_target_dependencies(jparse_velocity_controller
  geometry_msgs
  kdl_parser
  orocos_kdl
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(jparse_velocity_controller Eigen3::Eigen)

install(TARGETS jparse_velocity_controller DESTINATION lib/am_jparse_controller)
install(DIRECTORY launch DESTINATION share/am_jparse_controller)

ament_package()

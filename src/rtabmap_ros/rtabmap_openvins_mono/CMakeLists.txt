cmake_minimum_required(VERSION 3.8)
project(rtabmap_openvins_mono)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(RTABMap REQUIRED COMPONENTS core)

add_executable(mono_odometry
  src/mono_odometry_node.cpp
)

target_compile_features(mono_odometry PUBLIC cxx_std_17)

ament_target_dependencies(mono_odometry
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  message_filters
)

target_link_libraries(mono_odometry
  rtabmap::core
  ${OpenCV_LIBS}
)

install(TARGETS mono_odometry
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

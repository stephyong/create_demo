cmake_minimum_required(VERSION 3.8)
project(ur16e_handover)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(ur_msgs REQUIRED)       # ADDITION 1
find_package(Eigen3 REQUIRED)

add_executable(handover src/handover.cpp)
ament_target_dependencies(handover
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  ur_msgs
)

install(TARGETS handover
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

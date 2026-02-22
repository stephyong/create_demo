cmake_minimum_required(VERSION 3.8)
project(ur16e_handover)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# REMOVED: find_package(shape_msgs REQUIRED) — no longer used

add_executable(handover_demo src/handover.cpp)
ament_target_dependencies(handover_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  ur_msgs
)

target_link_libraries(handover_demo
  Eigen3::Eigen
)

install(TARGETS handover_demo
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config            # ADDED — installs your yaml params file
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
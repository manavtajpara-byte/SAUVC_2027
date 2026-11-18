cmake_minimum_required(VERSION 3.8)
project(sauvc_arena_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(sauvc_arena_sim_node src/sauvc_arena_sim_node.cpp)
ament_target_dependencies(sauvc_arena_sim_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

add_executable(sauvc_mission_controller_node src/sauvc_mission_controller_node.cpp)
ament_target_dependencies(sauvc_mission_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  gazebo_msgs
  tf2
  tf2_ros
)

install(TARGETS
  sauvc_arena_sim_node
  sauvc_mission_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_control_panel.py
  scripts/auto_train_evaluator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  rviz
  worlds
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

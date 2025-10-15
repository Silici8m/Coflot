cmake_minimum_required(VERSION 3.8)
project(my_fleet_manager)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Génération des messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Mission.msg"
  "msg/Tache.msg"
  DEPENDENCIES geometry_msgs
)

install(PROGRAMS
  src/mission_to_task_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

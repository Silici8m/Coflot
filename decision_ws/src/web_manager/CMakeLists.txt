cmake_minimum_required(VERSION 3.8)
project(web_manager)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(PROGRAMS
  src/web_reception.py
  src/web_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(
  DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

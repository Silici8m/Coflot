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
  DESTINATION share/${PROJECT_NAME}/
)

# Conversion automatique du fichier map.pgm en map.png lors du build
find_program(CONVERT_BIN convert)
if(CONVERT_BIN)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/web/map/map.png
    COMMAND ${CONVERT_BIN} ${CMAKE_CURRENT_SOURCE_DIR}/web/map/map.pgm ${CMAKE_CURRENT_SOURCE_DIR}/web/map/map.png
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/web/map/map.pgm
    COMMENT "Conversion de map.pgm en map.png"
  )
  add_custom_target(convert_map ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/web/map/map.png)
endif()

ament_package()

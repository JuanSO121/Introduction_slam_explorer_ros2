cmake_minimum_required(VERSION 3.8)
project(tutorial_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Instalar módulo Python
ament_python_install_package(${PROJECT_NAME})

# NO instalar los archivos Python manualmente aquí - setup.py lo maneja
# ELIMINAR estas líneas que tienes actualmente:
# install(PROGRAMS tutorial_pkg/ai_voice_commander.py ...)

# Solo instalar directorios de datos
install(DIRECTORY
  launch
  config
  maps
  models
  rviz
  worlds
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
cmake_minimum_required(VERSION 3.8)
project(tutorial_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Instalar módulo Python (carpeta tutorial_pkg/)
ament_python_install_package(${PROJECT_NAME})

# Instalar scripts Python opcionales (si no van en console_scripts)
install(PROGRAMS
  tutorial_pkg/exploration_monitor.py
  tutorial_pkg/exploration_restarter.py
  tutorial_pkg/exploration_visualizer.py
  # Añade aquí otros nodos .py que quieras ejecutar directamente
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar carpetas de datos y configuración
install(DIRECTORY
  config
  launch
  maps
  models
  rviz
  worlds
  scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

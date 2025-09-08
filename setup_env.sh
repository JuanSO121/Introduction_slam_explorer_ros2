#!/bin/bash
# Configuración de entorno para tutorial_pkg
# Source este archivo antes de ejecutar los launch files

# Variables de TurtleBot3
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/tutorial_pkg/models

# Variables de exploración
export TUTORIAL_PKG_CONFIG_DIR=~/ros2_ws/src/tutorial_pkg/config
export TUTORIAL_PKG_MAPS_DIR=~/ros2_ws/src/tutorial_pkg/maps
export TUTORIAL_PKG_LOGS_DIR=~/ros2_ws/src/tutorial_pkg/logs

# Variables de servicios de voz (opcional)
export FLASK_PORT=8000
export VOICE_SERVICES_DIR=~/ros2_ws/src/tutorial_pkg/voice_services

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "✅ Entorno tutorial_pkg configurado"
echo "🤖 TurtleBot3 Model: $TURTLEBOT3_MODEL" 
echo "📁 Config Dir: $TUTORIAL_PKG_CONFIG_DIR"
echo "🗺️ Maps Dir: $TUTORIAL_PKG_MAPS_DIR"

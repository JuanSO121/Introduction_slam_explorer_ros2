#!/bin/bash
# Script para iniciar tutorial_pkg con integración Flutter
# Ubicación: ~/ros2_ws/src/tutorial_pkg/start_flutter_integration.sh

echo "🚀 Iniciando tutorial_pkg con integración Flutter..."

# Source ROS2 y workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Obtener IP de WSL2 si estamos en WSL
if grep -qi microsoft /proc/version; then
    WSL2_IP=$(hostname -I | awk '{print $1}')
    echo "🌐 WSL2 IP: $WSL2_IP"
    echo "📱 URLs para Flutter:"
    echo "   HTTP: http://$WSL2_IP:8000"
    echo "   WebSocket: ws://$WSL2_IP:8000"
    echo "   Rosbridge: ws://$WSL2_IP:9090"
else
    echo "🌐 URLs para Flutter:"
    echo "   HTTP: http://localhost:8000"
    echo "   WebSocket: ws://localhost:8000"
    echo "   Rosbridge: ws://localhost:9090"
fi

echo ""
echo "🎯 Iniciando sistema completo..."

# Lanzar el sistema completo con Flutter Bridge
ros2 launch tutorial_pkg flutter_integration_launch.py

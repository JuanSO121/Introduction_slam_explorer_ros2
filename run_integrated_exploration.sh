#!/bin/bash
# Script de ejecución integrada para tutorial_pkg

# Configurar entorno
source ~/ros2_ws/src/tutorial_pkg/setup_env.sh

# Mostrar información
echo "🚀 Iniciando sistema integrado tutorial_pkg"
echo "🤖 Robot: TurtleBot3 Waffle"
echo "🗺️ Modo: Exploración autónoma con control por voz"
echo ""

# Opciones de ejecución
echo "Seleccione modo de ejecución:"
echo "1) Solo exploración (sin control por voz)"
echo "2) Exploración + Control por voz"
echo "3) Exploración + Control por voz + Servidor Flask"
read -p "Opción [1]: " option

case $option in
    2)
        echo "🎤 Iniciando con control por voz..."
        ros2 launch tutorial_pkg integrated_exploration_launch.py enable_voice_control:=true
        ;;
    3)
        echo "🎤📱 Iniciando con control por voz y servidor Flask..."
        ros2 launch tutorial_pkg integrated_exploration_launch.py enable_voice_control:=true enable_flask_server:=true
        ;;
    *)
        echo "🗺️ Iniciando solo exploración..."
        ros2 launch tutorial_pkg integrated_exploration_launch.py enable_voice_control:=false
        ;;
esac

#!/bin/bash
# Script para probar el sistema de voz sin exploración completa
# Uso: ./scripts/test_voice_system.sh

echo "🧪 Probando Sistema de Control por Voz"

# Verificar directorio
if [[ ! -f "package.xml" ]] || ! grep -q "tutorial_pkg" package.xml; then
    echo "❌ Error: Ejecutar desde directorio tutorial_pkg"
    exit 1
fi

# Configurar ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Solo lanzar nodos de voz
echo "🎤 Iniciando solo el sistema de voz..."
ros2 launch tutorial_pkg voice_control_launch.py use_sim_time:=false &
VOICE_PID=$!

sleep 3

# Probar comando manual
echo "📝 Probando comando manual..."
ros2 topic pub --once /voice_commands std_msgs/String "data: 'hola robot'"

sleep 2

# Mostrar tópicos activos
echo "📡 Tópicos activos:"
ros2 topic list | grep -E "(voice|ai)"

echo ""
echo "🧪 Sistema de prueba activo"
echo "   Probar con: ros2 topic pub /voice_commands std_msgs/String \"data: 'adelante'\""
echo "   Ver feedback: ros2 topic echo /voice_feedback"
echo ""
echo "⚠️  Presiona Ctrl+C para detener"

trap "kill $VOICE_PID 2>/dev/null; echo '👋 Prueba detenida'" SIGINT
wait

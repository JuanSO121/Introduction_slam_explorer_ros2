#!/bin/bash
# Script para iniciar control por voz en tutorial_pkg
# Uso: ./scripts/start_voice_control.sh

echo "🚀 Iniciando Control por Voz para tutorial_pkg"

# Verificar que estamos en tutorial_pkg
if [[ ! -f "package.xml" ]] || ! grep -q "tutorial_pkg" package.xml; then
    echo "❌ Error: Ejecutar desde directorio tutorial_pkg"
    exit 1
fi

# Configurar ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    if [ -d "~/ros2_ws" ]; then
        source ~/ros2_ws/install/setup.bash
    fi
else
    echo "⚠️ ROS2 no encontrado"
    exit 1
fi

# Lanzar sistema de exploración con control por voz
echo "🤖 Iniciando sistema de exploración TurtleBot3..."
ros2 launch tutorial_pkg explore_robust_fixed.launch.py &
EXPLORATION_PID=$!

# Esperar 15 segundos para que el sistema se inicialice
sleep 15

# Lanzar control por voz
echo "🎤 Iniciando control por voz IA..."
ros2 launch tutorial_pkg voice_control_launch.py &
VOICE_PID=$!

# Esperar 5 segundos más
sleep 5

# Iniciar servidor Flask (en directorio voice_services)
echo "🌐 Iniciando servidor Flask para Flutter..."
cd voice_services
source venv/bin/activate
python3 flask_server.py &
FLASK_PID=$!

echo ""
echo "✅ Servicios iniciados:"
echo "   - Sistema de exploración: PID $EXPLORATION_PID"
echo "   - Control por voz: PID $VOICE_PID"
echo "   - Servidor Flask: PID $FLASK_PID"
echo ""
echo "🌐 URLs disponibles:"
WSL_IP=$(hostname -I | awk '{print $1}')
echo "   - Flutter HTTP: http://${WSL_IP}:8000"
echo "   - Flutter WebSocket: ws://${WSL_IP}:8000"
echo ""
echo "🎤 Comandos de voz disponibles:"
echo "   - Movimiento: adelante, atrás, izquierda, derecha, parar"
echo "   - Exploración: explorar, pausar exploración, continuar, terminar"
echo "   - Estado: estado, estadísticas, progreso del mapa"
echo ""
echo "⚠️  Presiona Ctrl+C para detener todos los servicios"

# Función de limpieza
cleanup() {
    echo ""
    echo "🛑 Deteniendo servicios..."
    kill $EXPLORATION_PID 2>/dev/null || true
    kill $VOICE_PID 2>/dev/null || true
    kill $FLASK_PID 2>/dev/null || true
    sleep 2
    echo "👋 Servicios detenidos"
    exit 0
}

trap cleanup SIGINT
wait

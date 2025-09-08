#!/bin/bash
# Script de prueba del sistema tutorial_pkg

source ~/ros2_ws/src/tutorial_pkg/setup_env.sh

echo "🧪 PRUEBAS DEL SISTEMA TUTORIAL_PKG"
echo "=================================="

# Test 1: Verificar ROS2
echo -n "ROS2 disponible: "
if command -v ros2 &> /dev/null; then
    echo "✅"
else
    echo "❌"
fi

# Test 2: Verificar paquete compilado
echo -n "Paquete tutorial_pkg: "
if ros2 pkg list | grep -q "tutorial_pkg"; then
    echo "✅"
else
    echo "❌"
fi

# Test 3: Verificar archivos de configuración
echo -n "Archivos config: "
if [ -f "config/slam_simple.yaml" ] && [ -f "config/navigation_simple.yaml" ]; then
    echo "✅"
else
    echo "❌"
fi

# Test 4: Verificar launch files
echo -n "Launch files: "
if [ -f "launch/integrated_exploration_launch.py" ]; then
    echo "✅"
else
    echo "❌"
fi

# Test 5: Verificar nodos Python
echo -n "Nodos Python: "
if [ -f "tutorial_pkg/ai_voice_commander.py" ] && [ -f "tutorial_pkg/ai_response_node.py" ]; then
    echo "✅"
else
    echo "❌"
fi

# Test 6: Test de tópicos (requiere roscore)
echo "🔍 Probando sistema básico..."
timeout 10s ros2 topic list > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ ROS2 DDS funcionando"
else
    echo "⚠️ ROS2 DDS no responde (normal si no hay nodos corriendo)"
fi

echo ""
echo "📋 Para ejecutar el sistema completo:"
echo "./run_integrated_exploration.sh"
echo ""
echo "📋 Para ejecutar solo partes específicas:"
echo "ros2 launch tutorial_pkg integrated_exploration_launch.py"

#!/bin/bash

# Script para arreglar la integración IA en tutorial_pkg
# Ubicación: ~/ros2_ws/src/tutorial_pkg/fix_ai_integration.sh

echo "🔧 Arreglando integración IA en tutorial_pkg"
echo "============================================="

# Ir al directorio del paquete
cd ~/ros2_ws/src/tutorial_pkg

# 1. Verificar archivos necesarios
echo "📁 Verificando archivos necesarios..."

if [ ! -f "tutorial_pkg/__init__.py" ]; then
    echo "❌ Falta tutorial_pkg/__init__.py - creando..."
    # El contenido del __init__.py está en el artifact anterior
    touch tutorial_pkg/__init__.py
else
    echo "✅ __init__.py existe"
fi

if [ ! -f "tutorial_pkg/ai_response_node.py" ]; then
    echo "❌ Falta ai_response_node.py"
    exit 1
else
    echo "✅ ai_response_node.py existe"
fi

if [ ! -f "tutorial_pkg/ai_voice_commander.py" ]; then
    echo "❌ Falta ai_voice_commander.py"
    exit 1
else
    echo "✅ ai_voice_commander.py existe"
fi

# 2. Verificar que los archivos sean ejecutables
echo "🔒 Configurando permisos..."
chmod +x tutorial_pkg/ai_response_node.py
chmod +x tutorial_pkg/ai_voice_commander.py
chmod +x tutorial_pkg/voice_command_handler.py 2>/dev/null || true

# 3. Limpiar instalación anterior
echo "🧹 Limpiando instalación anterior..."
cd ~/ros2_ws
rm -rf build/tutorial_pkg install/tutorial_pkg 2>/dev/null || true

# 4. Reconstruir el paquete
echo "🔨 Reconstruyendo paquete..."
colcon build --packages-select tutorial_pkg --symlink-install

# 5. Verificar la instalación
echo "✅ Verificando instalación..."
if [ -d "install/tutorial_pkg/lib/tutorial_pkg" ]; then
    echo "✅ Directorio lib creado correctamente"
    ls -la install/tutorial_pkg/lib/tutorial_pkg/
else
    echo "❌ Error: No se creó el directorio lib"
    exit 1
fi

# 6. Source el workspace
echo "🔄 Actualizando workspace..."
source install/setup.bash

# 7. Probar los ejecutables
echo "🧪 Probando ejecutables..."

echo "Probando ai_response_node..."
ros2 pkg executables tutorial_pkg | grep ai_response_node
if [ $? -eq 0 ]; then
    echo "✅ ai_response_node disponible"
else
    echo "❌ ai_response_node NO disponible"
fi

echo "Probando ai_voice_commander..."
ros2 pkg executables tutorial_pkg | grep ai_voice_commander
if [ $? -eq 0 ]; then
    echo "✅ ai_voice_commander disponible"
else
    echo "❌ ai_voice_commander NO disponible"
fi

# 8. Mostrar todos los ejecutables disponibles
echo "📋 Ejecutables disponibles en tutorial_pkg:"
ros2 pkg executables tutorial_pkg

echo ""
echo "✅ Proceso completado!"
echo "Ahora puedes usar:"
echo "  ros2 run tutorial_pkg ai_response_node"
echo "  ros2 run tutorial_pkg ai_voice_commander"
echo ""
echo "Para probar el sistema completo:"
echo "  ./scripts/test_voice_system.sh"
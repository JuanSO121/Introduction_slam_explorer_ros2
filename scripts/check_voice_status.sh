#!/bin/bash
# Script para verificar estado del sistema de voz
# Uso: ./scripts/check_voice_status.sh

echo "🔍 Verificando Sistema de Control por Voz"
echo "========================================"

# ROS2
echo "🤖 ROS2:"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "  ✅ ROS2 Humble disponible"
    
    if ros2 node list >/dev/null 2>&1; then
        echo "  ✅ ROS2 daemon corriendo"
        
        # Verificar nodos de voz
        if ros2 node list | grep -q "ai_voice_commander"; then
            echo "  ✅ AI Voice Commander activo"
        else
            echo "  ❌ AI Voice Commander no activo"
        fi
        
        if ros2 node list | grep -q "ai_response_node"; then
            echo "  ✅ AI Response Node activo"
        else
            echo "  ❌ AI Response Node no activo"
        fi
    else
        echo "  ⚠️ ROS2 daemon no está corriendo"
    fi
else
    echo "  ❌ ROS2 no encontrado"
fi

echo ""
echo "🐍 Python (voice_services):"
if [ -d "voice_services/venv" ]; then
    cd voice_services
    source venv/bin/activate
    echo "  ✅ Entorno virtual encontrado"
    
    python3 -c "import flask; print('  ✅ Flask OK')" 2>/dev/null || echo "  ❌ Flask falta"
    python3 -c "import torch; print('  ✅ PyTorch OK')" 2>/dev/null || echo "  ❌ PyTorch falta"
    python3 -c "from transformers import WhisperProcessor; print('  ✅ Whisper OK')" 2>/dev/null || echo "  ❌ Whisper falta"
    
    cd ..
else
    echo "  ❌ Entorno virtual no encontrado en voice_services/"
fi

echo ""
echo "📡 Tópicos ROS2:"
if ros2 topic list >/dev/null 2>&1; then
    echo "  Tópicos de voz activos:"
    ros2 topic list | grep -E "(voice|ai)" | sed 's/^/    /'
else
    echo "  ⚠️ No se pueden listar tópicos"
fi

echo ""
echo "🌐 Red:"
WSL_IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "127.0.0.1")
echo "  IP local: $WSL_IP"
echo "  URLs Flutter:"
echo "    HTTP: http://$WSL_IP:8000"
echo "    WebSocket: ws://$WSL_IP:8000"

echo ""
echo "📋 Comandos útiles:"
echo "  Iniciar todo: ./scripts/start_voice_control.sh"
echo "  Solo prueba: ./scripts/test_voice_system.sh"
echo "  Enviar comando: ros2 topic pub /voice_commands std_msgs/String \"data: 'adelante'\""
echo "  Ver respuestas: ros2 topic echo /voice_feedback"

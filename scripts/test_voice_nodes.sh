#!/bin/bash
# Test completo de nodos de voz
echo "🧪 TEST COMPLETO NODOS DE VOZ"
echo "============================"

cd ~/ros2_ws
source install/setup.bash

echo "1. 📋 Nodos disponibles:"
ros2 pkg executables tutorial_pkg | grep -E "(ai_|voice)"

echo ""
echo "2. 🚀 Iniciando ai_voice_commander..."
ros2 run tutorial_pkg ai_voice_commander &
VOICE_PID=$!
sleep 3

echo ""  
echo "3. 🧠 Iniciando ai_response_node..."
ros2 run tutorial_pkg ai_response_node &
RESPONSE_PID=$!
sleep 3

echo ""
echo "4. 📢 Topics activos:"
ros2 topic list | grep -E "(voice|ai)"

echo ""
echo "5. 🎤 Enviando comando de test..."
ros2 topic pub --once /voice_commands std_msgs/msg/String '{data: "hola robot, adelante"}'

sleep 5

echo ""
echo "6. 🛑 Deteniendo nodos..."
kill $VOICE_PID $RESPONSE_PID 2>/dev/null

echo ""
echo "✅ Test completado"

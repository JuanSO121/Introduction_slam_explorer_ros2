#!/bin/bash
# Script para probar la integración Flutter
echo "🧪 Probando Flutter Bridge..."

# Source workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Obtener IP
if grep -qi microsoft /proc/version; then
    IP=$(hostname -I | awk '{print $1}')
else
    IP="localhost"
fi

PORT=8000
BASE_URL="http://$IP:$PORT"

echo "🌐 Probando conexión a $BASE_URL"

# Test health check
echo "1. Testing health check..."
curl -s "$BASE_URL/health" | python3 -m json.tool

echo ""
echo "2. Testing robot status..."
curl -s "$BASE_URL/robot_status" | python3 -m json.tool

echo ""
echo "3. Testing voice command..."
curl -s -X POST "$BASE_URL/send_voice_command" \
     -H "Content-Type: application/json" \
     -d '{"command": "hola robot"}' | python3 -m json.tool

echo ""
echo "✅ Pruebas completadas"

#!/bin/bash
# Script para ejecutar servicios de voz independientemente

VOICE_DIR="$HOME/ros2_ws/src/tutorial_pkg/voice_services"

if [ ! -d "$VOICE_DIR" ]; then
    echo "❌ Directorio voice_services no encontrado"
    exit 1
fi

if [ ! -d "$VOICE_DIR/venv" ]; then
    echo "❌ Entorno virtual no encontrado en $VOICE_DIR/venv"
    echo "💡 Ejecutar setup_integration.sh primero"
    exit 1
fi

cd "$VOICE_DIR"
source venv/bin/activate

echo "🎤 Iniciando servicios de voz..."
echo "🌐 Servidor disponible en: http://localhost:8000"
echo "🔌 WebSocket disponible en: ws://localhost:8000"
echo "⚠️ Presiona Ctrl+C para detener"

python3 flask_server.py

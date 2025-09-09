#!/bin/bash
# Script de instalación Flutter Bridge para tutorial_pkg
# Ubicación: ~/ros2_ws/src/tutorial_pkg/install_flutter_bridge.sh

echo "🚀 Instalando Flutter Bridge para tutorial_pkg"
echo "=============================================="

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Verificar directorio actual
if [[ ! -d "tutorial_pkg" && ! -f "package.xml" ]]; then
    echo -e "${RED}❌ Ejecuta este script desde ~/ros2_ws/src/tutorial_pkg/${NC}"
    exit 1
fi

echo -e "${BLUE}📍 Directorio actual: $(pwd)${NC}"

# === 1. CREAR ESTRUCTURA DE DIRECTORIOS ===
echo -e "${YELLOW}📁 Creando estructura de directorios...${NC}"

mkdir -p config
mkdir -p launch
mkdir -p tutorial_pkg  # asegurar que existe el directorio del paquete

echo -e "${GREEN}✅ Directorios creados${NC}"

# === 2. INSTALAR DEPENDENCIAS PYTHON ===
echo -e "${YELLOW}📦 Instalando dependencias Python para Flutter Bridge...${NC}"

pip3 install --user \
    flask>=2.0.0 \
    flask-cors>=4.0.0 \
    flask-socketio>=5.0.0 \
    requests>=2.25.0 \
    werkzeug>=2.0.0 \
    eventlet>=0.33.0 \
    python-socketio>=5.0.0

echo -e "${GREEN}✅ Dependencias Python instaladas${NC}"

# === 3. CREAR ARCHIVO FLUTTER_BRIDGE_NODE.PY ===
echo -e "${YELLOW}📝 Creando flutter_bridge_node.py...${NC}"

# El contenido del archivo ya está en el artefact anterior
# Aquí solo verificamos si existe y lo creamos si es necesario

if [ ! -f "tutorial_pkg/flutter_bridge_node.py" ]; then
    echo -e "${BLUE}Copiando flutter_bridge_node.py...${NC}"
    # El usuario debe copiar el contenido del artefact anterior
    echo -e "${YELLOW}⚠️ ACCIÓN REQUERIDA: Copia el contenido del flutter_bridge_node.py del artefact anterior${NC}"
else
    echo -e "${GREEN}✅ flutter_bridge_node.py ya existe${NC}"
fi

# === 4. CREAR CONFIGURACIÓN ===
echo -e "${YELLOW}⚙️ Creando archivo de configuración...${NC}"

cat > config/flutter_bridge_config.yaml << 'EOF'
# Configuración para Flutter Bridge Node
# Ubicación: ~/ros2_ws/src/tutorial_pkg/config/flutter_bridge_config.yaml

flutter_bridge_node:
  ros__parameters:
    # Configuración del servidor Flask
    flask_port: 8000
    flask_host: "0.0.0.0"
    
    # Características habilitadas
    enable_websocket: true
    enable_voice_processing: true
    enable_ai_integration: true
    
    # Configuración de procesamiento de voz
    voice_config:
      whisper_model: "openai/whisper-small"
      language: "es"
      max_audio_duration: 30
      audio_sample_rate: 16000
    
    # Configuración de IA
    ai_config:
      gemini_api_key: "AIzaSyABBoS19JdslBti21hcRT3tT7ASrc8EWsE"
      max_response_length: 200
      context_memory: 5
    
    # Configuración de seguridad
    security_config:
      max_file_size_mb: 10
      allowed_audio_formats: ["wav", "mp3", "m4a", "ogg"]
      rate_limit_requests_per_minute: 30
    
    # Configuración específica de tutorial_pkg
    tutorial_pkg_integration:
      subscribed_topics:
        - "/voice_feedback"
        - "/ai_status" 
        - "/robot_state"
        - "/cmd_vel"
        - "/map"
        - "/exploration_control"
      
      published_topics:
        - "/voice_commands"
        - "/exploration_control"
      
      valid_exploration_commands:
        - "start_exploration"
        - "pause_exploration"
        - "resume_exploration"
        - "finish_exploration"
        - "emergency_stop"
EOF

echo -e "${GREEN}✅ Configuración creada${NC}"

# === 5. CREAR LAUNCH FILE ===
echo -e "${YELLOW}🚀 Creando launch file...${NC}"

if [ ! -f "launch/flutter_integration_launch.py" ]; then
    echo -e "${YELLOW}⚠️ ACCIÓN REQUERIDA: Copia el contenido del flutter_integration_launch.py del artefact anterior${NC}"
else
    echo -e "${GREEN}✅ Launch file ya existe${NC}"
fi

# === 6. ACTUALIZAR SETUP.PY ===
echo -e "${YELLOW}📝 Verificando setup.py...${NC}"

if grep -q "flutter_bridge_node" setup.py; then
    echo -e "${GREEN}✅ setup.py ya está actualizado${NC}"
else
    echo -e "${YELLOW}⚠️ ACCIÓN REQUERIDA: Actualiza setup.py con el contenido del artefact anterior${NC}"
    echo -e "${BLUE}   Necesitas agregar 'flutter_bridge_node = tutorial_pkg.flutter_bridge_node:main' en entry_points${NC}"
fi

# === 7. VERIFICAR DEPENDENCIAS ROS2 ===
echo -e "${YELLOW}🔍 Verificando dependencias ROS2...${NC}"

# Verificar rosbridge
if ros2 pkg list | grep -q "rosbridge_server"; then
    echo -e "${GREEN}✅ rosbridge_server disponible${NC}"
else
    echo -e "${RED}❌ rosbridge_server no encontrado${NC}"
    echo -e "${BLUE}   Instalando rosbridge...${NC}"
    sudo apt update && sudo apt install -y ros-humble-rosbridge-suite
fi

# === 8. CONSTRUIR EL PAQUETE ===
echo -e "${YELLOW}🔨 Construyendo paquete tutorial_pkg...${NC}"

cd ~/ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Construir solo tutorial_pkg
colcon build --packages-select tutorial_pkg --symlink-install

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Paquete construido exitosamente${NC}"
    
    # Source el workspace
    source install/setup.bash
    
    echo -e "${BLUE}🧪 Verificando instalación...${NC}"
    
    # Verificar que el ejecutable existe
    if ros2 pkg executables tutorial_pkg | grep -q "flutter_bridge_node"; then
        echo -e "${GREEN}✅ flutter_bridge_node instalado correctamente${NC}"
    else
        echo -e "${RED}❌ flutter_bridge_node no se instaló correctamente${NC}"
    fi
else
    echo -e "${RED}❌ Error construyendo el paquete${NC}"
    exit 1
fi

# === 9. CREAR SCRIPT DE INICIO ===
echo -e "${YELLOW}📝 Creando script de inicio...${NC}"

cat > ~/ros2_ws/src/tutorial_pkg/start_flutter_integration.sh << 'EOF'
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
EOF

chmod +x ~/ros2_ws/src/tutorial_pkg/start_flutter_integration.sh

echo -e "${GREEN}✅ Script de inicio creado${NC}"

# === 10. CREAR SCRIPT DE PRUEBA ===
echo -e "${YELLOW}🧪 Creando script de prueba...${NC}"

cat > ~/ros2_ws/src/tutorial_pkg/test_flutter_bridge.sh << 'EOF'
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
EOF

chmod +x ~/ros2_ws/src/tutorial_pkg/test_flutter_bridge.sh

# === 11. RESUMEN FINAL ===
echo ""
echo -e "${GREEN}✅ ¡Instalación de Flutter Bridge completada!${NC}"
echo ""
echo -e "${BLUE}📋 Próximos pasos:${NC}"
echo "1. Copia el contenido de los artefacts anteriores a los archivos correspondientes"
echo "2. Prueba el sistema: ./start_flutter_integration.sh"
echo "3. Verifica la conexión: ./test_flutter_bridge.sh"
echo ""
echo -e "${BLUE}🔧 Archivos creados:${NC}"
echo "   - config/flutter_bridge_config.yaml"
echo "   - start_flutter_integration.sh"
echo "   - test_flutter_bridge.sh"
echo ""
echo -e "${BLUE}📱 URLs para Flutter:${NC}"
if grep -qi microsoft /proc/version; then
    WSL2_IP=$(hostname -I | awk '{print $1}')
    echo "   HTTP: http://$WSL2_IP:8000"
    echo "   WebSocket: ws://$WSL2_IP:8000"
    echo "   Rosbridge: ws://$WSL2_IP:9090"
else
    echo "   HTTP: http://localhost:8000"
    echo "   WebSocket: ws://localhost:8000"
    echo "   Rosbridge: ws://localhost:9090"
fi
echo ""
echo -e "${YELLOW}⚠️ IMPORTANTE:${NC}"
echo "   - El sistema tutorial_pkg existente NO se modifica"
echo "   - Flutter Bridge actúa como puente entre Flutter y tutorial_pkg"
echo "   - Todos los comandos pasan a través del sistema de voz existente"

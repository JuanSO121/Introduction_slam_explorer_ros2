#!/bin/bash
# Script de diagnóstico y reparación para Flutter Bridge
# Ubicación: ~/ros2_ws/src/tutorial_pkg/flutter_bridge_diagnostics.sh

echo "🔍 DIAGNÓSTICO FLUTTER BRIDGE PARA TUTORIAL_PKG"
echo "==============================================="

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

ERRORS_FOUND=0
WARNINGS_FOUND=0

log_error() {
    echo -e "${RED}❌ ERROR: $1${NC}"
    ((ERRORS_FOUND++))
}

log_warning() {
    echo -e "${YELLOW}⚠️ WARNING: $1${NC}"
    ((WARNINGS_FOUND++))
}

log_success() {
    echo -e "${GREEN}✅ OK: $1${NC}"
}

log_info() {
    echo -e "${BLUE}ℹ️ INFO: $1${NC}"
}

log_action() {
    echo -e "${PURPLE}🔧 ACCIÓN: $1${NC}"
}

# === 1. VERIFICAR UBICACIÓN Y ESTRUCTURA ===
echo -e "${BLUE}1. Verificando ubicación y estructura del proyecto...${NC}"

if [[ ! -d "tutorial_pkg" && ! -f "package.xml" ]]; then
    log_error "No estás en el directorio correcto. Ve a ~/ros2_ws/src/tutorial_pkg/"
    exit 1
fi

log_success "Ubicación correcta: $(pwd)"

# Verificar estructura
REQUIRED_DIRS=("tutorial_pkg" "config" "launch")
for dir in "${REQUIRED_DIRS[@]}"; do
    if [[ -d "$dir" ]]; then
        log_success "Directorio $dir existe"
    else
        log_warning "Directorio $dir no existe"
        log_action "Creando directorio $dir..."
        mkdir -p "$dir"
    fi
done

# === 2. VERIFICAR ARCHIVOS CLAVE ===
echo -e "\n${BLUE}2. Verificando archivos clave...${NC}"

# Archivo principal del bridge
if [[ -f "tutorial_pkg/flutter_bridge_node.py" ]]; then
    log_success "flutter_bridge_node.py existe"
    
    # Verificar permisos
    if [[ -x "tutorial_pkg/flutter_bridge_node.py" ]]; then
        log_success "flutter_bridge_node.py es ejecutable"
    else
        log_warning "flutter_bridge_node.py no es ejecutable"
        log_action "Haciendo ejecutable..."
        chmod +x tutorial_pkg/flutter_bridge_node.py
    fi
    
    # Verificar contenido básico
    if grep -q "class FlutterBridgeNode" tutorial_pkg/flutter_bridge_node.py; then
        log_success "Estructura de clase correcta"
    else
        log_error "Estructura de clase incorrecta en flutter_bridge_node.py"
    fi
else
    log_error "flutter_bridge_node.py no encontrado"
    log_action "Crear el archivo con el contenido del artifact anterior"
fi

# Launch file
if [[ -f "launch/flutter_integration_launch.py" ]]; then
    log_success "Launch file existe"
else
    log_error "Launch file no encontrado"
    log_action "Crear launch/flutter_integration_launch.py con el contenido del artifact"
fi

# Config file
if [[ -f "config/flutter_bridge_config.yaml" ]]; then
    log_success "Archivo de configuración existe"
else
    log_warning "Archivo de configuración no encontrado"
    log_action "Crear config/flutter_bridge_config.yaml"
fi

# === 3. VERIFICAR SETUP.PY ===
echo -e "\n${BLUE}3. Verificando setup.py...${NC}"

if [[ -f "setup.py" ]]; then
    log_success "setup.py existe"
    
    if grep -q "flutter_bridge_node" setup.py; then
        log_success "flutter_bridge_node configurado en setup.py"
    else
        log_error "flutter_bridge_node NO configurado en setup.py"
        log_action "Necesitas agregar 'flutter_bridge_node = tutorial_pkg.flutter_bridge_node:main' en entry_points"
        
        echo -e "${YELLOW}Ejemplo de setup.py corregido:${NC}"
        cat << 'EOF'
        entry_points={
            'console_scripts': [
                'voice_control_node = tutorial_pkg.voice_control_node:main',
                'gemini_ai_node = tutorial_pkg.gemini_ai_node:main',
                'integrated_exploration_node = tutorial_pkg.integrated_exploration_node:main',
                'flutter_bridge_node = tutorial_pkg.flutter_bridge_node:main',  # <-- AGREGAR ESTA LÍNEA
            ],
        },
EOF
    fi
else
    log_error "setup.py no encontrado"
fi

# === 4. VERIFICAR DEPENDENCIAS ROS2 ===
echo -e "\n${BLUE}4. Verificando dependencias ROS2...${NC}"

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || {
    log_error "No se puede source ROS2 Humble"
    exit 1
}

log_success "ROS2 Humble sourced"

# Verificar rosbridge
if ros2 pkg list | grep -q "rosbridge_server"; then
    log_success "rosbridge_server disponible"
else
    log_error "rosbridge_server no encontrado"
    log_action "Instalando rosbridge..."
    sudo apt update && sudo apt install -y ros-humble-rosbridge-suite
fi

# Verificar tutorial_pkg
if ros2 pkg list | grep -q "tutorial_pkg"; then
    log_success "tutorial_pkg disponible en ROS2"
else
    log_warning "tutorial_pkg no encontrado en ROS2 (necesita compilarse)"
fi

# === 5. VERIFICAR DEPENDENCIAS PYTHON ===
echo -e "\n${BLUE}5. Verificando dependencias Python...${NC}"

PYTHON_DEPS=("flask" "flask_cors" "flask_socketio" "eventlet" "requests" "werkzeug")

for dep in "${PYTHON_DEPS[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        log_success "$dep instalado"
    else
        log_error "$dep NO instalado"
        log_action "Instalar con: pip3 install --user $dep"
    fi
done

# === 6. VERIFICAR LAUNCH FILES DE TUTORIAL_PKG ===
echo -e "\n${BLUE}6. Verificando launch files de tutorial_pkg...${NC}"

LAUNCH_DIR="launch"
if [[ -d "$LAUNCH_DIR" ]]; then
    echo -e "${BLUE}Archivos launch encontrados:${NC}"
    find "$LAUNCH_DIR" -name "*.py" -exec basename {} \; | sort
    
    # Buscar el launch principal
    POSSIBLE_LAUNCHES=("integrated_exploration_launch_fixed.py" "integrated_exploration_launch.py" "main_launch.py")
    FOUND_MAIN=false
    
    for launch in "${POSSIBLE_LAUNCHES[@]}"; do
        if [[ -f "$LAUNCH_DIR/$launch" ]]; then
            log_success "Launch principal encontrado: $launch"
            FOUND_MAIN=true
            break
        fi
    done
    
    if [[ "$FOUND_MAIN" == false ]]; then
        log_warning "No se encontró launch principal de tutorial_pkg"
        log_info "El flutter_integration_launch.py puede funcionar sin el launch principal"
    fi
else
    log_error "Directorio launch no encontrado"
fi

# === 7. VERIFICAR COMPILACIÓN ===
echo -e "\n${BLUE}7. Verificando compilación del paquete...${NC}"

cd ~/ros2_ws

# Source workspace si existe
if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
    log_success "Workspace sourced"
else
    log_warning "Workspace no compilado aún"
fi

# Verificar si el ejecutable existe
if command -v ros2 >/dev/null && ros2 pkg executables tutorial_pkg 2>/dev/null | grep -q "flutter_bridge_node"; then
    log_success "flutter_bridge_node ejecutable encontrado"
else
    log_error "flutter_bridge_node ejecutable NO encontrado"
    log_action "Necesitas compilar el paquete con: colcon build --packages-select tutorial_pkg"
fi

# === 8. VERIFICAR PUERTOS ===
echo -e "\n${BLUE}8. Verificando puertos...${NC}"

# Puerto 8000 (Flask)
if netstat -tuln 2>/dev/null | grep -q ":8000 "; then
    log_warning "Puerto 8000 ya está en uso"
    log_info "Esto puede ser normal si Flutter Bridge ya está corriendo"
else
    log_success "Puerto 8000 disponible"
fi

# Puerto 9090 (Rosbridge)
if netstat -tuln 2>/dev/null | grep -q ":9090 "; then
    log_warning "Puerto 9090 ya está en uso"
    log_info "Esto puede ser normal si Rosbridge ya está corriendo"
else
    log_success "Puerto 9090 disponible"
fi

# === 9. CREAR ARCHIVOS DE REPARACIÓN ===
echo -e "\n${BLUE}9. Creando archivos de reparación...${NC}"

# Script de reparación automática
cat > repair_flutter_bridge.sh << 'EOF'
#!/bin/bash
# Script de reparación automática para Flutter Bridge

echo "🔧 Reparando Flutter Bridge..."

# 1. Instalar dependencias Python
echo "📦 Instalando dependencias Python..."
pip3 install --user flask flask-cors flask-socketio eventlet requests werkzeug python-socketio

# 2. Instalar rosbridge si no existe
if ! ros2 pkg list | grep -q "rosbridge_server"; then
    echo "📦 Instalando rosbridge..."
    sudo apt update && sudo apt install -y ros-humble-rosbridge-suite
fi

# 3. Compilar paquete
echo "🔨 Compilando tutorial_pkg..."
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tutorial_pkg --symlink-install

# 4. Source workspace
source install/setup.bash

echo "✅ Reparación completada"
EOF

chmod +x repair_flutter_bridge.sh
log_success "Script de reparación creado: repair_flutter_bridge.sh"

# Script de prueba rápida
cat > quick_test_flutter_bridge.sh << 'EOF'
#!/bin/bash
# Prueba rápida del Flutter Bridge

echo "🧪 Prueba rápida de Flutter Bridge"

# Source environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Verificar que el node existe
if ros2 pkg executables tutorial_pkg | grep -q "flutter_bridge_node"; then
    echo "✅ Ejecutable encontrado"
    
    # Intentar lanzar solo el bridge (sin tutorial_pkg)
    echo "🚀 Lanzando Flutter Bridge standalone..."
    timeout 10s ros2 run tutorial_pkg flutter_bridge_node --ros-args -p flask_port:=8000 -p enable_websocket:=true -p enable_voice_processing:=false
    
    echo "✅ Prueba completada"
else
    echo "❌ Ejecutable no encontrado"
    echo "🔧 Ejecuta: ./repair_flutter_bridge.sh"
fi
EOF

chmod +x quick_test_flutter_bridge.sh
log_success "Script de prueba creado: quick_test_flutter_bridge.sh"

# === 10. RESUMEN FINAL ===
echo -e "\n${PURPLE}📋 RESUMEN DEL DIAGNÓSTICO${NC}"
echo "=========================="

if [[ $ERRORS_FOUND -eq 0 ]]; then
    log_success "No se encontraron errores críticos"
else
    log_error "$ERRORS_FOUND errores encontrados"
fi

if [[ $WARNINGS_FOUND -eq 0 ]]; then
    log_success "No se encontraron advertencias"
else
    log_warning "$WARNINGS_FOUND advertencias encontradas"
fi

echo ""
echo -e "${BLUE}🚀 PRÓXIMOS PASOS RECOMENDADOS:${NC}"

if [[ $ERRORS_FOUND -gt 0 ]]; then
    echo "1. 🔧 Ejecutar reparación automática: ./repair_flutter_bridge.sh"
    echo "2. 📝 Revisar y corregir los errores listados arriba"
    echo "3. 🔨 Compilar el paquete: cd ~/ros2_ws && colcon build --packages-select tutorial_pkg"
    echo "4. 🧪 Probar el sistema: ./quick_test_flutter_bridge.sh"
else
    echo "1. 🧪 Probar el sistema: ./quick_test_flutter_bridge.sh"
    echo "2. 🚀 Lanzar sistema completo: ./start_flutter_integration.sh"
    echo "3. 🔍 Verificar conexión: ./test_flutter_bridge.sh"
fi

echo ""
echo -e "${BLUE}📡 URLs de conexión (si todo funciona):${NC}"
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
echo "🎯 Ejecuta este script regularmente para verificar el estado del sistema"
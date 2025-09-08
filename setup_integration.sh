#!/bin/bash
# Script de configuración integral para tutorial_pkg
# Ubicación: ~/ros2_ws/src/tutorial_pkg/setup_integration.sh
# Autor: Sistema Integrado de Control por Voz IA

set -e  # Salir en caso de error

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Función de logging
log() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE} $1 ${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# Variables de configuración
WORKSPACE_DIR="$HOME/ros2_ws"
PACKAGE_DIR="$WORKSPACE_DIR/src/tutorial_pkg"
VOICE_SERVICES_DIR="$PACKAGE_DIR/voice_services"

# Verificar que estamos en el directorio correcto
if [ ! -d "$PACKAGE_DIR" ]; then
    error "No se encuentra el directorio del paquete: $PACKAGE_DIR"
    error "Por favor ejecutar desde: cd $PACKAGE_DIR && ./setup_integration.sh"
    exit 1
fi

cd "$PACKAGE_DIR"

header "CONFIGURACIÓN INTEGRAL TUTORIAL_PKG"
echo "🤖 Configurando sistema integrado de exploración con control por voz IA"
echo "📦 Paquete: tutorial_pkg"
echo "🏠 Directorio: $PACKAGE_DIR"

# 1. VERIFICAR DEPENDENCIAS DE ROS2
header "1. VERIFICACIÓN DE DEPENDENCIAS ROS2"

# Verificar ROS2 Humble
if ! command -v ros2 &> /dev/null; then
    error "ROS2 no está instalado o no está en el PATH"
    exit 1
fi

ros2_version=$(ros2 --version 2>/dev/null | head -n 1)
log "ROS2 detectado: $ros2_version"

# Verificar workspace
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    error "Workspace ROS2 no compilado. Ejecutar: cd $WORKSPACE_DIR && colcon build"
    exit 1
fi

# Source del workspace
source "$WORKSPACE_DIR/install/setup.bash"
log "Workspace cargado correctamente"

# 2. CREAR ESTRUCTURA DE DIRECTORIOS
header "2. CONFIGURACIÓN DE ESTRUCTURA DE DIRECTORIOS"

# Directorios principales
directories=(
    "config"
    "launch" 
    "rviz"
    "worlds"
    "maps"
    "scripts"
    "voice_services"
    "logs"
    "data"
)

for dir in "${directories[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        log "Creado directorio: $dir"
    else
        log "Directorio existente: $dir"
    fi
done

# Hacer ejecutables los scripts
if [ -d "scripts" ]; then
    chmod +x scripts/*.py 2>/dev/null || true
    log "Scripts marcados como ejecutables"
fi

# 3. VERIFICAR PAQUETES ROS2 NECESARIOS
header "3. VERIFICACIÓN DE PAQUETES ROS2"

required_packages=(
    "turtlebot3_gazebo"
    "turtlebot3_description" 
    "slam_toolbox"
    "nav2_bringup"
    "nav2_rviz_plugins"
    "rviz2"
    "gazebo_ros"
)

missing_packages=()

for package in "${required_packages[@]}"; do
    if ros2 pkg list | grep -q "^$package$"; then
        log "✓ Paquete encontrado: $package"
    else
        warn "✗ Paquete faltante: $package"
        missing_packages+=("$package")
    fi
done

if [ ${#missing_packages[@]} -ne 0 ]; then
    error "Paquetes faltantes detectados. Instalar con:"
    echo "sudo apt update"
    echo "sudo apt install -y \\"
    for package in "${missing_packages[@]}"; do
        echo "  ros-humble-${package//_/-} \\"
    done
    echo ""
    read -p "¿Desea continuar sin estos paquetes? (y/N): " continue_choice
    if [[ ! "$continue_choice" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 4. CONFIGURACIÓN DE VARIABLES DE ENTORNO
header "4. CONFIGURACIÓN DE VARIABLES DE ENTORNO"

# Archivo de configuración de entorno
ENV_FILE="$PACKAGE_DIR/setup_env.sh"

cat > "$ENV_FILE" << 'EOF'
#!/bin/bash
# Configuración de entorno para tutorial_pkg
# Source este archivo antes de ejecutar los launch files

# Variables de TurtleBot3
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/tutorial_pkg/models

# Variables de exploración
export TUTORIAL_PKG_CONFIG_DIR=~/ros2_ws/src/tutorial_pkg/config
export TUTORIAL_PKG_MAPS_DIR=~/ros2_ws/src/tutorial_pkg/maps
export TUTORIAL_PKG_LOGS_DIR=~/ros2_ws/src/tutorial_pkg/logs

# Variables de servicios de voz (opcional)
export FLASK_PORT=8000
export VOICE_SERVICES_DIR=~/ros2_ws/src/tutorial_pkg/voice_services

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "✅ Entorno tutorial_pkg configurado"
echo "🤖 TurtleBot3 Model: $TURTLEBOT3_MODEL" 
echo "📁 Config Dir: $TUTORIAL_PKG_CONFIG_DIR"
echo "🗺️ Maps Dir: $TUTORIAL_PKG_MAPS_DIR"
EOF

chmod +x "$ENV_FILE"
log "Archivo de entorno creado: $ENV_FILE"

# Source el archivo de entorno
source "$ENV_FILE"

# 5. CREAR ARCHIVOS DE CONFIGURACIÓN BÁSICOS
header "5. CONFIGURACIÓN DE ARCHIVOS BÁSICOS"

# Crear slam_simple.yaml si no existe
if [ ! -f "config/slam_simple.yaml" ]; then
    cat > "config/slam_simple.yaml" << 'EOF'
# Configuración SLAM para tutorial_pkg
slam_toolbox:
  ros__parameters:
    # Basic SLAM parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    
    # SLAM behavior
    mode: mapping
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    
    # Map parameters  
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    
    # Loop closure
    do_loop_closing: true
    loop_match_minimum_confidence: 0.35
    loop_match_maximum_distance: 4.0
    
    # Correlation parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Motion model
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    
    # Publishing
    publish_period_sec: 1.0
EOF
    log "Creado: config/slam_simple.yaml"
fi

# Crear navigation_simple.yaml si no existe  
if [ ! -f "config/navigation_simple.yaml" ]; then
    cat > "config/navigation_simple.yaml" << 'EOF'
# Configuración de navegación para tutorial_pkg
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    
    # DWB Controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      
planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: true
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
EOF
    log "Creado: config/navigation_simple.yaml"
fi

# 6. CONFIGURACIÓN DEL SISTEMA DE VOZ (OPCIONAL)
header "6. CONFIGURACIÓN DEL SISTEMA DE VOZ"

if [ -d "$VOICE_SERVICES_DIR" ]; then
    log "Directorio voice_services encontrado"
    
    # Verificar si existe entorno virtual
    if [ ! -d "$VOICE_SERVICES_DIR/venv" ]; then
        warn "Entorno virtual no encontrado para servicios de voz"
        read -p "¿Desea crear el entorno virtual para servicios de IA? (y/N): " create_venv
        
        if [[ "$create_venv" =~ ^[Yy]$ ]]; then
            cd "$VOICE_SERVICES_DIR"
            
            # Crear entorno virtual
            python3 -m venv venv
            source venv/bin/activate
            
            # Actualizar pip
            pip install --upgrade pip
            
            # Instalar dependencias básicas
            pip install flask flask-cors flask-socketio transformers torch librosa scipy opencv-python requests
            
            log "Entorno virtual creado en: $VOICE_SERVICES_DIR/venv"
            log "Para activar: cd $VOICE_SERVICES_DIR && source venv/bin/activate"
            
            # Crear script de activación rápida
            cat > "$VOICE_SERVICES_DIR/activate_voice_env.sh" << 'EOF'
#!/bin/bash
# Script para activar entorno de servicios de voz
cd "$(dirname "$0")"
source venv/bin/activate
echo "✅ Entorno de servicios de voz activado"
echo "🎤 Para ejecutar servidor: python3 flask_server.py"
echo "🔌 Para ejecutar bridge ROS2: python3 ros2_voice_bridge.py"
EOF
            chmod +x "$VOICE_SERVICES_DIR/activate_voice_env.sh"
            
            cd "$PACKAGE_DIR"
        fi
    else
        log "Entorno virtual encontrado para servicios de voz"
    fi
else
    warn "Directorio voice_services no encontrado - servicios de IA no disponibles"
fi

# 7. VERIFICAR Y CORREGIR PERMISOS
header "7. CONFIGURACIÓN DE PERMISOS"

# Hacer ejecutables todos los scripts de Python
find . -name "*.py" -type f -exec chmod +x {} \;
log "Scripts Python marcados como ejecutables"

# Hacer ejecutables los launch files
find . -name "*.launch.py" -type f -exec chmod +x {} \;
log "Launch files marcados como ejecutables"

# Crear directorio de logs con permisos correctos
mkdir -p logs
chmod 755 logs
log "Directorio de logs configurado"

# 8. COMPILAR EL PAQUETE
header "8. COMPILACIÓN DEL PAQUETE"

cd "$WORKSPACE_DIR"

log "Compilando tutorial_pkg..."
if colcon build --packages-select tutorial_pkg --symlink-install; then
    log "✅ Compilación exitosa"
else
    warn "⚠️ Problemas durante la compilación - revisar errores arriba"
fi

# Source del workspace actualizado
source install/setup.bash

# 9. CREAR SCRIPTS DE EJECUCIÓN RÁPIDA
header "9. CREACIÓN DE SCRIPTS DE EJECUCIÓN"

cd "$PACKAGE_DIR"

# Script de lanzamiento completo
cat > "run_integrated_exploration.sh" << 'EOF'
#!/bin/bash
# Script de ejecución integrada para tutorial_pkg

# Configurar entorno
source ~/ros2_ws/src/tutorial_pkg/setup_env.sh

# Mostrar información
echo "🚀 Iniciando sistema integrado tutorial_pkg"
echo "🤖 Robot: TurtleBot3 Waffle"
echo "🗺️ Modo: Exploración autónoma con control por voz"
echo ""

# Opciones de ejecución
echo "Seleccione modo de ejecución:"
echo "1) Solo exploración (sin control por voz)"
echo "2) Exploración + Control por voz"
echo "3) Exploración + Control por voz + Servidor Flask"
read -p "Opción [1]: " option

case $option in
    2)
        echo "🎤 Iniciando con control por voz..."
        ros2 launch tutorial_pkg integrated_exploration_launch.py enable_voice_control:=true
        ;;
    3)
        echo "🎤📱 Iniciando con control por voz y servidor Flask..."
        ros2 launch tutorial_pkg integrated_exploration_launch.py enable_voice_control:=true enable_flask_server:=true
        ;;
    *)
        echo "🗺️ Iniciando solo exploración..."
        ros2 launch tutorial_pkg integrated_exploration_launch.py enable_voice_control:=false
        ;;
esac
EOF

chmod +x "run_integrated_exploration.sh"
log "Creado: run_integrated_exploration.sh"

# Script de prueba rápida
cat > "test_system.sh" << 'EOF'
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
EOF

chmod +x "test_system.sh"
log "Creado: test_system.sh"

# Script para servicios de voz
cat > "run_voice_services.sh" << 'EOF'
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
EOF

chmod +x "run_voice_services.sh"
log "Creado: run_voice_services.sh"

# 10. VERIFICACIÓN FINAL Y RESUMEN
header "10. VERIFICACIÓN FINAL"

log "Ejecutando verificaciones finales..."

# Verificar estructura de directorios
for dir in "${directories[@]}"; do
    if [ -d "$dir" ]; then
        log "✓ $dir/"
    else
        warn "✗ $dir/ - faltante"
    fi
done

# Verificar archivos críticos
critical_files=(
    "setup_env.sh"
    "run_integrated_exploration.sh"
    "test_system.sh"
    "config/voice_control.yaml"
    "config/slam_simple.yaml"
    "config/navigation_simple.yaml"
)

log "Verificando archivos críticos:"
for file in "${critical_files[@]}"; do
    if [ -f "$file" ]; then
        log "✓ $file"
    else
        warn "✗ $file - faltante"
    fi
done

# Resumen final
header "CONFIGURACIÓN COMPLETADA"

echo "🎉 Sistema tutorial_pkg configurado exitosamente!"
echo ""
echo "📁 Directorio del paquete: $PACKAGE_DIR"
echo "🔧 Archivo de entorno: $PACKAGE_DIR/setup_env.sh"
echo ""
echo "🚀 COMANDOS DE EJECUCIÓN:"
echo "  • Sistema completo:    ./run_integrated_exploration.sh"
echo "  • Solo servicios voz:  ./run_voice_services.sh"
echo "  • Pruebas sistema:     ./test_system.sh"
echo ""
echo "📋 LAUNCH FILES DISPONIBLES:"
echo "  • Exploración integrada: ros2 launch tutorial_pkg integrated_exploration_launch.py"
echo "  • Control por voz solo:  ros2 launch tutorial_pkg voice_control_launch.py"
echo ""
echo "🎤 CONTROL POR VOZ:"
if [ -d "$VOICE_SERVICES_DIR/venv" ]; then
    echo "  ✓ Entorno virtual configurado"
    echo "  • Activar: cd $VOICE_SERVICES_DIR && source venv/bin/activate"
    echo "  • Servidor: python3 flask_server.py"
else
    echo "  ⚠️ Entorno virtual no configurado (opcional)"
fi
echo ""
echo "📖 DOCUMENTACIÓN:"
echo "  • README: $PACKAGE_DIR/README.md"
echo "  • Configuración: $PACKAGE_DIR/config/"
echo "  • Logs: $PACKAGE_DIR/logs/"
echo ""
echo "⚡ PRÓXIMOS PASOS:"
echo "1. Source del entorno: source ~/ros2_ws/src/tutorial_pkg/setup_env.sh"
echo "2. Ejecutar pruebas: ./test_system.sh"
echo "3. Iniciar exploración: ./run_integrated_exploration.sh"
echo ""
echo "🔧 En caso de problemas:"
echo "  • Verificar dependencias ROS2"
echo "  • Recompilar: cd ~/ros2_ws && colcon build --packages-select tutorial_pkg"
echo "  • Revisar logs en: $PACKAGE_DIR/logs/"

# Crear archivo de estado de instalación
cat > ".installation_status" << EOF
# Estado de instalación tutorial_pkg
INSTALLATION_DATE=$(date)
INSTALLATION_VERSION=1.1.0
ROS2_VERSION=$ros2_version
WORKSPACE_DIR=$WORKSPACE_DIR
VOICE_SERVICES_AVAILABLE=$([ -d "$VOICE_SERVICES_DIR/venv" ] && echo "true" || echo "false")
MISSING_PACKAGES=$(IFS=,; echo "${missing_packages[*]}")
EOF

log "Estado de instalación guardado en: .installation_status"

echo ""
echo "✅ Configuración integral completada exitosamente!"
echo "🤖 Sistema tutorial_pkg listo para usar"
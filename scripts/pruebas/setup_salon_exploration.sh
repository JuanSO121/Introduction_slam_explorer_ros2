#!/bin/bash

# Script de configuración para exploración del salón
# Autor: Juan Sanchez
# Descripción: Configura el entorno para exploración robusta con TurtleBot3

set -e  # Salir si algún comando falla

echo "=========================================="
echo "CONFIGURACIÓN DE EXPLORACIÓN DEL SALÓN"
echo "=========================================="

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Función para logging
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Verificar que estamos en el workspace correcto
if [ ! -d "src/tutorial_pkg" ]; then
    log_error "Este script debe ejecutarse desde la raíz del workspace ROS2"
    log_error "Asegúrate de estar en ~/ros2_ws/ o tu workspace correspondiente"
    exit 1
fi

log_info "Verificando estructura del proyecto..."

# Verificar archivos críticos
CRITICAL_FILES=(
    "src/tutorial_pkg/worlds/salon_world.world"
    "src/tutorial_pkg/models/salon_model/model.sdf"
    "src/tutorial_pkg/models/salon_model/model.config"
    "src/tutorial_pkg/models/salon_model/meshes/salon.dae"
    "src/tutorial_pkg/config/explore_optimized.yaml"
    "src/tutorial_pkg/config/slam_optimized.yaml"
    "src/tutorial_pkg/launch/explore_salon_corrected.launch.py"
)

missing_files=()
for file in "${CRITICAL_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        missing_files+=("$file")
        log_warning "Archivo faltante: $file"
    else
        log_success "✓ $file"
    fi
done

if [ ${#missing_files[@]} -gt 0 ]; then
    log_error "Faltan archivos críticos. Por favor, créalos antes de continuar."
    exit 1
fi

# Verificar nodos Python
log_info "Verificando nodos Python..."
PYTHON_NODES=(
    "src/tutorial_pkg/tutorial_pkg/exploration_monitor.py"
    "src/tutorial_pkg/tutorial_pkg/exploration_restarter.py"
    "src/tutorial_pkg/tutorial_pkg/exploration_visualizer.py"
    "src/tutorial_pkg/tutorial_pkg/__init__.py"
)

for node in "${PYTHON_NODES[@]}"; do
    if [ ! -f "$node" ]; then
        log_warning "Nodo Python faltante: $node"
        # Crear __init__.py si no existe
        if [[ "$node" == *"__init__.py" ]]; then
            touch "$node"
            log_success "Creado: $node"
        fi
    else
        log_success "✓ $node"
    fi
done

# Verificar permisos de ejecución
log_info "Configurando permisos de ejecución..."
chmod +x src/tutorial_pkg/tutorial_pkg/*.py
chmod +x src/tutorial_pkg/launch/*.py
chmod +x src/tutorial_pkg/scripts/*.sh
log_success "Permisos configurados"

# Configurar variables de entorno
log_info "Configurando variables de entorno de Gazebo..."

# Obtener path absoluto del directorio de modelos
MODELS_PATH=$(realpath src/tutorial_pkg/models)

# Verificar si ya está en GAZEBO_MODEL_PATH
if [[ ":$GAZEBO_MODEL_PATH:" != *":$MODELS_PATH:"* ]]; then
    export GAZEBO_MODEL_PATH="$MODELS_PATH:$GAZEBO_MODEL_PATH"
    log_success "GAZEBO_MODEL_PATH actualizado: $MODELS_PATH"
    
    # Sugerir añadirlo al bashrc
    echo ""
    log_warning "Para hacer permanente la configuración, añade esta línea a tu ~/.bashrc:"
    echo "export GAZEBO_MODEL_PATH=\"$MODELS_PATH:\$GAZEBO_MODEL_PATH\""
else
    log_success "GAZEBO_MODEL_PATH ya configurado correctamente"
fi

# Verificar modelo TurtleBot3
log_info "Verificando configuración de TurtleBot3..."
if [ -z "$TURTLEBOT3_MODEL" ]; then
    export TURTLEBOT3_MODEL=burger
    log_success "TURTLEBOT3_MODEL configurado como: burger"
    log_warning "Para hacer permanente, añade a tu ~/.bashrc:"
    echo "export TURTLEBOT3_MODEL=burger"
else
    log_success "TURTLEBOT3_MODEL ya configurado: $TURTLEBOT3_MODEL"
fi

# Compilar el paquete
log_info "Compilando el paquete..."
if colcon build --packages-select tutorial_pkg --symlink-install; then
    log_success "Compilación exitosa"
else
    log_error "Error en la compilación"
    exit 1
fi

# Source del workspace
log_info "Sourcing workspace..."
source install/setup.bash
log_success "Workspace sourced correctamente"

# Verificar dependencias
log_info "Verificando dependencias de ROS2..."
REQUIRED_PACKAGES=(
    "nav2_bringup"
    "slam_toolbox" 
    "explore_lite"
    "gazebo_ros"
    "turtlebot3_gazebo"
)

missing_packages=()
for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! ros2 pkg list | grep -q "^$package$"; then
        missing_packages+=("$package")
        log_warning "Paquete faltante: $package"
    else
        log_success "✓ $package"
    fi
done

if [ ${#missing_packages[@]} -gt 0 ]; then
    log_warning "Instala los paquetes faltantes con:"
    echo "sudo apt update"
    for package in "${missing_packages[@]}"; do
        echo "sudo apt install ros-humble-${package//_/-}"
    done
    echo ""
fi

# Resumen final
echo ""
echo "=========================================="
log_success "CONFIGURACIÓN COMPLETADA"
echo "=========================================="
echo ""
log_info "Para lanzar la exploración del salón, ejecuta:"
echo "ros2 launch tutorial_pkg explore_salon_corrected.launch.py"
echo ""
log_info "Para monitorear el proceso:"
echo "ros2 topic echo /exploration_status"
echo ""
log_warning "Asegúrate de que el archivo salon.dae esté en:"
echo "src/tutorial_pkg/models/salon_model/meshes/salon.dae"
echo ""

# Verificar si el archivo DAE existe
if [ ! -f "src/tutorial_pkg/models/salon_model/meshes/salon.dae" ]; then
    log_error "¡IMPORTANTE! No se encontró el archivo salon.dae"
    log_error "Debes colocar tu modelo 3D en:"
    log_error "src/tutorial_pkg/models/salon_model/meshes/salon.dae"
    exit 1
else
    log_success "Modelo salon.dae encontrado correctamente"
fi

log_success "¡Sistema listo para exploración!"
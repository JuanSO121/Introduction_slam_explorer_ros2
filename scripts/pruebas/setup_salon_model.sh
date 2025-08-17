#!/bin/bash

# Script de configuración para el modelo de salón .dae
# Archivo: ~/ros2_ws/src/tutorial_pkg/scripts/setup_salon_model.sh

echo "Configurando modelo de salón para ROS2 Navigation..."

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Directorios
TUTORIAL_PKG_DIR="$HOME/ros2_ws/src/tutorial_pkg"
MODELS_DIR="$TUTORIAL_PKG_DIR/models/salon_model"
MESHES_DIR="$MODELS_DIR/meshes"

echo -e "${YELLOW}Creando estructura de directorios...${NC}"

# Crear directorios necesarios
mkdir -p "$MODELS_DIR"
mkdir -p "$MESHES_DIR"
mkdir -p "$TUTORIAL_PKG_DIR/config"
mkdir -p "$TUTORIAL_PKG_DIR/launch"
mkdir -p "$TUTORIAL_PKG_DIR/worlds"
mkdir -p "$TUTORIAL_PKG_DIR/rviz"

echo -e "${GREEN}✓ Directorios creados${NC}"

# Verificar que existe el archivo .dae
DAE_FILE=""
echo -e "${YELLOW}Buscando archivo .dae en el directorio actual...${NC}"

# Buscar archivos .dae en el directorio actual
for file in *.dae; do
    if [[ -f "$file" ]]; then
        DAE_FILE="$file"
        echo -e "${GREEN}✓ Encontrado: $file${NC}"
        break
    fi
done

if [[ -z "$DAE_FILE" ]]; then
    echo -e "${RED}✗ No se encontró archivo .dae en el directorio actual${NC}"
    echo -e "${YELLOW}Por favor, coloca tu archivo salon.dae en: $MESHES_DIR${NC}"
    echo -e "${YELLOW}O ejecuta este script desde el directorio que contiene tu archivo .dae${NC}"
else
    # Copiar archivo .dae al directorio correcto
    cp "$DAE_FILE" "$MESHES_DIR/salon.dae"
    echo -e "${GREEN}✓ Archivo .dae copiado a $MESHES_DIR/salon.dae${NC}"
fi

# Configurar variable de entorno GAZEBO_MODEL_PATH
echo -e "${YELLOW}Configurando variables de entorno...${NC}"

# Añadir al bashrc si no existe
BASHRC_LINE="export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$TUTORIAL_PKG_DIR/models"
if ! grep -q "$TUTORIAL_PKG_DIR/models" ~/.bashrc; then
    echo "$BASHRC_LINE" >> ~/.bashrc
    echo -e "${GREEN}✓ Variable GAZEBO_MODEL_PATH añadida a ~/.bashrc${NC}"
else
    echo -e "${GREEN}✓ Variable GAZEBO_MODEL_PATH ya configurada${NC}"
fi

# Configurar para la sesión actual
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$TUTORIAL_PKG_DIR/models

echo -e "${YELLOW}Verificando configuración...${NC}"

# Verificar estructura de archivos
REQUIRED_FILES=(
    "$MODELS_DIR/model.config"
    "$MODELS_DIR/model.sdf"
    "$TUTORIAL_PKG_DIR/worlds/salon_world.world"
    "$TUTORIAL_PKG_DIR/config/salon_explore.yaml"
    "$TUTORIAL_PKG_DIR/launch/salon_explore.launch.py"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [[ -f "$file" ]]; then
        echo -e "${GREEN}✓ $file${NC}"
    else
        echo -e "${RED}✗ $file (faltante)${NC}"
    fi
done

echo -e "${YELLOW}Pasos siguientes:${NC}"
echo "1. Asegúrate de que salon.dae esté en: $MESHES_DIR"
echo "2. Ajusta la escala del modelo en model.sdf si es necesario"
echo "3. Verifica las dimensiones del salón en la configuración"
echo "4. Compila el workspace: cd ~/ros2_ws && colcon build"
echo "5. Source el workspace: source ~/ros2_ws/install/setup.bash"
echo "6. Lanza la exploración: ros2 launch tutorial_pkg salon_explore.launch.py"

echo -e "${GREEN}¡Configuración completada!${NC}"

# Función para verificar el modelo
verify_model() {
    echo -e "${YELLOW}Verificando modelo de Gazebo...${NC}"
    
    if [[ -f "$MESHES_DIR/salon.dae" ]]; then
        echo -e "${GREEN}✓ Archivo .dae encontrado${NC}"
        
        # Verificar tamaño del archivo
        size=$(ls -lh "$MESHES_DIR/salon.dae" | awk '{print $5}')
        echo -e "${GREEN}  Tamaño: $size${NC}"
        
        # Verificar permisos
        if [[ -r "$MESHES_DIR/salon.dae" ]]; then
            echo -e "${GREEN}✓ Permisos de lectura OK${NC}"
        else
            echo -e "${RED}✗ Problemas de permisos${NC}"
            chmod 644 "$MESHES_DIR/salon.dae"
            echo -e "${GREEN}✓ Permisos corregidos${NC}"
        fi
    else
        echo -e "${RED}✗ Archivo .dae no encontrado${NC}"
    fi
}

# Función para ajustar configuración según dimensiones del salón
configure_salon_dimensions() {
    echo -e "${YELLOW}Configuración de dimensiones del salón:${NC}"
    echo "Si tu salón tiene dimensiones específicas, ajusta estos parámetros:"
    echo "  - En salon_explore.yaml:"
    echo "    * local_costmap width/height (líneas 116-117)"
    echo "    * laser_max_range según el tamaño del salón"
    echo "    * min_frontier_size según la complejidad"
    echo "  - En model.sdf:"
    echo "    * scale en las líneas de geometry (líneas 19 y 32)"
}

# Ejecutar verificaciones
verify_model
configure_salon_dimensions

echo -e "${GREEN}Script completado. ¡Listo para explorar el salón!${NC}"
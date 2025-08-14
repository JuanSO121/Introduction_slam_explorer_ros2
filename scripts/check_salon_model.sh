#!/bin/bash

echo "=== Verificación del modelo salon_model ==="
echo ""

# Verificar estructura de directorios
MODEL_PATH="$HOME/ros2_ws/src/tutorial_pkg/models/salon_model"

if [ -d "$MODEL_PATH" ]; then
    echo "✓ Directorio del modelo encontrado: $MODEL_PATH"
else
    echo "✗ Directorio del modelo NO encontrado: $MODEL_PATH"
    echo "  Ejecuta: mkdir -p $MODEL_PATH/meshes"
    exit 1
fi

# Verificar archivos de configuración
if [ -f "$MODEL_PATH/model.config" ]; then
    echo "✓ Archivo model.config encontrado"
else
    echo "✗ Archivo model.config NO encontrado"
fi

if [ -f "$MODEL_PATH/model.sdf" ]; then
    echo "✓ Archivo model.sdf encontrado"
else
    echo "✗ Archivo model.sdf NO encontrado"
fi

# Verificar directorio meshes
if [ -d "$MODEL_PATH/meshes" ]; then
    echo "✓ Directorio meshes encontrado"
    
    # Verificar archivo DAE
    if [ -f "$MODEL_PATH/meshes/salon.dae" ]; then
        echo "✓ Archivo salon.dae encontrado"
        
        # Verificar tamaño del archivo
        FILE_SIZE=$(stat -f%z "$MODEL_PATH/meshes/salon.dae" 2>/dev/null || stat -c%s "$MODEL_PATH/meshes/salon.dae" 2>/dev/null)
        if [ "$FILE_SIZE" -gt 0 ]; then
            echo "✓ Archivo salon.dae tiene contenido (${FILE_SIZE} bytes)"
        else
            echo "✗ Archivo salon.dae está vacío"
        fi
        
        # Verificar permisos
        if [ -r "$MODEL_PATH/meshes/salon.dae" ]; then
            echo "✓ Archivo salon.dae es legible"
        else
            echo "✗ Archivo salon.dae NO es legible"
            echo "  Ejecuta: chmod 644 $MODEL_PATH/meshes/salon.dae"
        fi
    else
        echo "✗ Archivo salon.dae NO encontrado en $MODEL_PATH/meshes/"
        echo "  Debes copiar tu archivo exportado desde Blender aquí"
    fi
else
    echo "✗ Directorio meshes NO encontrado"
    echo "  Ejecuta: mkdir -p $MODEL_PATH/meshes"
fi

echo ""
echo "=== Verificación de variables de entorno ==="

if [ -n "$GAZEBO_MODEL_PATH" ]; then
    echo "✓ GAZEBO_MODEL_PATH está configurado"
    if echo "$GAZEBO_MODEL_PATH" | grep -q "$HOME/ros2_ws/src/tutorial_pkg/models"; then
        echo "✓ Ruta de modelos incluida en GAZEBO_MODEL_PATH"
    else
        echo "✗ Ruta de modelos NO incluida en GAZEBO_MODEL_PATH"
        echo "  Verifica tu ~/.bashrc"
    fi
else
    echo "✗ GAZEBO_MODEL_PATH NO está configurado"
fi

echo ""
echo "=== Contenido del directorio models ==="
ls -la "$HOME/ros2_ws/src/tutorial_pkg/models/" 2>/dev/null || echo "Directorio models no existe"

echo ""
echo "=== Verificación completa ==="

# Contar problemas
ERRORS=0

[ ! -d "$MODEL_PATH" ] && ((ERRORS++))
[ ! -f "$MODEL_PATH/model.config" ] && ((ERRORS++))
[ ! -f "$MODEL_PATH/model.sdf" ] && ((ERRORS++))
[ ! -f "$MODEL_PATH/meshes/salon.dae" ] && ((ERRORS++))

if [ $ERRORS -eq 0 ]; then
    echo "✓ Todo parece estar en orden. Puedes probar lanzar Gazebo."
    echo ""
    echo "Comandos para probar:"
    echo "  cd ~/ros2_ws && source install/setup.bash"
    echo "  ros2 launch tutorial_pkg salon_gazebo.launch.py"
else
    echo "✗ Se encontraron $ERRORS problemas. Revisa los errores anteriores."
fi
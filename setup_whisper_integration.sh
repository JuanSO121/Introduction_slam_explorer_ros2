#!/bin/bash
# Setup script para integrar Whisper con tutorial_pkg
# Ubicación: ~/ros2_ws/src/tutorial_pkg/setup_whisper_integration.sh

set -e

echo "🤖 Configurando integración Whisper para tutorial_pkg"
echo "======================================================"

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

# Variables
TUTORIAL_PKG_DIR="$HOME/ros2_ws/src/tutorial_pkg"
VENV_DIR="$TUTORIAL_PKG_DIR/whisper_venv"
REQUIREMENTS_FILE="$TUTORIAL_PKG_DIR/requirements_whisper.txt"

# Verificar que estamos en el directorio correcto
if [ ! -d "$TUTORIAL_PKG_DIR" ]; then
    log_error "Directorio tutorial_pkg no encontrado en $TUTORIAL_PKG_DIR"
    exit 1
fi

cd "$TUTORIAL_PKG_DIR"

echo ""
echo "📋 Paso 1: Verificar dependencias del sistema"
echo "--------------------------------------------"

# Verificar Python 3.8+
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
log_info "Versión de Python detectada: $PYTHON_VERSION"

if python3 -c "import sys; exit(0 if sys.version_info >= (3, 8) else 1)"; then
    log_success "Python 3.8+ disponible"
else
    log_error "Se requiere Python 3.8 o superior"
    exit 1
fi

# Verificar ROS2
if [ -z "$ROS_DISTRO" ]; then
    log_warning "ROS_DISTRO no está configurado"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        log_info "Configurando ROS2 Humble..."
        source /opt/ros/humble/setup.bash
        export ROS_DISTRO=humble
    else
        log_error "ROS2 no encontrado. Por favor instala ROS2 Humble"
        exit 1
    fi
fi

log_success "ROS2 $ROS_DISTRO disponible"

# Verificar ffmpeg para audio
if command -v ffmpeg &> /dev/null; then
    log_success "FFmpeg disponible"
else
    log_warning "FFmpeg no encontrado, instalando..."
    sudo apt update && sudo apt install -y ffmpeg
fi

echo ""
echo "🐍 Paso 2: Configurar entorno virtual Python"
echo "--------------------------------------------"

# Crear entorno virtual si no existe
if [ ! -d "$VENV_DIR" ]; then
    log_info "Creando entorno virtual en $VENV_DIR"
    python3 -m venv "$VENV_DIR"
else
    log_info "Entorno virtual ya existe"
fi

# Activar entorno virtual
log_info "Activando entorno virtual"
source "$VENV_DIR/bin/activate"

# Actualizar pip
log_info "Actualizando pip"
pip install --upgrade pip

echo ""
echo "📦 Paso 3: Instalar dependencias Python"
echo "---------------------------------------"

# Crear archivo de requirements si no existe
if [ ! -f "$REQUIREMENTS_FILE" ]; then
    log_info "Creando archivo de requirements"
    cat > "$REQUIREMENTS_FILE" << EOF
# Whisper y dependencias de audio
openai-whisper>=20231117
torch>=2.0.0
torchaudio>=2.0.0
librosa>=0.10.0
soundfile>=0.12.0

# FastAPI y servidor web
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
python-multipart>=0.0.6

# ROS2 Python (si no está instalado)
rclpy

# Utilidades adicionales
numpy>=1.21.0
scipy>=1.7.0
requests
EOF
fi

log_info "Instalando dependencias de Python..."
pip install -r "$REQUIREMENTS_FILE"

# Verificar instalación de Whisper
log_info "Verificando instalación de Whisper"
python3 -c "import whisper; print(f'Whisper version: {whisper.__version__}')" || {
    log_error "Error instalando Whisper"
    exit 1
}

log_success "Dependencias Python instaladas correctamente"

echo ""
echo "📁 Paso 4: Configurar archivos del proyecto"
echo "------------------------------------------"

# Crear directorio para ejecutables si no existe
mkdir -p "$TUTORIAL_PKG_DIR/tutorial_pkg"

# Verificar que el servicio FastAPI esté en su lugar
FASTAPI_SERVICE="$TUTORIAL_PKG_DIR/tutorial_pkg/whisper_fastapi_service.py"
if [ ! -f "$FASTAPI_SERVICE" ]; then
    log_error "Archivo whisper_fastapi_service.py no encontrado"
    log_error "Por favor, copia el archivo al directorio tutorial_pkg/tutorial_pkg/"
    exit 1
fi

# Hacer ejecutable el servicio FastAPI
chmod +x "$FASTAPI_SERVICE"
log_success "Servicio FastAPI configurado"

# Crear script de lanzamiento conveniente
LAUNCH_SCRIPT="$TUTORIAL_PKG_DIR/start_whisper_service.sh"
log_info "Creando script de lanzamiento: $LAUNCH_SCRIPT"

cat > "$LAUNCH_SCRIPT" << 'EOF'
#!/bin/bash
# Script para iniciar el servicio Whisper FastAPI

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/whisper_venv"
SERVICE_FILE="$SCRIPT_DIR/tutorial_pkg/whisper_fastapi_service.py"

echo "🤖 Iniciando Whisper FastAPI Service para tutorial_pkg"
echo "======================================================"

# Verificar entorno virtual
if [ ! -d "$VENV_DIR" ]; then
    echo "❌ Entorno virtual no encontrado en $VENV_DIR"
    echo "   Ejecuta setup_whisper_integration.sh primero"
    exit 1
fi

# Verificar archivo del servicio
if [ ! -f "$SERVICE_FILE" ]; then
    echo "❌ Servicio no encontrado en $SERVICE_FILE"
    exit 1
fi

# Configurar ROS2 si está disponible
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "🔧 Configurando ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Configurar workspace de ROS2
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    echo "🔧 Configurando workspace ROS2..."
    source "$HOME/ros2_ws/install/setup.bash"
fi

# Activar entorno virtual
echo "🐍 Activando entorno virtual Python..."
source "$VENV_DIR/bin/activate"

# Mostrar información del sistema
echo ""
echo "📊 Información del sistema:"
echo "   - Python: $(python3 --version)"
echo "   - ROS_DISTRO: ${ROS_DISTRO:-'No configurado'}"
echo "   - Whisper disponible: $(python3 -c "import whisper; print('✅')" 2>/dev/null || echo '❌')"
echo "   - ROS2 disponible: $(python3 -c "import rclpy; print('✅')" 2>/dev/null || echo '❌')"
echo ""

# Iniciar servicio
echo "🚀 Iniciando servicio en http://0.0.0.0:8000"
echo "   - Health check: http://localhost:8000/health"
echo "   - Documentación: http://localhost:8000/docs"
echo ""
echo "📱 La app Flutter puede conectarse a:"
echo "   - WSL2 IP (recomendado): http://$(hostname -I | awk '{print $1}'):8000"
echo "   - Localhost: http://localhost:8000"
echo ""
echo "Presiona Ctrl+C para detener el servicio"
echo "========================================="

cd "$SCRIPT_DIR"
python3 "$SERVICE_FILE"
EOF

chmod +x "$LAUNCH_SCRIPT"
log_success "Script de lanzamiento creado: $LAUNCH_SCRIPT"

echo ""
echo "⚙️  Paso 5: Configurar ejecutables ROS2"
echo "-------------------------------------"

# Actualizar setup.py para incluir el nuevo ejecutable
SETUP_PY="$TUTORIAL_PKG_DIR/setup.py"
if [ -f "$SETUP_PY" ]; then
    log_info "Verificando setup.py..."
    
    # Verificar si whisper_fastapi_service ya está en entry_points
    if grep -q "whisper_fastapi_service" "$SETUP_PY"; then
        log_info "whisper_fastapi_service ya está configurado en setup.py"
    else
        log_warning "Agregando whisper_fastapi_service a setup.py"
        log_info "Por favor, agrega manualmente a la sección entry_points:"
        log_info "'whisper_fastapi_service = tutorial_pkg.whisper_fastapi_service:main',"
    fi
else
    log_warning "setup.py no encontrado, creando uno básico..."
    cat > "$SETUP_PY" << 'EOF'
from setuptools import setup

package_name = 'tutorial_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    py_modules=[
        'tutorial_pkg.whisper_fastapi_service',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tutorial_pkg',
    maintainer_email='tutorial_pkg@todo.todo',
    description='Tutorial package with Whisper integration',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'whisper_fastapi_service = tutorial_pkg.whisper_fastapi_service:main',
        ],
    },
)
EOF
fi

echo ""
echo "🏗️  Paso 6: Compilar el workspace"
echo "--------------------------------"

# Navegar al workspace y compilar
cd "$HOME/ros2_ws"

log_info "Compilando workspace ROS2..."
if colcon build --packages-select tutorial_pkg; then
    log_success "Compilación exitosa"
else
    log_error "Error en la compilación"
    log_info "Puedes compilar manualmente con: cd ~/ros2_ws && colcon build --packages-select tutorial_pkg"
fi

# Configurar entorno
log_info "Configurando entorno ROS2..."
source install/setup.bash

echo ""
echo "🧪 Paso 7: Verificar instalación"
echo "-------------------------------"

cd "$TUTORIAL_PKG_DIR"

log_info "Verificando servicios..."

# Verificar entorno virtual
if [ -f "$VENV_DIR/bin/activate" ]; then
    log_success "Entorno virtual: ✅"
else
    log_error "Entorno virtual: ❌"
fi

# Verificar dependencias (en el entorno virtual)
source "$VENV_DIR/bin/activate"

DEPS_OK=true

if python3 -c "import whisper" 2>/dev/null; then
    log_success "Whisper: ✅"
else
    log_error "Whisper: ❌"
    DEPS_OK=false
fi

if python3 -c "import fastapi" 2>/dev/null; then
    log_success "FastAPI: ✅"
else
    log_error "FastAPI: ❌"
    DEPS_OK=false
fi

if python3 -c "import rclpy" 2>/dev/null; then
    log_success "ROS2 Python: ✅"
else
    log_error "ROS2 Python: ❌"
    DEPS_OK=false
fi

if python3 -c "import librosa" 2>/dev/null; then
    log_success "Librosa (audio processing): ✅"
else
    log_warning "Librosa (audio processing): ❌ (opcional)"
fi

echo ""
echo "📋 Paso 8: Crear documentación de uso"
echo "------------------------------------"

USAGE_DOC="$TUTORIAL_PKG_DIR/WHISPER_INTEGRATION_USAGE.md"
cat > "$USAGE_DOC" << 'EOF'
# Integración Whisper - Guía de Uso

## 🚀 Inicio Rápido

### 1. Iniciar el servicio Whisper FastAPI

```bash
cd ~/ros2_ws/src/tutorial_pkg
./start_whisper_service.sh
```

El servicio estará disponible en:
- **WSL2**: `http://<WSL2_IP>:8000` (recomendado para Flutter)
- **Local**: `http://localhost:8000`

### 2. Verificar que funciona

Abre en tu navegador: `http://localhost:8000/health`

Deberías ver algo como:
```json
{
  "status": "healthy",
  "services": {
    "whisper": true,
    "ros2": true,
    "audio_processing": true
  },
  "whisper_available": true
}
```

### 3. Conectar la app Flutter

La app Flutter se conectará automáticamente al servicio. Asegúrate de que:
- WSL2 esté ejecutándose
- El servicio FastAPI esté activo
- El sistema tutorial_pkg esté funcionando

## 🔧 Comandos Útiles

### Verificar estado de servicios
```bash
curl http://localhost:8000/health
```

### Ver estadísticas
```bash
curl http://localhost:8000/stats
```

### Probar transcripción (desde línea de comandos)
```bash
curl -X POST "http://localhost:8000/transcribe" \
     -H "accept: application/json" \
     -H "Content-Type: multipart/form-data" \
     -F "audio=@mi_audio.wav"
```

### Enviar comando de texto
```bash
curl -X POST "http://localhost:8000/send_text_command" \
     -H "Content-Type: application/json" \
     -d '{"command": "hola robot"}'
```

## 🐛 Solución de Problemas

### Error: "Whisper no disponible"
```bash
# Reinstalar Whisper
source ~/ros2_ws/src/tutorial_pkg/whisper_venv/bin/activate
pip install --upgrade openai-whisper
```

### Error: "ROS2 no disponible"
```bash
# Configurar ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Error: "No se puede conectar desde Flutter"
```bash
# Verificar IP de WSL2
hostname -I
# Usar esta IP en la app Flutter
```

### Performance lento
- Whisper usará GPU automáticamente si está disponible
- Para CPU, considera usar el modelo "tiny" o "base"
- Edita `whisper_fastapi_service.py` y cambia `model_name = "base"`

## 📱 Uso con Flutter

1. La app detectará automáticamente el servicio
2. Usa comandos de voz o texto
3. Los comandos se envían automáticamente al sistema ROS2 existente
4. No necesitas modificar nada del sistema actual

## 🔄 Integración con el Sistema Existente

El servicio Whisper actúa como un bridge:
- **Flutter** → **Whisper FastAPI** → **ROS2 /voice_commands** → **Sistema tutorial_pkg**

Todo funciona sin modificar el sistema existente.
EOF

log_success "Documentación creada: $USAGE_DOC"

echo ""
echo "✅ INSTALACIÓN COMPLETADA"
echo "========================="

if [ "$DEPS_OK" = true ]; then
    echo ""
    log_success "🎉 Integración Whisper instalada exitosamente!"
    echo ""
    echo "📋 Próximos pasos:"
    echo "1. Inicia el servicio: ./start_whisper_service.sh"
    echo "2. Verifica el estado: curl http://localhost:8000/health"
    echo "3. Conecta la app Flutter"
    echo ""
    echo "📖 Documentación completa en: $USAGE_DOC"
    echo ""
    echo "🚀 Para iniciar ahora:"
    echo "   cd $TUTORIAL_PKG_DIR"
    echo "   ./start_whisper_service.sh"
else
    echo ""
    log_error "⚠️  Instalación completada con errores"
    echo ""
    echo "Revisa los errores arriba y ejecuta el script de nuevo"
    echo "o instala manualmente las dependencias faltantes."
fi

deactivate 2>/dev/null || true
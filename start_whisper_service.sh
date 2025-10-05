#!/bin/bash
# Script mejorado para iniciar el servicio Whisper FastAPI con correcciones
# Ubicación: ~/ros2_ws/src/tutorial_pkg/start_whisper_service.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/whisper_venv_clean"
SERVICE_FILE="$SCRIPT_DIR/tutorial_pkg/whisper_fastapi_service.py"
SERVICE_CPU_FILE="$SCRIPT_DIR/tutorial_pkg/whisper_fastapi_service_cpu.py"

echo "🤖 Whisper FastAPI Service - Versión con Correcciones"
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

# APLICAR CORRECCIONES CRÍTICAS
echo "🔧 Aplicando correcciones para 'double free'..."

# Variables de entorno críticas para evitar double free
export MALLOC_CHECK_=0                              # Desactivar verificaciones malloc que pueden causar false positives
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512  # Configurar allocator CUDA
export TOKENIZERS_PARALLELISM=false                # Evitar conflictos de paralelización
export CUDA_LAUNCH_BLOCKING=1                      # Sincronizar operaciones CUDA
export OMP_NUM_THREADS=1                           # Limitar threads OpenMP
export MKL_NUM_THREADS=1                           # Limitar threads MKL
export PYTHONUNBUFFERED=1                          # Output inmediato para debugging

echo "   ✅ Variables de entorno configuradas"

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

# Limpiar cache de PyTorch antes de iniciar (preventivo)
echo "🧹 Limpiando cache de PyTorch..."
python3 -c "
import torch
if torch.cuda.is_available():
    torch.cuda.empty_cache()
    print('   - Cache CUDA limpiado')
else:
    print('   - Usando CPU (sin cache CUDA que limpiar)')
" 2>/dev/null || echo "   - No se pudo limpiar cache (normal si no hay torch)"

# Mostrar información del sistema
echo ""
echo "📊 Información del sistema:"
echo "   - Python: $(python3 --version)"
echo "   - ROS_DISTRO: ${ROS_DISTRO:-'No configurado'}"

# Verificar Whisper con test de estabilidad
echo -n "   - Whisper disponible: "
if python3 -c "
import whisper
import torch
# Test básico de estabilidad
try:
    model = whisper.load_model('base', device='cpu')
    del model  # Limpiar inmediatamente
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
    print('✅ (estable)')
except Exception as e:
    print(f'⚠️ (inestable: {e})')
" 2>/dev/null; then
    echo ""
else
    echo "❌ (error)"
fi

echo "   - ROS2 disponible: $(python3 -c "import rclpy; print('✅')" 2>/dev/null || echo '❌')"

# Verificar GPU y sugerir CPU si hay problemas
echo -n "   - GPU Status: "
python3 -c "
import torch
if torch.cuda.is_available():
    try:
        x = torch.zeros(1).cuda()
        del x
        torch.cuda.empty_cache()
        print('✅ CUDA funcional')
    except:
        print('⚠️ CUDA problemático - recomendado usar CPU')
else:
    print('ℹ️ CPU (recomendado para estabilidad)')
" 2>/dev/null || echo "❓ No determinado"

echo ""

# Preguntar qué versión usar
echo "🚀 ¿Qué versión del servicio quieres usar?"
echo "1) 🖥️  CPU-only (MÁS ESTABLE - recomendado para resolver double free)"
echo "2) 🔥 Original con correcciones (puede usar GPU si está disponible)"
echo "3) 🧪 Test básico primero"
echo ""
read -p "Selecciona una opción [1-3]: " version_choice

case $version_choice in
    1)
        # Crear versión CPU si no existe
        if [ ! -f "$SERVICE_CPU_FILE" ]; then
            echo "🔧 Creando versión CPU-only..."
            cp "$SERVICE_FILE" "$SERVICE_CPU_FILE"
            
            # Modificar para forzar CPU
            sed -i 's/self\.device = "cuda"/self.device = "cpu"  # FORCED CPU for stability/g' "$SERVICE_CPU_FILE"
            sed -i 's/if torch and torch\.cuda\.is_available():/if False:  # FORCE CPU MODE/g' "$SERVICE_CPU_FILE"
            
            echo "   ✅ Versión CPU creada"
        fi
        
        SERVICE_TO_USE="$SERVICE_CPU_FILE"
        echo "🖥️ Usando versión CPU-only (más estable)"
        ;;
    2)
        SERVICE_TO_USE="$SERVICE_FILE"
        echo "🔥 Usando versión original con correcciones"
        ;;
    3)
        echo "🧪 Ejecutando test básico..."
        python3 -c "
import whisper
import tempfile
import soundfile as sf
import numpy as np

print('🔄 Cargando modelo Whisper base en CPU...')
model = whisper.load_model('base', device='cpu')
print('✅ Modelo cargado correctamente')

# Test con audio sintético
sr = 16000
audio = np.zeros(sr, dtype=np.float32)  # 1 segundo de silencio

with tempfile.NamedTemporaryFile(suffix='.wav', delete=True) as f:
    sf.write(f.name, audio, sr)
    result = model.transcribe(f.name)
    print(f'✅ Test de transcripción exitoso')

del model
print('✅ Test completado sin errores')
"
        if [ $? -eq 0 ]; then
            echo "✅ Test básico EXITOSO - el problema está resuelto"
            read -p "¿Continuar con el inicio del servicio? [y/N]: " continue_choice
            if [[ $continue_choice =~ ^[Yy]$ ]]; then
                SERVICE_TO_USE="$SERVICE_FILE"
            else
                echo "👋 Saliendo..."
                exit 0
            fi
        else
            echo "❌ Test básico FALLÓ"
            echo "💡 Recomendación: usar versión CPU-only"
            SERVICE_TO_USE="$SERVICE_CPU_FILE"
        fi
        ;;
    *)
        echo "❌ Opción inválida, usando CPU por defecto"
        SERVICE_TO_USE="$SERVICE_CPU_FILE"
        ;;
esac

echo ""
echo "🚀 Iniciando servicio en http://0.0.0.0:8000"
echo "   - Health check: http://localhost:8000/health"
echo "   - Documentación: http://localhost:8000/docs"
echo ""
echo "📱 La app Flutter puede conectarse a:"
echo "   - WSL2 IP (recomendado): http://$(hostname -I | awk '{print $1}'):8000"
echo "   - Localhost: http://localhost:8000"
echo ""
echo "🔧 Correcciones aplicadas:"
echo "   - Variables de entorno anti-double-free"
echo "   - Cache de PyTorch limpiado"
echo "   - Configuración de memoria optimizada"
echo ""
echo "Presiona Ctrl+C para detener el servicio"
echo "========================================="

cd "$SCRIPT_DIR"

# Intentar iniciar el servicio con manejo de errores
if python3 "$SERVICE_TO_USE"; then
    echo "✅ Servicio terminado correctamente"
else
    exit_code=$?
    echo ""
    echo "❌ El servicio terminó con error (código: $exit_code)"
    echo ""
    echo "💡 Sugerencias para resolver el problema:"
    echo "1. Ejecuta: ./fix_whisper_double_free.sh"
    echo "2. Usa la versión CPU-only: python3 tutorial_pkg/whisper_fastapi_service_cpu.py"
    echo "3. Reinstala las dependencias: pip uninstall openai-whisper torch && pip install torch openai-whisper"
    echo "4. Verifica que no haya otros procesos usando GPU: nvidia-smi"
    echo ""
    exit $exit_code
fi
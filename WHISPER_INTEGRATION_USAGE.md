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

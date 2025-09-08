# Tutorial PKG - Sistema Integrado de Exploración Autónoma con Control por Voz IA

## Descripción General

Sistema integral para TurtleBot3 que combina:
- **Exploración autónoma** con SLAM en Gazebo
- **Control por voz inteligente** con IA (Whisper + Gemini)
- **Monitor avanzado de exploración** con recuperación automática
- **Integración con apps móviles** via Flask/WebSocket
- **Visualización completa** en RViz

## Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                    TUTORIAL_PKG INTEGRADO                   │
├─────────────────┬─────────────────┬─────────────────────────┤
│   SIMULACIÓN    │   NAVEGACIÓN    │    CONTROL POR VOZ      │
│                 │                 │                         │
│ • Gazebo        │ • SLAM Toolbox  │ • Whisper (STT)         │
│ • TurtleBot3    │ • Nav2 Stack    │ • Gemini AI (NLP)       │
│ • Mundo custom  │ • Explore Lite  │ • Flask Server          │
│                 │ • Costmaps      │ • WebSocket Real-time   │
├─────────────────┼─────────────────┼─────────────────────────┤
│             NODOS DE INTEGRACIÓN                            │
│                                                             │
│ • ai_voice_commander.py    - Procesamiento de comandos     │
│ • ai_response_node.py      - Respuestas inteligentes       │
│ • exploration_monitor.py   - Monitor avanzado              │
│ • flask_server.py          - API para apps móviles         │
└─────────────────────────────────────────────────────────────┘
```

## Instalación Rápida

### 1. Configuración Automática
```bash
cd ~/ros2_ws/src/tutorial_pkg
chmod +x setup_integration.sh
./setup_integration.sh
```

### 2. Configuración Manual (si es necesaria)

#### Dependencias ROS2
```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-description \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-rviz-plugins \
  ros-humble-explore-lite
```

#### Compilar el paquete
```bash
cd ~/ros2_ws
colcon build --packages-select tutorial_pkg --symlink-install
source install/setup.bash
```

## Uso del Sistema

### Método 1: Script de Ejecución Rápida
```bash
cd ~/ros2_ws/src/tutorial_pkg
./run_integrated_exploration.sh
```

### Método 2: Launch File Directo
```bash
# Solo exploración autónoma
ros2 launch tutorial_pkg integrated_exploration_launch.py

# Con control por voz
ros2 launch tutorial_pkg integrated_exploration_launch.py enable_voice_control:=true

# Con control por voz + servidor Flask
ros2 launch tutorial_pkg integrated_exploration_launch.py \
  enable_voice_control:=true \
  enable_flask_server:=true
```

### Método 3: Componentes Individuales
```bash
# Solo el sistema de control por voz
ros2 launch tutorial_pkg voice_control_launch.py

# Solo el monitor de exploración
ros2 run tutorial_pkg exploration_monitor.py

# Solo servicios de voz (Flask)
cd voice_services && source venv/bin/activate
python3 flask_server.py
```

## Comandos de Voz Soportados

### Movimiento Básico
- **"adelante"**, "avanzar", "forward" → Mover hacia adelante
- **"atrás"**, "retroceder", "backward" → Mover hacia atrás  
- **"izquierda"**, "girar izquierda" → Girar a la izquierda
- **"derecha"**, "girar derecha" → Girar a la derecha
- **"parar"**, "stop", "detener" → Detener movimiento

### Control de Exploración
- **"explorar"**, "mapear" → Iniciar exploración autónoma
- **"pausar exploración"** → Pausar exploración
- **"continuar exploración"** → Reanudar exploración
- **"terminar exploración"** → Finalizar exploración

### Control de Velocidad
- **"más rápido"**, "acelerar" → Aumentar velocidad
- **"más lento"**, "despacio" → Reducir velocidad
- **"velocidad normal"** → Restablecer velocidad por defecto

### Información y Estado
- **"estado"**, "cómo estás" → Reporte de estado del robot
- **"estadísticas"** → Estadísticas de exploración
- **"progreso del mapa"** → Información del mapeo

### Emergencia
- **"emergencia"**, "ayuda" → Parada de emergencia

## Integración con Apps Móviles

### API REST (Flask)
El servidor Flask proporciona endpoints para integración:

```http
GET  /health              - Estado del sistema
POST /process_audio       - Procesar audio (desde app móvil)
POST /send_text_command   - Enviar comando de texto
GET  /robot_status        - Estado detallado del robot
GET  /server_stats        - Estadísticas del servidor
```

### WebSocket Real-time
```javascript
// Conectar a WebSocket
const socket = io('http://IP_ROBOT:8000');

// Enviar comando de voz
socket.emit('voice_command_ws', {
    command: 'explorar',
    process_ai: true,
    send_robot: true
});

// Recibir respuesta
socket.on('command_result', (data) => {
    console.log('Respuesta IA:', data.ai_response);
    console.log('Acción robot:', data.robot_action);
});
```

## Configuración Avanzada

### Parámetros de Exploración
Editar `config/voice_control.yaml`:
```yaml
exploration_monitor:
  ros__parameters:
    map_growth_timeout: 45.0
    position_stuck_timeout: 30.0
    min_frontier_distance: 1.2
    max_exploration_distance: 8.0
    enable_auto_recovery: true
```

### Parámetros de Voz
```yaml
ai_voice_commander:
  ros__parameters:
    linear_speed_default: 0.22
    angular_speed_default: 0.35
    auto_stop_timeout: 5.0
    enable_ai_responses: true
```

### Variables de Entorno
```bash
# En ~/.bashrc o antes de ejecutar
export TURTLEBOT3_MODEL=waffle
export TUTORIAL_PKG_CONFIG_DIR=~/ros2_ws/src/tutorial_pkg/config
export FLASK_PORT=8000
export GEMINI_API_KEY=tu_api_key_aquí  # Opcional
```

## Estructura del Proyecto

```
tutorial_pkg/
├── launch/
│   ├── integrated_exploration_launch.py  # Launch principal integrado
│   ├── voice_control_launch.py           # Solo control por voz
│   └── explore_robust_fixed.launch.py    # Solo exploración
├── config/
│   ├── voice_control.yaml                # Configuración sistema de voz
│   ├── slam_simple.yaml                  # Configuración SLAM
│   └── navigation_simple.yaml            # Configuración Nav2
├── tutorial_pkg/
│   ├── ai_voice_commander.py             # Nodo principal de voz
│   ├── ai_response_node.py               # Respuestas IA
│   └── exploration_monitor.py            # Monitor avanzado
├── voice_services/
│   ├── flask_server.py                   # Servidor web
│   ├── whisper_service.py               # Servicio STT
│   ├── gemini_service.py                # Servicio IA
│   └── command_processor.py             # Procesador comandos
├── rviz/
│   └── integrated_exploration.rviz      # Configuración RViz
├── worlds/
│   └── salon_world.world                # Mundo Gazebo
├── scripts/
│   ├── run_integrated_exploration.sh    # Script ejecución
│   ├── test_system.sh                   # Script pruebas
│   └── setup_integration.sh             # Script instalación
└── maps/                                # Mapas generados
```

## Tópicos ROS2 Principales

### Comandos y Control
- `/voice_commands` (std_msgs/String) - Comandos de voz entrantes
- `/voice_feedback` (std_msgs/String) - Respuestas del sistema
- `/exploration_control` (std_msgs/String) - Control de exploración
- `/cmd_vel` (geometry_msgs/Twist) - Comandos de velocidad

### Estado y Monitoreo  
- `/ai_status` (std_msgs/String) - Estado de IA
- `/robot_state` (std_msgs/String) - Estado completo del robot
- `/ai_context` (std_msgs/String) - Contexto para IA

### Navegación y SLAM
- `/map` (nav_msgs/OccupancyGrid) - Mapa SLAM
- `/scan` (sensor_msgs/LaserScan) - Datos del lidar
- `/odom` (nav_msgs/Odometry) - Odometría
- `/goal_pose` (geometry_msgs/PoseStamped) - Objetivos de navegación

## Solución de Problemas

### Problemas Comunes

#### 1. ROS2 no encuentra el paquete
```bash
cd ~/ros2_ws
colcon build --packages-select tutorial_pkg
source install/setup.bash
```

#### 2. Gazebo no carga el mundo
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/tutorial_pkg/models
```

#### 3. Servicios de voz no funcionan
```bash
cd ~/ros2_ws/src/tutorial_pkg/voice_services
source venv/bin/activate
pip install --upgrade -r requirements.txt
```

#### 4. Robot se queda atascado
- El monitor de exploración debería recuperarlo automáticamente
- Si no, usar comando de voz: "emergencia" o "parar"

#### 5. No hay respuesta de IA
- Verificar conexión a internet (Gemini API)
- Revisar logs: `tail -f ~/ros2_ws/src/tutorial_pkg/logs/voice_control.log`

### Logs y Debugging
```bash
# Ver logs en tiempo real
tail -f ~/ros2_ws/src/tutorial_pkg/logs/voice_control.log
tail -f ~/.ros/log/latest/tutorial_pkg/

# Verificar tópicos activos
ros2 topic list | grep -E "(voice|ai|exploration)"

# Monitorear estado del robot
ros2 topic echo /robot_state

# Ver comandos de voz recibidos
ros2 topic echo /voice_commands
```

## Desarrollo y Extensión

### Agregar Nuevos Comandos de Voz
Editar `tutorial_pkg/ai_voice_commander.py`:
```python
self.command_map = {
    r'\b(mi_comando|my_command)\b': self.my_custom_action,
    # ... otros comandos
}

def my_custom_action(self):
    # Implementar acción personalizada
    pass
```

### Personalizar Respuestas de IA
Editar `voice_services/gemini_service.py`:
```python
system_prompt = """
Eres un robot personalizado que puede:
- Tu funcionalidad personalizada aquí
- Responder en el estilo que prefieras
"""
```

### Crear Nuevos Nodos de Integración
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tutorial_pkg.base_integration_node import BaseIntegrationNode

class MyCustomNode(BaseIntegrationNode):
    def __init__(self):
        super().__init__('my_custom_node')
        # Tu implementación aquí
```

## Contribuir

1. Fork el repositorio
2. Crear branch: `git checkout -b feature/nueva-funcionalidad`
3. Commit cambios: `git commit -am 'Agregar nueva funcionalidad'`
4. Push al branch: `git push origin feature/nueva-funcionalidad`
5. Crear Pull Request

## Licencia

Este proyecto está bajo la licencia MIT. Ver archivo LICENSE para detalles.

## Contacto y Soporte

- **GitHub Issues**: Para reportar bugs o solicitar funcionalidades
- **Documentación**: Wiki del repositorio para información detallada
- **Ejemplos**: Directorio `examples/` para casos de uso específicos

---

**Versión**: 1.1.0  
**Compatibilidad**: ROS2 Humble, Ubuntu 22.04  
**Última actualización**: 2024
# ROS2 SLAM Explorer con TurtleBot3 🤖🗺️

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/download/desktop)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()

> **Exploración automática de entornos con TurtleBot3 usando SLAM en ROS2**

Este proyecto permite que un robot TurtleBot3 explore automáticamente un entorno usando sensores LIDAR, genere un mapa mediante SLAM (Simultaneous Localization and Mapping) y guarde el resultado. Compatible con entornos predefinidos y modelos 3D personalizados en formato `.dae`.

## 📋 Tabla de Contenidos

- [🚀 Instalación Rápida](#-instalación-rápida)
- [📋 Prerrequisitos](#-prerrequisitos)
- [🛠️ Instalación Manual](#️-instalación-manual)
- [🎮 Uso del Sistema](#-uso-del-sistema)
- [📊 Monitoreo y Visualización](#-monitoreo-y-visualización)
- [🧩 Estructura del Proyecto](#-estructura-del-proyecto)
- [🔧 Personalización](#-personalización)
- [🐛 Solución de Problemas](#-solución-de-problemas)
- [📚 Recursos Adicionales](#-recursos-adicionales)
- [🤝 Contribuir](#-contribuir)

## 🚀 Instalación Rápida

### Opción 1: Script Automático (Recomendada)

```bash
# Descargar e instalar automáticamente
wget https://raw.githubusercontent.com/JuanSO121/Introduction_slam_explorer_ros2/main/install_slam_explorer.sh
chmod +x install_slam_explorer.sh
./install_slam_explorer.sh
```

**El script automático realizará:**
- ✅ Verificación de ROS2 Humble
- ✅ Instalación de dependencias del sistema
- ✅ Clonado de repositorios necesarios
- ✅ Compilación del workspace
- ✅ Configuración del entorno
- ✅ Verificación de la instalación

## 📋 Prerrequisitos

### Sistema Operativo
- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- **ROS2 Humble** instalado

### Hardware Mínimo Recomendado
| Componente | Mínimo | Recomendado |
|------------|--------|-------------|
| **RAM** | 8 GB | 16 GB |
| **CPU** | Intel i5 / AMD equivalente | Intel i7 / AMD Ryzen 7 |
| **GPU** | Integrada | Dedicada (para Gazebo) |
| **Espacio** | 10 GB libres | 20 GB libres |

## 🛠️ Instalación Manual

<details>
<summary><b>Click para ver instrucciones paso a paso</b></summary>

### Paso 1: Verificar ROS2 Humble

```bash
# Verificar que ROS2 Humble esté instalado
source /opt/ros/humble/setup.bash
ros2 --version
```

> **Nota:** Si no tienes ROS2 Humble, sigue la [guía oficial de instalación](https://docs.ros.org/en/humble/Installation.html)

### Paso 2: Instalar Dependencias del Sistema

```bash
# Actualizar sistema
sudo apt update && sudo apt upgrade -y

# Instalar herramientas básicas
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git wget curl build-essential cmake

# Instalar paquetes ROS2 necesarios
sudo apt install -y \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-navigation2 \
    ros-humble-nav2-map-server \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs
```

### Paso 3: Crear y Configurar Workspace

```bash
# Crear workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clonar repositorio m-explore-ros2
git clone https://github.com/robo-friends/m-explore-ros2.git

# Clonar el proyecto principal
git clone https://github.com/JuanSO121/Introduction_slam_explorer_ros2.git

# Mover tutorial_pkg al nivel correcto
mv Introduction_slam_explorer_ros2/tutorial_pkg ./
rm -rf Introduction_slam_explorer_ros2
```

### Paso 4: Instalar Dependencias de Python

```bash
pip3 install --user \
    transforms3d \
    numpy \
    matplotlib \
    scipy
```

### Paso 5: Compilar el Workspace

```bash
cd ~/ros2_ws

# Inicializar rosdep (solo la primera vez)
sudo rosdep init  # Omitir si ya está inicializado
rosdep update

# Instalar dependencias
rosdep install --from-paths src --ignore-src -r -y

# Compilar
colcon build --parallel-workers 4

# Source el workspace
source install/setup.bash
```

### Paso 6: Configurar Variables de Entorno

Añadir al final de `~/.bashrc`:

```bash
nano ~/.bashrc
```

```bash
# ============================================================================
# ROS2 HUMBLE CONFIGURATION - SLAM EXPLORER PROJECT
# ============================================================================

# Locale configuration
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# Source ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source local workspace if available
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Gazebo configuration
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/tutorial_pkg/models

# ROS2 Domain ID
export ROS_DOMAIN_ID=30

# Aliases útiles
alias cdws='cd ~/ros2_ws'
alias build-ws='cd ~/ros2_ws && colcon build && source install/setup.bash'
alias start-salon-gazebo='ros2 launch tutorial_pkg salon_gazebo_fixed.launch.py'
alias start-exploration='ros2 launch tutorial_pkg explore_robust_fixed.launch.py'
alias monitor-exploration='ros2 run tutorial_pkg exploration_monitor'
alias save-map='ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map'
```

```bash
# Recargar bashrc
source ~/.bashrc
```

</details>

## 🎮 Uso del Sistema

### Verificar Instalación

```bash
# Verificar que los paquetes estén disponibles
ros2 pkg list | grep -E "(tutorial_pkg|explore_lite|slam_toolbox)"

# Estado del sistema
ros2 node list
ros2 topic list
```

### Ejecutar Exploración del Salón

Abre **4 terminales** y ejecuta los siguientes comandos:

#### 📺 Terminal 1: Iniciar Simulador de Gazebo
```bash
start-salon-gazebo
# O alternativamente:
# ros2 launch tutorial_pkg salon_gazebo_fixed.launch.py
```

#### 🗺️ Terminal 2: Iniciar Exploración Automática
```bash
start-exploration
# O alternativamente:
# ros2 launch tutorial_pkg explore_robust_fixed.launch.py
```

#### 📊 Terminal 3: Monitorear Exploración (Opcional)
```bash
monitor-exploration
# O alternativamente:
# ros2 run tutorial_pkg exploration_monitor
```

#### 🎮 Terminal 4: Control Manual (Opcional)
```bash
# Si necesitas controlar manualmente el robot
ros2 run turtlebot3_teleop teleop_keyboard
```

### 💾 Guardar el Mapa

Una vez completada la exploración:

```bash
# Guardar mapa en directorio actual
ros2 run nav2_map_server map_saver_cli -f salon_map

# O guardar en directorio específico con timestamp
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/mi_salon_$(date +%Y%m%d_%H%M%S)
```

## 📊 Monitoreo y Visualización

### RViz para Visualización
- **RViz se abre automáticamente** con la configuración óptima
- **Temas importantes a monitorear**:
  - `/map` - Mapa generado
  - `/scan` - Datos del LIDAR
  - `/explore/frontiers` - Fronteras de exploración
  - `/move_base/global_costmap/costmap` - Mapa de costos

### Tópicos ROS2 Útiles

```bash
# Ver mapa en tiempo real
ros2 topic echo /map

# Ver datos del LIDAR
ros2 topic echo /scan

# Ver objetivos de exploración
ros2 topic echo /move_base_simple/goal

# Estado de la navegación
ros2 topic echo /navigate_to_pose/_action/status
```

## 🧩 Estructura del Proyecto

```
tutorial_pkg/
├── 🚀 launch/                     # Archivos de lanzamiento
│   ├── salon_gazebo_fixed.launch.py    # Gazebo con modelo de salón
│   └── explore_robust_fixed.launch.py  # Exploración automática
├── 🏠 models/                     # Modelos 3D
│   └── salon_model/               # Modelo del salón (.dae)
├── ⚙️ config/                     # Configuraciones
│   ├── slam_salon.yaml            # Configuración SLAM
│   └── navigation_simple.yaml     # Configuración navegación
├── 🌍 worlds/                     # Mundos de Gazebo
│   └── salon_world.world          # Mundo del salón
├── 👁️ rviz/                       # Configuraciones RViz
├── 🗺️ maps/                       # Mapas guardados
├── 🐍 tutorial_pkg/               # Nodos Python
│   ├── exploration_monitor.py
│   ├── exploration_restarter.py
│   └── exploration_visualizer.py
└── 📜 scripts/                    # Scripts auxiliares
```

## 🔧 Personalización

<details>
<summary><b>Usar tu Propio Modelo 3D</b></summary>

### 1. Convertir modelo a .dae
```bash
# Usando Blender (recomendado)
# File → Export → Collada (.dae)
```

### 2. Colocar modelo
```bash
cp tu_modelo.dae ~/ros2_ws/src/tutorial_pkg/models/salon_model/meshes/
```

### 3. Modificar configuración
```bash
# Editar el archivo SDF del modelo
nano ~/ros2_ws/src/tutorial_pkg/models/salon_model/model.sdf
```

</details>

<details>
<summary><b>Ajustar Parámetros de Exploración</b></summary>

```bash
# Editar configuración de exploración
nano ~/ros2_ws/src/tutorial_pkg/config/slam_salon.yaml
```

**Parámetros importantes:**
- `planner_frequency`: Frecuencia de planificación
- `gain_scale`: Ganancia para fronteras
- `min_frontier_size`: Tamaño mínimo de frontera

</details>

## 🐛 Solución de Problemas

<details>
<summary><b>❌ Gazebo no inicia correctamente</b></summary>

```bash
# Limpiar cache de Gazebo
rm -rf ~/.gazebo/models
rm -rf /tmp/.gazebo

# Reiniciar con software rendering
export LIBGL_ALWAYS_SOFTWARE=1
start-salon-gazebo
```

</details>

<details>
<summary><b>❌ Robot no se mueve</b></summary>

```bash
# Verificar nodos activos
ros2 node list

# Verificar tópicos
ros2 topic list | grep cmd_vel

# Probar movimiento manual
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.1}" --once
```

</details>

<details>
<summary><b>❌ SLAM no funciona</b></summary>

```bash
# Verificar que el LIDAR publique datos
ros2 topic echo /scan --once

# Reiniciar SLAM
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

</details>

<details>
<summary><b>❌ Exploración se detiene</b></summary>

```bash
# Usar el monitor de exploración
monitor-exploration

# O reiniciar exploración
ros2 run tutorial_pkg exploration_restarter
```

</details>

## 📚 Recursos Adicionales

### 📖 Documentación Oficial
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

### 🎓 Tutoriales Relacionados
- [Husarion Exploration Tutorial](https://husarion.com/tutorials/ros2-tutorials/10-exploration/) - Inspiración principal
- [ROS2 Navigation Tutorial](https://navigation.ros.org/tutorials/)

### 🔗 Repositorios Útiles
- [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) - Paquete de exploración
- [TurtleBot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

## 🤝 Contribuir

Las contribuciones son bienvenidas. Para contribuir:

1. **Fork** el repositorio
2. **Crea** una rama para tu feature (`git checkout -b feature/nueva-feature`)
3. **Commit** tus cambios (`git commit -am 'Añadir nueva feature'`)
4. **Push** a la rama (`git push origin feature/nueva-feature`)
5. **Crea** un Pull Request

## 📄 Licencia

Este proyecto está bajo la Licencia Apache 2.0. Ver archivo [LICENSE](LICENSE) para más detalles.

## 🙏 Agradecimientos

- **[Husarion](https://husarion.com/)** por el tutorial de exploración que sirvió de inspiración
- **[ROS2 Community](https://docs.ros.org/)** por el excelente ecosistema
- **[m-explore-ros2 team](https://github.com/robo-friends/m-explore-ros2)** por el paquete de exploración

## 📞 Soporte

Si tienes problemas con la instalación o uso:

1. **📖 Revisa** la sección de [solución de problemas](#-solución-de-problemas)
2. **🔍 Busca** en los [issues del repositorio](../../issues)
3. **➕ Crea** un [nuevo issue](../../issues/new) con detalles del problema

**🚨 Al reportar problemas, incluye siempre:**
- Versión de Ubuntu
- Versión de ROS2
- Logs de error completos
- Pasos para reproducir el problema

---

<div align="center">

**¡Disfruta explorando con tu TurtleBot3! 🤖🗺️**

[![GitHub stars](https://img.shields.io/github/stars/JuanSO121/Introduction_slam_explorer_ros2?style=social)](https://github.com/JuanSO121/Introduction_slam_explorer_ros2/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/JuanSO121/Introduction_slam_explorer_ros2?style=social)](https://github.com/JuanSO121/Introduction_slam_explorer_ros2/network)

</div>

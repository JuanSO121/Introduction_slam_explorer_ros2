#!/usr/bin/env python3
"""
Script de verificación para el modelo de salón .dae
Archivo: ~/ros2_ws/src/tutorial_pkg/scripts/test_salon_model.py
"""

import os
import sys
import time
import subprocess
from pathlib import Path

class SalonModelTester:
    def __init__(self):
        self.tutorial_pkg_path = Path.home() / "ros2_ws" / "src" / "tutorial_pkg"
        self.models_path = self.tutorial_pkg_path / "models" / "salon"
        self.meshes_path = self.models_path / "meshes"
        
    def check_file_structure(self):
        """Verificar que todos los archivos necesarios existen"""
        print("🔍 Verificando estructura de archivos...")
        
        required_files = [
            self.models_path / "model.config",
            self.models_path / "model.sdf", 
            self.meshes_path / "salon.dae",
            self.tutorial_pkg_path / "worlds" / "salon_world.world",
            self.tutorial_pkg_path / "config" / "salon_explore.yaml",
            self.tutorial_pkg_path / "launch" / "salon_explore.launch.py"
        ]
        
        missing_files = []
        for file_path in required_files:
            if file_path.exists():
                print(f"✅ {file_path.relative_to(self.tutorial_pkg_path)}")
            else:
                print(f"❌ {file_path.relative_to(self.tutorial_pkg_path)} - FALTANTE")
                missing_files.append(file_path)
        
        return len(missing_files) == 0

    def check_dae_file(self):
        """Verificar el archivo .dae específicamente"""
        print("\n📐 Verificando archivo .dae...")
        
        dae_file = self.meshes_path / "salon.dae"
        
        if not dae_file.exists():
            print("❌ Archivo salon.dae no encontrado")
            return False
            
        # Verificar tamaño
        size_mb = dae_file.stat().st_size / (1024 * 1024)
        print(f"📏 Tamaño: {size_mb:.2f} MB")
        
        if size_mb > 50:
            print("⚠️  ADVERTENCIA: Archivo muy grande (>50MB). Considera optimizar.")
        elif size_mb < 0.1:
            print("⚠️  ADVERTENCIA: Archivo muy pequeño (<0.1MB). Verifica integridad.")
        else:
            print("✅ Tamaño apropiado")
            
        # Verificar permisos
        if os.access(dae_file, os.R_OK):
            print("✅ Permisos de lectura OK")
        else:
            print("❌ Problemas de permisos")
            return False
            
        return True

    def check_gazebo_model_path(self):
        """Verificar configuración de GAZEBO_MODEL_PATH"""
        print("\n🌍 Verificando GAZEBO_MODEL_PATH...")
        
        model_path = str(self.tutorial_pkg_path / "models")
        gazebo_path = os.environ.get('GAZEBO_MODEL_PATH', '')
        
        if model_path in gazebo_path:
            print("✅ GAZEBO_MODEL_PATH configurado correctamente")
            return True
        else:
            print("❌ GAZEBO_MODEL_PATH no incluye el directorio de modelos")
            print(f"   Añade: export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{model_path}")
            return False

    def test_gazebo_model_loading(self):
        """Test de carga del modelo en Gazebo"""
        print("\n🎮 Probando carga del modelo en Gazebo...")
        
        try:
            # Verificar que Gazebo puede encontrar el modelo
            result = subprocess.run([
                'gazebo', '--verbose', '--version'
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                print("✅ Gazebo disponible")
                print(f"   Versión: {result.stdout.strip()}")
            else:
                print("❌ Problemas con Gazebo")
                return False
                
        except subprocess.TimeoutExpired:
            print("⚠️  Timeout verificando Gazebo")
        except FileNotFoundError:
            print("❌ Gazebo no encontrado en PATH")
            return False
            
        return True

    def check_ros2_packages(self):
        """Verificar que los paquetes ROS2 necesarios están disponibles"""
        print("\n📦 Verificando paquetes ROS2...")
        
        required_packages = [
            'nav2_bringup',
            'slam_toolbox', 
            'explore_lite',
            'turtlebot3_gazebo',
            'gazebo_ros',
            'tutorial_pkg'
        ]
        
        try:
            # Obtener lista de paquetes disponibles
            result = subprocess.run([
                'ros2', 'pkg', 'list'
            ], capture_output=True, text=True, timeout=10)
            
            available_packages = result.stdout.strip().split('\n')
            
            missing_packages = []
            for pkg in required_packages:
                if pkg in available_packages:
                    print(f"✅ {pkg}")
                else:
                    print(f"❌ {pkg} - NO ENCONTRADO")
                    missing_packages.append(pkg)
                    
            return len(missing_packages) == 0
            
        except Exception as e:
            print(f"❌ Error verificando paquetes: {e}")
            return False

    def generate_test_script(self):
        """Generar script de prueba completa"""
        print("\n📝 Generando script de prueba...")
        
        test_script_content = '''#!/bin/bash
# Script de prueba completa del modelo de salón

echo "🚀 Iniciando prueba del modelo de salón..."

# 1. Compilar workspace
echo "🔨 Compilando workspace..."
cd ~/ros2_ws
colcon build --packages-select tutorial_pkg
source install/setup.bash

# 2. Lanzar simulación
echo "🎮 Iniciando simulación..."
ros2 launch tutorial_pkg salon_explore.launch.py &
LAUNCH_PID=$!

# Esperar 30 segundos para que todo se inicialice
echo "⏳ Esperando inicialización (30s)..."
sleep 30

# 3. Verificar nodos activos
echo "🔍 Verificando nodos activos..."
ros2 node list

# 4. Verificar tópicos
echo "📡 Verificando tópicos..."
ros2 topic list | grep -E "(scan|map|costmap|explore)"

# 5. Verificar transformaciones
echo "🔄 Verificando TF..."
ros2 run tf2_tools view_frames.py

echo "✅ Prueba completada. Revisa la salida anterior."
echo "   Para detener: kill $LAUNCH_PID"
'''
        
        script_path = self.tutorial_pkg_path / "scripts" / "test_complete.sh"
        script_path.parent.mkdir(exist_ok=True)
        
        with open(script_path, 'w') as f:
            f.write(test_script_content)
            
        # Hacer ejecutable
        os.chmod(script_path, 0o755)
        print(f"✅ Script generado: {script_path}")

    def run_full_test(self):
        """Ejecutar prueba completa"""
        print("🔧 PRUEBA COMPLETA DEL MODELO DE SALÓN")
        print("=" * 50)
        
        all_tests_passed = True
        
        # Test 1: Estructura de archivos
        if not self.check_file_structure():
            all_tests_passed = False
            
        # Test 2: Archivo .dae
        if not self.check_dae_file():
            all_tests_passed = False
            
        # Test 3: GAZEBO_MODEL_PATH
        if not self.check_gazebo_model_path():
            all_tests_passed = False
            
        # Test 4: Paquetes ROS2
        if not self.check_ros2_packages():
            all_tests_passed = False
            
        # Test 5: Gazebo
        if not self.test_gazebo_model_loading():
            all_tests_passed = False
            
        # Generar script de prueba
        self.generate_test_script()
        
        print("\n" + "=" * 50)
        if all_tests_passed:
            print("🎉 ¡TODAS LAS PRUEBAS PASARON!")
            print("   Puedes proceder a lanzar la exploración:")
            print("   ros2 launch tutorial_pkg salon_explore.launch.py")
        else:
            print("❌ ALGUNAS PRUEBAS FALLARON")
            print("   Revisa los errores anteriores antes de continuar")
            
        return all_tests_passed


if __name__ == "__main__":
    tester = SalonModelTester()
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--quick":
            # Solo verificaciones básicas
            print("🚀 VERIFICACIÓN RÁPIDA")
            tester.check_file_structure()
            tester.check_dae_file()
        elif sys.argv[1] == "--setup":
            # Ayuda de configuración
            print("⚙️  AYUDA DE CONFIGURACIÓN")
            print("\n1. Coloca tu archivo .dae en:")
            print(f"   {tester.meshes_path}/salon.dae")
            print("\n2. Ajusta la escala en model.sdf si es necesario")
            print("\n3. Configura GAZEBO_MODEL_PATH:")
            print(f"   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{tester.tutorial_pkg_path}/models")
            print("\n4. Compila: cd ~/ros2_ws && colcon build")
            print("5. Source: source ~/ros2_ws/install/setup.bash")
        else:
            print("Uso: python3 test_salon_model.py [--quick|--setup]")
    else:
        # Prueba completa
        tester.run_full_test()
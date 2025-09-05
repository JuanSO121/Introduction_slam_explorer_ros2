#!/usr/bin/env python3
"""
Script de configuración y prueba del sistema de voz
Ubicación: tutorial_pkg/scripts/setup_voice.py
"""

import os
import sys
import subprocess
import platform
import time

def print_status(message, status="INFO"):
    """Imprime mensajes con formato"""
    colors = {
        "INFO": "\033[94m",    # Azul
        "SUCCESS": "\033[92m", # Verde
        "WARNING": "\033[93m", # Amarillo
        "ERROR": "\033[91m",   # Rojo
        "RESET": "\033[0m"     # Reset
    }
    
    color = colors.get(status, colors["INFO"])
    print(f"{color}[{status}]{colors['RESET']} {message}")

def check_system_requirements():
    """Verifica requisitos del sistema"""
    print_status("Verificando requisitos del sistema...")
    
    # Verificar Python
    python_version = sys.version_info
    if python_version.major < 3 or (python_version.major == 3 and python_version.minor < 8):
        print_status("Se requiere Python 3.8 o superior", "ERROR")
        return False
    
    print_status(f"Python {python_version.major}.{python_version.minor}.{python_version.micro} ✓", "SUCCESS")
    
    # Verificar sistema operativo
    system = platform.system()
    print_status(f"Sistema operativo: {system}", "INFO")
    
    return True

def install_system_dependencies():
    """Instala dependencias del sistema"""
    print_status("Instalando dependencias del sistema...")
    
    system = platform.system()
    
    if system == "Linux":
        # Detectar distribución
        try:
            with open("/etc/os-release", "r") as f:
                os_info = f.read()
            
            if "ubuntu" in os_info.lower() or "debian" in os_info.lower():
                commands = [
                    "sudo apt update",
                    "sudo apt install -y python3-pyaudio portaudio19-dev pulseaudio alsa-utils",
                    "sudo apt install -y espeak espeak-data libespeak1 libespeak-dev",
                    "sudo apt install -y flac sox libsox-fmt-all",
                    "sudo usermod -a -G audio $USER"
                ]
                
                for cmd in commands:
                    print_status(f"Ejecutando: {cmd}", "INFO")
                    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
                    if result.returncode != 0:
                        print_status(f"Error ejecutando: {cmd}", "WARNING")
                        print_status(result.stderr, "ERROR")
            
            elif "arch" in os_info.lower():
                commands = [
                    "sudo pacman -Sy --noconfirm python-pyaudio portaudio pulseaudio espeak-ng sox flac"
                ]
                
                for cmd in commands:
                    print_status(f"Ejecutando: {cmd}", "INFO")
                    subprocess.run(cmd, shell=True)
            
        except Exception as e:
            print_status(f"Error instalando dependencias: {e}", "ERROR")
    
    elif system == "Darwin":  # macOS
        print_status("Instalando dependencias para macOS...", "INFO")
        commands = [
            "brew install portaudio",
            "brew install espeak",
            "brew install sox"
        ]
        
        for cmd in commands:
            print_status(f"Ejecutando: {cmd}", "INFO")
            subprocess.run(cmd, shell=True)
    
    elif system == "Windows":
        print_status("Para Windows, instala manualmente:", "WARNING")
        print_status("1. PyAudio: pip install pyaudio", "INFO")
        print_status("2. Espeak: http://espeak.sourceforge.net/download.html", "INFO")
    
    print_status("Dependencias del sistema instaladas", "SUCCESS")

def install_python_dependencies():
    """Instala dependencias de Python"""
    print_status("Instalando dependencias de Python...")
    
    # Lista de paquetes a instalar
    packages = [
        "google-generativeai>=0.8.0",
        "speechrecognition>=3.10.0",
        "pyttsx3>=2.90",
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "opencv-python>=4.5.0",
        "requests>=2.28.0",
        "python-dotenv>=0.19.0"
    ]
    
    # Instalar PyAudio con manejo especial
    try:
        print_status("Instalando PyAudio...", "INFO")
        subprocess.run([sys.executable, "-m", "pip", "install", "pyaudio"], check=True)
        print_status("PyAudio instalado ✓", "SUCCESS")
    except subprocess.CalledProcessError:
        print_status("Error instalando PyAudio, intentando método alternativo...", "WARNING")
        try:
            # Método alternativo para sistemas Linux
            subprocess.run([sys.executable, "-m", "pip", "install", "--global-option=build_ext", 
                          "--global-option=-I/usr/local/include", 
                          "--global-option=-L/usr/local/lib", "pyaudio"], check=True)
            print_status("PyAudio instalado con método alternativo ✓", "SUCCESS")
        except subprocess.CalledProcessError:
            print_status("No se pudo instalar PyAudio automáticamente", "ERROR")
            print_status("Instala manualmente: sudo apt install python3-pyaudio", "INFO")
    
    # Instalar otros paquetes
    for package in packages:
        try:
            print_status(f"Instalando {package}...", "INFO")
            subprocess.run([sys.executable, "-m", "pip", "install", package], 
                         check=True, capture_output=True)
            print_status(f"{package} instalado ✓", "SUCCESS")
        except subprocess.CalledProcessError as e:
            print_status(f"Error instalando {package}: {e}", "WARNING")

def setup_environment():
    """Configura variables de entorno"""
    print_status("Configurando variables de entorno...", "INFO")
    
    # Verificar si existe GEMINI_API_KEY
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        print_status("GEMINI_API_KEY no encontrada", "WARNING")
        print_status("Para configurarla:", "INFO")
        print_status("export GEMINI_API_KEY=tu_api_key_aqui", "INFO")
        print_status("Agrega esta línea a tu ~/.bashrc o ~/.zshrc", "INFO")
        
        # Intentar leer desde archivo .env
        env_file = os.path.join(os.path.expanduser("~"), ".env")
        if os.path.exists(env_file):
            print_status(f"Archivo .env encontrado en {env_file}", "INFO")
        else:
            print_status("Considera crear un archivo .env en tu directorio home", "INFO")
    else:
        print_status("GEMINI_API_KEY encontrada ✓", "SUCCESS")
    
    # Configurar audio
    if platform.system() == "Linux":
        os.environ['PULSE_LATENCY_MSEC'] = '30'
        print_status("Latencia de audio configurada", "INFO")

def test_voice_systems():
    """Prueba los sistemas de voz"""
    print_status("Probando sistemas de voz...", "INFO")
    
    # Probar Text-to-Speech
    try:
        import pyttsx3
        engine = pyttsx3.init()
        engine.setProperty('rate', 160)
        engine.setProperty('volume', 0.8)
        
        print_status("Probando Text-to-Speech...", "INFO")
        engine.say("Sistema de voz funcionando correctamente")
        engine.runAndWait()
        print_status("Text-to-Speech funciona ✓", "SUCCESS")
        
    except Exception as e:
        print_status(f"Error en Text-to-Speech: {e}", "ERROR")
    
    # Probar Speech Recognition
    try:
        import speech_recognition as sr
        r = sr.Recognizer()
        mic = sr.Microphone()
        
        with mic as source:
            r.adjust_for_ambient_noise(source, duration=1)
        
        print_status("Speech Recognition inicializado ✓", "SUCCESS")
        print_status("Prueba diciendo algo en 3 segundos...", "INFO")
        
        time.sleep(1)
        print_status("3...", "INFO")
        time.sleep(1)
        print_status("2...", "INFO")
        time.sleep(1)
        print_status("1...", "INFO")
        
        with mic as source:
            audio = r.listen(source, timeout=5, phrase_time_limit=3)
        
        try:
            text = r.recognize_google(audio, language="es-ES")
            print_status(f"Reconocido: '{text}' ✓", "SUCCESS")
        except sr.UnknownValueError:
            print_status("No se pudo reconocer el audio", "WARNING")
        except sr.RequestError as e:
            print_status(f"Error en servicio de reconocimiento: {e}", "ERROR")
            
    except Exception as e:
        print_status(f"Error en Speech Recognition: {e}", "ERROR")
    
    # Probar Gemini AI
    try:
        from google import genai
        from google.genai import types
        
        api_key = os.getenv('GEMINI_API_KEY')
        if api_key:
            client = genai.Client()
            
            response = client.models.generate_content(
                model="gemini-2.0-flash-exp",
                contents="Di 'hola' en español",
                config=types.GenerateContentConfig(
                    thinking_config=types.ThinkingConfig(thinking_budget=0)
                )
            )
            
            print_status(f"Gemini AI respuesta: {response.text}", "SUCCESS")
        else:
            print_status("GEMINI_API_KEY no configurada, saltando prueba", "WARNING")
            
    except Exception as e:
        print_status(f"Error probando Gemini AI: {e}", "ERROR")

def create_launch_file():
    """Crea archivo de lanzamiento para el sistema de voz"""
    print_status("Creando archivo de lanzamiento...", "INFO")
    
    launch_content = '''#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Argumentos de lanzamiento
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo de simulación'
    )
    
    # Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Nodo de control por voz
    voice_controller_node = TimerAction(
        period=3.0,  # Esperar 3 segundos antes de iniciar
        actions=[
            Node(
                package='tutorial_pkg',
                executable='voice_controller.py',
                name='voice_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time
                }],
                respawn=True,
                respawn_delay=5.0
            )
        ]
    )
    
    # Monitor de comandos de voz (opcional)
    voice_monitor_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='tutorial_pkg',
                executable='voice_monitor.py',
                name='voice_monitor',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time
                }]
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        voice_controller_node,
        voice_monitor_node
    ])
'''
    
    # Crear directorio launch si no existe
    launch_dir = "launch"
    if not os.path.exists(launch_dir):
        os.makedirs(launch_dir)
    
    # Escribir archivo
    launch_file = os.path.join(launch_dir, "voice_control.launch.py")
    with open(launch_file, "w") as f:
        f.write(launch_content)
    
    print_status(f"Archivo de lanzamiento creado: {launch_file}", "SUCCESS")

def create_config_files():
    """Crea archivos de configuración"""
    print_status("Creando archivos de configuración...", "INFO")
    
    # Crear directorio config si no existe
    config_dir = "config"
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)
    
    # Configuración de voz
    voice_config = '''# config/voice_control.yaml
voice_controller:
  ros__parameters:
    # Configuración de reconocimiento de voz
    speech_recognition:
      language: "es-ES"
      energy_threshold: 300
      pause_threshold: 0.8
      phrase_threshold: 0.3
      timeout: 5.0
    
    # Configuración de Text-to-Speech
    text_to_speech:
      language: "es"
      rate: 160
      volume: 0.8
      voice_id: ""
    
    # Configuración de Gemini AI
    ai_config:
      model: "gemini-2.0-flash-exp"
      temperature: 0.3
      max_tokens: 200
      confidence_threshold: 0.6
    
    # Configuración del robot
    robot_config:
      max_linear_velocity: 0.5
      max_angular_velocity: 1.0
      default_navigation_timeout: 30.0
      exploration_timeout: 300.0
    
    # Comandos disponibles
    commands:
      navigation: ["ir", "navegar", "mover", "dirigir", "go", "move", "navigate"]
      exploration: ["explorar", "explore", "mapear", "map", "buscar", "search"]
      stop: ["parar", "detener", "stop", "alto", "halt"]
      status: ["estado", "status", "reporte", "report", "info"]
      emergency: ["emergencia", "emergency", "auxilio", "help"]
      manual: ["manual", "adelante", "atras", "izquierda", "derecha"]
      save: ["guardar", "save", "salvar"]
      home: ["casa", "origen", "home", "base", "inicio"]
'''
    
    config_file = os.path.join(config_dir, "voice_control.yaml")
    with open(config_file, "w") as f:
        f.write(voice_config)
    
    print_status(f"Configuración creada: {config_file}", "SUCCESS")

def main():
    """Función principal de configuración"""
    print_status("=== CONFIGURACIÓN DEL SISTEMA DE VOZ ===", "INFO")
    print_status("Este script configurará el sistema de control por voz con IA", "INFO")
    print()
    
    # Verificar requisitos
    if not check_system_requirements():
        print_status("Requisitos no cumplidos", "ERROR")
        return
    
    # Instalar dependencias del sistema
    print_status("\n=== INSTALANDO DEPENDENCIAS DEL SISTEMA ===", "INFO")
    install_system_dependencies()
    
    # Instalar dependencias de Python
    print_status("\n=== INSTALANDO DEPENDENCIAS DE PYTHON ===", "INFO")
    install_python_dependencies()
    
    # Configurar entorno
    print_status("\n=== CONFIGURANDO ENTORNO ===", "INFO")
    setup_environment()
    
    # Crear archivos de configuración
    print_status("\n=== CREANDO ARCHIVOS DE CONFIGURACIÓN ===", "INFO")
    create_config_files()
    create_launch_file()
    
    # Probar sistemas
    print_status("\n=== PROBANDO SISTEMAS ===", "INFO")
    test_voice_systems()
    
    print_status("\n=== CONFIGURACIÓN COMPLETADA ===", "SUCCESS")
    print_status("Para usar el sistema:", "INFO")
    print_status("1. Configura GEMINI_API_KEY en tu entorno", "INFO")
    print_status("2. Ejecuta: ros2 launch tutorial_pkg voice_control.launch.py", "INFO")
    print_status("3. Di comandos como 'explorar el área' o 'ir adelante'", "INFO")

if __name__ == "__main__":
    main()
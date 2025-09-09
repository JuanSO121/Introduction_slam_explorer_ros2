#!/usr/bin/env python3
"""
Servidor Flask adaptado para tutorial_pkg con integración completa de IA por voz
Ubicación: ~/ros2_ws/src/tutorial_pkg/voice_services/flask_server.py
Versión optimizada para TurtleBot3 Explorer
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import os
import sys
import logging
import tempfile
import time
import json
import threading
from pathlib import Path
from werkzeug.utils import secure_filename
import subprocess

# Configurar logging antes de importar servicios
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('/tmp/tutorial_flask_server.log')
    ]
)
logger = logging.getLogger(__name__)

# Verificar que estamos en el entorno virtual correcto
def verify_environment():
    """Verificar que las dependencias están disponibles"""
    try:
        import torch
        import transformers
        import librosa
        import flask_socketio
        logger.info("✅ Entorno virtual verificado correctamente")
        return True
    except ImportError as e:
        logger.error(f"❌ Error: Dependencia faltante: {e}")
        logger.error("💡 Activar entorno virtual: cd voice_services && source venv/bin/activate")
        return False

if not verify_environment():
    print("❌ Por favor activa el entorno virtual primero:")
    print("cd ~/ros2_ws/src/tutorial_pkg/voice_services")
    print("source venv/bin/activate")
    print("python3 flask_server.py")
    sys.exit(1)

# Importar servicios después de verificar entorno
try:
    from whisper_service import WhisperService
    from gemini_service import GeminiService
    from command_processor import CommandProcessor
    logger.info("✅ Servicios de IA importados correctamente")
except ImportError as e:
    logger.error(f"❌ Error importando servicios de IA: {e}")
    sys.exit(1)

# === CONFIGURACIÓN DE FLASK ===
app = Flask(__name__)
app.config['SECRET_KEY'] = 'tutorial_pkg_voice_control_2024'
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024  # 16MB max para audio

# CORS para permitir conexiones desde Flutter
CORS(app, origins=["*"])

# SocketIO con configuración optimizada para tutorial_pkg
socketio = SocketIO(
    app, 
    cors_allowed_origins="*", 
    async_mode='eventlet',
    logger=False,  # Evitar spam de logs
    engineio_logger=False,
    ping_timeout=60,
    ping_interval=25
)

# Directorio temporal para archivos de audio
TEMP_DIR = Path("/tmp/tutorial_voice_control")
TEMP_DIR.mkdir(exist_ok=True)

# === VARIABLES GLOBALES DE SERVICIOS ===
whisper_service = None
gemini_service = None
command_processor = None
ros_bridge = None
server_stats = {
    'start_time': time.time(),
    'requests_processed': 0,
    'audio_files_processed': 0,
    'errors_count': 0,
    'connected_clients': 0
}

# === BRIDGE ROS2 SIMPLE ===
class SimplifiedROSBridge:
    """Bridge simplificado para ROS2 usando subprocess"""
    
    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.ROSBridge")
        self.is_ros_available = self._check_ros2_availability()
        
    def _check_ros2_availability(self):
        """Verificar si ROS2 está disponible"""
        try:
            result = subprocess.run(['ros2', '--version'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.logger.info("✅ ROS2 detectado y disponible")
                return True
        except Exception as e:
            self.logger.warning(f"⚠️ ROS2 no disponible: {e}")
        return False
    
    def send_voice_command(self, command_text: str) -> bool:
        """Enviar comando de voz a través de ROS2 topic"""
        if not self.is_ros_available:
            self.logger.warning("ROS2 no disponible - simulando envío")
            return False
            
        try:
            # Escapar comillas en el comando
            escaped_command = command_text.replace('"', '\\"')
            
            # Construir comando ros2 topic pub
            ros_cmd = [
                'ros2', 'topic', 'pub', '--once',
                '/voice_commands',
                'std_msgs/String',
                f'data: "{escaped_command}"'
            ]
            
            # Ejecutar comando con timeout
            result = subprocess.run(ros_cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self.logger.info(f"✅ Comando enviado a ROS2: '{command_text}'")
                return True
            else:
                self.logger.error(f"❌ Error enviando a ROS2: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            self.logger.error("⏰ Timeout enviando comando a ROS2")
            return False
        except Exception as e:
            self.logger.error(f"❌ Error inesperado en ROS2: {e}")
            return False
    
    def get_robot_feedback(self) -> str:
        """Intentar obtener último feedback del robot"""
        if not self.is_ros_available:
            return "ROS2 no disponible"
            
        try:
            # Intentar leer último mensaje del tópico feedback
            result = subprocess.run([
                'ros2', 'topic', 'echo', '/voice_feedback', '--once'
            ], capture_output=True, text=True, timeout=3)
            
            if result.returncode == 0 and result.stdout.strip():
                # Extraer el contenido del mensaje
                output = result.stdout.strip()
                if 'data:' in output:
                    feedback = output.split('data:')[1].strip().strip('"\'')
                    return feedback
                    
        except subprocess.TimeoutExpired:
            self.logger.debug("Timeout obteniendo feedback del robot")
        except Exception as e:
            self.logger.debug(f"Error obteniendo feedback: {e}")
            
        return "Sin feedback disponible"
    
    def is_connected(self) -> bool:
        """Verificar si hay conexión activa con ROS2"""
        return self.is_ros_available

# === INICIALIZACIÓN DE SERVICIOS ===
def initialize_services():
    """Inicializar todos los servicios de forma segura y progresiva"""
    global whisper_service, gemini_service, command_processor, ros_bridge
    
    try:
        logger.info("🔧 Inicializando servicios para tutorial_pkg...")
        
        # 1. ROS Bridge (primero, es más rápido)
        logger.info("🌉 Inicializando bridge ROS2...")
        ros_bridge = SimplifiedROSBridge()
        
        # 2. Gemini Service (rápido de inicializar)
        logger.info("🤖 Inicializando Gemini AI...")
        try:
            gemini_service = GeminiService()
            # Test rápido de conectividad
            test_response = gemini_service.generate_response("test")
            if test_response and "error" not in test_response.lower():
                logger.info("✅ Gemini AI inicializado y probado")
            else:
                logger.warning("⚠️ Gemini responde pero con posibles limitaciones")
        except Exception as e:
            logger.warning(f"⚠️ Gemini AI no disponible: {e}")
            gemini_service = None
        
        # 3. Whisper Service (más lento, al final)
        logger.info("🎤 Inicializando Whisper (puede tardar 30-60 segundos)...")
        try:
            whisper_service = WhisperService(model_name="openai/whisper-small")
            
            if whisper_service.is_ready():
                logger.info("✅ Whisper inicializado correctamente")
                
                # Test rápido con audio sintético
                import numpy as np
                test_audio = np.zeros(16000, dtype=np.float32)  # 1 segundo de silencio
                test_file = TEMP_DIR / "whisper_test.wav"
                
                from scipy.io import wavfile
                wavfile.write(str(test_file), 16000, test_audio)
                
                test_result = whisper_service.transcribe_audio(str(test_file))
                test_file.unlink()  # Limpiar archivo de prueba
                
                logger.info("✅ Whisper probado y funcionando")
            else:
                logger.error("❌ Whisper falló la inicialización")
                whisper_service = None
        except Exception as e:
            logger.error(f"❌ Error crítico en Whisper: {e}")
            whisper_service = None
        
        # 4. Command Processor
        logger.info("🧠 Inicializando procesador de comandos...")
        command_processor = CommandProcessor(gemini_service, ros_bridge)
        
        # Resumen de servicios
        services_ready = {
            'ros2': ros_bridge and ros_bridge.is_connected(),
            'gemini': gemini_service is not None,
            'whisper': whisper_service is not None and whisper_service.is_ready(),
            'command_processor': command_processor is not None
        }
        
        ready_count = sum(services_ready.values())
        total_services = len(services_ready)
        
        logger.info(f"📊 Servicios inicializados: {ready_count}/{total_services}")
        for service, status in services_ready.items():
            status_symbol = "✅" if status else "❌"
            logger.info(f"  {status_symbol} {service}")
        
        if ready_count >= 2:  # Al menos ROS2 + uno más
            logger.info("🎉 Sistema listo para recibir peticiones")
            return True
        else:
            logger.warning("⚠️ Servicios mínimos no disponibles")
            return False
        
    except Exception as e:
        logger.error(f"❌ Error crítico inicializando servicios: {e}")
        return False

# === ENDPOINTS HTTP PARA FLUTTER ===

@app.route('/health', methods=['GET'])
def health_check():
    """Health check completo optimizado para tutorial_pkg"""
    try:
        current_time = time.time()
        uptime = current_time - server_stats['start_time']
        
        # Estado de servicios
        services_status = {
            'ros2': ros_bridge is not None and ros_bridge.is_connected(),
            'gemini': gemini_service is not None,
            'whisper': whisper_service is not None and whisper_service.is_ready(),
            'command_processor': command_processor is not None,
            'flask_server': True
        }
        
        # Contar servicios funcionando
        services_ok = sum(services_status.values())
        total_services = len(services_status)
        
        # Determinar estado general
        if services_ok == total_services:
            overall_status = 'healthy'
            http_code = 200
        elif services_ok >= 3:  # Al menos 3 de 5 servicios
            overall_status = 'degraded'
            http_code = 206  # Partial Content
        else:
            overall_status = 'critical'
            http_code = 503  # Service Unavailable
        
        # Información del sistema
        system_info = {
            'package': 'tutorial_pkg',
            'robot_type': 'turtlebot3_explorer',
            'server_version': '1.1.0',
            'uptime_seconds': int(uptime),
            'uptime_formatted': f"{int(uptime//3600)}h {int((uptime%3600)//60)}m {int(uptime%60)}s"
        }
        
        # Información específica de Whisper si está disponible
        whisper_info = {}
        if whisper_service:
            whisper_info = {
                'model': 'openai/whisper-small',
                'device': 'cpu',
                'language': 'spanish',
                'ready': whisper_service.is_ready()
            }
        
        response = {
            'status': overall_status,
            'services': services_status,
            'services_ready': f"{services_ok}/{total_services}",
            'system_info': system_info,
            'whisper_info': whisper_info,
            'server_stats': server_stats.copy(),
            'timestamp': current_time,
            'message': f'tutorial_pkg Voice Control Server - {overall_status.upper()}'
        }
        
        # Agregar información de red
        try:
            import socket
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
            response['network_info'] = {
                'hostname': hostname,
                'local_ip': local_ip,
                'server_port': request.environ.get('SERVER_PORT', '8000')
            }
        except:
            pass
        
        return jsonify(response), http_code
        
    except Exception as e:
        logger.error(f"Error en health check: {e}")
        server_stats['errors_count'] += 1
        return jsonify({
            'status': 'error',
            'error': str(e),
            'timestamp': time.time(),
            'message': 'Health check failed'
        }), 500

@app.route('/process_audio', methods=['POST'])
def process_audio_flutter():
    """
    Procesamiento principal de audio desde Flutter
    Optimizado para tutorial_pkg con mejor manejo de errores
    """
    start_time = time.time()
    temp_file = None
    
    try:
        server_stats['requests_processed'] += 1
        
        # Verificar que Whisper esté listo
        if not whisper_service or not whisper_service.is_ready():
            return jsonify({
                'success': False, 
                'error': 'whisper_not_ready',
                'ai_response': 'El sistema de reconocimiento de voz no está listo. Intenta de nuevo en unos segundos.',
                'retry_after': 30
            }), 503
        
        # Verificar archivo de audio
        if 'audio' not in request.files:
            return jsonify({
                'success': False, 
                'error': 'no_audio_file',
                'ai_response': 'No se recibió archivo de audio. Verifica la grabación.'
            }), 400
        
        audio_file = request.files['audio']
        if not audio_file or audio_file.filename == '':
            return jsonify({
                'success': False, 
                'error': 'empty_audio_file',
                'ai_response': 'El archivo de audio está vacío.'
            }), 400
        
        # Parámetros opcionales
        process_with_ai = request.form.get('process_ai', 'true').lower() == 'true'
        send_to_robot = request.form.get('send_robot', 'true').lower() == 'true'
        
        # Crear archivo temporal seguro
        timestamp = int(time.time() * 1000)
        filename = f"tutorial_audio_{timestamp}_{secure_filename(audio_file.filename)}"
        if not filename.endswith(('.wav', '.mp3', '.m4a', '.ogg')):
            filename += '.wav'
        
        temp_file = TEMP_DIR / filename
        
        try:
            # Guardar archivo
            audio_file.save(str(temp_file))
            file_size = temp_file.stat().st_size
            
            logger.info(f"📱 Audio recibido de Flutter: {filename} ({file_size} bytes)")
            
            # Validaciones del archivo
            if file_size == 0:
                raise ValueError("Archivo de audio vacío")
            if file_size > 10 * 1024 * 1024:  # 10MB max
                raise ValueError("Archivo demasiado grande (máximo 10MB)")
            if file_size < 1000:  # Menos de 1KB probablemente no tiene audio útil
                raise ValueError("Archivo demasiado pequeño")
            
            # Transcripción con Whisper
            logger.info("🎤 Iniciando transcripción...")
            transcription_start = time.time()
            transcription = whisper_service.transcribe_audio(str(temp_file))
            transcription_time = time.time() - transcription_start
            
            server_stats['audio_files_processed'] += 1
            
            # Verificar transcripción
            if not transcription or len(transcription.strip()) < 2:
                return jsonify({
                    'success': True,
                    'transcription': '',
                    'ai_response': 'No pude entender lo que dijiste. Habla más claro y cerca del micrófono.',
                    'command_type': 'silence_or_unclear',
                    'confidence': 0.0,
                    'robot_action': False,
                    'processing_time': time.time() - start_time
                })
            
            transcription = transcription.strip()
            logger.info(f"📝 Transcripción obtenida ({transcription_time:.2f}s): '{transcription}'")
            
            # Procesar con IA y enviar a robot si está habilitado
            ai_result = {}
            if process_with_ai and command_processor:
                logger.info("🧠 Procesando con IA...")
                ai_result = command_processor.process_voice_input(transcription)
                
                # Enviar comando a ROS2 si es apropiado
                if send_to_robot and ai_result.get('type') in ['robot_command', 'unknown']:
                    try:
                        robot_sent = ros_bridge.send_voice_command(transcription)
                        ai_result['ros_sent'] = robot_sent
                        
                        if robot_sent:
                            logger.info("🤖 Comando enviado al robot via ROS2")
                        else:
                            logger.warning("⚠️ No se pudo enviar comando al robot")
                            
                    except Exception as e:
                        logger.error(f"❌ Error enviando a robot: {e}")
                        ai_result['ros_sent'] = False
            else:
                # Respuesta básica sin IA
                ai_result = {
                    'ai_response': f'Escuché: "{transcription}". Procesando...',
                    'type': 'transcription_only',
                    'confidence': 1.0,
                    'ros_sent': False
                }
                
                # Enviar a robot de todas formas si está habilitado
                if send_to_robot:
                    ai_result['ros_sent'] = ros_bridge.send_voice_command(transcription)
            
            # Preparar respuesta completa para Flutter
            total_processing_time = time.time() - start_time
            
            response_data = {
                'success': True,
                'transcription': transcription,
                'ai_response': ai_result.get('ai_response', 'Comando procesado correctamente'),
                'command_type': ai_result.get('type', 'unknown'),
                'confidence': float(ai_result.get('confidence', 0.5)),
                'robot_action': bool(ai_result.get('ros_sent', False)),
                'processing_time': round(total_processing_time, 2),
                'transcription_time': round(transcription_time, 2),
                'timestamp': time.time(),
                'server_info': {
                    'package': 'tutorial_pkg',
                    'audio_file_size': file_size,
                    'temp_filename': filename
                }
            }
            
            # Obtener feedback del robot si es posible
            try:
                robot_feedback = ros_bridge.get_robot_feedback()
                if robot_feedback and robot_feedback != "Sin feedback disponible":
                    response_data['robot_feedback'] = robot_feedback
            except:
                pass
            
            logger.info(f"✅ Procesamiento completo ({total_processing_time:.2f}s)")
            return jsonify(response_data)
            
        finally:
            # Limpiar archivo temporal
            if temp_file and temp_file.exists():
                try:
                    temp_file.unlink()
                    logger.debug(f"🗑️ Archivo temporal eliminado: {filename}")
                except Exception as e:
                    logger.warning(f"⚠️ No se pudo eliminar {filename}: {e}")
    
    except Exception as e:
        server_stats['errors_count'] += 1
        logger.error(f"❌ Error procesando audio: {e}")
        
        # Limpiar en caso de error
        if temp_file and temp_file.exists():
            try:
                temp_file.unlink()
            except:
                pass
        
        return jsonify({
            'success': False,
            'error': str(e),
            'error_type': type(e).__name__,
            'ai_response': 'Lo siento, tuve problemas procesando tu audio. Intenta grabar de nuevo.',
            'processing_time': time.time() - start_time,
            'timestamp': time.time()
        }), 500

@app.route('/send_text_command', methods=['POST'])
def send_text_command():
    """Enviar comando de texto directamente (útil para testing)"""
    try:
        server_stats['requests_processed'] += 1
        
        data = request.get_json()
        if not data or 'command' not in data:
            return jsonify({
                'success': False, 
                'error': 'no_command_data',
                'message': 'Se requiere campo "command" en JSON'
            }), 400
        
        command_text = data.get('command', '').strip()
        if not command_text:
            return jsonify({
                'success': False, 
                'error': 'empty_command'
            }), 400
        
        # Parámetros opcionales
        process_ai = data.get('process_ai', True)
        send_robot = data.get('send_robot', True)
        
        logger.info(f"📝 Comando texto recibido: '{command_text}'")
        
        # Procesar con IA si está disponible
        ai_result = {}
        if process_ai and command_processor:
            ai_result = command_processor.process_voice_input(command_text)
        else:
            ai_result = {
                'ai_response': f'Comando recibido: "{command_text}"',
                'type': 'text_command',
                'confidence': 1.0,
                'ros_sent': False
            }
        
        # Enviar a ROS2 si está habilitado
        if send_robot and ros_bridge:
            try:
                robot_sent = ros_bridge.send_voice_command(command_text)
                ai_result['ros_sent'] = robot_sent
            except Exception as e:
                logger.error(f"Error enviando comando a robot: {e}")
                ai_result['ros_sent'] = False
        
        response = {
            'success': True,
            'command': command_text,
            'ai_response': ai_result.get('ai_response', ''),
            'command_type': ai_result.get('type', 'text'),
            'robot_action': bool(ai_result.get('ros_sent', False)),
            'confidence': float(ai_result.get('confidence', 1.0)),
            'timestamp': time.time()
        }
        
        # Agregar feedback del robot si disponible
        try:
            robot_feedback = ros_bridge.get_robot_feedback()
            if robot_feedback and "Sin feedback" not in robot_feedback:
                response['robot_feedback'] = robot_feedback
        except:
            pass
        
        return jsonify(response)
        
    except Exception as e:
        server_stats['errors_count'] += 1
        logger.error(f"❌ Error procesando comando texto: {e}")
        return jsonify({
            'success': False,
            'error': str(e),
            'timestamp': time.time()
        }), 500

@app.route('/robot_status', methods=['GET'])
def get_robot_status():
    """Obtener estado detallado del robot y sistema tutorial_pkg"""
    try:
        # Estado básico de conexión
        ros_connected = ros_bridge and ros_bridge.is_connected()
        
        status_data = {
            'connected': ros_connected,
            'package': 'tutorial_pkg',
            'robot_type': 'turtlebot3_explorer',
            'timestamp': time.time()
        }
        
        # Información de servicios
        status_data['services'] = {
            'ros2_bridge': ros_connected,
            'whisper_ready': whisper_service and whisper_service.is_ready(),
            'gemini_ready': gemini_service is not None,
            'command_processor': command_processor is not None
        }
        
        # Intentar obtener feedback del robot
        if ros_bridge:
            try:
                latest_feedback = ros_bridge.get_robot_feedback()
                if latest_feedback and "Sin feedback" not in latest_feedback:
                    status_data['latest_feedback'] = latest_feedback
                    status_data['feedback_timestamp'] = time.time()
            except Exception as e:
                logger.debug(f"No se pudo obtener feedback: {e}")
        
        # Estadísticas del servidor
        status_data['server_stats'] = {
            'uptime': int(time.time() - server_stats['start_time']),
            'requests_total': server_stats['requests_processed'],
            'audio_files_processed': server_stats['audio_files_processed'],
            'errors_count': server_stats['errors_count'],
            'connected_clients': server_stats['connected_clients']
        }
        
        # Información del sistema
        try:
            import psutil
            status_data['system_info'] = {
                'cpu_percent': psutil.cpu_percent(),
                'memory_percent': psutil.virtual_memory().percent,
                'disk_usage': psutil.disk_usage('/').percent
            }
        except ImportError:
            pass
        
        return jsonify(status_data)
        
    except Exception as e:
        server_stats['errors_count'] += 1
        logger.error(f"Error obteniendo estado del robot: {e}")
        return jsonify({
            'connected': False,
            'error': str(e),
            'timestamp': time.time(),
            'package': 'tutorial_pkg'
        }), 500

@app.route('/server_stats', methods=['GET'])
def get_server_statistics():
    """Obtener estadísticas detalladas del servidor"""
    uptime = time.time() - server_stats['start_time']
    
    stats = server_stats.copy()
    stats.update({
        'uptime_seconds': int(uptime),
        'uptime_formatted': f"{int(uptime//3600)}h {int((uptime%3600)//60)}m",
        'requests_per_minute': round(stats['requests_processed'] / max(uptime/60, 1), 2),
        'error_rate': round((stats['errors_count'] / max(stats['requests_processed'], 1)) * 100, 2),
        'timestamp': time.time()
    })
    
    return jsonify(stats)

@app.route('/clear_cache', methods=['POST'])
def clear_system_cache():
    """Limpiar caches del sistema (Whisper, archivos temporales, etc.)"""
    try:
        cleared_items = []
        
        # Limpiar cache de Whisper
        if whisper_service:
            whisper_service.clear_cache()
            cleared_items.append('whisper_cache')
        
        # Limpiar archivos temporales
        temp_files_removed = 0
        try:
            for temp_file in TEMP_DIR.glob("tutorial_audio_*"):
                temp_file.unlink()
                temp_files_removed += 1
            cleared_items.append(f'temp_files_{temp_files_removed}')
        except Exception as e:
            logger.warning(f"Error limpiando archivos temporales: {e}")
        
        # Reset de estadísticas de error
        server_stats['errors_count'] = 0
        cleared_items.append('error_stats')
        
        logger.info(f"🧹 Cache limpiado: {cleared_items}")
        
        return jsonify({
            'success': True,
            'cleared_items': cleared_items,
            'message': 'Cache del sistema limpiado correctamente',
            'timestamp': time.time()
        })
        
    except Exception as e:
        logger.error(f"Error limpiando cache: {e}")
        return jsonify({
            'success': False,
            'error': str(e),
            'timestamp': time.time()
        }), 500

# === WEBSOCKET PARA COMUNICACIÓN EN TIEMPO REAL ===

@socketio.on('connect')
def handle_flutter_connect():
    """Flutter se conecta vía WebSocket"""
    server_stats['connected_clients'] += 1
    client_id = request.sid
    
    logger.info(f"📱 Cliente Flutter conectado: {client_id}")
    
    # Enviar estado inicial inmediatamente
    emit('connection_status', {
        'status': 'connected',
        'client_id': client_id,
        'server_info': {
            'package': 'tutorial_pkg',
            'version': '1.1.0',
            'uptime': int(time.time() - server_stats['start_time'])
        },
        'services': {
            'whisper_ready': whisper_service and whisper_service.is_ready(),
            'gemini_ready': gemini_service is not None,
            'ros2_ready': ros_bridge and ros_bridge.is_connected(),
            'command_processor_ready': command_processor is not None
        },
        'capabilities': [
            'voice_transcription',
            'ai_conversation', 
            'robot_commands',
            'turtlebot3_exploration',
            'real_time_communication'
        ],
        'timestamp': time.time()
    })

@socketio.on('disconnect')
def handle_flutter_disconnect():
    """Flutter se desconecta del WebSocket"""
    server_stats['connected_clients'] = max(0, server_stats['connected_clients'] - 1)
    client_id = request.sid
    logger.info(f"📱 Cliente Flutter desconectado: {client_id}")

@socketio.on('voice_command_ws')
def handle_voice_command_websocket(data):
    """Procesar comando de voz recibido vía WebSocket"""
    try:
        if not isinstance(data, dict):
            emit('command_error', {
                'error': 'invalid_data_format',
                'message': 'Datos deben ser JSON válido'
            })
            return
        
        command_text = data.get('command', '').strip()
        if not command_text:
            emit('command_error', {
                'error': 'empty_command',
                'message': 'Comando vacío'
            })
            return
        
        process_ai = data.get('process_ai', True)
        send_robot = data.get('send_robot', True)
        
        logger.info(f"🎤 Comando WebSocket: '{command_text}'")
        
        # Procesar comando
        start_time = time.time()
        
        # IA processing
        if process_ai and command_processor:
            ai_result = command_processor.process_voice_input(command_text)
        else:
            ai_result = {
                'ai_response': f'Comando recibido vía WebSocket: "{command_text}"',
                'type': 'websocket_command',
                'confidence': 1.0,
                'ros_sent': False
            }
        
        # Enviar a robot si está habilitado
        if send_robot and ros_bridge:
            try:
                robot_sent = ros_bridge.send_voice_command(command_text)
                ai_result['ros_sent'] = robot_sent
            except Exception as e:
                logger.error(f"Error enviando a robot vía WS: {e}")
                ai_result['ros_sent'] = False
        
        processing_time = time.time() - start_time
        
        # Responder inmediatamente
        emit('command_result', {
            'success': True,
            'original_command': command_text,
            'ai_response': ai_result.get('ai_response', ''),
            'command_type': ai_result.get('type', 'unknown'),
            'robot_action': bool(ai_result.get('ros_sent', False)),
            'confidence': float(ai_result.get('confidence', 0.0)),
            'processing_time': round(processing_time, 3),
            'timestamp': time.time()
        })
        
        # Intentar obtener feedback del robot después de un momento
        def get_delayed_feedback():
            time.sleep(1)  # Esperar un momento para que el robot procese
            try:
                feedback = ros_bridge.get_robot_feedback()
                if feedback and "Sin feedback" not in feedback:
                    socketio.emit('robot_feedback', {
                        'feedback': feedback,
                        'related_command': command_text,
                        'timestamp': time.time()
                    }, room=request.sid)
            except Exception as e:
                logger.debug(f"No se pudo obtener feedback diferido: {e}")
        
        # Ejecutar en hilo separado para no bloquear
        if ai_result.get('ros_sent', False):
            threading.Thread(target=get_delayed_feedback, daemon=True).start()
        
    except Exception as e:
        logger.error(f"Error en comando WebSocket: {e}")
        emit('command_error', {
            'error': str(e),
            'error_type': type(e).__name__,
            'timestamp': time.time()
        })

@socketio.on('request_status')
def handle_status_request():
    """Flutter solicita estado actual del sistema"""
    try:
        # Estado completo del sistema
        status_data = {
            'services_online': {
                'whisper': whisper_service and whisper_service.is_ready(),
                'gemini': gemini_service is not None,
                'ros2': ros_bridge and ros_bridge.is_connected(),
                'command_processor': command_processor is not None
            },
            'server_stats': {
                'uptime': int(time.time() - server_stats['start_time']),
                'requests_processed': server_stats['requests_processed'],
                'connected_clients': server_stats['connected_clients'],
                'errors_count': server_stats['errors_count']
            },
            'system_info': {
                'package': 'tutorial_pkg',
                'robot_type': 'turtlebot3_explorer',
                'version': '1.1.0'
            },
            'timestamp': time.time()
        }
        
        # Agregar feedback del robot si está disponible
        if ros_bridge:
            try:
                latest_feedback = ros_bridge.get_robot_feedback()
                if latest_feedback and "Sin feedback" not in latest_feedback:
                    status_data['robot_feedback'] = latest_feedback
            except:
                pass
        
        emit('system_status', status_data)
        
    except Exception as e:
        logger.error(f"Error enviando estado vía WebSocket: {e}")
        emit('system_status', {
            'error': str(e),
            'timestamp': time.time()
        })

@socketio.on('ping')
def handle_ping(data=None):
    """Responder a ping de Flutter para verificar conectividad"""
    emit('pong', {
        'timestamp': time.time(),
        'server_ready': True,
        'data': data  # Echo back any data sent
    })

@socketio.on('request_capabilities')
def handle_capabilities_request():
    """Enviar información detallada de capacidades del sistema"""
    capabilities = {
        'voice_processing': {
            'transcription': whisper_service and whisper_service.is_ready(),
            'supported_languages': ['spanish', 'english'],
            'max_audio_duration': 30,
            'supported_formats': ['.wav', '.mp3', '.m4a', '.ogg']
        },
        'ai_features': {
            'conversational_ai': gemini_service is not None,
            'command_classification': command_processor is not None,
            'context_awareness': True,
            'multi_language_support': True
        },
        'robot_integration': {
            'ros2_commands': ros_bridge and ros_bridge.is_connected(),
            'real_time_feedback': True,
            'exploration_control': True,
            'navigation_commands': True
        },
        'communication': {
            'http_api': True,
            'websocket_realtime': True,
            'flutter_optimized': True,
            'cors_enabled': True
        },
        'system_info': {
            'package': 'tutorial_pkg',
            'robot_platform': 'turtlebot3',
            'slam_support': True,
            'autonomous_exploration': True
        }
    }
    
    emit('capabilities_info', capabilities)

# === FUNCIONES DE UTILIDAD ===

def get_network_info():
    """Obtener información de red para conexiones externas"""
    try:
        import socket
        import subprocess
        
        # Información básica
        hostname = socket.gethostname()
        
        # Intentar obtener IP de WSL2 si estamos en WSL
        try:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3)
            if result.returncode == 0 and result.stdout.strip():
                wsl_ip = result.stdout.strip().split()[0]
                return {
                    'hostname': hostname,
                    'wsl_ip': wsl_ip,
                    'localhost': '127.0.0.1',
                    'is_wsl': True
                }
        except:
            pass
        
        # Fallback a IP local estándar
        try:
            local_ip = socket.gethostbyname(hostname)
            return {
                'hostname': hostname,
                'local_ip': local_ip,
                'localhost': '127.0.0.1',
                'is_wsl': False
            }
        except:
            return {
                'hostname': hostname,
                'localhost': '127.0.0.1',
                'is_wsl': False
            }
        
    except Exception as e:
        logger.warning(f"Error obteniendo info de red: {e}")
        return {
            'hostname': 'unknown',
            'localhost': '127.0.0.1',
            'is_wsl': False
        }

def cleanup_old_temp_files():
    """Limpiar archivos temporales antiguos (más de 1 hora)"""
    try:
        current_time = time.time()
        cleaned_count = 0
        
        for temp_file in TEMP_DIR.glob("tutorial_audio_*"):
            try:
                if current_time - temp_file.stat().st_mtime > 3600:  # 1 hora
                    temp_file.unlink()
                    cleaned_count += 1
            except Exception as e:
                logger.debug(f"Error eliminando {temp_file}: {e}")
        
        if cleaned_count > 0:
            logger.info(f"🧹 {cleaned_count} archivos temporales antiguos eliminados")
            
    except Exception as e:
        logger.warning(f"Error en limpieza de archivos temporales: {e}")

def periodic_maintenance():
    """Mantenimiento periódico del servidor"""
    def maintenance_task():
        while True:
            try:
                # Limpiar archivos temporales cada 30 minutos
                cleanup_old_temp_files()
                
                # Limpiar cache de Whisper si hay muchos errores
                if (server_stats['errors_count'] > 10 and 
                    server_stats['errors_count'] % 20 == 0 and 
                    whisper_service):
                    logger.info("🧹 Limpiando cache de Whisper por errores acumulados")
                    whisper_service.clear_cache()
                
                # Esperar 30 minutos
                time.sleep(30 * 60)
                
            except Exception as e:
                logger.error(f"Error en mantenimiento periódico: {e}")
                time.sleep(60)  # Esperar 1 minuto si hay error
    
    # Ejecutar en hilo de fondo
    maintenance_thread = threading.Thread(target=maintenance_task, daemon=True)
    maintenance_thread.start()
    logger.info("🔧 Hilo de mantenimiento periódico iniciado")

# === MANEJO DE ERRORES GLOBALES ===

@app.errorhandler(404)
def not_found(error):
    return jsonify({
        'error': 'endpoint_not_found',
        'message': 'Endpoint no encontrado',
        'available_endpoints': [
            '/health',
            '/process_audio',
            '/send_text_command', 
            '/robot_status',
            '/server_stats',
            '/clear_cache'
        ],
        'websocket_available': True,
        'package': 'tutorial_pkg'
    }), 404

@app.errorhandler(413)
def request_entity_too_large(error):
    return jsonify({
        'error': 'file_too_large',
        'message': 'Archivo demasiado grande (máximo 16MB)',
        'max_size_mb': 16
    }), 413

@app.errorhandler(500)
def internal_server_error(error):
    server_stats['errors_count'] += 1
    logger.error(f"Error interno del servidor: {error}")
    return jsonify({
        'error': 'internal_server_error',
        'message': 'Error interno del servidor',
        'timestamp': time.time(),
        'package': 'tutorial_pkg'
    }), 500

# === PUNTO DE ENTRADA PRINCIPAL ===

def print_startup_info(network_info, port):
    """Mostrar información de inicio del servidor"""
    print("\n" + "="*60)
    print("🚀 SERVIDOR FLASK TUTORIAL_PKG INICIADO")
    print("="*60)
    print(f"📦 Paquete: tutorial_pkg")
    print(f"🤖 Robot: TurtleBot3 Explorer")
    print(f"🔧 Versión: 1.1.0")
    print(f"⏰ Iniciado: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("\n🌐 URLs DE CONEXIÓN:")
    
    if network_info.get('wsl_ip'):
        print(f"   📱 Flutter (WSL): http://{network_info['wsl_ip']}:{port}")
        print(f"   🔌 WebSocket: ws://{network_info['wsl_ip']}:{port}")
    elif network_info.get('local_ip'):
        print(f"   📱 Flutter (LAN): http://{network_info['local_ip']}:{port}")
        print(f"   🔌 WebSocket: ws://{network_info['local_ip']}:{port}")
    
    print(f"   🏠 Local: http://localhost:{port}")
    
    print(f"\n🧪 PRUEBAS RÁPIDAS:")
    if network_info.get('wsl_ip'):
        test_ip = network_info['wsl_ip']
    else:
        test_ip = 'localhost'
    print(f"   curl http://{test_ip}:{port}/health")
    print(f"   curl -X POST http://{test_ip}:{port}/send_text_command \\")
    print(f"        -H 'Content-Type: application/json' \\")
    print(f"        -d '{{\"command\": \"hola robot\"}}'")
    
    print(f"\n📡 INTEGRACIÓN ROS2:")
    print(f"   Tópico entrada: /voice_commands")
    print(f"   Tópico respuesta: /voice_feedback") 
    print(f"   Bridge: {'✅ Activo' if ros_bridge and ros_bridge.is_connected() else '❌ No disponible'}")
    
    print(f"\n🎤 SERVICIOS IA:")
    print(f"   Whisper: {'✅ Listo' if whisper_service and whisper_service.is_ready() else '❌ No disponible'}")
    print(f"   Gemini: {'✅ Listo' if gemini_service else '❌ No disponible'}")
    
    print(f"\n📋 COMANDOS DE VOZ SOPORTADOS:")
    print(f"   • Movimiento: adelante, atrás, izquierda, derecha, parar")
    print(f"   • Exploración: explorar, pausar exploración, continuar")
    print(f"   • Estado: estado, estadísticas, progreso del mapa")
    print(f"   • Conversación: hola, gracias, cómo estás")
    
    print("\n" + "="*60)
    print("✅ SERVIDOR LISTO - Esperando conexiones...")
    print("⚠️  Presiona Ctrl+C para detener")
    print("="*60 + "\n")

if __name__ == '__main__':
    try:
        print("🚀 Iniciando Servidor Flask para tutorial_pkg...")
        
        # Verificar que estamos en el directorio correcto
        current_dir = Path.cwd()
        if not (current_dir / "whisper_service.py").exists():
            print("❌ Error: Ejecutar desde directorio voice_services/")
            print("cd ~/ros2_ws/src/tutorial_pkg/voice_services")
            print("source venv/bin/activate") 
            print("python3 flask_server.py")
            sys.exit(1)
        
        # Limpiar archivos temporales antiguos al inicio
        cleanup_old_temp_files()
        
        # Inicializar servicios
        print("🔧 Inicializando servicios...")
        services_ready = initialize_services()
        
        if not services_ready:
            print("❌ Error crítico: Servicios mínimos no disponibles")
            print("💡 Verifica que ROS2 esté corriendo y las dependencias instaladas")
            sys.exit(1)
        
        # Obtener información de red
        network_info = get_network_info()
        port = int(os.getenv('FLASK_PORT', 8000))
        
        # Iniciar mantenimiento periódico
        periodic_maintenance()
        
        # Mostrar información de inicio
        print_startup_info(network_info, port)
        
        # Ejecutar servidor
        socketio.run(
            app,
            host='0.0.0.0',  # Escuchar en todas las interfaces
            port=port,
            debug=False,     # Cambiar a True solo para desarrollo
            use_reloader=False,  # Evitar problemas con servicios de IA
            allow_unsafe_werkzeug=True  # Para desarrollo con SocketIO
        )
        
    except KeyboardInterrupt:
        print("\n🛑 Deteniendo servidor...")
    except Exception as e:
        logger.error(f"Error crítico ejecutando servidor: {e}")
        print(f"❌ Error crítico: {e}")
    finally:
        # Cleanup final
        print("🧹 Limpiando recursos...")
        try:
            cleanup_old_temp_files()
            if whisper_service:
                whisper_service.clear_cache()
            if ros_bridge:
                # No hay método close en SimplifiedROSBridge, pero está bien
                pass
        except Exception as e:
            logger.debug(f"Error en cleanup final: {e}")
        
        print("👋 Servidor cerrado correctamente")
        print("🔄 Para reiniciar: python3 flask_server.py")
            
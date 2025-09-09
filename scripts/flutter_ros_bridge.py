#!/usr/bin/env python3
"""
Flutter-ROS2 Bridge para tutorial_pkg
Ubicación: ~/ros2_ws/src/tutorial_pkg/scripts/flutter_ros_bridge.py

Este bridge conecta la app Flutter con el sistema ROS2 de tutorial_pkg
mediante WebSockets y rosbridge, permitiendo control por voz y monitoreo.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import json
import time
import threading
import websocket
import logging
from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import tempfile
from pathlib import Path
from werkzeug.utils import secure_filename

# Configurar logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class FlutterROSBridge(Node):
    """
    Bridge que conecta Flutter con ROS2 tutorial_pkg
    """
    def __init__(self):
        super().__init__('flutter_ros_bridge')
        
        # Parámetros configurables
        self.declare_parameter('flask_port', 8000)
        self.declare_parameter('enable_voice_processing', True)
        self.declare_parameter('max_audio_size_mb', 10)
        
        # Obtener parámetros
        self.flask_port = self.get_parameter('flask_port').value
        self.enable_voice = self.get_parameter('enable_voice_processing').value
        self.max_audio_size = self.get_parameter('max_audio_size_mb').value * 1024 * 1024
        
        # Estado del sistema
        self.robot_state = {
            'exploration_status': 'idle',
            'current_velocity': {'linear': 0.0, 'angular': 0.0},
            'map_progress': 0.0,
            'battery_level': 100.0,  # Simulado
            'last_command': '',
            'ai_responses': [],
            'slam_active': False,
            'navigation_active': False,
            'obstacles_detected': [],
            'current_position': {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        }
        
        # Suscriptores ROS2
        self.create_subscription(String, '/voice_feedback', self.voice_feedback_callback, 10)
        self.create_subscription(String, '/ai_status', self.ai_status_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(String, '/exploration_control', self.exploration_control_callback, 10)
        
        # Publicadores ROS2
        self.voice_cmd_pub = self.create_publisher(String, '/voice_commands', 10)
        self.exploration_ctrl_pub = self.create_publisher(String, '/exploration_control', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Configurar Flask y WebSocket en thread separado
        self.setup_flask_server()
        
        # Configurar servicios de voz si están habilitados
        if self.enable_voice:
            self.setup_voice_services()
        
        self.get_logger().info('🌉 Flutter-ROS2 Bridge iniciado')
        self.get_logger().info(f'   - Puerto Flask: {self.flask_port}')
        self.get_logger().info(f'   - Procesamiento de voz: {"Habilitado" if self.enable_voice else "Deshabilitado"}')
    
    def setup_flask_server(self):
        """Configurar servidor Flask con WebSocket"""
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'tutorial_pkg_bridge_2024'
        CORS(self.app)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Directorio temporal para audio
        self.temp_dir = Path("/tmp/tutorial_pkg_audio")
        self.temp_dir.mkdir(exist_ok=True)
        
        # Configurar rutas Flask
        self.setup_flask_routes()
        self.setup_websocket_handlers()
        
        # Ejecutar Flask en thread separado
        self.flask_thread = threading.Thread(
            target=self.run_flask_server, 
            daemon=True
        )
        self.flask_thread.start()
    
    def setup_voice_services(self):
        """Configurar servicios de procesamiento de voz"""
        try:
            # Importar servicios de voz del proyecto voice_control
            import sys
            import os
            voice_control_path = os.path.expanduser('~/ros2_ws/src/voice_control/services')
            sys.path.append(voice_control_path)
            
            from whisper_service import WhisperService
            from gemini_service import GeminiService
            from command_processor import CommandProcessor
            
            self.whisper_service = WhisperService()
            self.gemini_service = GeminiService()
            self.command_processor = CommandProcessor(self.gemini_service, None)
            
            self.get_logger().info('✅ Servicios de voz inicializados')
            
        except Exception as e:
            self.get_logger().warning(f'⚠️ No se pudieron cargar servicios de voz: {e}')
            self.enable_voice = False
    
    def setup_flask_routes(self):
        """Configurar rutas HTTP de Flask"""
        
        @self.app.route('/health', methods=['GET'])
        def health_check():
            """Health check para Flutter"""
            return jsonify({
                'status': 'healthy',
                'package': 'tutorial_pkg',
                'bridge_version': '1.0.0',
                'services': {
                    'ros2_bridge': True,
                    'voice_processing': self.enable_voice,
                    'websocket': True
                },
                'robot_state': self.robot_state,
                'timestamp': time.time()
            })
        
        @self.app.route('/robot/status', methods=['GET'])
        def get_robot_status():
            """Obtener estado completo del robot"""
            return jsonify(self.robot_state)
        
        @self.app.route('/robot/command', methods=['POST'])
        def send_robot_command():
            """Enviar comando directo al robot"""
            try:
                data = request.get_json()
                command_type = data.get('type', '')
                command_data = data.get('data', {})
                
                if command_type == 'movement':
                    return self.handle_movement_command(command_data)
                elif command_type == 'exploration':
                    return self.handle_exploration_command(command_data)
                elif command_type == 'voice_text':
                    return self.handle_voice_text_command(command_data)
                else:
                    return jsonify({'success': False, 'error': 'Tipo de comando desconocido'})
                    
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)})
        
        @self.app.route('/robot/voice', methods=['POST'])
        def process_voice_command():
            """Procesar comando de voz desde Flutter"""
            if not self.enable_voice:
                return jsonify({
                    'success': False, 
                    'error': 'Procesamiento de voz no disponible'
                })
            
            try:
                if 'audio' not in request.files:
                    return jsonify({'success': False, 'error': 'No se envió archivo de audio'})
                
                audio_file = request.files['audio']
                if audio_file.filename == '':
                    return jsonify({'success': False, 'error': 'Archivo de audio vacío'})
                
                # Guardar archivo temporal
                filename = secure_filename(f"flutter_audio_{int(time.time() * 1000)}.wav")
                temp_path = self.temp_dir / filename
                
                try:
                    audio_file.save(str(temp_path))
                    
                    # Verificar tamaño
                    if temp_path.stat().st_size > self.max_audio_size:
                        return jsonify({'success': False, 'error': 'Archivo de audio demasiado grande'})
                    
                    # Procesar con Whisper
                    transcription = self.whisper_service.transcribe_audio(str(temp_path))
                    
                    if not transcription:
                        return jsonify({
                            'success': True,
                            'transcription': '',
                            'ai_response': 'No pude entender el audio. ¿Puedes repetir?',
                            'robot_action': False
                        })
                    
                    # Procesar con IA
                    ai_result = self.command_processor.process_voice_input(transcription)
                    
                    # Enviar comando a ROS2 si es necesario
                    ros_sent = False
                    if ai_result.get('type') == 'robot_command':
                        ros_sent = self.send_voice_command_to_ros(transcription)
                    
                    return jsonify({
                        'success': True,
                        'transcription': transcription,
                        'ai_response': ai_result.get('ai_response', 'Comando procesado'),
                        'command_type': ai_result.get('type', 'unknown'),
                        'confidence': float(ai_result.get('confidence', 0.0)),
                        'robot_action': ros_sent
                    })
                
                finally:
                    # Limpiar archivo temporal
                    try:
                        temp_path.unlink()
                    except:
                        pass
                        
            except Exception as e:
                self.get_logger().error(f'Error procesando audio: {e}')
                return jsonify({
                    'success': False, 
                    'error': 'Error procesando audio',
                    'details': str(e)
                })
    
    def setup_websocket_handlers(self):
        """Configurar manejadores de WebSocket"""
        
        @self.socketio.on('connect')
        def handle_connect():
            self.get_logger().info('📱 Flutter conectado via WebSocket')
            emit('robot_state', self.robot_state)
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.get_logger().info('📱 Flutter desconectado')
        
        @self.socketio.on('subscribe_updates')
        def handle_subscribe(data):
            """Suscribir Flutter a actualizaciones en tiempo real"""
            topics = data.get('topics', [])
            self.get_logger().info(f'📡 Flutter suscrito a: {topics}')
            # Enviar estado inicial
            emit('robot_state_update', self.robot_state)
        
        @self.socketio.on('send_command')
        def handle_websocket_command(data):
            """Manejar comando via WebSocket"""
            try:
                command_type = data.get('type')
                command_data = data.get('data', {})
                
                result = None
                if command_type == 'movement':
                    result = self.handle_movement_command(command_data)
                elif command_type == 'exploration':
                    result = self.handle_exploration_command(command_data)
                elif command_type == 'voice_text':
                    result = self.handle_voice_text_command(command_data)
                
                emit('command_result', result)
                
            except Exception as e:
                emit('command_result', {'success': False, 'error': str(e)})
    
    def handle_movement_command(self, command_data):
        """Manejar comandos de movimiento"""
        try:
            action = command_data.get('action', '')
            
            cmd_vel = Twist()
            
            if action == 'forward':
                cmd_vel.linear.x = command_data.get('speed', 0.2)
            elif action == 'backward':
                cmd_vel.linear.x = -command_data.get('speed', 0.2)
            elif action == 'turn_left':
                cmd_vel.angular.z = command_data.get('angular_speed', 0.3)
            elif action == 'turn_right':
                cmd_vel.angular.z = -command_data.get('angular_speed', 0.3)
            elif action == 'stop':
                pass  # cmd_vel ya está en ceros
            else:
                return {'success': False, 'error': f'Acción desconocida: {action}'}
            
            # Publicar comando
            self.cmd_vel_pub.publish(cmd_vel)
            
            return {
                'success': True,
                'action': action,
                'message': f'Comando de movimiento {action} enviado'
            }
            
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def handle_exploration_command(self, command_data):
        """Manejar comandos de exploración"""
        try:
            action = command_data.get('action', '')
            
            # Crear mensaje de control
            control_msg = String()
            
            if action == 'start':
                control_msg.data = 'start_exploration'
                message = 'Exploración iniciada'
            elif action == 'pause':
                control_msg.data = 'pause_exploration'
                message = 'Exploración pausada'
            elif action == 'resume':
                control_msg.data = 'resume_exploration'
                message = 'Exploración reanudada'
            elif action == 'stop':
                control_msg.data = 'finish_exploration'
                message = 'Exploración finalizada'
            else:
                return {'success': False, 'error': f'Acción de exploración desconocida: {action}'}
            
            # Publicar comando de control
            self.exploration_ctrl_pub.publish(control_msg)
            
            return {
                'success': True,
                'action': action,
                'message': message
            }
            
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def handle_voice_text_command(self, command_data):
        """Manejar comando de texto como si fuera voz"""
        try:
            text_command = command_data.get('text', '')
            
            if not text_command:
                return {'success': False, 'error': 'Texto vacío'}
            
            # Procesar con IA si está disponible
            if self.enable_voice and hasattr(self, 'command_processor'):
                ai_result = self.command_processor.process_voice_input(text_command)
                ai_response = ai_result.get('ai_response', 'Comando procesado')
                
                # Enviar a ROS2 si es comando de robot
                ros_sent = False
                if ai_result.get('type') == 'robot_command':
                    ros_sent = self.send_voice_command_to_ros(text_command)
            else:
                ai_response = f'Comando recibido: {text_command}'
                ros_sent = self.send_voice_command_to_ros(text_command)
            
            return {
                'success': True,
                'text_command': text_command,
                'ai_response': ai_response,
                'robot_action': ros_sent
            }
            
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def send_voice_command_to_ros(self, command_text):
        """Enviar comando de voz a ROS2"""
        try:
            voice_msg = String()
            voice_msg.data = command_text
            self.voice_cmd_pub.publish(voice_msg)
            
            self.get_logger().info(f'📤 Comando enviado a ROS2: {command_text}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error enviando comando a ROS2: {e}')
            return False
    
    def run_flask_server(self):
        """Ejecutar servidor Flask"""
        try:
            self.socketio.run(
                self.app,
                host='0.0.0.0',
                port=self.flask_port,
                debug=False,
                use_reloader=False,
                allow_unsafe_werkzeug=True
            )
        except Exception as e:
            self.get_logger().error(f'Error ejecutando Flask: {e}')
    
    # === CALLBACKS ROS2 ===
    
    def voice_feedback_callback(self, msg):
        """Procesar feedback de comandos de voz"""
        feedback = msg.data
        self.robot_state['last_command'] = feedback
        self.robot_state['ai_responses'].append({
            'message': feedback,
            'timestamp': time.time()
        })
        
        # Mantener solo las últimas 10 respuestas
        if len(self.robot_state['ai_responses']) > 10:
            self.robot_state['ai_responses'] = self.robot_state['ai_responses'][-10:]
        
        # Notificar a Flutter via WebSocket
        try:
            self.socketio.emit('voice_feedback', {
                'feedback': feedback,
                'timestamp': time.time()
            })
        except:
            pass
    
    def ai_status_callback(self, msg):
        """Procesar estado de IA"""
        try:
            ai_data = json.loads(msg.data)
            
            # Actualizar estado según datos de IA
            if 'exploration_active' in ai_data:
                self.robot_state['exploration_status'] = (
                    'exploring' if ai_data['exploration_active'] else 'idle'
                )
            
            # Notificar a Flutter
            self.socketio.emit('ai_status', ai_data)
            
        except json.JSONDecodeError:
            # Si no es JSON, tratar como mensaje simple
            self.socketio.emit('ai_status', {'message': msg.data})
    
    def cmd_vel_callback(self, msg):
        """Monitorear comandos de velocidad"""
        self.robot_state['current_velocity'] = {
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }
        
        # Notificar cambios significativos a Flutter
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            try:
                self.socketio.emit('robot_movement', {
                    'velocity': self.robot_state['current_velocity'],
                    'timestamp': time.time()
                })
            except:
                pass
    
    def map_callback(self, msg):
        """Procesar información del mapa"""
        # Calcular progreso aproximado del mapa
        total_cells = msg.info.width * msg.info.height
        if total_cells > 0:
            known_cells = sum(1 for cell in msg.data if cell != -1)
            self.robot_state['map_progress'] = (known_cells / total_cells) * 100
        
        # Actualizar info del mapa
        self.robot_state.update({
            'slam_active': True,
            'map_info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution
            }
        })
        
        # Notificar a Flutter cada 5 segundos para evitar spam
        current_time = time.time()
        if not hasattr(self, '_last_map_update') or current_time - self._last_map_update > 5:
            try:
                self.socketio.emit('map_update', {
                    'progress': self.robot_state['map_progress'],
                    'map_info': self.robot_state['map_info'],
                    'timestamp': current_time
                })
                self._last_map_update = current_time
            except:
                pass
    
    def laser_callback(self, msg):
        """Procesar datos del láser para detectar obstáculos"""
        # Analizar ranges para detectar obstáculos cercanos
        close_obstacles = [r for r in msg.ranges if 0.1 < r < 0.5]  # Obstáculos a menos de 0.5m
        
        self.robot_state['obstacles_detected'] = len(close_obstacles)
        
        # Notificar solo si hay cambios significativos
        if len(close_obstacles) > 5:  # Umbral de obstáculos
            try:
                self.socketio.emit('obstacle_alert', {
                    'obstacle_count': len(close_obstacles),
                    'min_distance': min(close_obstacles) if close_obstacles else float('inf'),
                    'timestamp': time.time()
                })
            except:
                pass
    
    def exploration_control_callback(self, msg):
        """Procesar comandos de control de exploración"""
        control_data = msg.data
        
        # Actualizar estado según comando
        if 'start' in control_data.lower():
            self.robot_state['exploration_status'] = 'exploring'
        elif 'pause' in control_data.lower():
            self.robot_state['exploration_status'] = 'paused'
        elif 'finish' in control_data.lower() or 'complete' in control_data.lower():
            self.robot_state['exploration_status'] = 'completed'
        elif 'emergency' in control_data.lower():
            self.robot_state['exploration_status'] = 'emergency'
        
        # Notificar a Flutter
        try:
            self.socketio.emit('exploration_control', {
                'command': control_data,
                'new_status': self.robot_state['exploration_status'],
                'timestamp': time.time()
            })
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge_node = FlutterROSBridge()
        
        # Obtener IP para mostrar en logs
        import subprocess
        try:
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            ip = result.stdout.strip().split()[0]
        except:
            ip = 'localhost'
        
        bridge_node.get_logger().info('🌉 Bridge iniciado correctamente')
        bridge_node.get_logger().info(f'📱 URLs para Flutter:')
        bridge_node.get_logger().info(f'   HTTP: http://{ip}:{bridge_node.flask_port}')
        bridge_node.get_logger().info(f'   WebSocket: ws://{ip}:{bridge_node.flask_port}')
        bridge_node.get_logger().info(f'🧪 Test: curl http://{ip}:{bridge_node.flask_port}/health')
        
        rclpy.spin(bridge_node)
        
    except KeyboardInterrupt:
        print('🔄 Cerrando Flutter-ROS2 Bridge...')
    except Exception as e:
        print(f'❌ Error crítico: {e}')
    finally:
        try:
            bridge_node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
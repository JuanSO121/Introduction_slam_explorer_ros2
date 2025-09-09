#!/usr/bin/env python3
"""
Flutter Bridge para tutorial_pkg
Integra Flutter con el sistema existente sin modificar la funcionalidad actual
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/flutter_bridge_node.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import json
import time
import threading
from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import tempfile
from pathlib import Path
from werkzeug.utils import secure_filename
import logging

# Configurar logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class FlutterBridgeNode(Node):
    def __init__(self):
        super().__init__('flutter_bridge_node')
        
        # Parámetros configurables
        self.declare_parameter('flask_port', 8000)
        self.declare_parameter('enable_websocket', True)
        self.declare_parameter('enable_voice_processing', True)
        
        self.flask_port = self.get_parameter('flask_port').get_parameter_value().integer_value
        self.enable_websocket = self.get_parameter('enable_websocket').get_parameter_value().bool_value
        self.enable_voice = self.get_parameter('enable_voice_processing').get_parameter_value().bool_value
        
        # Estado del sistema
        self.robot_state = {
            'exploration_status': 'idle',
            'current_velocity': {'linear': 0.0, 'angular': 0.0},
            'map_progress': 0.0,
            'ai_responses': [],
            'last_command': '',
            'connected_clients': 0
        }
        
        # Suscriptores para monitorear tutorial_pkg
        self.voice_feedback_sub = self.create_subscription(
            String, '/voice_feedback', self.voice_feedback_callback, 10)
        self.ai_status_sub = self.create_subscription(
            String, '/ai_status', self.ai_status_callback, 10)
        self.robot_state_sub = self.create_subscription(
            String, '/robot_state', self.robot_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Publicadores para enviar comandos a tutorial_pkg
        self.voice_commands_pub = self.create_publisher(String, '/voice_commands', 10)
        self.exploration_control_pub = self.create_publisher(String, '/exploration_control', 10)
        
        # Inicializar servicios de voz si están habilitados
        self.whisper_service = None
        self.gemini_service = None
        
        if self.enable_voice:
            try:
                from tutorial_pkg.voice_services.whisper_service import WhisperService
                from tutorial_pkg.voice_services.gemini_service import GeminiService
                self.whisper_service = WhisperService()
                self.gemini_service = GeminiService()
                self.get_logger().info('✅ Servicios de voz inicializados')
            except ImportError as e:
                self.get_logger().warn(f'⚠️ Servicios de voz no disponibles: {e}')
                self.enable_voice = False
        
        # Inicializar Flask en hilo separado
        self.flask_thread = threading.Thread(target=self.run_flask_server, daemon=True)
        self.flask_thread.start()
        
        self.get_logger().info(f'🌉 Flutter Bridge Node iniciado en puerto {self.flask_port}')
        self.get_logger().info(f'🎤 Procesamiento de voz: {"habilitado" if self.enable_voice else "deshabilitado"}')
    
    # === CALLBACKS ROS2 ===
    
    def voice_feedback_callback(self, msg: String):
        """Recibir feedback del sistema de voz de tutorial_pkg"""
        feedback = msg.data
        self.robot_state['last_command'] = feedback
        self.robot_state['ai_responses'].append({
            'message': feedback,
            'timestamp': time.time(),
            'type': 'voice_feedback'
        })
        
        # Mantener solo los últimos 10 mensajes
        if len(self.robot_state['ai_responses']) > 10:
            self.robot_state['ai_responses'] = self.robot_state['ai_responses'][-10:]
        
        # Notificar clientes Flutter via WebSocket
        if hasattr(self, 'socketio'):
            self.socketio.emit('robot_feedback', {
                'message': feedback,
                'timestamp': time.time()
            })
    
    def ai_status_callback(self, msg: String):
        """Recibir estado de IA de tutorial_pkg"""
        try:
            ai_data = json.loads(msg.data)
            self.robot_state.update(ai_data)
        except:
            self.robot_state['ai_status'] = msg.data
    
    def robot_state_callback(self, msg: String):
        """Recibir estado completo del robot"""
        try:
            state_data = json.loads(msg.data)
            self.robot_state.update(state_data)
        except:
            pass
    
    def cmd_vel_callback(self, msg: Twist):
        """Monitorear velocidad actual"""
        self.robot_state['current_velocity'] = {
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }
    
    def map_callback(self, msg: OccupancyGrid):
        """Monitorear progreso del mapa"""
        total_cells = msg.info.width * msg.info.height
        if total_cells > 0:
            known_cells = sum(1 for cell in msg.data if cell != -1)
            self.robot_state['map_progress'] = (known_cells / total_cells) * 100
    
    # === SERVIDOR FLASK ===
    
    def run_flask_server(self):
        """Ejecutar servidor Flask en hilo separado"""
        app = Flask(__name__)
        app.config['SECRET_KEY'] = 'tutorial_pkg_flutter_bridge'
        CORS(app)
        
        # Configurar WebSocket si está habilitado
        if self.enable_websocket:
            self.socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
            self.setup_websocket_handlers()
        
        self.setup_flask_routes(app)
        
        # Directorio temporal para archivos de audio
        self.temp_dir = Path("/tmp/tutorial_pkg_flutter")
        self.temp_dir.mkdir(exist_ok=True)
        
        try:
            if self.enable_websocket:
                self.socketio.run(
                    app,
                    host="0.0.0.0",
                    port=self.flask_port,
                    debug=False,
                    allow_unsafe_werkzeug=True
                )

            else:
                app.run(host='0.0.0.0', port=self.flask_port, debug=False)
        except Exception as e:
            self.get_logger().error(f'Error iniciando servidor Flask: {e}')
    
    def setup_flask_routes(self, app):
        """Configurar rutas Flask"""
        
        @app.route('/health', methods=['GET'])
        def health_check():
            """Health check para Flutter"""
            return jsonify({
                'status': 'healthy',
                'services': {
                    'ros2_bridge': True,
                    'voice_processing': self.enable_voice and self.whisper_service is not None,
                    'websocket': self.enable_websocket,
                    'tutorial_pkg_connected': True
                },
                'robot_state': self.robot_state,
                'timestamp': time.time()
            })
        
        @app.route('/send_voice_command', methods=['POST'])
        def send_voice_command():
            """Enviar comando de voz directamente a tutorial_pkg"""
            try:
                data = request.get_json()
                if not data or 'command' not in data:
                    return jsonify({'success': False, 'error': 'No command provided'}), 400
                
                command = data['command'].strip()
                if not command:
                    return jsonify({'success': False, 'error': 'Empty command'}), 400
                
                # Enviar comando directamente al sistema tutorial_pkg
                cmd_msg = String()
                cmd_msg.data = command
                self.voice_commands_pub.publish(cmd_msg)
                
                self.get_logger().info(f'📤 Comando enviado a tutorial_pkg: "{command}"')
                
                return jsonify({
                    'success': True,
                    'command_sent': command,
                    'timestamp': time.time()
                })
                
            except Exception as e:
                self.get_logger().error(f'Error enviando comando: {e}')
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @app.route('/send_text_command', methods=['POST'])
        def send_text_command():
            """Enviar comando de texto directamente a tutorial_pkg - ENDPOINT FALTANTE"""
            try:
                data = request.get_json()
                if not data or 'command' not in data:
                    return jsonify({'success': False, 'error': 'No command provided'}), 400
                
                command = data['command'].strip()
                if not command:
                    return jsonify({'success': False, 'error': 'Empty command'}), 400
                
                self.get_logger().info(f'💬 Procesando comando de texto: "{command}"')
                
                # Si tenemos el servicio de IA habilitado, procesar el comando
                ai_response = "Comando recibido correctamente"
                if self.enable_voice and self.gemini_service:
                    try:
                        # Procesar comando con IA
                        ai_response = self.gemini_service.process_command(command)
                        self.get_logger().info(f'🤖 Respuesta IA: "{ai_response}"')
                    except Exception as e:
                        self.get_logger().warn(f'⚠️ Error procesando con IA: {e}')
                        ai_response = f"Comando '{command}' recibido pero sin procesamiento IA"
                
                # Enviar comando al sistema tutorial_pkg
                cmd_msg = String()
                cmd_msg.data = command
                self.voice_commands_pub.publish(cmd_msg)
                
                # Actualizar estado interno
                self.robot_state['last_command'] = command
                self.robot_state['ai_responses'].append({
                    'message': ai_response,
                    'timestamp': time.time(),
                    'type': 'text_command',
                    'original_command': command
                })
                
                # Mantener historial limitado
                if len(self.robot_state['ai_responses']) > 10:
                    self.robot_state['ai_responses'] = self.robot_state['ai_responses'][-10:]
                
                # Notificar via WebSocket si está disponible
                if hasattr(self, 'socketio'):
                    self.socketio.emit('command_processed', {
                        'command': command,
                        'ai_response': ai_response,
                        'timestamp': time.time()
                    })
                
                return jsonify({
                    'success': True,
                    'command_sent': command,
                    'ai_response': ai_response,
                    'timestamp': time.time()
                })
                
            except Exception as e:
                self.get_logger().error(f'Error procesando comando de texto: {e}')
                return jsonify({
                    'success': False, 
                    'error': str(e),
                    'ai_response': f'Error procesando comando: {str(e)}'
                }), 500
        
        @app.route('/process_audio', methods=['POST'])
        def process_audio():
            """Procesar audio si el servicio de voz está habilitado"""
            if not self.enable_voice or not self.whisper_service:
                return jsonify({
                    'success': False,
                    'error': 'Voice processing not available'
                }), 503
            
            try:
                if 'audio' not in request.files:
                    return jsonify({'success': False, 'error': 'No audio file'}), 400
                
                audio_file = request.files['audio']
                if audio_file.filename == '':
                    return jsonify({'success': False, 'error': 'Empty file'}), 400
                
                # Guardar archivo temporal
                filename = secure_filename(f"audio_{int(time.time() * 1000)}.wav")
                temp_path = self.temp_dir / filename
                
                try:
                    audio_file.save(str(temp_path))
                    
                    # Procesar con Whisper
                    transcription = self.whisper_service.transcribe_audio(str(temp_path))
                    
                    if transcription:
                        # Procesar con IA si está disponible
                        ai_response = "Transcripción procesada"
                        if self.gemini_service:
                            try:
                                ai_response = self.gemini_service.process_command(transcription)
                            except Exception as e:
                                self.get_logger().warn(f'Error IA: {e}')
                                ai_response = f"Comando '{transcription}' transcrito correctamente"
                        
                        # Enviar transcripción al sistema tutorial_pkg
                        cmd_msg = String()
                        cmd_msg.data = transcription
                        self.voice_commands_pub.publish(cmd_msg)
                        
                        self.get_logger().info(f'🎤 Audio procesado: "{transcription}"')
                        
                        return jsonify({
                            'success': True,
                            'transcription': transcription,
                            'ai_response': ai_response,
                            'command_sent': True,
                            'timestamp': time.time()
                        })
                    else:
                        return jsonify({
                            'success': False,
                            'error': 'Could not transcribe audio'
                        }), 400
                
                finally:
                    # Limpiar archivo temporal
                    if temp_path.exists():
                        temp_path.unlink()
                
            except Exception as e:
                self.get_logger().error(f'Error procesando audio: {e}')
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @app.route('/robot_status', methods=['GET'])
        def get_robot_status():
            """Obtener estado actual del robot"""
            return jsonify({
                'status': 'connected',
                'robot_state': self.robot_state,
                'timestamp': time.time()
            })
        
        @app.route('/exploration_control', methods=['POST'])
        def exploration_control():
            """Controlar exploración"""
            try:
                data = request.get_json()
                if not data or 'action' not in data:
                    return jsonify({'success': False, 'error': 'No action provided'}), 400
                
                action = data['action']
                valid_actions = ['start_exploration', 'pause_exploration', 
                               'resume_exploration', 'finish_exploration', 'emergency_stop']
                
                if action not in valid_actions:
                    return jsonify({
                        'success': False, 
                        'error': f'Invalid action. Valid actions: {valid_actions}'
                    }), 400
                
                # Enviar comando de control
                control_msg = String()
                control_msg.data = action
                self.exploration_control_pub.publish(control_msg)
                
                self.get_logger().info(f'🎮 Control de exploración: {action}')
                
                return jsonify({
                    'success': True,
                    'action': action,
                    'timestamp': time.time()
                })
                
            except Exception as e:
                self.get_logger().error(f'Error en control de exploración: {e}')
                return jsonify({'success': False, 'error': str(e)}), 500
    
    def setup_websocket_handlers(self):
        """Configurar manejadores WebSocket"""
        
        @self.socketio.on('connect')
        def handle_connect():
            self.robot_state['connected_clients'] += 1
            self.get_logger().info(f'📱 Cliente Flutter conectado ({self.robot_state["connected_clients"]} total)')
            emit('connection_status', {
                'status': 'connected',
                'robot_state': self.robot_state,
                'timestamp': time.time()
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.robot_state['connected_clients'] = max(0, self.robot_state['connected_clients'] - 1)
            self.get_logger().info(f'📱 Cliente Flutter desconectado ({self.robot_state["connected_clients"]} restantes)')
        
        @self.socketio.on('send_command')
        def handle_command(data):
            try:
                command = data.get('command', '').strip()
                if command:
                    cmd_msg = String()
                    cmd_msg.data = command
                    self.voice_commands_pub.publish(cmd_msg)
                    
                    emit('command_result', {
                        'success': True,
                        'command': command,
                        'timestamp': time.time()
                    })
                else:
                    emit('command_result', {
                        'success': False,
                        'error': 'Empty command',
                        'timestamp': time.time()
                    })
            except Exception as e:
                emit('command_result', {
                    'success': False,
                    'error': str(e),
                    'timestamp': time.time()
                })
        
        @self.socketio.on('request_status')
        def handle_status_request():
            emit('robot_status', {
                'robot_state': self.robot_state,
                'timestamp': time.time()
            })


def main(args=None):
    rclpy.init(args=args)
    
    try:
        flutter_bridge = FlutterBridgeNode()
        
        # Ejecutar en hilo principal
        rclpy.spin(flutter_bridge)
        
    except KeyboardInterrupt:
        print('🔄 Cerrando Flutter Bridge Node...')
    except Exception as e:
        print(f'❌ Error crítico: {e}')
    finally:
        if 'flutter_bridge' in locals():
            flutter_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Integración de Control por Voz con IA para TurtleBot3 Explorer
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/ai_voice_commander.py
Basado en voice_commander.py pero adaptado para tutorial_pkg
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import re
import time
import json
from typing import Dict, Callable

class AIVoiceCommander(Node):
    def __init__(self):
        super().__init__('ai_voice_commander')
        
        # Suscriptores
        self.voice_subscription = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )
        
        # Suscriptor para estado de IA
        self.ai_status_subscription = self.create_subscription(
            String,
            '/ai_status',
            self.ai_status_callback,
            10
        )
        
        # Publicadores
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_publisher = self.create_publisher(String, '/voice_feedback', 10)
        self.ai_context_publisher = self.create_publisher(String, '/ai_context', 10)
        
        # Específicos para tutorial_pkg
        self.exploration_control_publisher = self.create_publisher(String, '/exploration_control', 10)
        
        # Variables de estado
        self.current_velocity = Twist()
        self.exploration_active = False
        self.last_command_time = time.time()
        self.ai_integration_active = True
        
        # Velocidades configurables (más seguras para exploración)
        self.linear_speed = 0.2  # m/s - reducida para mayor precisión
        self.angular_speed = 0.3  # rad/s - reducida para mayor precisión
        
        # Estadísticas para IA
        self.command_stats = {
            'total_commands': 0,
            'successful_commands': 0,
            'failed_commands': 0,
            'command_types': {},
            'session_start': time.time()
        }
        
        # Mapeo de comandos específico para exploración
        self.command_map = {
            # Movimiento básico
            r'\b(adelante|avanzar|forward|move forward|ve adelante|muévete adelante)\b': self.move_forward,
            r'\b(atras|atrás|retroceder|backward|move backward|ve atrás|muévete atrás)\b': self.move_backward,
            r'\b(izquierda|left|girar izquierda|turn left|voltea izquierda|gira izquierda)\b': self.turn_left,
            r'\b(derecha|right|girar derecha|turn right|voltea derecha|gira derecha)\b': self.turn_right,
            r'\b(parar|stop|detener|halt|alto|quieto|para)\b': self.stop_robot,
            
            # Control de velocidad
            r'\b(rapido|rápido|fast|faster|acelera|más rápido)\b': self.speed_up,
            r'\b(lento|slow|slower|desacelera|más lento|despacio)\b': self.speed_down,
            r'\b(velocidad normal|normal speed|velocidad media)\b': self.normal_speed,
            
            # Exploración específica para tutorial_pkg
            r'\b(iniciar exploracion|start exploration|comenzar exploración|explorar|mapear)\b': self.start_exploration,
            r'\b(pausar exploracion|pause exploration|detener exploración|pausa)\b': self.pause_exploration,
            r'\b(continuar exploracion|resume exploration|reanudar exploración|continuar)\b': self.resume_exploration,
            r'\b(terminar exploracion|finish exploration|finalizar exploración|completar)\b': self.finish_exploration,
            
            # Estado e información
            r'\b(estado|status|información|info|cómo estás|qué haces)\b': self.report_status,
            r'\b(donde estas|dónde estás|where are you|posicion|position|ubicación)\b': self.report_position,
            r'\b(estadisticas|statistics|resumen|summary|reporte)\b': self.report_statistics,
            r'\b(mapa|map|progreso del mapa|map progress)\b': self.report_map_progress,
            
            # Comandos específicos de IA
            r'\b(modo ia|ai mode|inteligente|smart mode)\b': self.enable_ai_mode,
            r'\b(modo manual|manual mode|básico|basic mode)\b': self.disable_ai_mode,
            
            # Comandos de emergencia
            r'\b(emergencia|emergency|ayuda|help|problema)\b': self.emergency_stop,
        }
        
        # Timer de seguridad
        self.create_timer(0.1, self.safety_timer_callback)
        
        self.get_logger().info('🤖 AI Voice Commander para TurtleBot3 Explorer iniciado')
        self.get_logger().info('🧠 Integración IA activa')
        self.get_logger().info('👂 Escuchando comandos en /voice_commands')
    
    def voice_command_callback(self, msg: String):
        """Procesar comando de voz con contexto IA"""
        command_text = msg.data.lower().strip()
        self.get_logger().info(f'🎤 Comando recibido: "{command_text}"')
        
        # Incrementar estadísticas
        self.command_stats['total_commands'] += 1
        
        # Detectar si es comando conversacional
        if self.is_conversational_message(command_text):
            self.handle_conversational_message(command_text)
            return
        
        # Buscar comando de robot coincidente
        command_found = False
        executed_command = None
        
        for pattern, action in self.command_map.items():
            if re.search(pattern, command_text, re.IGNORECASE):
                self.get_logger().info(f'✅ Ejecutando: {action.__name__}')
                try:
                    action()
                    command_found = True
                    executed_command = action.__name__
                    self.last_command_time = time.time()
                    
                    # Actualizar estadísticas
                    self.command_stats['successful_commands'] += 1
                    if executed_command in self.command_stats['command_types']:
                        self.command_stats['command_types'][executed_command] += 1
                    else:
                        self.command_stats['command_types'][executed_command] = 1
                    
                    # Enviar contexto a IA
                    self.send_ai_context(command_text, executed_command, True)
                    break
                    
                except Exception as e:
                    self.get_logger().error(f'❌ Error ejecutando comando: {e}')
                    self.command_stats['failed_commands'] += 1
                    self.send_ai_context(command_text, None, False, str(e))
        
        if not command_found:
            self.get_logger().warning(f'⚠️ Comando no reconocido: "{command_text}"')
            self.command_stats['failed_commands'] += 1
            self.handle_unknown_command(command_text)
    
    def ai_status_callback(self, msg: String):
        """Recibir actualizaciones de estado de IA"""
        try:
            ai_data = json.loads(msg.data)
            ai_message = ai_data.get('message', '')
            self.get_logger().info(f'🧠 Estado IA: {ai_message}')
        except:
            self.get_logger().debug(f'🧠 IA: {msg.data}')
    
    def is_conversational_message(self, text: str) -> bool:
        """Detectar si el mensaje es conversacional"""
        conversational_patterns = [
            r'\b(hola|hello|hi|buenos dias|buenas tardes|buenas noches)\b',
            r'\b(como estas|how are you|que tal|wassup)\b',
            r'\b(gracias|thank you|thanks|de nada)\b',
            r'\b(adios|goodbye|bye|hasta luego|nos vemos)\b',
            r'\b(quien eres|who are you|tu nombre|your name)\b',
            r'\b(me gusta|i like|esta bien|ok|cool)\b'
        ]
        
        for pattern in conversational_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                return True
        
        return False
    
    def handle_conversational_message(self, text: str):
        """Manejar mensajes conversacionales"""
        self.get_logger().info(f'💬 Mensaje conversacional: {text}')
        
        # Enviar a IA para respuesta
        context = {
            'type': 'conversational',
            'message': text,
            'robot_state': self.get_robot_state_summary()
        }
        
        context_msg = String()
        context_msg.data = json.dumps(context)
        self.ai_context_publisher.publish(context_msg)
        
        # Feedback local simple
        self.publish_feedback('Mensaje conversacional procesado por IA')
    
    def handle_unknown_command(self, command_text: str):
        """Manejar comando no reconocido con ayuda de IA"""
        self.get_logger().info(f'❓ Procesando comando desconocido con IA: {command_text}')
        
        # Enviar a IA para análisis
        context = {
            'type': 'unknown_command',
            'command': command_text,
            'available_commands': list(self.command_map.keys()),
            'robot_state': self.get_robot_state_summary()
        }
        
        context_msg = String()
        context_msg.data = json.dumps(context)
        self.ai_context_publisher.publish(context_msg)
        
        self.publish_feedback(f'Comando no reconocido: "{command_text}" - Consultando IA...')
    
    def send_ai_context(self, original_command: str, executed_action: str, success: bool, error: str = None):
        """Enviar contexto a IA después de ejecutar comando"""
        context = {
            'type': 'command_executed',
            'original_command': original_command,
            'executed_action': executed_action,
            'success': success,
            'error': error,
            'robot_state': self.get_robot_state_summary(),
            'timestamp': time.time()
        }
        
        context_msg = String()
        context_msg.data = json.dumps(context)
        self.ai_context_publisher.publish(context_msg)
    
    def get_robot_state_summary(self) -> Dict:
        """Obtener resumen del estado actual del robot"""
        return {
            'exploration_active': self.exploration_active,
            'current_velocity': {
                'linear': self.current_velocity.linear.x,
                'angular': self.current_velocity.angular.z
            },
            'speed_settings': {
                'linear_speed': self.linear_speed,
                'angular_speed': self.angular_speed
            },
            'ai_integration': self.ai_integration_active,
            'last_command_time': self.last_command_time,
            'session_duration': time.time() - self.command_stats['session_start']
        }
    
    # === COMANDOS DE MOVIMIENTO ===
    def move_forward(self):
        """Mover robot hacia adelante"""
        self.current_velocity.linear.x = self.linear_speed
        self.current_velocity.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback(f'Avanzando a {self.linear_speed:.2f} m/s')
        self.get_logger().info('🚀 Moviendo adelante')
    
    def move_backward(self):
        """Mover robot hacia atrás"""
        self.current_velocity.linear.x = -self.linear_speed
        self.current_velocity.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback(f'Retrocediendo a {self.linear_speed:.2f} m/s')
        self.get_logger().info('⬅️ Moviendo atrás')
    
    def turn_left(self):
        """Girar robot a la izquierda"""
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback(f'Girando izquierda a {self.angular_speed:.2f} rad/s')
        self.get_logger().info('↰ Girando izquierda')
    
    def turn_right(self):
        """Girar robot a la derecha"""
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = -self.angular_speed
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback(f'Girando derecha a {self.angular_speed:.2f} rad/s')
        self.get_logger().info('↱ Girando derecha')
    
    def stop_robot(self):
        """Detener completamente el robot"""
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback('Robot detenido completamente')
        self.get_logger().info('🛑 Robot detenido')
    
    def speed_up(self):
        """Aumentar velocidad"""
        self.linear_speed = min(self.linear_speed * 1.3, 0.5)  # Max 0.5 para seguridad
        self.angular_speed = min(self.angular_speed * 1.3, 0.8)
        self.publish_feedback(f'Velocidad aumentada: {self.linear_speed:.2f} m/s')
        self.get_logger().info(f'⬆️ Velocidad: {self.linear_speed:.2f} m/s')
    
    def speed_down(self):
        """Reducir velocidad"""
        self.linear_speed = max(self.linear_speed * 0.7, 0.1)
        self.angular_speed = max(self.angular_speed * 0.7, 0.1)
        self.publish_feedback(f'Velocidad reducida: {self.linear_speed:.2f} m/s')
        self.get_logger().info(f'⬇️ Velocidad: {self.linear_speed:.2f} m/s')
    
    def normal_speed(self):
        """Restablecer velocidad normal"""
        self.linear_speed = 0.2
        self.angular_speed = 0.3
        self.publish_feedback('Velocidad restablecida a valores normales')
        self.get_logger().info('🔄 Velocidad normal')
    
    # === COMANDOS DE EXPLORACIÓN ===
    def start_exploration(self):
        """Iniciar exploración autónoma"""
        try:
            self.exploration_active = True
            
            # Enviar comando a exploration_control
            control_msg = String()
            control_msg.data = "start_exploration"
            self.exploration_control_publisher.publish(control_msg)
            
            self.publish_feedback('Exploración autónoma iniciada - Mapeando entorno')
            self.get_logger().info('🗺️ Exploración iniciada')
        except Exception as e:
            self.get_logger().error(f'❌ Error iniciando exploración: {e}')
    
    def pause_exploration(self):
        """Pausar exploración"""
        try:
            self.exploration_active = False
            self.stop_robot()
            
            # Enviar comando a exploration_control
            control_msg = String()
            control_msg.data = "pause_exploration"
            self.exploration_control_publisher.publish(control_msg)
            
            self.publish_feedback('Exploración pausada - Robot en espera')
            self.get_logger().info('⏸️ Exploración pausada')
        except Exception as e:
            self.get_logger().error(f'❌ Error pausando exploración: {e}')
    
    def resume_exploration(self):
        """Reanudar exploración"""
        try:
            if not self.exploration_active:
                self.exploration_active = True
                
                # Enviar comando a exploration_control
                control_msg = String()
                control_msg.data = "resume_exploration"
                self.exploration_control_publisher.publish(control_msg)
                
                self.publish_feedback('Exploración reanudada')
                self.get_logger().info('▶️ Exploración reanudada')
            else:
                self.publish_feedback('La exploración ya está activa')
        except Exception as e:
            self.get_logger().error(f'❌ Error reanudando exploración: {e}')
    
    def finish_exploration(self):
        """Terminar exploración y guardar mapa"""
        try:
            self.exploration_active = False
            self.stop_robot()
            
            # Enviar comando a exploration_control
            control_msg = String()
            control_msg.data = "finish_exploration"
            self.exploration_control_publisher.publish(control_msg)
            
            self.publish_feedback('Exploración completada - Mapa final guardado')
            self.get_logger().info('🏁 Exploración terminada')
        except Exception as e:
            self.get_logger().error(f'❌ Error terminando exploración: {e}')
    
    # === COMANDOS DE ESTADO ===
    def report_status(self):
        """Reportar estado completo del robot"""
        uptime = time.time() - self.command_stats['session_start']
        status_msg = f'Estado: {"Explorando" if self.exploration_active else "Manual"}, '
        status_msg += f'Velocidad: {self.linear_speed:.2f} m/s, '
        status_msg += f'Comandos: {self.command_stats["successful_commands"]}/{self.command_stats["total_commands"]}, '
        status_msg += f'Tiempo activo: {uptime:.0f}s'
        
        self.publish_feedback(status_msg)
        self.get_logger().info(f'📊 {status_msg}')
    
    def report_position(self):
        """Reportar posición y orientación del robot"""
        position_msg = f'Velocidad actual: L:{self.current_velocity.linear.x:.2f} A:{self.current_velocity.angular.z:.2f}'
        self.publish_feedback(position_msg)
        self.get_logger().info(f'📍 {position_msg}')
    
    def report_statistics(self):
        """Reportar estadísticas de la sesión"""
        stats = self.command_stats
        stats_msg = f'Estadísticas: {stats["total_commands"]} comandos, '
        stats_msg += f'{stats["successful_commands"]} exitosos, '
        stats_msg += f'{stats["failed_commands"]} fallidos'
        
        self.publish_feedback(stats_msg)
        self.get_logger().info(f'📈 {stats_msg}')
    
    def report_map_progress(self):
        """Reportar progreso del mapeo"""
        # Este método puede ser expandido con información real del mapa
        progress_msg = f'Modo exploración: {"Activo" if self.exploration_active else "Inactivo"}'
        self.publish_feedback(progress_msg)
        self.get_logger().info(f'🗺️ {progress_msg}')
    
    # === COMANDOS DE IA ===
    def enable_ai_mode(self):
        """Activar modo IA inteligente"""
        self.ai_integration_active = True
        self.publish_feedback('Modo IA activado - Respuestas inteligentes habilitadas')
        self.get_logger().info('🧠 Modo IA activado')
    
    def disable_ai_mode(self):
        """Desactivar modo IA"""
        self.ai_integration_active = False
        self.publish_feedback('Modo manual activado - Solo comandos básicos')
        self.get_logger().info('🔧 Modo manual activado')
    
    def emergency_stop(self):
        """Parada de emergencia"""
        self.stop_robot()
        self.exploration_active = False
        
        # Enviar comando de emergencia
        control_msg = String()
        control_msg.data = "emergency_stop"
        self.exploration_control_publisher.publish(control_msg)
        
        self.publish_feedback('¡PARADA DE EMERGENCIA ACTIVADA!')
        self.get_logger().error('🚨 PARADA DE EMERGENCIA')
    
    def publish_feedback(self, message: str):
        """Publicar feedback al usuario"""
        feedback_msg = String()
        feedback_msg.data = message
        self.feedback_publisher.publish(feedback_msg)
    
    def safety_timer_callback(self):
        """Timer de seguridad - auto stop después de 5 segundos sin comandos"""
        if time.time() - self.last_command_time > 5.0:
            if (self.current_velocity.linear.x != 0.0 or 
                self.current_velocity.angular.z != 0.0):
                
                # Auto-stop por seguridad (no durante exploración autónoma)
                if not self.exploration_active:
                    self.stop_robot()
                    self.get_logger().info('⏰ Auto-stop por seguridad')

def main(args=None):
    rclpy.init(args=args)
    
    ai_voice_commander = AIVoiceCommander()
    
    try:
        rclpy.spin(ai_voice_commander)
    except KeyboardInterrupt:
        ai_voice_commander.get_logger().info('🔄 Cerrando AI Voice Commander...')
    finally:
        ai_voice_commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
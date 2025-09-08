#!/usr/bin/env python3
"""
Integración de Control por Voz con IA para TurtleBot3 Explorer - CORREGIDO
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/ai_voice_commander.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re
import time
import json
from typing import Dict, Callable

class AIVoiceCommander(Node):
    def __init__(self):
        super().__init__('ai_voice_commander')
        
        # Declarar parámetros con valores por defecto (IMPORTANTE para evitar errores)
        self.declare_parameter('linear_speed_default', 0.2)
        self.declare_parameter('angular_speed_default', 0.3)
        self.declare_parameter('auto_stop_timeout', 5.0)
        self.declare_parameter('enable_ai_responses', True)
        self.declare_parameter('log_voice_commands', True)
        self.declare_parameter('emergency_stop_enabled', True)
        
        # Obtener parámetros de forma segura
        try:
            self.linear_speed = self.get_parameter('linear_speed_default').get_parameter_value().double_value
            self.angular_speed = self.get_parameter('angular_speed_default').get_parameter_value().double_value
            self.auto_stop_timeout = self.get_parameter('auto_stop_timeout').get_parameter_value().double_value
            self.ai_integration_active = self.get_parameter('enable_ai_responses').get_parameter_value().bool_value
            self.log_commands = self.get_parameter('log_voice_commands').get_parameter_value().bool_value
        except Exception as e:
            self.get_logger().warn(f"Error obteniendo parámetros: {e} - Usando valores por defecto")
            self.linear_speed = 0.2
            self.angular_speed = 0.3
            self.auto_stop_timeout = 5.0
            self.ai_integration_active = True
            self.log_commands = True
        
        # Suscriptores
        self.voice_subscription = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )
        
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
        self.exploration_control_publisher = self.create_publisher(String, '/exploration_control', 10)
        
        # Variables de estado
        self.current_velocity = Twist()
        self.exploration_active = False
        self.last_command_time = time.time()
        
        # Estadísticas
        self.command_stats = {
            'total_commands': 0,
            'successful_commands': 0,
            'failed_commands': 0,
            'session_start': time.time()
        }
        
        # Mapeo de comandos más robusto
        self.command_map = {
            # Movimiento básico
            r'\b(adelante|avanzar|forward)\b': self.move_forward,
            r'\b(atras|atrás|retroceder|backward)\b': self.move_backward,
            r'\b(izquierda|left|girar izquierda)\b': self.turn_left,
            r'\b(derecha|right|girar derecha)\b': self.turn_right,
            r'\b(parar|stop|detener|halt|alto)\b': self.stop_robot,
            
            # Control de velocidad
            r'\b(rapido|rápido|fast|acelera)\b': self.speed_up,
            r'\b(lento|slow|despacio|desacelera)\b': self.speed_down,
            r'\b(velocidad normal|normal)\b': self.normal_speed,
            
            # Exploración
            r'\b(explorar|mapear|iniciar exploracion|start exploration)\b': self.start_exploration,
            r'\b(pausar|pause|detener exploracion)\b': self.pause_exploration,
            r'\b(continuar|resume|reanudar exploracion)\b': self.resume_exploration,
            r'\b(terminar|finish|finalizar exploracion)\b': self.finish_exploration,
            
            # Estado
            r'\b(estado|status|información|info)\b': self.report_status,
            r'\b(estadisticas|statistics|resumen)\b': self.report_statistics,
            
            # Emergencia
            r'\b(emergencia|emergency|ayuda|help)\b': self.emergency_stop,
        }
        
        # Timer de seguridad
        self.create_timer(0.1, self.safety_timer_callback)
        
        self.get_logger().info('🤖 AI Voice Commander iniciado correctamente')
        self.get_logger().info(f'   - Velocidades: L={self.linear_speed:.2f} A={self.angular_speed:.2f}')
        self.get_logger().info(f'   - IA habilitada: {self.ai_integration_active}')
        self.get_logger().info('👂 Escuchando comandos en /voice_commands')
    
    def voice_command_callback(self, msg: String):
        """Procesar comando de voz con manejo de errores mejorado"""
        command_text = msg.data.lower().strip()
        
        if self.log_commands:
            self.get_logger().info(f'🎤 Comando: "{command_text}"')
        
        self.command_stats['total_commands'] += 1
        
        # Procesar comando
        command_found = False
        for pattern, action in self.command_map.items():
            if re.search(pattern, command_text, re.IGNORECASE):
                try:
                    action()
                    command_found = True
                    self.command_stats['successful_commands'] += 1
                    self.last_command_time = time.time()
                    
                    if self.ai_integration_active:
                        self.send_ai_context(command_text, action.__name__, True)
                    break
                except Exception as e:
                    self.get_logger().error(f'❌ Error ejecutando {action.__name__}: {e}')
                    self.command_stats['failed_commands'] += 1
                    self.publish_feedback(f'Error ejecutando comando: {e}')
        
        if not command_found:
            self.command_stats['failed_commands'] += 1
            self.handle_unknown_command(command_text)
    
    def ai_status_callback(self, msg: String):
        """Callback mejorado para estado de IA"""
        try:
            data = json.loads(msg.data)
            message = data.get('message', msg.data)
            self.get_logger().info(f'🧠 IA: {message}')
        except:
            self.get_logger().debug(f'🧠 IA: {msg.data}')
    
    def handle_unknown_command(self, command_text: str):
        """Manejo mejorado de comandos desconocidos"""
        self.get_logger().warning(f'⚠️ Comando no reconocido: "{command_text}"')
        
        # Sugerencias basadas en patrones comunes
        suggestions = []
        if any(word in command_text for word in ['mov', 'ir', 'go']):
            suggestions.append('Intenta: adelante, atrás, izquierda, derecha')
        if any(word in command_text for word in ['map', 'explor']):
            suggestions.append('Intenta: explorar, pausar exploración')
        if any(word in command_text for word in ['vel', 'speed']):
            suggestions.append('Intenta: rápido, lento, velocidad normal')
        
        feedback_msg = f'Comando no reconocido: "{command_text}"'
        if suggestions:
            feedback_msg += f'. {". ".join(suggestions)}'
        
        self.publish_feedback(feedback_msg)
        
        if self.ai_integration_active:
            self.send_ai_context(command_text, None, False)
    
    def send_ai_context(self, command: str, action: str, success: bool, error: str = None):
        """Enviar contexto a IA"""
        try:
            context = {
                'type': 'command_executed' if success else 'command_failed',
                'command': command,
                'action': action,
                'success': success,
                'error': error,
                'robot_state': {
                    'exploration_active': self.exploration_active,
                    'current_velocity': {
                        'linear': self.current_velocity.linear.x,
                        'angular': self.current_velocity.angular.z
                    }
                },
                'timestamp': time.time()
            }
            
            context_msg = String()
            context_msg.data = json.dumps(context)
            self.ai_context_publisher.publish(context_msg)
        except Exception as e:
            self.get_logger().error(f'Error enviando contexto IA: {e}')
    
    # === COMANDOS DE MOVIMIENTO ===
    def move_forward(self):
        """Mover adelante con velocidad configurada"""
        self.current_velocity.linear.x = self.linear_speed
        self.current_velocity.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback(f'Avanzando a {self.linear_speed:.2f} m/s')
    
    def move_backward(self):
        """Mover atrás"""
        self.current_velocity.linear.x = -self.linear_speed
        self.current_velocity.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback(f'Retrocediendo a {self.linear_speed:.2f} m/s')
    
    def turn_left(self):
        """Girar izquierda"""
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback(f'Girando izquierda a {self.angular_speed:.2f} rad/s')
    
    def turn_right(self):
        """Girar derecha"""
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = -self.angular_speed
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback(f'Girando derecha a {self.angular_speed:.2f} rad/s')
    
    def stop_robot(self):
        """Detener robot completamente"""
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.current_velocity)
        self.publish_feedback('Robot detenido')
        self.get_logger().info('🛑 Robot detenido por comando')
    
    def speed_up(self):
        """Aumentar velocidad"""
        self.linear_speed = min(self.linear_speed * 1.2, 0.5)  # Max 0.5 m/s
        self.angular_speed = min(self.angular_speed * 1.2, 0.8)  # Max 0.8 rad/s
        self.publish_feedback(f'Velocidad aumentada: {self.linear_speed:.2f} m/s')
    
    def speed_down(self):
        """Reducir velocidad"""
        self.linear_speed = max(self.linear_speed * 0.8, 0.05)  # Min 0.05 m/s
        self.angular_speed = max(self.angular_speed * 0.8, 0.1)  # Min 0.1 rad/s
        self.publish_feedback(f'Velocidad reducida: {self.linear_speed:.2f} m/s')
    
    def normal_speed(self):
        """Restablecer velocidad normal"""
        self.linear_speed = self.get_parameter('linear_speed_default').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed_default').get_parameter_value().double_value
        self.publish_feedback('Velocidad restablecida a normal')
    
    # === COMANDOS DE EXPLORACIÓN ===
    def start_exploration(self):
        """Iniciar exploración autónoma"""
        self.exploration_active = True
        control_msg = String()
        control_msg.data = "start_exploration"
        self.exploration_control_publisher.publish(control_msg)
        self.publish_feedback('Exploración iniciada - Comenzando mapeo')
        self.get_logger().info('🗺️ Exploración iniciada')
    
    def pause_exploration(self):
        """Pausar exploración"""
        self.exploration_active = False
        self.stop_robot()
        control_msg = String()
        control_msg.data = "pause_exploration"
        self.exploration_control_publisher.publish(control_msg)
        self.publish_feedback('Exploración pausada')
        self.get_logger().info('⏸️ Exploración pausada')
    
    def resume_exploration(self):
        """Reanudar exploración"""
        self.exploration_active = True
        control_msg = String()
        control_msg.data = "resume_exploration"
        self.exploration_control_publisher.publish(control_msg)
        self.publish_feedback('Exploración reanudada')
        self.get_logger().info('▶️ Exploración reanudada')
    
    def finish_exploration(self):
        """Terminar exploración"""
        self.exploration_active = False
        self.stop_robot()
        control_msg = String()
        control_msg.data = "finish_exploration"
        self.exploration_control_publisher.publish(control_msg)
        self.publish_feedback('Exploración completada - Mapa guardado')
        self.get_logger().info('🏁 Exploración completada')
    
    # === COMANDOS DE ESTADO ===
    def report_status(self):
        """Reportar estado del robot"""
        uptime = time.time() - self.command_stats['session_start']
        status = f'Estado: {"Explorando" if self.exploration_active else "Manual"}, '
        status += f'Velocidad: {self.linear_speed:.2f} m/s, '
        status += f'Comandos: {self.command_stats["successful_commands"]}/{self.command_stats["total_commands"]}, '
        status += f'Tiempo: {uptime:.0f}s'
        
        self.publish_feedback(status)
        self.get_logger().info(f'📊 {status}')
    
    def report_statistics(self):
        """Reportar estadísticas detalladas"""
        stats = self.command_stats
        uptime = time.time() - stats['session_start']
        
        stats_msg = f'Estadísticas: {stats["total_commands"]} comandos totales, '
        stats_msg += f'{stats["successful_commands"]} exitosos, '
        stats_msg += f'{stats["failed_commands"]} fallidos, '
        stats_msg += f'Tiempo activo: {uptime:.0f}s'
        
        self.publish_feedback(stats_msg)
        self.get_logger().info(f'📈 {stats_msg}')
    
    def emergency_stop(self):
        """Parada de emergencia"""
        self.stop_robot()
        self.exploration_active = False
        
        control_msg = String()
        control_msg.data = "emergency_stop"
        self.exploration_control_publisher.publish(control_msg)
        
        self.publish_feedback('¡PARADA DE EMERGENCIA ACTIVADA!')
        self.get_logger().error('🚨 PARADA DE EMERGENCIA')
    
    def publish_feedback(self, message: str):
        """Publicar feedback con timestamp"""
        try:
            feedback_msg = String()
            feedback_msg.data = f"[{time.strftime('%H:%M:%S')}] {message}"
            self.feedback_publisher.publish(feedback_msg)
        except Exception as e:
            self.get_logger().error(f'Error publicando feedback: {e}')
    
    def safety_timer_callback(self):
        """Timer de seguridad - auto-stop por inactividad"""
        if time.time() - self.last_command_time > self.auto_stop_timeout:
            if (self.current_velocity.linear.x != 0.0 or 
                self.current_velocity.angular.z != 0.0):
                
                # Solo auto-stop en modo manual (no durante exploración)
                if not self.exploration_active:
                    self.stop_robot()
                    self.get_logger().info('⏰ Auto-stop por seguridad (inactividad)')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        ai_voice_commander = AIVoiceCommander()
        rclpy.spin(ai_voice_commander)
    except KeyboardInterrupt:
        print('🔄 Cerrando AI Voice Commander...')
    except Exception as e:
        print(f'❌ Error crítico: {e}')
    finally:
        try:
            ai_voice_commander.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Nodo ROS2 para manejar respuestas de IA y feedback en tutorial_pkg
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/ai_response_node.py
Adaptado para integración con TurtleBot3 Explorer
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import json
import time
from typing import Dict

class AIResponseNode(Node):
    def __init__(self):
        super().__init__('ai_response_node')
        
        # Suscriptores
        self.voice_feedback_subscription = self.create_subscription(
            String,
            '/voice_feedback',
            self.voice_feedback_callback,
            10
        )
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Suscriptor específico para tutorial_pkg - control de exploración
        self.exploration_control_subscription = self.create_subscription(
            String,
            '/exploration_control',
            self.exploration_control_callback,
            10
        )
        
        # Publicadores
        self.ai_status_publisher = self.create_publisher(
            String,
            '/ai_status',
            10
        )
        
        self.robot_state_publisher = self.create_publisher(
            String,
            '/robot_state',
            10
        )
        
        # Estado del robot específico para tutorial_pkg
        self.robot_state = {
            'current_velocity': {'linear': 0.0, 'angular': 0.0},
            'last_command': '',
            'last_command_time': 0,
            'map_info': {'width': 0, 'height': 0, 'resolution': 0.0},
            'exploration_status': 'idle',
            'ai_responses': [],
            'total_commands': 0,
            'exploration_active': False,
            'map_completion_percentage': 0.0,
            'slam_status': 'unknown',
            'navigation_status': 'idle'
        }
        
        # Timer para publicar estado periódicamente
        self.create_timer(2.0, self.publish_robot_state)
        
        # Timer para detectar inactividad
        self.create_timer(1.0, self.check_robot_activity)
        
        self.get_logger().info('🧠 AI Response Node (tutorial_pkg) iniciado')
        self.get_logger().info('📡 Monitoreando sistema de exploración TurtleBot3')
        
    def voice_feedback_callback(self, msg: String):
        """Procesar feedback de comandos de voz"""
        feedback = msg.data
        self.get_logger().info(f'🗣️ Feedback recibido: {feedback}')
        
        # Actualizar estado basado en feedback
        current_time = time.time()
        
        # Interpretar estado basado en feedback
        feedback_lower = feedback.lower()
        
        if any(word in feedback_lower for word in ['explorando', 'mapeando', 'exploración iniciada']):
            self.robot_state['exploration_status'] = 'exploring'
            self.robot_state['exploration_active'] = True
        elif any(word in feedback_lower for word in ['detenido', 'parado', 'pausada']):
            self.robot_state['exploration_status'] = 'stopped'
            self.robot_state['exploration_active'] = False
        elif any(word in feedback_lower for word in ['navegando', 'moviendo']):
            self.robot_state['exploration_status'] = 'navigating'
        elif 'completada' in feedback_lower or 'finalizada' in feedback_lower:
            self.robot_state['exploration_status'] = 'completed'
            self.robot_state['exploration_active'] = False
        elif 'emergencia' in feedback_lower:
            self.robot_state['exploration_status'] = 'emergency'
            self.robot_state['exploration_active'] = False
        
        # Guardar último comando y feedback
        self.robot_state['last_command'] = feedback
        self.robot_state['last_command_time'] = current_time
        self.robot_state['total_commands'] += 1
        
        # Agregar a historial de respuestas IA (mantener últimas 5)
        self.robot_state['ai_responses'].append({
            'feedback': feedback,
            'timestamp': current_time,
            'response_id': len(self.robot_state['ai_responses'])
        })
        
        if len(self.robot_state['ai_responses']) > 5:
            self.robot_state['ai_responses'] = self.robot_state['ai_responses'][-5:]
        
        # Publicar actualización de estado
        self.publish_ai_status(f"Procesado: {feedback}")
    
    def exploration_control_callback(self, msg: String):
        """Manejar comandos de control de exploración"""
        control_command = msg.data
        self.get_logger().info(f'🎮 Control de exploración: {control_command}')
        
        # Actualizar estado según el comando de control
        if control_command == "start_exploration":
            self.robot_state['exploration_active'] = True
            self.robot_state['exploration_status'] = 'starting'
            self.publish_ai_status('Iniciando exploración autónoma')
            
        elif control_command == "pause_exploration":
            self.robot_state['exploration_active'] = False
            self.robot_state['exploration_status'] = 'paused'
            self.publish_ai_status('Exploración pausada')
            
        elif control_command == "resume_exploration":
            self.robot_state['exploration_active'] = True
            self.robot_state['exploration_status'] = 'exploring'
            self.publish_ai_status('Exploración reanudada')
            
        elif control_command == "finish_exploration":
            self.robot_state['exploration_active'] = False
            self.robot_state['exploration_status'] = 'completed'
            self.publish_ai_status('Exploración completada')
            
        elif control_command == "emergency_stop":
            self.robot_state['exploration_active'] = False
            self.robot_state['exploration_status'] = 'emergency'
            self.publish_ai_status('PARADA DE EMERGENCIA ACTIVADA')
    
    def cmd_vel_callback(self, msg: Twist):
        """Monitorear comandos de velocidad"""
        self.robot_state['current_velocity'] = {
            'linear': msg.linear.x,
            'angular': msg.angular.z
        }
        
        # Determinar tipo de movimiento
        if msg.linear.x > 0.01:
            movement = 'forward'
        elif msg.linear.x < -0.01:
            movement = 'backward'
        elif msg.angular.z > 0.01:
            movement = 'turning_left'
        elif msg.angular.z < -0.01:
            movement = 'turning_right'
        else:
            movement = 'stopped'
        
        # Log solo cambios significativos
        if hasattr(self, '_last_movement') and self._last_movement != movement:
            self.get_logger().debug(f'🎮 Movimiento: {movement}')
        
        self._last_movement = movement
    
    def map_callback(self, msg: OccupancyGrid):
        """Procesar información del mapa SLAM"""
        self.robot_state['map_info'] = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y
        }
        
        # Calcular área mapeada (aproximada)
        total_cells = msg.info.width * msg.info.height
        if total_cells > 0:
            known_cells = sum(1 for cell in msg.data if cell != -1)
            mapped_percentage = (known_cells / total_cells) * 100
            self.robot_state['map_completion_percentage'] = mapped_percentage
            
            # Determinar estado de SLAM basado en progreso
            if mapped_percentage < 5:
                self.robot_state['slam_status'] = 'initializing'
            elif mapped_percentage < 80:
                self.robot_state['slam_status'] = 'mapping'
            else:
                self.robot_state['slam_status'] = 'nearly_complete'
        
        # Log periódico del progreso de mapeo (cada 10 segundos)
        if not hasattr(self, '_last_map_log') or time.time() - self._last_map_log > 10:
            self.get_logger().info(f'🗺️ Progreso del mapa: {self.robot_state["map_completion_percentage"]:.1f}%')
            self._last_map_log = time.time()
    
    def publish_robot_state(self):
        """Publicar estado completo del robot"""
        try:
            # Enriquecer estado con información específica de tutorial_pkg
            enhanced_state = self.robot_state.copy()
            enhanced_state['package'] = 'tutorial_pkg'
            enhanced_state['robot_type'] = 'turtlebot3_explorer'
            enhanced_state['capabilities'] = [
                'autonomous_exploration',
                'slam_mapping',
                'voice_control',
                'ai_integration'
            ]
            
            state_json = json.dumps(enhanced_state, indent=2)
            
            state_msg = String()
            state_msg.data = state_json
            self.robot_state_publisher.publish(state_msg)
            
            # Log estado resumido
            velocity = self.robot_state['current_velocity']
            exploration = self.robot_state['exploration_status']
            map_progress = self.robot_state['map_completion_percentage']
            
            status_summary = f"Estado: {exploration}, Vel: {velocity['linear']:.2f}m/s"
            if map_progress > 0:
                status_summary += f", Mapa: {map_progress:.1f}%"
            
            self.get_logger().debug(status_summary)
            
        except Exception as e:
            self.get_logger().error(f'Error publicando estado: {e}')
    
    def check_robot_activity(self):
        """Verificar actividad del robot y detectar problemas"""
        current_time = time.time()
        last_command_time = self.robot_state['last_command_time']
        
        # Detectar inactividad prolongada (sin comandos por más de 60 segundos)
        if last_command_time > 0 and (current_time - last_command_time) > 60:
            if self.robot_state['exploration_status'] not in ['idle', 'completed']:
                self.robot_state['exploration_status'] = 'idle'
                self.publish_ai_status('Robot inactivo - esperando comandos')
                self.get_logger().info('😴 Robot inactivo detectado')
        
        # Verificar si exploración se ha atascado
        if (self.robot_state['exploration_active'] and 
            self.robot_state['current_velocity']['linear'] == 0.0 and 
            self.robot_state['current_velocity']['angular'] == 0.0):
            
            if not hasattr(self, '_stuck_start_time'):
                self._stuck_start_time = current_time
            elif current_time - self._stuck_start_time > 30:  # 30 segundos atascado
                self.get_logger().warning('⚠️ Robot parece atascado durante exploración')
                self.publish_ai_status('Posible problema: Robot atascado durante exploración')
                self._stuck_start_time = current_time  # Reset timer
        else:
            # Robot se está moviendo, resetear timer
            if hasattr(self, '_stuck_start_time'):
                delattr(self, '_stuck_start_time')
    
    def publish_ai_status(self, status_message: str):
        """Publicar estado de IA con información específica de tutorial_pkg"""
        try:
            status_data = {
                'message': status_message,
                'timestamp': time.time(),
                'robot_state': self.robot_state['exploration_status'],
                'total_commands': self.robot_state['total_commands'],
                'exploration_active': self.robot_state['exploration_active'],
                'map_progress': self.robot_state['map_completion_percentage'],
                'slam_status': self.robot_state['slam_status'],
                'package': 'tutorial_pkg'
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.ai_status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publicando estado IA: {e}')
    
    def get_exploration_summary(self) -> str:
        """Obtener resumen específico del estado de exploración"""
        velocity = self.robot_state['current_velocity']
        map_info = self.robot_state['map_info']
        
        summary = f"""
        Estado de Exploración TurtleBot3:
        - Estado: {self.robot_state['exploration_status']}
        - Exploración activa: {'Sí' if self.robot_state['exploration_active'] else 'No'}
        - Velocidad: {velocity['linear']:.2f} m/s lineal, {velocity['angular']:.2f} rad/s angular
        - Progreso del mapa: {self.robot_state['map_completion_percentage']:.1f}%
        - Estado SLAM: {self.robot_state['slam_status']}
        - Dimensiones del mapa: {map_info.get('width', 0)}x{map_info.get('height', 0)} píxeles
        - Último comando: {self.robot_state['last_command']}
        - Comandos totales: {self.robot_state['total_commands']}
        """
        
        return summary.strip()

def main(args=None):
    rclpy.init(args=args)
    
    ai_response_node = AIResponseNode()
    
    try:
        ai_response_node.get_logger().info('🧠 AI Response Node (tutorial_pkg) ejecutándose...')
        rclpy.spin(ai_response_node)
    except KeyboardInterrupt:
        ai_response_node.get_logger().info('🛑 Deteniendo AI Response Node...')
    finally:
        ai_response_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
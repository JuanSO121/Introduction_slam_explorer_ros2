#!/usr/bin/env python3
"""
Exploration Bridge - Intermediario entre Explore Lite y Robot Control Coordinator
Convierte comandos de velocidad de explore_lite en solicitudes estructuradas al coordinador
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/exploration_bridge.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time
import math
from typing import Dict, Any, Optional


class ExplorationBridge(Node):
    """
    Bridge que intercepta comandos de explore_lite y los convierte en solicitudes
    estructuradas al coordinador central, evitando conflictos de control
    """
    
    def __init__(self):
        super().__init__('exploration_bridge')
        
        # Configuración
        self.declare_parameter('movement_threshold', 0.01)
        self.declare_parameter('angular_threshold', 0.01) 
        self.declare_parameter('command_timeout', 5.0)
        self.declare_parameter('bridge_enabled', True)
        
        self.movement_threshold = self.get_parameter('movement_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.bridge_enabled = self.get_parameter('bridge_enabled').value
        
        # Estado del bridge
        self.coordinator_state = {
            'exploration_active': False,
            'manual_override_active': False,
            'voice_control_active': False,
            'current_state': 'UNKNOWN'
        }
        
        self.last_cmd_vel = Twist()
        self.last_request_time = 0.0
        self.commands_bridged = 0
        self.commands_blocked = 0
        
        # Publishers - Comunicación con coordinador
        self.exploration_request_pub = self.create_publisher(
            String, '/exploration_request', 10)
        
        self.bridge_status_pub = self.create_publisher(
            String, '/bridge_status', 10)
        
        # Subscribers
        # Interceptar comandos de explore_lite
        self.exploration_cmd_sub = self.create_subscription(
            Twist, '/exploration_cmd_vel', self._exploration_cmd_callback, 10)
        
        # Monitorear estado del coordinador
        self.coordinator_status_sub = self.create_subscription(
            String, '/coordinator_status', self._coordinator_status_callback, 10)
        
        # Timer para monitoreo
        self.status_timer = self.create_timer(5.0, self._publish_bridge_status)
        
        self.get_logger().info('Bridge de exploración iniciado')
        self.get_logger().info(f'Interceptando comandos de /exploration_cmd_vel')
        
    def _coordinator_status_callback(self, msg: String):
        """Actualizar estado conocido del coordinador"""
        try:
            status_data = json.loads(msg.data)
            self.coordinator_state.update({
                'exploration_active': 'EXPLORING_AUTO' in status_data.get('state', ''),
                'manual_override_active': status_data.get('manual_override_active', False),
                'voice_control_active': status_data.get('voice_control_active', False),
                'current_state': status_data.get('state', 'UNKNOWN')
            })
        except Exception as e:
            self.get_logger().error(f'Error procesando estado coordinador: {e}')
    
    def _exploration_cmd_callback(self, msg: Twist):
        """
        CALLBACK CRÍTICO: Interceptar comandos de explore_lite
        y convertirlos en solicitudes al coordinador
        """
        if not self.bridge_enabled:
            return
            
        current_time = time.time()
        
        # Verificar si el coordinador permite exploración automática
        if not self._should_forward_command():
            self.commands_blocked += 1
            self.get_logger().debug(
                f'Comando de exploración bloqueado - Estado coordinador: {self.coordinator_state["current_state"]}'
            )
            return
        
        # Analizar el comando de velocidad
        command_analysis = self._analyze_velocity_command(msg)
        
        if command_analysis['significant']:
            # Crear solicitud estructurada para el coordinador
            request_data = {
                'type': 'exploration_movement',
                'movement_type': command_analysis['movement_type'],
                'velocity_data': {
                    'linear_x': float(msg.linear.x),
                    'angular_z': float(msg.angular.z)
                },
                'source': 'explore_lite',
                'timestamp': current_time,
                'analysis': command_analysis
            }
            
            # Enviar solicitud al coordinador
            self._send_exploration_request(request_data)
            
            self.last_cmd_vel = msg
            self.last_request_time = current_time
            self.commands_bridged += 1
            
            self.get_logger().debug(
                f'Comando bridgeado: {command_analysis["movement_type"]} '
                f'(lin: {msg.linear.x:.3f}, ang: {msg.angular.z:.3f})'
            )
    
    def _should_forward_command(self) -> bool:
        """Determinar si se debe enviar el comando al coordinador"""
        
        # No enviar si hay override manual o de voz activo
        if (self.coordinator_state.get('manual_override_active', False) or 
            self.coordinator_state.get('voice_control_active', False)):
            return False
        
        # Solo enviar si el coordinador está en modo de exploración automática
        current_state = self.coordinator_state.get('current_state', 'UNKNOWN')
        if current_state not in ['EXPLORING_AUTO', 'IDLE']:
            return False
        
        return True
    
    def _analyze_velocity_command(self, cmd_vel: Twist) -> Dict[str, Any]:
        """Analizar comando de velocidad para categorización"""
        
        linear_x = cmd_vel.linear.x
        angular_z = cmd_vel.angular.z
        
        # Determinar tipo de movimiento
        movement_type = 'stop'
        significant = False
        
        if abs(linear_x) > self.movement_threshold:
            if linear_x > 0:
                movement_type = 'forward'
            else:
                movement_type = 'backward'
            significant = True
        
        if abs(angular_z) > self.angular_threshold:
            if angular_z > 0:
                movement_type = 'turn_left' if movement_type == 'stop' else f'{movement_type}_turn_left'
            else:
                movement_type = 'turn_right' if movement_type == 'stop' else f'{movement_type}_turn_right'
            significant = True
        
        # Calcular magnitudes para priorización
        linear_magnitude = abs(linear_x)
        angular_magnitude = abs(angular_z)
        total_magnitude = math.sqrt(linear_x**2 + angular_z**2)
        
        return {
            'movement_type': movement_type,
            'significant': significant,
            'linear_magnitude': linear_magnitude,
            'angular_magnitude': angular_magnitude,
            'total_magnitude': total_magnitude,
            'is_pure_rotation': abs(linear_x) < self.movement_threshold and abs(angular_z) > self.angular_threshold,
            'is_pure_linear': abs(angular_z) < self.angular_threshold and abs(linear_x) > self.movement_threshold,
            'is_combined': abs(linear_x) > self.movement_threshold and abs(angular_z) > self.angular_threshold
        }
    
    def _send_exploration_request(self, request_data: Dict[str, Any]):
        """Enviar solicitud de movimiento al coordinador"""
        try:
            request_msg = String()
            request_msg.data = json.dumps(request_data)
            self.exploration_request_pub.publish(request_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error enviando solicitud exploración: {e}')
    
    def _publish_bridge_status(self):
        """Publicar estado del bridge periódicamente"""
        try:
            uptime = time.time() - (getattr(self, '_start_time', time.time()))
            
            status_data = {
                'bridge_enabled': self.bridge_enabled,
                'coordinator_state': self.coordinator_state,
                'commands_bridged': self.commands_bridged,
                'commands_blocked': self.commands_blocked,
                'bridge_rate': self.commands_bridged / max(1, self.commands_bridged + self.commands_blocked),
                'last_command_time': self.last_request_time,
                'uptime': uptime,
                'timestamp': time.time()
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.bridge_status_pub.publish(status_msg)
            
            # Log periódico
            if int(time.time()) % 60 == 0:  # Cada minuto
                self.get_logger().info(
                    f'Bridge status: {self.commands_bridged} bridged, '
                    f'{self.commands_blocked} blocked, '
                    f'rate: {status_data["bridge_rate"]:.1%}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error publicando estado bridge: {e}')
    
    def get_bridge_statistics(self) -> Dict[str, Any]:
        """Obtener estadísticas del bridge"""
        total_commands = self.commands_bridged + self.commands_blocked
        bridge_rate = self.commands_bridged / max(1, total_commands)
        
        return {
            'total_commands': total_commands,
            'commands_bridged': self.commands_bridged,
            'commands_blocked': self.commands_blocked,
            'bridge_rate': bridge_rate,
            'coordinator_responsive': self.coordinator_state['current_state'] != 'UNKNOWN',
            'exploration_enabled': self.coordinator_state.get('exploration_active', False)
        }


def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = ExplorationBridge()
        bridge._start_time = time.time()  # Para estadísticas de uptime
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print('Cerrando Exploration Bridge...')
    except Exception as e:
        print(f'Error en Exploration Bridge: {e}')
    finally:
        try:
            bridge.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
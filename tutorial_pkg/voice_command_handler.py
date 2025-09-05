#!/usr/bin/env python3
"""
Nodo ROS2 para manejar comandos de voz y convertirlos en acciones del robot
Ubicación: tutorial_pkg/tutorial_pkg/voice_command_handler.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
import json
import time
import math
from typing import Dict, Optional, List
import threading


class VoiceCommandHandler(Node):
    def __init__(self):
        super().__init__('voice_command_handler')
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Estado del robot
        self.robot_state = {
            "current_position": None,
            "exploration_active": False,
            "last_map_update": 0,
            "navigation_active": False,
            "emergency_stop": False
        }
        
        # Configuración de comandos
        self.movement_speed = {
            "linear": 0.3,  # m/s
            "angular": 0.8   # rad/s
        }
        
        self.movement_duration = 2.0  # segundos para movimientos básicos
        
        # Subscribers para comandos de voz
        self.voice_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            qos_reliable
        )
        
        self.structured_voice_sub = self.create_subscription(
            String,  # Cambiaremos a custom msg después
            '/structured_voice_commands',
            self.structured_command_callback,
            qos_reliable
        )
        
        # Subscribers para estado del robot
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            qos_reliable
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_reliable
        )
        
        # Publishers para control del robot
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_reliable
        )
        
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_reliable
        )
        
        self.exploration_control_pub = self.create_publisher(
            Bool,
            '/exploration_control',
            qos_reliable
        )
        
        # Publishers para respuestas al sistema de voz
        self.status_response_pub = self.create_publisher(
            String,
            '/robot_status_response',
            qos_reliable
        )
        
        self.voice_feedback_pub = self.create_publisher(
            String,
            '/voice_feedback',
            qos_reliable
        )
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Service clients
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, 
            '/local_costmap/clear_entirely_local_costmap'
        )
        
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, 
            '/global_costmap/clear_entirely_global_costmap'
        )
        
        # Timer para movimientos temporales
        self.movement_timer = None
        self.movement_lock = threading.Lock()
        
        # Historial de comandos
        self.command_history = []
        self.max_history = 20
        
        self.get_logger().info("🎤 Manejador de Comandos de Voz inicializado")
    
    def voice_command_callback(self, msg):
        """Procesar comando de voz básico (string simple)"""
        command = msg.data.strip().lower()
        self.get_logger().info(f"🎙️ Comando de voz recibido: '{command}'")
        
        try:
            success, response = self.process_basic_command(command)
            
            # Agregar al historial
            self.add_to_history(command, "basic", success, response)
            
            # Enviar feedback
            self.send_voice_feedback(response)
            
        except Exception as e:
            self.get_logger().error(f"Error procesando comando básico: {e}")
            self.send_voice_feedback(f"Error ejecutando comando: {str(e)}")
    
    def structured_command_callback(self, msg):
        """Procesar comando estructurado con parámetros"""
        try:
            # Parsear JSON del comando estructurado
            command_data = json.loads(msg.data)
            
            command = command_data.get("command", "")
            cmd_type = command_data.get("command_type", "unknown")
            parameters = command_data.get("parameters", {})
            
            self.get_logger().info(f"📡 Comando estructurado: {command} (tipo: {cmd_type})")
            
            success, response = self.process_structured_command(command, cmd_type, parameters)
            
            # Agregar al historial
            self.add_to_history(command, cmd_type, success, response, parameters)
            
            # Enviar feedback
            self.send_voice_feedback(response)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parseando comando estructurado: {e}")
            self.send_voice_feedback("Error: Comando malformado")
        except Exception as e:
            self.get_logger().error(f"Error procesando comando estructurado: {e}")
            self.send_voice_feedback(f"Error: {str(e)}")
    
    def process_basic_command(self, command: str) -> tuple[bool, str]:
        """Procesar comando básico de voz"""
        
        # Comandos de movimiento básico
        if any(word in command for word in ["adelante", "avanzar", "forward"]):
            return self.move_robot("forward")
        
        elif any(word in command for word in ["atras", "retroceder", "backward"]):
            return self.move_robot("backward")
        
        elif any(word in command for word in ["izquierda", "left"]):
            return self.move_robot("left")
        
        elif any(word in command for word in ["derecha", "right"]):
            return self.move_robot("right")
        
        elif any(word in command for word in ["parar", "stop", "detener"]):
            return self.stop_robot()
        
        # Comandos de exploración
        elif any(word in command for word in ["explorar", "explore", "mapear"]):
            return self.control_exploration("start")
        
        elif "pausar" in command and "exploracion" in command.replace("ó", "o"):
            return self.control_exploration("pause")
        
        elif "continuar" in command and "exploracion" in command.replace("ó", "o"):
            return self.control_exploration("resume")
        
        # Comandos de estado
        elif any(word in command for word in ["estado", "status"]):
            return self.get_robot_status()
        
        elif any(word in command for word in ["posicion", "donde", "position", "where"]):
            return self.get_robot_position()
        
        # Comandos de sistema
        elif "emergencia" in command or "emergency" in command:
            return self.emergency_stop()
        
        elif "reiniciar" in command or "restart" in command:
            return self.restart_navigation()
        
        else:
            return False, f"Comando no reconocido: '{command}'"
    
    def process_structured_command(self, command: str, cmd_type: str, parameters: Dict) -> tuple[bool, str]:
        """Procesar comando estructurado con parámetros"""
        
        if cmd_type == "movement":
            return self.handle_movement_command(command, parameters)
        
        elif cmd_type == "exploration":
            return self.handle_exploration_command(command, parameters)
        
        elif cmd_type == "navigation":
            return self.handle_navigation_command(command, parameters)
        
        elif cmd_type == "status":
            return self.handle_status_command(command, parameters)
        
        elif cmd_type == "system":
            return self.handle_system_command(command, parameters)
        
        else:
            return False, f"Tipo de comando no soportado: {cmd_type}"
    
    def handle_movement_command(self, command: str, parameters: Dict) -> tuple[bool, str]:
        """Manejar comandos de movimiento con parámetros"""
        
        distance = parameters.get("distance", self.movement_duration)
        speed_multiplier = parameters.get("speed", 1.0)
        
        # Ajustar velocidades
        linear_speed = self.movement_speed["linear"] * speed_multiplier
        angular_speed = self.movement_speed["angular"] * speed_multiplier
        
        if command == "move_forward":
            return self.move_robot_with_params("forward", distance, linear_speed)
        
        elif command == "move_backward":
            return self.move_robot_with_params("backward", distance, linear_speed)
        
        elif command == "turn_left":
            return self.turn_robot_with_params("left", distance, angular_speed)
        
        elif command == "turn_right":
            return self.turn_robot_with_params("right", distance, angular_speed)
        
        elif command == "stop":
            return self.stop_robot()
        
        else:
            return False, f"Comando de movimiento no reconocido: {command}"
    
    def handle_exploration_command(self, command: str, parameters: Dict) -> tuple[bool, str]:
        """Manejar comandos de exploración"""
        
        if command == "start_exploration":
            return self.control_exploration("start")
        
        elif command == "pause_exploration":
            return self.control_exploration("pause")
        
        elif command == "resume_exploration":
            return self.control_exploration("resume")
        
        elif command == "stop_exploration":
            return self.control_exploration("stop")
        
        else:
            return False, f"Comando de exploración no reconocido: {command}"
    
    def handle_navigation_command(self, command: str, parameters: Dict) -> tuple[bool, str]:
        """Manejar comandos de navegación a puntos específicos"""
        
        if command == "go_to_point":
            x = parameters.get("x")
            y = parameters.get("y")
            yaw = parameters.get("yaw", 0.0)
            
            if x is not None and y is not None:
                return self.navigate_to_point(float(x), float(y), float(yaw))
            else:
                return False, "Coordenadas x,y requeridas para navegación"
        
        elif command == "return_home":
            return self.navigate_to_point(0.0, 0.0, 0.0)
        
        else:
            return False, f"Comando de navegación no reconocido: {command}"
    
    def handle_status_command(self, command: str, parameters: Dict) -> tuple[bool, str]:
        """Manejar comandos de consulta de estado"""
        
        if command == "get_position":
            return self.get_robot_position()
        
        elif command == "get_map_status":
            return self.get_map_status()
        
        elif command == "get_exploration_progress":
            return self.get_exploration_progress()
        
        else:
            return self.get_robot_status()
    
    def handle_system_command(self, command: str, parameters: Dict) -> tuple[bool, str]:
        """Manejar comandos de sistema"""
        
        if command == "emergency_stop":
            return self.emergency_stop()
        
        elif command == "restart_navigation":
            return self.restart_navigation()
        
        elif command == "clear_costmaps":
            return self.clear_costmaps()
        
        else:
            return False, f"Comando de sistema no reconocido: {command}"
    
    def move_robot(self, direction: str) -> tuple[bool, str]:
        """Movimiento básico del robot"""
        return self.move_robot_with_params(direction, self.movement_duration, None)
    
    def move_robot_with_params(self, direction: str, duration: float, speed: Optional[float] = None) -> tuple[bool, str]:
        """Movimiento del robot con parámetros"""
        
        if self.robot_state["emergency_stop"]:
            return False, "Robot en parada de emergencia"
        
        try:
            with self.movement_lock:
                # Cancelar movimiento anterior si existe
                if self.movement_timer is not None:
                    self.movement_timer.cancel()
                
                # Crear comando de velocidad
                cmd = Twist()
                
                linear_speed = speed or self.movement_speed["linear"]
                angular_speed = speed or self.movement_speed["angular"]
                
                if direction == "forward":
                    cmd.linear.x = linear_speed
                    response = f"Moviendo adelante por {duration:.1f} segundos"
                    
                elif direction == "backward":
                    cmd.linear.x = -linear_speed
                    response = f"Moviendo atrás por {duration:.1f} segundos"
                    
                elif direction == "left":
                    cmd.angular.z = angular_speed
                    response = f"Girando izquierda por {duration:.1f} segundos"
                    
                elif direction == "right":
                    cmd.angular.z = -angular_speed
                    response = f"Girando derecha por {duration:.1f} segundos"
                
                else:
                    return False, f"Dirección no válida: {direction}"
                
                # Publicar comando
                self.cmd_vel_pub.publish(cmd)
                
                # Programar parada automática
                self.movement_timer = self.create_timer(
                    duration,
                    lambda: self.stop_robot_timer()
                )
                
                self.get_logger().info(f"🚗 {response}")
                return True, response
                
        except Exception as e:
            self.get_logger().error(f"Error en movimiento: {e}")
            return False, f"Error ejecutando movimiento: {str(e)}"
    
    def turn_robot_with_params(self, direction: str, angle_or_duration: float, angular_speed: float) -> tuple[bool, str]:
        """Giro del robot con parámetros específicos"""
        # Similar a move_robot_with_params pero optimizado para giros
        return self.move_robot_with_params(direction, angle_or_duration, angular_speed)
    
    def stop_robot(self) -> tuple[bool, str]:
        """Parar el robot inmediatamente"""
        try:
            with self.movement_lock:
                # Cancelar timer de movimiento
                if self.movement_timer is not None:
                    self.movement_timer.cancel()
                    self.movement_timer = None
                
                # Enviar comando de parada
                stop_cmd = Twist()  # Todas las velocidades en 0
                self.cmd_vel_pub.publish(stop_cmd)
                
                self.get_logger().info("⏹️ Robot detenido")
                return True, "Robot detenido"
                
        except Exception as e:
            self.get_logger().error(f"Error deteniendo robot: {e}")
            return False, f"Error deteniendo robot: {str(e)}"
    
    def stop_robot_timer(self):
        """Callback para parar robot después del timer"""
        try:
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            
            if self.movement_timer is not None:
                self.movement_timer.cancel()
                self.movement_timer = None
                
        except Exception as e:
            self.get_logger().error(f"Error en stop_robot_timer: {e}")
    
    def control_exploration(self, action: str) -> tuple[bool, str]:
        """Controlar exploración automática"""
        try:
            control_msg = Bool()
            
            if action == "start":
                control_msg.data = True
                self.robot_state["exploration_active"] = True
                response = "Iniciando exploración automática"
                
            elif action == "pause" or action == "stop":
                control_msg.data = False
                self.robot_state["exploration_active"] = False
                response = "Pausando exploración"
                
            elif action == "resume":
                control_msg.data = True
                self.robot_state["exploration_active"] = True
                response = "Reanudando exploración"
            
            else:
                return False, f"Acción de exploración no válida: {action}"
            
            self.exploration_control_pub.publish(control_msg)
            self.get_logger().info(f"🗺️ {response}")
            
            return True, response
            
        except Exception as e:
            self.get_logger().error(f"Error controlando exploración: {e}")
            return False, f"Error en exploración: {str(e)}"
    
    def navigate_to_point(self, x: float, y: float, yaw: float = 0.0) -> tuple[bool, str]:
        """Navegar a un punto específico"""
        try:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = 0.0
            
            # Convertir yaw a quaternion
            goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
            
            self.goal_pub.publish(goal_msg)
            
            response = f"Navegando a punto ({x:.2f}, {y:.2f})"
            self.get_logger().info(f"🎯 {response}")
            
            return True, response
            
        except Exception as e:
            self.get_logger().error(f"Error navegando a punto: {e}")
            return False, f"Error en navegación: {str(e)}"
    
    def get_robot_status(self) -> tuple[bool, str]:
        """Obtener estado general del robot"""
        try:
            status_parts = []
            
            if self.robot_state["current_position"]:
                pos = self.robot_state["current_position"]
                status_parts.append(f"Posición: ({pos['x']:.2f}, {pos['y']:.2f})")
            
            status_parts.append(f"Exploración: {'Activa' if self.robot_state['exploration_active'] else 'Inactiva'}")
            
            if self.robot_state["emergency_stop"]:
                status_parts.append("⚠️ PARADA DE EMERGENCIA ACTIVA")
            
            response = "Estado del robot: " + ", ".join(status_parts)
            return True, response
            
        except Exception as e:
            return False, f"Error obteniendo estado: {str(e)}"
    
    def get_robot_position(self) -> tuple[bool, str]:
        """Obtener posición actual del robot"""
        try:
            if self.robot_state["current_position"]:
                pos = self.robot_state["current_position"]
                response = f"Posición actual: X={pos['x']:.2f}m, Y={pos['y']:.2f}m, Orientación={pos['yaw']:.2f}rad"
            else:
                response = "Posición no disponible"
            
            return True, response
            
        except Exception as e:
            return False, f"Error obteniendo posición: {str(e)}"
    
    def get_map_status(self) -> tuple[bool, str]:
        """Obtener estado del mapa"""
        try:
            time_since_update = time.time() - self.robot_state["last_map_update"]
            
            if time_since_update < 60:  # Menos de 1 minuto
                map_status = "Actualizado recientemente"
            elif time_since_update < 300:  # Menos de 5 minutos
                map_status = f"Actualizado hace {time_since_update/60:.1f} minutos"
            else:
                map_status = "No actualizado recientemente"
            
            response = f"Estado del mapa: {map_status}"
            return True, response
            
        except Exception as e:
            return False, f"Error obteniendo estado del mapa: {str(e)}"
    
    def get_exploration_progress(self) -> tuple[bool, str]:
        """Obtener progreso de exploración"""
        # Esta información debería venir del nodo de exploración
        return True, f"Exploración {'en progreso' if self.robot_state['exploration_active'] else 'pausada'}"
    
    def emergency_stop(self) -> tuple[bool, str]:
        """Parada de emergencia completa"""
        try:
            self.robot_state["emergency_stop"] = True
            
            # Parar movimiento inmediatamente
            self.stop_robot()
            
            # Parar exploración
            self.control_exploration("stop")
            
            # Publicar múltiples veces para asegurar
            for _ in range(3):
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
            
            response = "🚨 PARADA DE EMERGENCIA ACTIVADA"
            self.get_logger().warn(response)
            
            return True, response
            
        except Exception as e:
            return False, f"Error en parada de emergencia: {str(e)}"
    
    def restart_navigation(self) -> tuple[bool, str]:
        """Reiniciar sistema de navegación"""
        try:
            # Resetear estado de emergencia
            self.robot_state["emergency_stop"] = False
            
            # Limpiar costmaps
            self.clear_costmaps()
            
            response = "Sistema de navegación reiniciado"
            self.get_logger().info(response)
            
            return True, response
            
        except Exception as e:
            return False, f"Error reiniciando navegación: {str(e)}"
    
    def clear_costmaps(self) -> tuple[bool, str]:
        """Limpiar costmaps locales y globales"""
        try:
            success_count = 0
            
            # Limpiar costmap local
            if self.clear_local_costmap.service_is_ready():
                req = ClearEntireCostmap.Request()
                future = self.clear_local_costmap.call_async(req)
                # No esperamos la respuesta para no bloquear
                success_count += 1
            
            # Limpiar costmap global  
            if self.clear_global_costmap.service_is_ready():
                req = ClearEntireCostmap.Request()
                future = self.clear_global_costmap.call_async(req)
                success_count += 1
            
            if success_count > 0:
                response = "Costmaps limpiados"
                self.get_logger().info("🧹 " + response)
                return True, response
            else:
                return False, "Servicios de limpieza no disponibles"
                
        except Exception as e:
            return False, f"Error limpiando costmaps: {str(e)}"
    
    def pose_callback(self, msg):
        """Actualizar posición del robot"""
        try:
            self.robot_state["current_position"] = {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
                "yaw": 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
            }
        except Exception as e:
            self.get_logger().error(f"Error actualizando posición: {e}")
    
    def map_callback(self, msg):
        """Callback para actualizaciones del mapa"""
        self.robot_state["last_map_update"] = time.time()
    
    def send_voice_feedback(self, message: str):
        """Enviar feedback al sistema de voz"""
        try:
            feedback_msg = String()
            feedback_msg.data = message
            self.voice_feedback_pub.publish(feedback_msg)
        except Exception as e:
            self.get_logger().error(f"Error enviando feedback: {e}")
    
    def add_to_history(self, command: str, cmd_type: str, success: bool, response: str, parameters: Optional[Dict] = None):
        """Agregar comando al historial"""
        try:
            history_entry = {
                "timestamp": time.time(),
                "command": command,
                "type": cmd_type,
                "success": success,
                "response": response,
                "parameters": parameters or {}
            }
            
            self.command_history.append(history_entry)
            
            # Mantener tamaño del historial
            if len(self.command_history) > self.max_history:
                self.command_history = self.command_history[-self.max_history:]
                
        except Exception as e:
            self.get_logger().error(f"Error agregando al historial: {e}")
    
    def get_command_history(self) -> List[Dict]:
        """Obtener historial de comandos"""
        return self.command_history.copy()


def main(args=None):
    rclpy.init(args=args)
    
    voice_handler = VoiceCommandHandler()
    
    try:
        rclpy.spin(voice_handler)
    except KeyboardInterrupt:
        voice_handler.get_logger().info("Nodo detenido por usuario")
    finally:
        voice_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
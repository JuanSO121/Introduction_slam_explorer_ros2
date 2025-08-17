#!/usr/bin/env python3
"""
Nodo para monitorear la exploración y reiniciarla automáticamente
si se detiene por falta de fronteras.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Parámetros
        self.last_exploration_time = time.time()
        self.last_map_size = 0
        self.robot_stuck_counter = 0
        self.exploration_timeout = 60.0  # 60 segundos sin progreso
        self.min_map_growth = 10  # Mínimo crecimiento del mapa
        self.max_stuck_time = 30.0  # Tiempo máximo sin movimiento
        
        # Estado del robot
        self.current_velocity = Twist()
        self.last_position = None
        self.position_history = []
        self.last_scan_ranges = None
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            qos_profile
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            qos_profile
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # Publishers
        self.exploration_goal_pub = self.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            qos_profile
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )
        
        # Timer para monitoreo
        self.monitor_timer = self.create_timer(5.0, self.monitor_exploration)
        
        self.get_logger().info("Monitor de exploración iniciado")
    
    def map_callback(self, msg):
        """Monitorea el crecimiento del mapa"""
        current_map_size = sum(1 for cell in msg.data if cell >= 0)  # Celdas conocidas
        
        if current_map_size > self.last_map_size + self.min_map_growth:
            self.last_exploration_time = time.time()
            self.last_map_size = current_map_size
            self.get_logger().debug(f"Mapa creciendo: {current_map_size} celdas")
    
    def cmd_vel_callback(self, msg):
        """Monitorea el movimiento del robot"""
        self.current_velocity = msg
        
        # Verificar si el robot se está moviendo
        is_moving = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)
        
        if is_moving:
            self.robot_stuck_counter = 0
        else:
            self.robot_stuck_counter += 1
    
    def scan_callback(self, msg):
        """Analiza datos del láser para detectar espacios explorables"""
        self.last_scan_ranges = msg.ranges
    
    def find_exploration_point(self):
        """Encuentra un punto para continuar la exploración"""
        if not self.last_scan_ranges:
            return None
        
        ranges = np.array(self.last_scan_ranges)
        angles = np.linspace(-math.pi, math.pi, len(ranges))
        
        # Buscar la dirección con mayor espacio libre
        max_range = 0
        best_angle = 0
        
        for i, (r, angle) in enumerate(zip(ranges, angles)):
            if not math.isinf(r) and r > max_range and r > 1.0:
                max_range = r
                best_angle = angle
        
        if max_range > 1.5:  # Si hay espacio suficiente
            # Crear objetivo en esa dirección
            goal = PoseStamped()
            goal.header.frame_id = "base_footprint"
            goal.header.stamp = self.get_clock().now().to_msg()
            
            # Objetivo a 70% de la distancia máxima detectada
            distance = min(max_range * 0.7, 3.0)
            goal.pose.position.x = distance * math.cos(best_angle)
            goal.pose.position.y = distance * math.sin(best_angle)
            goal.pose.position.z = 0.0
            
            # Orientación hacia el objetivo
            goal.pose.orientation.z = math.sin(best_angle / 2.0)
            goal.pose.orientation.w = math.cos(best_angle / 2.0)
            
            return goal
        
        return None
    
    def perform_exploration_spin(self):
        """Hace que el robot gire para buscar nuevas áreas"""
        self.get_logger().info("Realizando giro de exploración...")
        
        twist = Twist()
        twist.angular.z = 0.5  # Velocidad angular moderada
        
        # Publicar comando de giro por 3 segundos
        for _ in range(15):  # 3 segundos a 5Hz
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.2)
        
        # Detener el robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
    
    def send_random_exploration_goal(self):
        """Envía un objetivo aleatorio para continuar la exploración"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Objetivo aleatorio en un radio de 3 metros
        angle = np.random.uniform(-math.pi, math.pi)
        distance = np.random.uniform(1.5, 3.0)
        
        goal.pose.position.x = distance * math.cos(angle)
        goal.pose.position.y = distance * math.sin(angle)
        goal.pose.position.z = 0.0
        
        # Orientación aleatoria
        goal.pose.orientation.z = math.sin(angle / 2.0)
        goal.pose.orientation.w = math.cos(angle / 2.0)
        
        self.exploration_goal_pub.publish(goal)
        self.get_logger().info(f"Enviado objetivo de exploración: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}")
    
    def monitor_exploration(self):
        """Función principal de monitoreo"""
        current_time = time.time()
        time_since_exploration = current_time - self.last_exploration_time
        
        # Verificar si la exploración se ha detenido
        exploration_stalled = time_since_exploration > self.exploration_timeout
        robot_stuck = self.robot_stuck_counter > (self.max_stuck_time / 5.0)  # 5 segundos de timer
        
        if exploration_stalled or robot_stuck:
            self.get_logger().warn(f"Exploración detenida detectada:")
            self.get_logger().warn(f"  - Tiempo sin progreso: {time_since_exploration:.1f}s")
            self.get_logger().warn(f"  - Robot inmóvil por: {self.robot_stuck_counter * 5.0:.1f}s")
            
            # Reiniciar exploración
            self.restart_exploration()
    
    def restart_exploration(self):
        """Reinicia la exploración cuando se detecta que se ha detenido"""
        self.get_logger().info("🔄 REINICIANDO EXPLORACIÓN...")
        
        # 1. Primero hacer un giro para actualizar el mapa
        self.perform_exploration_spin()
        
        # 2. Buscar un punto de exploración basado en el láser
        exploration_goal = self.find_exploration_point()
        
        if exploration_goal:
            # Cambiar frame a map
            exploration_goal.header.frame_id = "map"
            self.exploration_goal_pub.publish(exploration_goal)
            self.get_logger().info("✅ Objetivo de exploración enviado basado en láser")
        else:
            # 3. Si no hay punto basado en láser, enviar objetivo aleatorio
            self.send_random_exploration_goal()
            self.get_logger().info("✅ Objetivo de exploración aleatorio enviado")
        
        # Resetear contadores
        self.last_exploration_time = time.time()
        self.robot_stuck_counter = 0
        
        self.get_logger().info("🚀 Exploración reiniciada exitosamente")


def main(args=None):
    rclpy.init(args=args)
    
    exploration_monitor = ExplorationMonitor()
    
    try:
        rclpy.spin(exploration_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        exploration_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
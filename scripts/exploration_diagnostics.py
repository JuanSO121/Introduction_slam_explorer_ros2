#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import threading
import time

class ExplorationDiagnostics(Node):
    def __init__(self):
        super().__init__('exploration_diagnostics')
        
        # QoS profiles
        self.map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            self.map_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)
        
        # Variables de estado
        self.map_data = None
        self.last_position = None
        self.last_cmd_vel = None
        self.last_laser = None
        self.exploration_started = False
        
        # Estadísticas
        self.total_area = 0
        self.explored_area = 0
        self.exploration_percentage = 0.0
        self.stuck_counter = 0
        self.movement_threshold = 0.05  # metros
        self.stuck_threshold = 20  # checks sin movimiento significativo
        
        # Timer para diagnósticos periódicos
        self.timer = self.create_timer(5.0, self.run_diagnostics)
        
        # Lock para thread safety
        self.lock = threading.Lock()
        
        self.get_logger().info('🔧 Sistema de diagnósticos iniciado')

    def map_callback(self, msg):
        with self.lock:
            self.map_data = msg
            self.calculate_exploration_progress()

    def odom_callback(self, msg):
        with self.lock:
            current_position = msg.pose.pose.position
            
            if self.last_position is not None:
                # Calcular distancia movida
                dx = current_position.x - self.last_position.x
                dy = current_position.y - self.last_position.y
                distance_moved = np.sqrt(dx*dx + dy*dy)
                
                if distance_moved < self.movement_threshold:
                    self.stuck_counter += 1
                else:
                    self.stuck_counter = 0
            
            self.last_position = current_position

    def cmd_vel_callback(self, msg):
        with self.lock:
            self.last_cmd_vel = msg

    def laser_callback(self, msg):
        with self.lock:
            self.last_laser = msg

    def calculate_exploration_progress(self):
        if self.map_data is None:
            return
        
        data = np.array(self.map_data.data)
        
        # -1: desconocido, 0: libre, 100: ocupado
        unknown_cells = np.sum(data == -1)
        free_cells = np.sum(data == 0)
        occupied_cells = np.sum(data == 100)
        
        self.total_area = len(data)
        self.explored_area = free_cells + occupied_cells
        
        if self.total_area > 0:
            self.exploration_percentage = (self.explored_area / self.total_area) * 100
        
        self.get_logger().info(
            f'📊 Progreso de exploración: {self.exploration_percentage:.1f}% '
            f'(Explorado: {self.explored_area}, Total: {self.total_area})'
        )

    def check_robot_health(self):
        """Verificar el estado de salud del robot"""
        health_status = []
        
        # 1. Verificar si recibimos datos del láser
        if self.last_laser is None:
            health_status.append("⚠️ No hay datos del láser")
        else:
            # Verificar si el láser detecta obstáculos
            ranges = np.array(self.last_laser.ranges)
            valid_ranges = ranges[~np.isinf(ranges)]
            if len(valid_ranges) == 0:
                health_status.append("⚠️ Láser no detecta obstáculos")
        
        # 2. Verificar comandos de velocidad
        if self.last_cmd_vel is None:
            health_status.append("⚠️ No hay comandos de velocidad")
        else:
            linear_vel = abs(self.last_cmd_vel.linear.x)
            angular_vel = abs(self.last_cmd_vel.angular.z)
            
            if linear_vel == 0 and angular_vel == 0:
                health_status.append("🛑 Robot detenido")
        
        # 3. Verificar si está atascado
        if self.stuck_counter > self.stuck_threshold:
            health_status.append(f"🚫 Robot posiblemente atascado ({self.stuck_counter} checks)")
        
        # 4. Verificar mapa
        if self.map_data is None:
            health_status.append("⚠️ No hay datos del mapa")
        
        return health_status

    def check_exploration_status(self):
        """Verificar el estado de la exploración"""
        status = []
        
        if self.exploration_percentage < 10:
            status.append("🚀 Exploración inicial")
        elif self.exploration_percentage < 50:
            status.append("🔄 Exploración en progreso")
        elif self.exploration_percentage < 80:
            status.append("📈 Exploración avanzada")
        elif self.exploration_percentage < 95:
            status.append("🏁 Exploración casi completa")
        else:
            status.append("✅ Exploración completa")
        
        # Verificar si hay fronteras disponibles
        if self.map_data is not None:
            # Aquí podrías implementar detección de fronteras
            pass
        
        return status

    def generate_recommendations(self, health_issues):
        """Generar recomendaciones basadas en los problemas detectados"""
        recommendations = []
        
        for issue in health_issues:
            if "atascado" in issue.lower():
                recommendations.append("💡 Recomendación: Reiniciar exploración o mover robot manualmente")
            elif "láser" in issue.lower():
                recommendations.append("💡 Recomendación: Verificar conexión del láser LiDAR")
            elif "velocidad" in issue.lower():
                recommendations.append("💡 Recomendación: Verificar navegación y planificador")
            elif "mapa" in issue.lower():
                recommendations.append("💡 Recomendación: Verificar SLAM y publicación del mapa")
        
        if not health_issues:
            recommendations.append("✅ Sistema funcionando correctamente")
        
        return recommendations

    def run_diagnostics(self):
        """Ejecutar diagnósticos completos"""
        self.get_logger().info('🔍 Ejecutando diagnósticos del sistema...')
        
        # Verificar salud del robot
        health_issues = self.check_robot_health()
        
        # Verificar estado de exploración
        exploration_status = self.check_exploration_status()
        
        # Generar recomendaciones
        recommendations = self.generate_recommendations(health_issues)
        
        # Log de resultados
        self.get_logger().info('=' * 60)
        self.get_logger().info('📋 REPORTE DE DIAGNÓSTICOS')
        self.get_logger().info('=' * 60)
        
        # Estado de salud
        if health_issues:
            self.get_logger().warning('⚠️ PROBLEMAS DETECTADOS:')
            for issue in health_issues:
                self.get_logger().warning(f'  {issue}')
        else:
            self.get_logger().info('✅ ESTADO DE SALUD: OK')
        
        # Estado de exploración
        self.get_logger().info('📊 ESTADO DE EXPLORACIÓN:')
        for status in exploration_status:
            self.get_logger().info(f'  {status}')
        
        # Recomendaciones
        self.get_logger().info('💡 RECOMENDACIONES:')
        for rec in recommendations:
            self.get_logger().info(f'  {rec}')
        
        self.get_logger().info('=' * 60)
        
        # Publicar estado
        status_msg = String()
        status_msg.data = f"Exploración: {self.exploration_percentage:.1f}%, Problemas: {len(health_issues)}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
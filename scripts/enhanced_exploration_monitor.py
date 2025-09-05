#!/usr/bin/env python3
"""
Monitor de exploración mejorado con limpieza automática de costmaps
y detección inteligente de áreas no exploradas
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import ClearEntireCostmap
import numpy as np
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sklearn.cluster import DBSCAN
import cv2


class EnhancedExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Estado del sistema
        self.current_map = None
        self.map_metadata = None
        self.robot_position = None
        self.last_exploration_time = time.time()
        self.last_map_size = 0
        self.stuck_counter = 0
        self.unexplored_frontiers = []
        self.visited_positions = []
        
        # Parámetros mejorados
        self.exploration_timeout = 45.0      # Tiempo sin progreso
        self.stuck_threshold = 25.0          # Tiempo sin movimiento
        self.min_frontier_size = 8           # Tamaño mínimo de frontera
        self.frontier_distance_threshold = 1.5  # Distancia mínima entre fronteras
        self.costmap_clear_interval = 30.0   # Intervalo limpieza costmaps
        self.last_costmap_clear = 0
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile
        )
        
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', 
            self.robot_pose_callback, qos_profile
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', qos_profile
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', qos_profile
        )
        
        # Clientes de servicio para limpiar costmaps
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )
        
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        
        # Timers
        self.monitor_timer = self.create_timer(3.0, self.monitor_exploration)
        self.frontier_timer = self.create_timer(10.0, self.find_unexplored_areas)
        self.cleanup_timer = self.create_timer(5.0, self.periodic_cleanup)
        
        self.get_logger().info("🚀 Monitor de exploración mejorado iniciado")
    
    def map_callback(self, msg):
        """Procesa actualizaciones del mapa"""
        self.current_map = msg
        self.map_metadata = msg.info
        
        # Contar celdas conocidas
        current_size = sum(1 for cell in msg.data if cell >= 0)
        
        if current_size > self.last_map_size + 50:  # Crecimiento significativo
            self.last_exploration_time = time.time()
            self.last_map_size = current_size
            self.get_logger().debug(f"📈 Mapa creciendo: {current_size} celdas")
    
    def robot_pose_callback(self, msg):
        """Actualiza la posición del robot"""
        self.robot_position = msg.pose.pose.position
        
        # Registrar posición visitada
        if self.robot_position:
            current_pos = (self.robot_position.x, self.robot_position.y)
            self.visited_positions.append(current_pos)
            
            # Mantener solo las últimas 100 posiciones
            if len(self.visited_positions) > 100:
                self.visited_positions.pop(0)
    
    def cmd_vel_callback(self, msg):
        """Monitorea movimiento del robot"""
        is_moving = abs(msg.linear.x) > 0.05 or abs(msg.angular.z) > 0.1
        
        if is_moving:
            self.stuck_counter = 0
        else:
            self.stuck_counter += 1
    
    def scan_callback(self, msg):
        """Procesa datos del láser"""
        # Aquí podrías implementar detección de obstáculos fantasma
        # comparando con el mapa conocido
        pass
    
    def clear_costmaps(self):
        """Limpia ambos costmaps para eliminar obstáculos fantasma"""
        current_time = time.time()
        
        if current_time - self.last_costmap_clear < self.costmap_clear_interval:
            return
        
        self.get_logger().info("🧹 Limpiando costmaps...")
        
        # Limpiar costmap local
        if self.clear_local_costmap.service_is_ready():
            req = ClearEntireCostmap.Request()
            future = self.clear_local_costmap.call_async(req)
        
        # Limpiar costmap global  
        if self.clear_global_costmap.service_is_ready():
            req = ClearEntireCostmap.Request()
            future = self.clear_global_costmap.call_async(req)
        
        self.last_costmap_clear = current_time
        time.sleep(0.5)  # Pequeña pausa para que se aplique la limpieza
    
    def find_frontiers(self, occupancy_grid):
        """Encuentra fronteras usando procesamiento de imágenes"""
        if not occupancy_grid:
            return []
        
        # Convertir a array numpy
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin = occupancy_grid.info.origin.position
        
        # Convertir datos del mapa
        map_array = np.array(occupancy_grid.data).reshape(height, width)
        
        # Crear imagen binaria (0=libre, 100=ocupado, -1=desconocido)
        free_space = (map_array == 0).astype(np.uint8) * 255
        unknown_space = (map_array == -1).astype(np.uint8) * 255
        
        # Encontrar fronteras (bordes entre espacio libre y desconocido)
        kernel = np.ones((3, 3), np.uint8)
        
        # Dilatar espacio libre
        free_dilated = cv2.dilate(free_space, kernel, iterations=1)
        
        # Fronteras = espacio libre dilatado ∩ espacio desconocido
        frontiers = cv2.bitwise_and(free_dilated, unknown_space)
        
        # Encontrar contornos de fronteras
        contours, _ = cv2.findContours(frontiers, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        frontier_points = []
        for contour in contours:
            # Solo considerar fronteras suficientemente grandes
            if cv2.contourArea(contour) > self.min_frontier_size:
                # Obtener centroide
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Convertir coordenadas de grid a mundo
                    world_x = origin.x + cx * resolution
                    world_y = origin.y + cy * resolution
                    
                    frontier_points.append((world_x, world_y, cv2.contourArea(contour)))
        
        return frontier_points
    
    def find_unexplored_areas(self):
        """Encuentra áreas no exploradas usando el mapa actual"""
        if not self.current_map:
            return
        
        frontiers = self.find_frontiers(self.current_map)
        
        # Filtrar fronteras por distancia mínima
        filtered_frontiers = []
        for frontier in frontiers:
            x, y, area = frontier
            too_close = False
            
            # Verificar distancia a fronteras ya conocidas
            for existing in filtered_frontiers:
                ex, ey, _ = existing
                distance = math.sqrt((x - ex)**2 + (y - ey)**2)
                if distance < self.frontier_distance_threshold:
                    too_close = True
                    break
            
            if not too_close:
                filtered_frontiers.append(frontier)
        
        self.unexplored_frontiers = filtered_frontiers
        
        if len(self.unexplored_frontiers) > 0:
            self.get_logger().info(f"🎯 Encontradas {len(self.unexplored_frontiers)} fronteras no exploradas")
        else:
            self.get_logger().info("🏁 No se encontraron más fronteras - exploración completa?")
    
    def get_best_frontier(self):
        """Selecciona la mejor frontera para explorar"""
        if not self.unexplored_frontiers or not self.robot_position:
            return None
        
        robot_x = self.robot_position.x
        robot_y = self.robot_position.y
        
        # Calcular score para cada frontera (distancia + tamaño)
        best_frontier = None
        best_score = float('inf')
        
        for frontier in self.unexplored_frontiers:
            x, y, area = frontier
            
            # Distancia al robot
            distance = math.sqrt((x - robot_x)**2 + (y - robot_y)**2)
            
            # Score combinado (menor distancia + mayor área = mejor)
            score = distance / (area + 1)  # +1 para evitar división por cero
            
            if score < best_score:
                best_score = score
                best_frontier = frontier
        
        return best_frontier
    
    def send_exploration_goal(self, x, y, yaw=None):
        """Envía objetivo de exploración"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        if yaw is None:
            # Orientación hacia el objetivo si no se especifica
            if self.robot_position:
                dx = x - self.robot_position.x
                dy = y - self.robot_position.y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
        
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f"🎯 Objetivo enviado: ({x:.2f}, {y:.2f})")
    
    def perform_recovery_spin(self):
        """Giro de recuperación para actualizar percepción"""
        self.get_logger().info("🔄 Realizando giro de recuperación...")
        
        twist = Twist()
        twist.angular.z = 1.0
        
        # Girar por 2 segundos
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.2)
        
        # Detener
        self.cmd_vel_pub.publish(Twist())
        
        # Limpiar costmaps después del giro
        time.sleep(1.0)
        self.clear_costmaps()
    
    def periodic_cleanup(self):
        """Limpieza periódica del sistema"""
        current_time = time.time()
        
        # Limpieza automática cada cierto tiempo
        if current_time - self.last_costmap_clear > self.costmap_clear_interval:
            self.clear_costmaps()
    
    def monitor_exploration(self):
        """Monitoreo principal de la exploración"""
        current_time = time.time()
        time_since_exploration = current_time - self.last_exploration_time
        
        exploration_stalled = time_since_exploration > self.exploration_timeout
        robot_stuck = self.stuck_counter > (self.stuck_threshold / 3.0)
        
        if exploration_stalled or robot_stuck:
            self.get_logger().warn(f"⚠️  Problema detectado:")
            self.get_logger().warn(f"   - Tiempo sin progreso: {time_since_exploration:.1f}s")
            self.get_logger().warn(f"   - Inmóvil por: {self.stuck_counter * 3.0:.1f}s")
            
            # Estrategia de recuperación
            self.restart_exploration()
    
    def restart_exploration(self):
        """Reinicia la exploración con estrategias múltiples"""
        self.get_logger().info("🚨 REINICIANDO EXPLORACIÓN")
        
        # 1. Limpiar costmaps primero
        self.clear_costmaps()
        
        # 2. Giro de recuperación
        self.perform_recovery_spin()
        
        # 3. Buscar nueva frontera
        self.find_unexplored_areas()
        
        best_frontier = self.get_best_frontier()
        
        if best_frontier:
            x, y, area = best_frontier
            self.send_exploration_goal(x, y)
            self.get_logger().info(f"✅ Enviado a frontera: ({x:.2f}, {y:.2f}) área={area:.0f}")
        else:
            # 4. Si no hay fronteras, explorar aleatoriamente
            self.explore_randomly()
        
        # Resetear contadores
        self.last_exploration_time = time.time()
        self.stuck_counter = 0
    
    def explore_randomly(self):
        """Exploración aleatoria cuando no hay fronteras"""
        self.get_logger().info("🎲 Explorando aleatoriamente...")
        
        if not self.robot_position:
            return
        
        # Generar múltiples objetivos aleatorios
        for i in range(3):
            angle = np.random.uniform(0, 2 * np.pi)
            distance = np.random.uniform(2.0, 4.0)
            
            x = self.robot_position.x + distance * np.cos(angle)
            y = self.robot_position.y + distance * np.sin(angle)
            
            self.send_exploration_goal(x, y, angle)
            time.sleep(1.0)  # Pausa entre objetivos


def main(args=None):
    rclpy.init(args=args)
    
    monitor = EnhancedExplorationMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Monitor detenido por usuario")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Nodo para diagnosticar obstáculos fantasma en los costmaps
comparando con datos del sensor láser
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav2_msgs.srv import ClearEntireCostmap
import numpy as np
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros
import tf2_geometry_msgs


class ObstacleDiagnostics(Node):
    def __init__(self):
        super().__init__('obstacle_diagnostics')
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # TF2 para transformaciones
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Estado del sistema
        self.current_scan = None
        self.local_costmap = None
        self.global_costmap = None
        self.robot_pose = None
        self.phantom_obstacles = []
        
        # Parámetros
        self.scan_timeout = 2.0  # Timeout para datos del láser
        self.obstacle_threshold = 80  # Umbral para considerar obstáculo en costmap
        self.phantom_distance_threshold = 0.5  # Distancia para considerar obstáculo fantasma
        self.clear_phantom_threshold = 5  # Número de obstáculos fantasma para limpiar
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile
        )
        
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', 
            self.local_costmap_callback, qos_profile
        )
        
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self.global_costmap_callback, qos_profile
        )
        
        # Publishers
        self.phantom_markers_pub = self.create_publisher(
            MarkerArray, '/phantom_obstacles_markers', 10
        )
        
        self.diagnostics_pub = self.create_publisher(
            Marker, '/obstacle_diagnostics', 10
        )
        
        # Servicios para limpiar costmaps
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )
        
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        
        # Timer para diagnósticos
        self.diagnostic_timer = self.create_timer(3.0, self.run_diagnostics)
        
        self.get_logger().info("🔍 Diagnóstico de obstáculos iniciado")
    
    def scan_callback(self, msg):
        """Guarda datos del láser"""
        self.current_scan = msg
    
    def local_costmap_callback(self, msg):
        """Guarda costmap local"""
        self.local_costmap = msg
    
    def global_costmap_callback(self, msg):
        """Guarda costmap global"""
        self.global_costmap = msg
    
    def get_robot_pose_in_costmap(self, costmap):
        """Obtiene la pose del robot en coordenadas del costmap"""
        try:
            # Transformar pose del robot al frame del costmap
            transform = self.tf_buffer.lookup_transform(
                costmap.header.frame_id,
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Convertir a coordenadas del grid
            origin_x = costmap.info.origin.position.x
            origin_y = costmap.info.origin.position.y
            resolution = costmap.info.resolution
            
            grid_x = int((robot_x - origin_x) / resolution)
            grid_y = int((robot_y - origin_y) / resolution)
            
            return (robot_x, robot_y, grid_x, grid_y)
            
        except Exception as e:
            self.get_logger().debug(f"Error obteniendo pose del robot: {e}")
            return None
    
    def scan_to_cartesian(self, scan_msg, robot_pose):
        """Convierte datos del láser a coordenadas cartesianas"""
        if not scan_msg or not robot_pose:
            return []
        
        robot_x, robot_y, _, _ = robot_pose
        points = []
        
        angle = scan_msg.angle_min
        for i, range_val in enumerate(scan_msg.ranges):
            if not math.isinf(range_val) and not math.isnan(range_val):
                if scan_msg.range_min <= range_val <= scan_msg.range_max:
                    # Coordenadas del obstáculo
                    obs_x = robot_x + range_val * math.cos(angle)
                    obs_y = robot_y + range_val * math.sin(angle)
                    points.append((obs_x, obs_y))
            
            angle += scan_msg.angle_increment
        
        return points
    
    def find_costmap_obstacles(self, costmap, robot_pose):
        """Encuentra obstáculos en el costmap"""
        if not costmap or not robot_pose:
            return []
        
        robot_x, robot_y, grid_x, grid_y = robot_pose
        obstacles = []
        
        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        
        # Buscar en área alrededor del robot
        search_radius = int(3.0 / resolution)  # 3 metros de radio
        
        for dy in range(-search_radius, search_radius):
            for dx in range(-search_radius, search_radius):
                gx = grid_x + dx
                gy = grid_y + dy
                
                if 0 <= gx < width and 0 <= gy < height:
                    index = gy * width + gx
                    if index < len(costmap.data):
                        cost = costmap.data[index]
                        
                        if cost > self.obstacle_threshold:
                            # Convertir a coordenadas del mundo
                            world_x = origin_x + gx * resolution
                            world_y = origin_y + gy * resolution
                            obstacles.append((world_x, world_y, cost))
        
        return obstacles
    
    def find_phantom_obstacles(self):
        """Encuentra obstáculos fantasma comparando costmap con láser"""
        phantom_obstacles = []
        
        if not self.current_scan or not self.local_costmap:
            return phantom_obstacles
        
        # Verificar que los datos del láser sean recientes
        scan_time = self.current_scan.header.stamp
        current_time = self.get_clock().now()
        time_diff = (current_time - rclpy.time.Time.from_msg(scan_time)).nanoseconds / 1e9
        
        if time_diff > self.scan_timeout:
            return phantom_obstacles
        
        # Obtener pose del robot
        robot_pose = self.get_robot_pose_in_costmap(self.local_costmap)
        if not robot_pose:
            return phantom_obstacles
        
        # Obtener obstáculos detectados por el láser
        laser_obstacles = self.scan_to_cartesian(self.current_scan, robot_pose)
        
        # Obtener obstáculos en el costmap
        costmap_obstacles = self.find_costmap_obstacles(self.local_costmap, robot_pose)
        
        # Encontrar obstáculos en costmap que no están en el láser
        for costmap_obs in costmap_obstacles:
            cx, cy, cost = costmap_obs
            
            # Verificar si hay un obstáculo láser cerca
            is_phantom = True
            for laser_obs in laser_obstacles:
                lx, ly = laser_obs
                distance = math.sqrt((cx - lx)**2 + (cy - ly)**2)
                
                if distance < self.phantom_distance_threshold:
                    is_phantom = False
                    break
            
            if is_phantom:
                phantom_obstacles.append((cx, cy, cost))
        
        return phantom_obstacles
    
    def create_phantom_markers(self, phantom_obstacles):
        """Crea marcadores visuales para obstáculos fantasma"""
        markers = MarkerArray()
        
        for i, (x, y, cost) in enumerate(phantom_obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "phantom_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0
            
            # Color basado en el costo
            intensity = min(float(cost) / 100.0, 1.0)
            marker.color = ColorRGBA(
                r=1.0, 
                g=1.0 - intensity, 
                b=0.0, 
                a=0.8
            )
            
            marker.lifetime.sec = 5
            markers.markers.append(marker)
        
        return markers
    
    def create_diagnostics_marker(self, num_phantoms):
        """Crea marcador de texto con diagnósticos"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "diagnostics"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = -3.0
        marker.pose.position.y = -2.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.4
        
        if num_phantoms == 0:
            marker.text = "✅ Sin obstáculos fantasma"
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        elif num_phantoms < 3:
            marker.text = f"⚠️ {num_phantoms} obstáculos fantasma"
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        else:
            marker.text = f"❌ {num_phantoms} obstáculos fantasma - CRÍTICO"
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        marker.lifetime.sec = 5
        return marker
    
    def clear_costmaps_if_needed(self, num_phantoms):
        """Limpia costmaps si hay demasiados obstáculos fantasma"""
        if num_phantoms >= self.clear_phantom_threshold:
            self.get_logger().warn(f"🧹 Limpiando costmaps - {num_phantoms} obstáculos fantasma detectados")
            
            # Limpiar costmap local
            if self.clear_local_costmap.service_is_ready():
                req = ClearEntireCostmap.Request()
                future = self.clear_local_costmap.call_async(req)
            
            # Limpiar costmap global
            if self.clear_global_costmap.service_is_ready():
                req = ClearEntireCostmap.Request()
                future = self.clear_global_costmap.call_async(req)
                
            time.sleep(0.5)  # Pausa para aplicar limpieza
    
    def run_diagnostics(self):
        """Ejecuta diagnósticos principales"""
        # Encontrar obstáculos fantasma
        phantom_obstacles = self.find_phantom_obstacles()
        num_phantoms = len(phantom_obstacles)
        
        if num_phantoms > 0:
            self.get_logger().debug(f"🚨 Detectados {num_phantoms} obstáculos fantasma")
            
            # Crear marcadores visuales
            phantom_markers = self.create_phantom_markers(phantom_obstacles)
            self.phantom_markers_pub.publish(phantom_markers)
        
        # Crear marcador de diagnósticos
        diagnostics_marker = self.create_diagnostics_marker(num_phantoms)
        self.diagnostics_pub.publish(diagnostics_marker)
        
        # Limpiar costmaps si es necesario
        self.clear_costmaps_if_needed(num_phantoms)
        
        # Log periódico
        if num_phantoms > 0:
            self.get_logger().info(f"Diagnóstico: {num_phantoms} obstáculos fantasma detectados")


def main(args=None):
    rclpy.init(args=args)
    
    diagnostics = ObstacleDiagnostics()
    
    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        diagnostics.get_logger().info("Diagnósticos detenidos por usuario")
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Monitor avanzado de exploración que soluciona problemas de atascamiento
y mejora la cobertura de exploración.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import time
import math
from scipy import ndimage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2


class AdvancedExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')  # Cambiado de 'advanced_exploration_monitor'
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Estado del sistema
        self.current_map = None
        self.robot_pose = None
        self.last_scan = None
        self.exploration_history = []
        self.stuck_counter = 0
        self.last_map_growth = time.time()
        self.last_position = None
        self.position_stuck_time = 0
        self.explored_frontiers = set()
        self.last_position_time = time.time()  # Inicializar esta variable
        
        # Parámetros optimizados
        self.map_growth_timeout = 45.0      # 45s sin crecimiento = problema
        self.position_stuck_timeout = 30.0   # 30s en misma posición = atascado
        self.min_frontier_distance = 1.5    # Distancia mínima entre fronteras
        self.max_exploration_distance = 8.0  # Máxima distancia para explorar
        self.frontier_history_size = 50     # Recordar últimas 50 fronteras
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_reliable)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_reliable)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_reliable)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_reliable)
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', qos_reliable)
        
        self.emergency_stop_pub = self.create_publisher(
            Twist, '/cmd_vel', qos_reliable)
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Service clients
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        
        # Timer para monitoreo
        self.monitor_timer = self.create_timer(10.0, self.monitor_exploration)
        self.recovery_timer = self.create_timer(5.0, self.check_recovery_needed)
        
        self.get_logger().info("🚀 Monitor Avanzado de Exploración iniciado")
    
    def map_callback(self, msg):
        """Procesa actualizaciones del mapa y detecta crecimiento"""
        old_known_cells = 0
        if self.current_map is not None:
            old_known_cells = np.sum(np.array(self.current_map.data) >= 0)
        
        self.current_map = msg
        new_known_cells = np.sum(np.array(msg.data) >= 0)
        
        # Detectar crecimiento significativo del mapa
        if new_known_cells > old_known_cells + 20:  # Mínimo 20 celdas nuevas
            self.last_map_growth = time.time()
            self.get_logger().debug(f"Mapa creció: {new_known_cells - old_known_cells} celdas")
    
    def odom_callback(self, msg):
        """Monitorea la posición del robot para detectar atascamiento"""
        current_pos = msg.pose.pose.position
        current_time = time.time()
        
        if self.last_position is not None:
            distance = math.sqrt(
                (current_pos.x - self.last_position.x)**2 + 
                (current_pos.y - self.last_position.y)**2
            )
            
            if distance < 0.1:  # Robot casi estático
                self.position_stuck_time += (current_time - self.last_position_time)
            else:
                self.position_stuck_time = 0
        
        self.robot_pose = msg.pose.pose
        self.last_position = current_pos
        self.last_position_time = current_time
    
    def scan_callback(self, msg):
        """Almacena datos del láser para análisis"""
        self.last_scan = msg
    
    def cmd_vel_callback(self, msg):
        """Monitorea comandos de velocidad"""
        # Detectar si el robot está recibiendo comandos pero no se mueve
        has_command = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        
        if has_command and self.position_stuck_time > 15.0:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
    
    def find_frontiers(self):
        """Encuentra fronteras de exploración usando OpenCV"""
        if self.current_map is None or self.robot_pose is None:
            return []
        
        try:
            # Convertir mapa a array numpy
            width = self.current_map.info.width
            height = self.current_map.info.height
            resolution = self.current_map.info.resolution
            origin = self.current_map.info.origin
            
            map_array = np.array(self.current_map.data).reshape((height, width))
            
            # Crear máscaras
            free_space = (map_array == 0)
            unknown_space = (map_array == -1)
            
            # Encontrar fronteras: celdas libres adyacentes a desconocidas
            kernel = np.ones((3, 3), np.uint8)
            
            # Dilatar espacio libre para encontrar bordes
            free_dilated = cv2.dilate(free_space.astype(np.uint8), kernel, iterations=1)
            
            # Fronteras = intersección de espacio libre dilatado y espacio desconocido
            frontiers = free_dilated & unknown_space
            
            # Encontrar contornos de fronteras
            contours, _ = cv2.findContours(
                frontiers.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            frontier_points = []
            
            for contour in contours:
                # Filtrar fronteras muy pequeñas
                if cv2.contourArea(contour) < 5:
                    continue
                
                # Obtener centroide de la frontera
                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue
                    
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Convertir a coordenadas del mundo
                world_x = origin.position.x + cx * resolution
                world_y = origin.position.y + cy * resolution
                
                # Verificar distancia al robot
                robot_x = self.robot_pose.position.x
                robot_y = self.robot_pose.position.y
                distance = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)
                
                # Filtrar fronteras por distancia
                if self.min_frontier_distance < distance < self.max_exploration_distance:
                    # Verificar que no hayamos explorado esta área recientemente
                    frontier_key = (round(world_x, 1), round(world_y, 1))
                    if frontier_key not in self.explored_frontiers:
                        frontier_points.append({
                            'x': world_x,
                            'y': world_y,
                            'distance': distance,
                            'size': cv2.contourArea(contour)
                        })
            
            # Ordenar por tamaño y distancia (priorizando fronteras grandes y cercanas)
            frontier_points.sort(key=lambda f: f['size'] / (f['distance'] + 0.1), reverse=True)
            
            return frontier_points[:10]  # Devolver las 10 mejores fronteras
            
        except Exception as e:
            self.get_logger().error(f"Error en find_frontiers: {e}")
            return []
    
    def send_exploration_goal(self, frontier):
        """Envía un objetivo de exploración a una frontera"""
        try:
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Posición objetivo
            goal_msg.pose.position.x = frontier['x']
            goal_msg.pose.position.y = frontier['y']
            goal_msg.pose.position.z = 0.0
            
            # Orientación hacia la frontera
            if self.robot_pose:
                dx = frontier['x'] - self.robot_pose.position.x
                dy = frontier['y'] - self.robot_pose.position.y
                yaw = math.atan2(dy, dx)
                
                goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
                goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                goal_msg.pose.orientation.w = 1.0
            
            # Publicar objetivo
            self.goal_pub.publish(goal_msg)
            
            # Agregar a historia de fronteras exploradas
            frontier_key = (round(frontier['x'], 1), round(frontier['y'], 1))
            self.explored_frontiers.add(frontier_key)
            
            # Mantener tamaño de historia limitado
            if len(self.explored_frontiers) > self.frontier_history_size:
                # Remover fronteras más antiguas (simplificado)
                self.explored_frontiers = set(list(self.explored_frontiers)[-self.frontier_history_size//2:])
            
            self.get_logger().info(
                f"🎯 Enviado objetivo a frontera: x={frontier['x']:.2f}, y={frontier['y']:.2f}, "
                f"tamaño={frontier['size']:.1f}, distancia={frontier['distance']:.2f}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error enviando objetivo: {e}")
            return False
    
    def clear_costmaps(self):
        """Limpia los costmaps para resolver problemas de navegación"""
        try:
            # Limpiar costmap local
            if self.clear_local_costmap.service_is_ready():
                req = ClearEntireCostmap.Request()
                self.clear_local_costmap.call_async(req)
                self.get_logger().info("🧹 Costmap local limpiado")
            
            # Limpiar costmap global
            if self.clear_global_costmap.service_is_ready():
                req = ClearEntireCostmap.Request()
                self.clear_global_costmap.call_async(req)
                self.get_logger().info("🧹 Costmap global limpiado")
            
        except Exception as e:
            self.get_logger().error(f"Error limpiando costmaps: {e}")
    
    def perform_recovery_maneuver(self):
        """Ejecuta maniobras de recuperación cuando el robot está atascado"""
        self.get_logger().warn("🔄 Ejecutando maniobra de recuperación...")
        
        try:
            # 1. Parar el robot
            stop_cmd = Twist()
            self.emergency_stop_pub.publish(stop_cmd)
            time.sleep(1.0)
            
            # 2. Limpiar costmaps
            self.clear_costmaps()
            time.sleep(2.0)
            
            # 3. Giro exploratorio
            spin_cmd = Twist()
            spin_cmd.angular.z = 0.5
            
            # Girar por 4 segundos para actualizar sensores
            for _ in range(20):
                self.emergency_stop_pub.publish(spin_cmd)
                time.sleep(0.2)
            
            # 4. Parar después del giro
            self.emergency_stop_pub.publish(stop_cmd)
            time.sleep(1.0)
            
            # 5. Buscar nueva frontera después de actualizar mapa
            self.get_logger().info("✅ Maniobra de recuperación completada")
            
        except Exception as e:
            self.get_logger().error(f"Error en maniobra de recuperación: {e}")
    
    def execute_systematic_exploration(self):
        """Ejecuta exploración sistemática cuando no hay fronteras obvias"""
        if not self.robot_pose:
            return False
        
        self.get_logger().info("🗺️ Iniciando exploración sistemática...")
        
        try:
            # Patrones de exploración sistemática
            patterns = [
                # Patrón en espiral
                [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0),
                 (2.0, 0.0), (0.0, 2.0), (-2.0, 0.0), (0.0, -2.0)],
                
                # Patrón en cuadrícula
                [(2.0, 2.0), (2.0, -2.0), (-2.0, -2.0), (-2.0, 2.0),
                 (3.0, 0.0), (0.0, 3.0), (-3.0, 0.0), (0.0, -3.0)],
                
                # Patrón radial
                [(3.0 * math.cos(i * math.pi/4), 3.0 * math.sin(i * math.pi/4)) 
                 for i in range(8)]
            ]
            
            # Seleccionar patrón basado en tiempo
            pattern_idx = int(time.time() / 120) % len(patterns)  # Cambiar cada 2 minutos
            pattern = patterns[pattern_idx]
            
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y
            
            for offset_x, offset_y in pattern:
                target_x = robot_x + offset_x
                target_y = robot_y + offset_y
                
                # Verificar que el objetivo esté en área válida
                if self.is_valid_exploration_target(target_x, target_y):
                    fake_frontier = {
                        'x': target_x,
                        'y': target_y,
                        'distance': math.sqrt(offset_x**2 + offset_y**2),
                        'size': 10.0  # Tamaño artificial
                    }
                    
                    return self.send_exploration_goal(fake_frontier)
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Error en exploración sistemática: {e}")
            return False
    
    def is_valid_exploration_target(self, x, y):
        """Verifica si un objetivo de exploración es válido"""
        if not self.current_map or not self.robot_pose:
            return True  # Asumir válido si no hay información
        
        try:
            # Convertir coordenadas del mundo a índices del mapa
            resolution = self.current_map.info.resolution
            origin = self.current_map.info.origin
            width = self.current_map.info.width
            height = self.current_map.info.height
            
            map_x = int((x - origin.position.x) / resolution)
            map_y = int((y - origin.position.y) / resolution)
            
            # Verificar límites del mapa
            if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
                return False
            
            # Verificar que no sea un obstáculo conocido
            map_index = map_y * width + map_x
            if map_index < len(self.current_map.data):
                cell_value = self.current_map.data[map_index]
                return cell_value != 100  # No es obstáculo
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validando objetivo: {e}")
            return True
    
    def analyze_exploration_coverage(self):
        """Analiza la cobertura de exploración y sugiere acciones"""
        if not self.current_map:
            return {
                "total_cells": 0,
                "known_cells": 0,
                "free_cells": 0,
                "obstacle_cells": 0,
                "unknown_cells": 0,
                "coverage_ratio": 0.0,
                "status": "no_map", 
                "action": "wait"
            }
        
        try:
            map_data = np.array(self.current_map.data)
            total_cells = len(map_data)
            known_cells = np.sum(map_data >= 0)
            free_cells = np.sum(map_data == 0)
            obstacle_cells = np.sum(map_data == 100)
            unknown_cells = np.sum(map_data == -1)
            
            coverage_ratio = known_cells / total_cells if total_cells > 0 else 0.0
            
            analysis = {
                "total_cells": int(total_cells),
                "known_cells": int(known_cells),
                "free_cells": int(free_cells),
                "obstacle_cells": int(obstacle_cells),
                "unknown_cells": int(unknown_cells),
                "coverage_ratio": float(coverage_ratio),  # Asegurar que sea float
                "status": "exploring",
                "action": "continue"
            }
            
            # Determinar estado y acción
            if coverage_ratio > 0.85:
                analysis["status"] = "nearly_complete"
                analysis["action"] = "systematic_search"
            elif coverage_ratio > 0.95:
                analysis["status"] = "complete"
                analysis["action"] = "finished"
            elif unknown_cells < 100:
                analysis["status"] = "few_unknowns"
                analysis["action"] = "systematic_search"
            
            return analysis
            
        except Exception as e:
            self.get_logger().error(f"Error analizando cobertura: {e}")
            return {
                "total_cells": 0,
                "known_cells": 0,
                "free_cells": 0,
                "obstacle_cells": 0,
                "unknown_cells": 0,
                "coverage_ratio": 0.0,
                "status": "error", 
                "action": "wait"
            }
    
    def check_recovery_needed(self):
        """Verifica si se necesita recuperación inmediata"""
        current_time = time.time()
        
        # Casos que requieren recuperación inmediata
        needs_recovery = False
        recovery_reason = ""
        
        if self.stuck_counter > 6:
            needs_recovery = True
            recovery_reason = "robot_stuck_commands"
        
        elif self.position_stuck_time > self.position_stuck_timeout:
            needs_recovery = True
            recovery_reason = "position_stuck"
        
        elif (current_time - self.last_map_growth) > self.map_growth_timeout:
            needs_recovery = True
            recovery_reason = "no_map_growth"
        
        if needs_recovery:
            self.get_logger().warn(f"🚨 Recuperación necesaria: {recovery_reason}")
            self.perform_recovery_maneuver()
            
            # Reset contadores
            self.stuck_counter = 0
            self.position_stuck_time = 0
            self.last_map_growth = current_time
    
    def monitor_exploration(self):
        """Función principal de monitoreo de exploración"""
        try:
            current_time = time.time()
            
            # Analizar cobertura actual
            coverage_analysis = self.analyze_exploration_coverage()
            
            # Verificar que coverage_ratio existe antes de usar
            if 'coverage_ratio' in coverage_analysis:
                self.get_logger().info(
                    f"📊 Cobertura: {coverage_analysis['coverage_ratio']:.1%}, "
                    f"Estado: {coverage_analysis['status']}, "
                    f"Celdas conocidas: {coverage_analysis['known_cells']}")
            else:
                self.get_logger().info(
                    f"📊 Cobertura: No disponible, "
                    f"Estado: {coverage_analysis['status']}, "
                    f"Celdas conocidas: {coverage_analysis['known_cells']}")
            
            # Encontrar fronteras disponibles
            frontiers = self.find_frontiers()
            
            self.get_logger().debug(f"🔍 Encontradas {len(frontiers)} fronteras")
            
            # Decidir acción basada en análisis
            if coverage_analysis["action"] == "finished":
                self.get_logger().info("🎉 Exploración completada!")
                return
            
            elif coverage_analysis["action"] == "systematic_search" or len(frontiers) == 0:
                # No hay fronteras claras, usar exploración sistemática
                if not self.execute_systematic_exploration():
                    self.get_logger().warn("⚠️ No se pudo ejecutar exploración sistemática")
            
            elif len(frontiers) > 0:
                # Hay fronteras disponibles, ir a la mejor
                best_frontier = frontiers[0]
                self.send_exploration_goal(best_frontier)
            
            # Logging periódico de estadísticas
            if int(current_time) % 60 == 0:  # Cada minuto
                time_since_growth = current_time - self.last_map_growth
                self.get_logger().info(
                    f"📈 Estadísticas - Cobertura: {coverage_analysis.get('coverage_ratio', 0):.1%}, "
                    f"Fronteras: {len(frontiers)}, "
                    f"Último crecimiento: {time_since_growth:.1f}s, "
                    f"Posición estática: {self.position_stuck_time:.1f}s")
                    
        except Exception as e:
            self.get_logger().error(f"Error en monitor_exploration: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    monitor = AdvancedExplorationMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Monitor detenido por usuario")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
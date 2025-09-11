#!/usr/bin/env python3
"""
Enhanced Exploration Monitor - VERSIÓN INTEGRADA CON COORDINADOR
Se comunica con el RobotControlCoordinator para evitar conflictos
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/enhanced_exploration_monitor.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math
import json
from typing import Dict, Any, Optional
import cv2


class EnhancedExplorationMonitor(Node):
    def __init__(self):
        super().__init__('enhanced_exploration_monitor')
        
        # =====================================================================
        # CONFIGURACIÓN
        # =====================================================================
        self.declare_parameter('map_growth_timeout', 60.0)
        self.declare_parameter('position_stuck_timeout', 45.0)
        self.declare_parameter('min_frontier_distance', 1.0)
        self.declare_parameter('max_exploration_distance', 8.0)
        self.declare_parameter('monitoring_enabled', True)
        
        # Estado del monitor
        self.monitoring_active = self.get_parameter('monitoring_enabled').value
        self.current_map = None
        self.robot_pose = None
        self.last_scan = None
        self.last_map_growth = time.time()
        self.last_position = None
        self.position_stuck_time = 0
        self.explored_frontiers = set()
        self.last_position_time = time.time()
        
        # Estado del coordinador - COMUNICACIÓN BIDIRECCIONAL
        self.coordinator_state = {
            'exploration_active': False,
            'manual_override_active': False,
            'voice_control_active': False,
            'emergency_active': False,
            'current_state': 'UNKNOWN'
        }
        
        # =====================================================================
        # PUBLISHERS - Comunicación con coordinador
        # =====================================================================
        self.exploration_request_pub = self.create_publisher(
            String, '/exploration_request', 10)
        
        self.monitor_status_pub = self.create_publisher(
            String, '/monitor_status', 10)
        
        # =====================================================================
        # SUBSCRIBERS - Escuchar coordinador y sensores
        # =====================================================================
        # Estado del coordinador
        self.coordinator_status_sub = self.create_subscription(
            String, '/coordinator_status', self._coordinator_status_callback, 10)
        
        # Sensores
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)
        
        # Monitorear velocidad para detectar conflictos
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        
        # =====================================================================
        # TIMERS - Monitoreo inteligente
        # =====================================================================
        self.monitor_timer = self.create_timer(3.0, self._monitor_exploration)
        self.status_publish_timer = self.create_timer(5.0, self._publish_monitor_status)
        
        self.get_logger().info('🔍 Enhanced Exploration Monitor iniciado (integrado con coordinador)')
    
    # =========================================================================
    # CALLBACKS DE ESTADO - Coordinación con sistema central
    # =========================================================================
    
    def _coordinator_status_callback(self, msg: String):
        """Recibir estado del coordinador - SINCRONIZACIÓN CRÍTICA"""
        try:
            status_data = json.loads(msg.data)
            
            # Actualizar estado conocido del coordinador
            old_exploration_state = self.coordinator_state.get('exploration_active', False)
            
            self.coordinator_state.update({
                'exploration_active': 'EXPLORING_AUTO' in status_data.get('state', ''),
                'manual_override_active': status_data.get('manual_override_active', False),
                'voice_control_active': status_data.get('voice_control_active', False),
                'emergency_active': status_data.get('emergency_active', False),
                'current_state': status_data.get('state', 'UNKNOWN')
            })
            
            # Detectar cambio en estado de exploración
            new_exploration_state = self.coordinator_state['exploration_active']
            if old_exploration_state != new_exploration_state:
                if new_exploration_state:
                    self.get_logger().info('▶️ Monitor: Exploración activada por coordinador')
                    self.last_map_growth = time.time()  # Reset timers
                    self.position_stuck_time = 0
                else:
                    self.get_logger().info('⏸️ Monitor: Exploración pausada por coordinador')
            
            # Ajustar monitoreo basado en estado
            self.monitoring_active = (
                new_exploration_state and 
                not self.coordinator_state['emergency_active']
            )
            
        except Exception as e:
            self.get_logger().error(f'Error procesando estado coordinador: {e}')
    
    def _map_callback(self, msg: OccupancyGrid):
        """Procesar actualizaciones del mapa"""
        old_known_cells = 0
        if self.current_map is not None:
            old_known_cells = np.sum(np.array(self.current_map.data) >= 0)
        
        self.current_map = msg
        new_known_cells = np.sum(np.array(msg.data) >= 0)
        
        # Detectar crecimiento significativo
        if new_known_cells > old_known_cells + 20:
            self.last_map_growth = time.time()
            self.get_logger().debug(f'Mapa creció: {new_known_cells - old_known_cells} celdas')
    
    def _odom_callback(self, msg: Odometry):
        """Monitorear posición para detectar robot atascado"""
        current_pos = msg.pose.pose.position
        current_time = time.time()
        
        if self.last_position is not None:
            distance = math.sqrt(
                (current_pos.x - self.last_position.x)**2 + 
                (current_pos.y - self.last_position.y)**2
            )
            
            if distance < 0.05:  # Robot casi estático (umbral reducido)
                self.position_stuck_time += (current_time - self.last_position_time)
            else:
                self.position_stuck_time = 0
        
        self.robot_pose = msg.pose.pose
        self.last_position = current_pos
        self.last_position_time = current_time
    
    def _scan_callback(self, msg: LaserScan):
        """Almacenar datos de laser scan"""
        self.last_scan = msg
    
    def _cmd_vel_callback(self, msg: Twist):
        """Monitorear comandos de velocidad para detectar conflictos"""
        # Este callback nos permite detectar si hay movimiento pero el robot está atascado
        has_command = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        
        if has_command and self.position_stuck_time > 20.0:
            # Robot recibe comandos pero no se mueve - posible problema
            self._report_stuck_condition()
    
    # =========================================================================
    # LÓGICA DE MONITOREO - Inteligente y coordinada
    # =========================================================================
    
    def _monitor_exploration(self):
        """Función principal de monitoreo - COORDINA CON SISTEMA CENTRAL"""
        if not self.monitoring_active:
            return
        
        current_time = time.time()
        
        # Solo actuar si la exploración está activa según el coordinador
        if not self.coordinator_state.get('exploration_active', False):
            return
        
        # Verificar si hay override manual/voz activo
        if (self.coordinator_state.get('manual_override_active', False) or 
            self.coordinator_state.get('voice_control_active', False)):
            self.get_logger().debug('Monitor pausado - control manual/voz activo')
            return
        
        # Análisis de condiciones de exploración
        conditions = self._analyze_exploration_conditions()
        
        # Decidir acción basada en análisis
        action_needed = self._determine_action_needed(conditions)
        
        if action_needed:
            self._request_exploration_action(action_needed, conditions)
    
    def _analyze_exploration_conditions(self) -> Dict[str, Any]:
        """Analizar condiciones actuales de exploración"""
        current_time = time.time()
        
        conditions = {
            'timestamp': current_time,
            'map_growth_stalled': (current_time - self.last_map_growth) > self.get_parameter('map_growth_timeout').value,
            'robot_stuck': self.position_stuck_time > self.get_parameter('position_stuck_timeout').value,
            'frontiers_available': False,
            'coverage_complete': False,
            'needs_recovery': False
        }
        
        # Analizar fronteras disponibles
        if self.current_map and self.robot_pose:
            frontiers = self._find_frontiers()
            conditions['frontiers_available'] = len(frontiers) > 0
            conditions['frontier_count'] = len(frontiers)
            conditions['best_frontier'] = frontiers[0] if frontiers else None
        
        # Analizar cobertura del mapa
        if self.current_map:
            coverage_analysis = self._analyze_map_coverage()
            conditions['coverage_ratio'] = coverage_analysis.get('coverage_ratio', 0.0)
            conditions['coverage_complete'] = coverage_analysis.get('coverage_ratio', 0.0) > 0.90
        
        # Determinar si se necesita recuperación
        conditions['needs_recovery'] = (
            conditions['map_growth_stalled'] or 
            conditions['robot_stuck']
        )
        
        return conditions
    
    def _determine_action_needed(self, conditions: Dict[str, Any]) -> Optional[str]:
        """Determinar qué acción se necesita basada en las condiciones"""
        
        # Si exploración está completa
        if conditions.get('coverage_complete', False):
            return 'complete_exploration'
        
        # Si robot está atascado o sin progreso
        if conditions.get('needs_recovery', False):
            return 'request_recovery'
        
        # Si hay fronteras disponibles para explorar
        if conditions.get('frontiers_available', False):
            return 'explore_frontier'
        
        # Si no hay fronteras pero cobertura es baja, búsqueda sistemática
        if conditions.get('coverage_ratio', 0.0) < 0.80:
            return 'systematic_exploration'
        
        return None
    
    def _request_exploration_action(self, action: str, conditions: Dict[str, Any]):
        """Solicitar acción al coordinador - COMUNICACIÓN ESTRUCTURADA"""
        request_data = {
            'type': action,
            'conditions': conditions,
            'timestamp': time.time(),
            'source': 'exploration_monitor'
        }
        
        # Agregar datos específicos según el tipo de acción
        if action == 'explore_frontier' and conditions.get('best_frontier'):
            request_data['frontier'] = conditions['best_frontier']
        
        elif action == 'request_recovery':
            request_data['recovery_reason'] = []
            if conditions.get('map_growth_stalled'):
                request_data['recovery_reason'].append('map_growth_stalled')
            if conditions.get('robot_stuck'):
                request_data['recovery_reason'].append('robot_stuck')
        
        # Enviar solicitud al coordinador
        request_msg = String()
        request_msg.data = json.dumps(request_data)
        self.exploration_request_pub.publish(request_msg)
        
        self.get_logger().info(
            f'📝 Solicitada acción: {action} '
            f'(fronteras: {conditions.get("frontier_count", 0)}, '
            f'cobertura: {conditions.get("coverage_ratio", 0)*100:.1f}%)'
        )
    
    def _report_stuck_condition(self):
        """Reportar condición de robot atascado"""
        report_data = {
            'type': 'robot_stuck_detected',
            'stuck_time': self.position_stuck_time,
            'timestamp': time.time(),
            'position': {
                'x': self.robot_pose.position.x,
                'y': self.robot_pose.position.y
            } if self.robot_pose else None
        }
        
        request_msg = String()
        request_msg.data = json.dumps(report_data)
        self.exploration_request_pub.publish(request_msg)
        
        self.get_logger().warn(f'🚨 Robot atascado detectado ({self.position_stuck_time:.1f}s)')
    
    # =========================================================================
    # ANÁLISIS DE FRONTERAS - Mejorado y eficiente
    # =========================================================================
    
    def _find_frontiers(self):
        """Encontrar fronteras de exploración disponibles"""
        if not self.current_map or not self.robot_pose:
            return []
        
        try:
            width = self.current_map.info.width
            height = self.current_map.info.height
            resolution = self.current_map.info.resolution
            origin = self.current_map.info.origin
            
            map_array = np.array(self.current_map.data).reshape((height, width))
            
            # Crear máscaras
            free_space = (map_array == 0)
            unknown_space = (map_array == -1)
            
            # Encontrar fronteras usando operaciones morfológicas
            kernel = np.ones((3, 3), np.uint8)
            free_dilated = cv2.dilate(free_space.astype(np.uint8), kernel, iterations=1)
            frontiers = free_dilated & unknown_space
            
            # Encontrar contornos
            contours, _ = cv2.findContours(
                frontiers.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            frontier_points = []
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y
            
            for contour in contours:
                if cv2.contourArea(contour) < 3:  # Filtrar fronteras muy pequeñas
                    continue
                
                # Calcular centroide
                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue
                    
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Convertir a coordenadas del mundo
                world_x = origin.position.x + cx * resolution
                world_y = origin.position.y + cy * resolution
                
                # Calcular distancia al robot
                distance = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)
                
                # Filtrar por distancia
                min_dist = self.get_parameter('min_frontier_distance').value
                max_dist = self.get_parameter('max_exploration_distance').value
                
                if min_dist < distance < max_dist:
                    # Verificar que no se haya explorado recientemente
                    frontier_key = (round(world_x, 0.5), round(world_y, 0.5))
                    if frontier_key not in self.explored_frontiers:
                        frontier_points.append({
                            'x': world_x,
                            'y': world_y,
                            'distance': distance,
                            'size': cv2.contourArea(contour),
                            'priority': cv2.contourArea(contour) / (distance + 0.1)
                        })
                        
                        # Agregar a historia (con límite)
                        self.explored_frontiers.add(frontier_key)
                        if len(self.explored_frontiers) > 30:
                            # Limpiar las más antigas
                            old_frontiers = list(self.explored_frontiers)[:10]
                            for old_frontier in old_frontiers:
                                self.explored_frontiers.discard(old_frontier)
            
            # Ordenar por prioridad (tamaño/distancia)
            frontier_points.sort(key=lambda f: f['priority'], reverse=True)
            
            return frontier_points[:5]  # Devolver las 5 mejores
            
        except Exception as e:
            self.get_logger().error(f'Error encontrando fronteras: {e}')
            return []
    
    def _analyze_map_coverage(self) -> Dict[str, Any]:
        """Analizar cobertura del mapa actual"""
        if not self.current_map:
            return {'coverage_ratio': 0.0, 'status': 'no_map'}
        
        try:
            map_data = np.array(self.current_map.data)
            total_cells = len(map_data)
            known_cells = np.sum(map_data >= 0)
            free_cells = np.sum(map_data == 0)
            obstacle_cells = np.sum(map_data == 100)
            unknown_cells = np.sum(map_data == -1)
            
            coverage_ratio = known_cells / total_cells if total_cells > 0 else 0.0
            
            return {
                'total_cells': total_cells,
                'known_cells': known_cells,
                'free_cells': free_cells,
                'obstacle_cells': obstacle_cells,
                'unknown_cells': unknown_cells,
                'coverage_ratio': coverage_ratio,
                'status': 'complete' if coverage_ratio > 0.90 else 'exploring'
            }
            
        except Exception as e:
            self.get_logger().error(f'Error analizando cobertura: {e}')
            return {'coverage_ratio': 0.0, 'status': 'error'}
    
    def _publish_monitor_status(self):
        """Publicar estado del monitor periódicamente"""
        try:
            status = {
                'monitoring_active': self.monitoring_active,
                'coordinator_state': self.coordinator_state,
                'last_map_growth': self.last_map_growth,
                'position_stuck_time': self.position_stuck_time,
                'frontiers_count': len(self._find_frontiers()) if self.current_map and self.robot_pose else 0,
                'timestamp': time.time()
            }
            
            # Agregar análisis de cobertura si hay mapa
            if self.current_map:
                coverage = self._analyze_map_coverage()
                status['coverage_ratio'] = coverage.get('coverage_ratio', 0.0)
                status['coverage_status'] = coverage.get('status', 'unknown')
            
            status_msg = String()
            status_msg.data = json.dumps(status)
            self.monitor_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publicando estado monitor: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = EnhancedExplorationMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print('Cerrando Enhanced Exploration Monitor...')
    except Exception as e:
        print(f'Error en Enhanced Exploration Monitor: {e}')
    finally:
        try:
            monitor.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Nodo para visualizar el estado de la exploración en RViz con marcadores
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA, Header
import numpy as np
import time
import math


class ExplorationVisualizer(Node):
    def __init__(self):
        super().__init__('exploration_visualizer')
        
        # Publishers para marcadores
        self.status_markers_pub = self.create_publisher(
            MarkerArray, 
            '/exploration_status_markers', 
            10
        )
        
        self.goals_markers_pub = self.create_publisher(
            MarkerArray, 
            '/exploration_goals_markers', 
            10
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Estado
        self.exploration_goals_history = []
        self.map_size_history = []
        self.last_map_update = time.time()
        self.exploration_start_time = time.time()
        
        # Timer para actualizar visualización
        self.viz_timer = self.create_timer(2.0, self.update_visualization)
        
        self.get_logger().info("Visualizador de exploración iniciado")
    
    def map_callback(self, msg):
        """Procesa actualizaciones del mapa"""
        try:
            current_time = time.time()
            known_cells = sum(1 for cell in msg.data if cell >= 0)
            
            self.map_size_history.append({
                'time': current_time,
                'size': known_cells,
                'resolution': msg.info.resolution
            })
            
            # Mantener solo los últimos 50 puntos
            if len(self.map_size_history) > 50:
                self.map_size_history.pop(0)
            
            self.last_map_update = current_time
            
        except Exception as e:
            self.get_logger().error(f"Error en map_callback: {e}")
    
    def goal_callback(self, msg):
        """Registra objetivos de exploración"""
        try:
            goal_info = {
                'time': time.time(),
                'position': [float(msg.pose.position.x), float(msg.pose.position.y)],  # Asegurar que sean float
                'frame': msg.header.frame_id
            }
            
            self.exploration_goals_history.append(goal_info)
            
            # Mantener solo los últimos 20 objetivos
            if len(self.exploration_goals_history) > 20:
                self.exploration_goals_history.pop(0)
                
        except Exception as e:
            self.get_logger().error(f"Error en goal_callback: {e}")
    
    def create_status_text_marker(self, marker_id, text, position, color):
        """Crea un marcador de texto para mostrar estado"""
        try:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "status"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            # Asegurar que las posiciones sean float
            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = float(position[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.3  # Tamaño del texto
            marker.color = color
            marker.text = str(text)  # Asegurar que sea string
            marker.lifetime.sec = 5  # Duración
            
            return marker
            
        except Exception as e:
            self.get_logger().error(f"Error creando marcador de texto: {e}")
            # Devolver un marcador básico en caso de error
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "status"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.3
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker.text = "Error"
            marker.lifetime.sec = 5
            return marker
    
    def create_goal_history_markers(self):
        """Crea marcadores para mostrar historial de objetivos"""
        markers = []
        current_time = time.time()
        
        try:
            for i, goal in enumerate(self.exploration_goals_history):
                # Edad del objetivo en segundos
                age = current_time - goal['time']
                if age > 60:  # Solo mostrar objetivos de los últimos 60 segundos
                    continue
                
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "exploration_goals"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # Asegurar que las posiciones sean float
                marker.pose.position.x = float(goal['position'][0])
                marker.pose.position.y = float(goal['position'][1])
                marker.pose.position.z = 0.1
                marker.pose.orientation.w = 1.0
                
                # Tamaño basado en la edad (más pequeño = más viejo)
                size = max(0.1, 0.3 - (age / 60.0) * 0.2)
                marker.scale.x = float(size)
                marker.scale.y = float(size)
                marker.scale.z = float(size)
                
                # Color basado en la edad (verde reciente → amarillo → rojo viejo)
                if age < 10:  # Verde
                    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                elif age < 30:  # Amarillo
                    marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)
                else:  # Rojo
                    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.4)
                
                marker.lifetime.sec = 65
                markers.append(marker)
            
        except Exception as e:
            self.get_logger().error(f"Error creando marcadores de objetivos: {e}")
        
        return markers
    
    def calculate_exploration_stats(self):
        """Calcula estadísticas de exploración"""
        try:
            current_time = time.time()
            exploration_duration = current_time - self.exploration_start_time
            
            stats = {
                'duration': float(exploration_duration),
                'goals_sent': int(len(self.exploration_goals_history)),
                'map_growth_rate': 0.0,
                'current_map_size': 0,
                'time_since_map_update': float(current_time - self.last_map_update)
            }
            
            if len(self.map_size_history) >= 2:
                recent_map = self.map_size_history[-1]
                old_map = self.map_size_history[0]
                
                stats['current_map_size'] = int(recent_map['size'])
                time_diff = recent_map['time'] - old_map['time']
                
                if time_diff > 0:
                    size_diff = recent_map['size'] - old_map['size']
                    stats['map_growth_rate'] = float(size_diff / time_diff)  # células/segundo
            
            return stats
            
        except Exception as e:
            self.get_logger().error(f"Error calculando estadísticas: {e}")
            return {
                'duration': 0.0,
                'goals_sent': 0,
                'map_growth_rate': 0.0,
                'current_map_size': 0,
                'time_since_map_update': 0.0
            }
    
    def update_visualization(self):
        """Actualiza todos los marcadores de visualización"""
        try:
            current_time = time.time()
            stats = self.calculate_exploration_stats()
            
            # Array para marcadores de estado
            status_markers = MarkerArray()
            
            # Marcador de duración de exploración
            duration_text = f"Exploración: {stats['duration']:.0f}s"
            status_markers.markers.append(
                self.create_status_text_marker(
                    0, duration_text, [-2.0, 3.0, 0.5],
                    ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # Cyan
                )
            )
            
            # Marcador de tamaño del mapa
            map_text = f"Mapa: {stats['current_map_size']} celdas"
            status_markers.markers.append(
                self.create_status_text_marker(
                    1, map_text, [-2.0, 2.5, 0.5],
                    ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Verde
                )
            )
            
            # Marcador de tasa de crecimiento
            growth_text = f"Crecimiento: {stats['map_growth_rate']:.1f} cel/s"
            color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Verde por defecto
            
            if stats['map_growth_rate'] < 1:
                color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Amarillo - lento
            if stats['map_growth_rate'] < 0.5:
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Rojo - muy lento
            
            status_markers.markers.append(
                self.create_status_text_marker(
                    2, growth_text, [-2.0, 2.0, 0.5], color
                )
            )
            
            # Marcador de objetivos enviados
            goals_text = f"Objetivos: {stats['goals_sent']}"
            status_markers.markers.append(
                self.create_status_text_marker(
                    3, goals_text, [-2.0, 1.5, 0.5],
                    ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Naranja
                )
            )
            
            # Advertencia si no hay actualizaciones del mapa
            if stats['time_since_map_update'] > 30:
                warning_text = "⚠️ Sin actualización del mapa!"
                status_markers.markers.append(
                    self.create_status_text_marker(
                        4, warning_text, [-2.0, 1.0, 0.5],
                        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Rojo
                    )
                )
            
            # Estado de exploración
            if stats['map_growth_rate'] > 1 and stats['time_since_map_update'] < 10:
                status_text = "✅ Explorando activamente"
                status_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif stats['map_growth_rate'] > 0.1:
                status_text = "⚠️ Exploración lenta"
                status_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            else:
                status_text = "❌ Exploración detenida"
                status_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            
            status_markers.markers.append(
                self.create_status_text_marker(
                    5, status_text, [-2.0, 0.5, 0.5], status_color
                )
            )
            
            # Publicar marcadores de estado
            self.status_markers_pub.publish(status_markers)
            
            # Array para marcadores de objetivos
            goals_markers = MarkerArray()
            goals_markers.markers = self.create_goal_history_markers()
            self.goals_markers_pub.publish(goals_markers)
            
            # Log periódico
            if int(current_time) % 30 == 0:  # Cada 30 segundos
                self.get_logger().info(
                    f"Estadísticas - Duración: {stats['duration']:.0f}s, "
                    f"Mapa: {stats['current_map_size']} celdas, "
                    f"Crecimiento: {stats['map_growth_rate']:.1f} cel/s, "
                    f"Objetivos: {stats['goals_sent']}"
                )
                
        except Exception as e:
            self.get_logger().error(f"Error en update_visualization: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    exploration_visualizer = ExplorationVisualizer()
    
    try:
        rclpy.spin(exploration_visualizer)
    except KeyboardInterrupt:
        exploration_visualizer.get_logger().info("Visualizador detenido por usuario")
    finally:
        exploration_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
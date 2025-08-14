#!/usr/bin/env python3
"""
Nodo para reiniciar automáticamente la exploración cuando explore_lite se detiene
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import subprocess
import psutil
import time
import math
import random


class ExplorationRestarter(Node):
    def __init__(self):
        super().__init__('exploration_restarter')
        
        # Cliente de acción para navegación
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher para objetivos de exploración
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Variables de estado
        self.last_activity_time = time.time()
        self.check_interval = 30.0  # Verificar cada 30 segundos
        self.restart_timeout = 120.0  # Reiniciar después de 2 minutos sin actividad
        
        # Timer para verificación periódica
        self.check_timer = self.create_timer(self.check_interval, self.check_exploration_status)
        
        self.get_logger().info("Reiniciador de exploración iniciado")
    
    def is_explore_node_running(self):
        """Verifica si el nodo explore está ejecutándose"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            return '/explore' in result.stdout
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return False
    
    def is_navigation_active(self):
        """Verifica si hay navegación activa"""
        try:
            # Verificar si hay un goal activo
            if self.nav_action_client.server_is_ready():
                return True
            return False
        except Exception:
            return False
    
    def send_exploration_goal(self, x=None, y=None, yaw=None):
        """Envía un objetivo de exploración"""
        goal = PoseStamped()
        goal.header = Header()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Si no se especifican coordenadas, usar posiciones aleatorias
        if x is None or y is None:
            # Generar posición aleatoria en un radio de 2-4 metros
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(2.0, 4.0)
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
        
        if yaw is None:
            yaw = random.uniform(-math.pi, math.pi)
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # Convertir yaw a quaternion
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Enviado objetivo de exploración: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
    
    def restart_explore_node(self):
        """Intenta reiniciar el nodo explore"""
        self.get_logger().warn("Intentando reiniciar el nodo explore...")
        
        try:
            # Matar procesos explore existentes
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if 'explore' in ' '.join(proc.info['cmdline']).lower():
                        proc.kill()
                        self.get_logger().info(f"Terminado proceso explore: {proc.info['pid']}")
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            
            time.sleep(2.0)  # Esperar que termine
            
            # Reiniciar explore_lite
            subprocess.Popen([
                'ros2', 'launch', 'explore_lite', 'explore.launch.py',
                'use_sim_time:=true'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            self.get_logger().info("Nodo explore reiniciado")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error al reiniciar explore: {e}")
            return False
    
    def force_exploration_movement(self):
        """Fuerza el movimiento del robot enviando múltiples objetivos"""
        self.get_logger().info("🚀 Forzando movimiento de exploración...")
        
        # Enviar varios objetivos en diferentes direcciones
        exploration_points = [
            (2.0, 0.0, 0.0),      # Adelante
            (0.0, 2.0, math.pi/2), # Izquierda
            (-2.0, 0.0, math.pi),  # Atrás
            (0.0, -2.0, -math.pi/2), # Derecha
            (1.5, 1.5, math.pi/4), # Diagonal
            (-1.5, 1.5, 3*math.pi/4), # Diagonal
        ]
        
        for i, (x, y, yaw) in enumerate(exploration_points):
            time.sleep(1.0)  # Pequeño delay entre objetivos
            self.send_exploration_goal(x, y, yaw)
            if i >= 2:  # Solo enviar los primeros 3 objetivos
                break
    
    def check_exploration_status(self):
        """Verifica el estado de la exploración y reinicia si es necesario"""
        current_time = time.time()
        time_since_activity = current_time - self.last_activity_time
        
        explore_running = self.is_explore_node_running()
        nav_active = self.is_navigation_active()
        
        self.get_logger().debug(f"Estado - Explore: {explore_running}, Nav: {nav_active}, "
                               f"Tiempo inactivo: {time_since_activity:.1f}s")
        
        # Si explore no está corriendo, reiniciarlo
        if not explore_running:
            self.get_logger().warn("❌ Nodo explore no está ejecutándose")
            if self.restart_explore_node():
                self.last_activity_time = current_time
                # Enviar objetivo inicial después de reiniciar
                time.sleep(5.0)  # Esperar que el nodo se inicialice
                self.send_exploration_goal()
        
        # Si no hay actividad por mucho tiempo, forzar movimiento
        elif time_since_activity > self.restart_timeout:
            self.get_logger().warn(f"⚠️  Sin actividad por {time_since_activity:.1f}s")
            self.force_exploration_movement()
            self.last_activity_time = current_time
        
        # Si hay navegación activa, actualizar tiempo de actividad
        elif nav_active:
            self.last_activity_time = current_time
            self.get_logger().debug("✅ Navegación activa detectada")
    
    def trigger_emergency_exploration(self):
        """Activa exploración de emergencia con múltiples estrategias"""
        self.get_logger().warn("🆘 ACTIVANDO EXPLORACIÓN DE EMERGENCIA")
        
        strategies = [
            "random_goals",
            "systematic_sweep",
            "frontier_search"
        ]
        
        for strategy in strategies:
            self.get_logger().info(f"Probando estrategia: {strategy}")
            
            if strategy == "random_goals":
                # Enviar múltiples objetivos aleatorios
                for _ in range(5):
                    self.send_exploration_goal()
                    time.sleep(2.0)
            
            elif strategy == "systematic_sweep":
                # Barrido sistemático en patrón de cuadrícula
                positions = [
                    (2.0, 2.0), (2.0, -2.0), (-2.0, -2.0), (-2.0, 2.0),
                    (3.0, 0.0), (0.0, 3.0), (-3.0, 0.0), (0.0, -3.0)
                ]
                for x, y in positions:
                    self.send_exploration_goal(x, y)
                    time.sleep(1.5)
            
            elif strategy == "frontier_search":
                # Buscar fronteras manualmente
                self.force_exploration_movement()
            
            # Verificar si la estrategia funcionó
            time.sleep(10.0)
            if self.is_navigation_active():
                self.get_logger().info(f"✅ Estrategia {strategy} exitosa")
                break
        
        self.last_activity_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    
    exploration_restarter = ExplorationRestarter()
    
    try:
        rclpy.spin(exploration_restarter)
    except KeyboardInterrupt:
        exploration_restarter.get_logger().info("Reiniciador detenido por usuario")
    finally:
        exploration_restarter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
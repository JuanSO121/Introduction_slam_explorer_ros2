#!/usr/bin/env python3
"""
Nodo especializado para limpiar costmaps y resolver problemas
de inflación excesiva que causan falsas detecciones de obstáculos.
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import numpy as np
import time
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CostmapCleaner(Node):
    def __init__(self):
        super().__init__('costmap_cleaner')
        
        # Parámetros configurables
        self.declare_parameter('cleaning_interval', 30.0)
        self.declare_parameter('inflation_recovery_enabled', True)
        self.declare_parameter('max_inflation_ratio', 0.15)  # Máximo 15% de celdas infladas
        self.declare_parameter('stuck_detection_time', 20.0)
        
        self.cleaning_interval = self.get_parameter('cleaning_interval').value
        self.inflation_recovery = self.get_parameter('inflation_recovery_enabled').value
        self.max_inflation_ratio = self.get_parameter('max_inflation_ratio').value
        self.stuck_detection_time = self.get_parameter('stuck_detection_time').value
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Estado del sistema
        self.last_cleaning_time = 0
        self.local_costmap = None
        self.global_costmap = None
        self.robot_velocity = Twist()
        self.stuck_start_time = None
        self.cleaning_in_progress = False
        
        # Subscribers para monitorear costmaps
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.local_costmap_callback,
            qos_reliable
        )
        
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.global_costmap_callback,
            qos_reliable
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_reliable
        )
        
        # Service clients para limpiar costmaps
        self.clear_local_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        
        self.clear_global_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        
        # Service client para reiniciar navegación si es necesario
        self.nav_lifecycle_client = self.create_client(
            Empty,
            '/controller_server/transition_event'
        )
        
        # Timer principal
        self.cleaning_timer = self.create_timer(5.0, self.check_costmap_health)
        
        # Timer para limpieza periódica
        self.periodic_timer = self.create_timer(
            self.cleaning_interval, 
            self.periodic_cleaning
        )
        
        self.get_logger().info(f"🧹 Costmap Cleaner iniciado - Intervalo: {self.cleaning_interval}s")
    
    def local_costmap_callback(self, msg):
        """Analiza el costmap local para detectar problemas"""
        self.local_costmap = msg
    
    def global_costmap_callback(self, msg):
        """Analiza el costmap global para detectar problemas"""
        self.global_costmap = msg
    
    def cmd_vel_callback(self, msg):
        """Monitorea velocidades del robot"""
        self.robot_velocity = msg
        
        # Detectar si el robot está atascado
        is_moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        
        if not is_moving and self.stuck_start_time is None:
            self.stuck_start_time = time.time()
        elif is_moving:
            self.stuck_start_time = None
    
    def analyze_costmap_inflation(self, costmap_msg):
        """Analiza el nivel de inflación en un costmap"""
        if not costmap_msg or not costmap_msg.data:
            return {"valid": False}
        
        data = np.array(costmap_msg.data)
        total_cells = len(data)
        
        # Clasificar celdas
        unknown_cells = np.sum(data == -1)
        free_cells = np.sum(data == 0)
        obstacle_cells = np.sum(data == 100)
        inflated_cells = np.sum((data > 0) & (data < 100))
        
        # Calcular ratios
        known_cells = total_cells - unknown_cells
        if known_cells == 0:
            return {"valid": False}
        
        inflation_ratio = inflated_cells / known_cells
        obstacle_ratio = obstacle_cells / known_cells
        
        analysis = {
            "valid": True,
            "total_cells": total_cells,
            "unknown_cells": unknown_cells,
            "free_cells": free_cells,
            "obstacle_cells": obstacle_cells,
            "inflated_cells": inflated_cells,
            "inflation_ratio": inflation_ratio,
            "obstacle_ratio": obstacle_ratio,
            "over_inflated": inflation_ratio > self.max_inflation_ratio
        }
        
        return analysis
    
    def clear_costmap_async(self, client, costmap_name):
        """Limpia un costmap de forma asíncrona"""
        if not client.service_is_ready():
            self.get_logger().warn(f"Servicio de limpieza para {costmap_name} no disponible")
            return False
        
        try:
            request = ClearEntireCostmap.Request()
            future = client.call_async(request)
            
            # Usar un hilo para no bloquear
            def handle_response():
                try:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                    if future.result() is not None:
                        self.get_logger().info(f"✅ {costmap_name} limpiado exitosamente")
                    else:
                        self.get_logger().warn(f"⚠️ Timeout limpiando {costmap_name}")
                except Exception as e:
                    self.get_logger().error(f"❌ Error limpiando {costmap_name}: {e}")
            
            thread = threading.Thread(target=handle_response)
            thread.daemon = True
            thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error al limpiar {costmap_name}: {e}")
            return False
    
    def perform_intelligent_cleaning(self, local_analysis, global_analysis):
        """Realiza limpieza inteligente basada en análisis de costmaps"""
        if self.cleaning_in_progress:
            return
        
        self.cleaning_in_progress = True
        cleaning_actions = []
        
        try:
            # Análisis del costmap local
            if local_analysis["valid"] and local_analysis["over_inflated"]:
                self.get_logger().warn(
                    f"🔴 Costmap local sobre-inflado: {local_analysis['inflation_ratio']:.1%}")
                cleaning_actions.append("local")
            
            # Análisis del costmap global
            if global_analysis["valid"] and global_analysis["over_inflated"]:
                self.get_logger().warn(
                    f"🔴 Costmap global sobre-inflado: {global_analysis['inflation_ratio']:.1%}")
                cleaning_actions.append("global")
            
            # Ejecutar limpiezas necesarias
            if "local" in cleaning_actions:
                self.clear_costmap_async(self.clear_local_client, "costmap local")
                time.sleep(1.0)  # Pequeña pausa entre limpiezas
            
            if "global" in cleaning_actions:
                self.clear_costmap_async(self.clear_global_client, "costmap global")
                time.sleep(1.0)
            
            if cleaning_actions:
                self.get_logger().info(f"🧹 Limpieza inteligente completada: {cleaning_actions}")
                self.last_cleaning_time = time.time()
            
        finally:
            # Pausa para permitir que los costmaps se regeneren
            threading.Timer(3.0, lambda: setattr(self, 'cleaning_in_progress', False)).start()
    
    def check_stuck_robot(self):
        """Verifica si el robot está atascado y necesita limpieza"""
        if self.stuck_start_time is None:
            return False
        
        stuck_duration = time.time() - self.stuck_start_time
        
        if stuck_duration > self.stuck_detection_time:
            self.get_logger().warn(f"🤖 Robot atascado por {stuck_duration:.1f}s - Limpiando costmaps")
            
            # Limpieza forzada cuando el robot está atascado
            self.clear_costmap_async(self.clear_local_client, "costmap local (robot atascado)")
            time.sleep(2.0)
            self.clear_costmap_async(self.clear_global_client, "costmap global (robot atascado)")
            
            # Reset del timer de atascamiento
            self.stuck_start_time = None
            self.last_cleaning_time = time.time()
            
            return True
        
        return False
    
    def check_costmap_health(self):
        """Función principal que verifica la salud de los costmaps"""
        current_time = time.time()
        
        # Verificar si el robot está atascado
        if self.check_stuck_robot():
            return
        
        # Verificar tiempo desde la última limpieza
        time_since_cleaning = current_time - self.last_cleaning_time
        
        # Análisis de costmaps solo si está habilitado
        if self.inflation_recovery and time_since_cleaning > 10.0:  # Mínimo 10s entre análisis
            local_analysis = {"valid": False}
            global_analysis = {"valid": False}
            
            if self.local_costmap:
                local_analysis = self.analyze_costmap_inflation(self.local_costmap)
            
            if self.global_costmap:
                global_analysis = self.analyze_costmap_inflation(self.global_costmap)
            
            # Realizar limpieza inteligente si es necesario
            if (local_analysis.get("over_inflated", False) or 
                global_analysis.get("over_inflated", False)):
                self.perform_intelligent_cleaning(local_analysis, global_analysis)
            
            # Log periódico de estadísticas
            if int(current_time) % 60 == 0:  # Cada minuto
                if local_analysis["valid"]:
                    self.get_logger().info(
                        f"📊 Local: {local_analysis['inflated_cells']} infladas "
                        f"({local_analysis['inflation_ratio']:.1%})")
                
                if global_analysis["valid"]:
                    self.get_logger().info(
                        f"📊 Global: {global_analysis['inflated_cells']} infladas "
                        f"({global_analysis['inflation_ratio']:.1%})")
    
    def periodic_cleaning(self):
        """Limpieza periódica preventiva"""
        current_time = time.time()
        
        if self.cleaning_in_progress:
            self.get_logger().debug("Limpieza periódica saltada - limpieza en progreso")
            return
        
        self.get_logger().info("🕒 Ejecutando limpieza periódica preventiva")
        
        # Limpieza suave - solo local costmap en limpieza periódica
        success = self.clear_costmap_async(self.clear_local_client, "costmap local (periódica)")
        
        if success:
            self.last_cleaning_time = current_time
            
            # Ocasionalmente limpiar también el global (cada 3 limpiezas periódicas)
            if int(current_time / self.cleaning_interval) % 3 == 0:
                time.sleep(2.0)  # Pausa entre limpiezas
                self.clear_costmap_async(self.clear_global_client, "costmap global (periódica)")
                self.get_logger().info("🧹 Limpieza periódica extendida (local + global)")
    
    def emergency_reset(self):
        """Reset de emergencia de todos los costmaps"""
        self.get_logger().warn("🚨 RESET DE EMERGENCIA DE COSTMAPS")
        
        # Limpiar ambos costmaps simultáneamente
        local_success = self.clear_costmap_async(self.clear_local_client, "costmap local (emergencia)")
        time.sleep(1.0)
        global_success = self.clear_costmap_async(self.clear_global_client, "costmap global (emergencia)")
        
        if local_success or global_success:
            self.last_cleaning_time = time.time()
            self.get_logger().info("✅ Reset de emergencia completado")
        else:
            self.get_logger().error("❌ Falló el reset de emergencia")
    
    def get_cleaning_statistics(self):
        """Obtiene estadísticas de limpieza para diagnóstico"""
        current_time = time.time()
        
        stats = {
            "last_cleaning_ago": current_time - self.last_cleaning_time,
            "cleaning_interval": self.cleaning_interval,
            "inflation_recovery_enabled": self.inflation_recovery,
            "max_inflation_ratio": self.max_inflation_ratio,
            "stuck_detection_time": self.stuck_detection_time,
            "cleaning_in_progress": self.cleaning_in_progress,
            "robot_stuck": self.stuck_start_time is not None
        }
        
        return stats


def main(args=None):
    rclpy.init(args=args)
    
    costmap_cleaner = CostmapCleaner()
    
    try:
        rclpy.spin(costmap_cleaner)
    except KeyboardInterrupt:
        costmap_cleaner.get_logger().info("Costmap Cleaner detenido por usuario")
    finally:
        costmap_cleaner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
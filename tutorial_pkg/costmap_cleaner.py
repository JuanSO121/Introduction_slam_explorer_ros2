#!/usr/bin/env python3
"""
Nodo limpiador de costmaps de emergencia
Se activa automáticamente cuando detecta problemas
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from nav2_msgs.srv import ClearEntireCostmap
import time
import numpy as np

class EmergencyCostmapCleaner(Node):
    def __init__(self):
        super().__init__('emergency_costmap_cleaner')
        
        # Service clients
        self.clear_local_client = self.create_client(
            ClearEntireCostmap, 
            '/local_costmap/clear_entirely_local_costmap'
        )
        self.clear_global_client = self.create_client(
            ClearEntireCostmap, 
            '/global_costmap/clear_entirely_global_costmap'
        )
        
        # Subscribers para monitorear costmaps
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, 
            '/local_costmap/costmap', 
            self.local_costmap_callback, 
            10
        )
        
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, 
            '/global_costmap/costmap', 
            self.global_costmap_callback, 
            10
        )
        
        # Estado
        self.last_local_costmap = None
        self.last_global_costmap = None
        self.last_cleaning_time = time.time()
        self.cleaning_interval = 30.0  # Limpiar cada 30 segundos como máximo
        self.obstacle_threshold = 0.8  # 80% de obstáculos = problema
        
        # Timer para limpieza periódica
        self.cleanup_timer = self.create_timer(20.0, self.periodic_cleanup)
        
        self.get_logger().info("🧹 Limpiador de costmaps de emergencia iniciado")

    def local_costmap_callback(self, msg):
        """Analiza costmap local para detectar problemas"""
        self.last_local_costmap = msg
        self.analyze_costmap(msg, "LOCAL")

    def global_costmap_callback(self, msg):
        """Analiza costmap global para detectar problemas"""
        self.last_global_costmap = msg
        self.analyze_costmap(msg, "GLOBAL")

    def analyze_costmap(self, costmap_msg, costmap_type):
        """Analiza si el costmap tiene demasiados obstáculos"""
        if len(costmap_msg.data) == 0:
            return
            
        try:
            # Convertir a numpy array
            costmap_data = np.array(costmap_msg.data, dtype=np.int8)
            
            # Contar celdas ocupadas (valor > 50 en nav2)
            occupied_cells = np.sum(costmap_data > 50)
            total_cells = len(costmap_data)
            
            if total_cells > 0:
                occupied_ratio = occupied_cells / total_cells
                
                # Si hay demasiados obstáculos, limpiar
                if occupied_ratio > self.obstacle_threshold:
                    current_time = time.time()
                    if current_time - self.last_cleaning_time > 10.0:  # No limpiar muy seguido
                        self.get_logger().warn(
                            f"🚨 {costmap_type} costmap saturado: {occupied_ratio*100:.1f}% ocupado - LIMPIANDO"
                        )
                        self.emergency_clean_costmaps()
                        
        except Exception as e:
            self.get_logger().warn(f"Error analizando costmap {costmap_type}: {e}")

    def periodic_cleanup(self):
        """Limpieza periódica preventiva"""
        current_time = time.time()
        if current_time - self.last_cleaning_time > self.cleaning_interval:
            self.get_logger().info("🧹 Limpieza periódica preventiva de costmaps")
            self.emergency_clean_costmaps()

    def emergency_clean_costmaps(self):
        """Limpia ambos costmaps de emergencia"""
        try:
            # Limpiar costmap local
            if self.clear_local_client.wait_for_service(timeout_sec=2.0):
                req = ClearEntireCostmap.Request()
                future = self.clear_local_client.call_async(req)
                self.get_logger().info("🧹 Costmap LOCAL limpiado")
            else:
                self.get_logger().warn("⚠️ Servicio de limpieza LOCAL no disponible")
                
            # Limpiar costmap global  
            if self.clear_global_client.wait_for_service(timeout_sec=2.0):
                req = ClearEntireCostmap.Request()
                future = self.clear_global_client.call_async(req)
                self.get_logger().info("🧹 Costmap GLOBAL limpiado")
            else:
                self.get_logger().warn("⚠️ Servicio de limpieza GLOBAL no disponible")
                
            self.last_cleaning_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"❌ Error limpiando costmaps: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    cleaner = EmergencyCostmapCleaner()
    
    try:
        rclpy.spin(cleaner)
    except KeyboardInterrupt:
        pass
    
    cleaner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
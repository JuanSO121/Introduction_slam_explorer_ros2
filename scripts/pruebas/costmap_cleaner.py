#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import threading
import time

class CostmapCleaner(Node):
    def __init__(self):
        super().__init__('costmap_cleaner')
        
        # Clientes de servicio para limpiar costmaps
        self.clear_local_costmap_client = self.create_client(
            Empty, 
            '/local_costmap/clear_entirely_local_costmap'
        )
        self.clear_global_costmap_client = self.create_client(
            Empty, 
            '/global_costmap/clear_entirely_global_costmap'
        )
        
        # Subscriber para monitorear el mapa
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Subscriber para monitorear velocidad
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Variables de estado
        self.last_cmd_vel = None
        self.stuck_counter = 0
        self.clean_counter = 0
        self.max_stuck_count = 10  # Limpiar después de 50 segundos sin movimiento
        
        # Timer para limpieza periódica
        self.periodic_clean_timer = self.create_timer(120.0, self.periodic_clean)  # Cada 2 minutos
        self.monitor_timer = self.create_timer(5.0, self.monitor_robot_status)
        
        self.lock = threading.Lock()
        
        self.get_logger().info('🧹 Limpiador de costmap iniciado')

    def map_callback(self, msg):
        # Callback para el mapa (puede ser útil para análisis futuro)
        pass

    def cmd_vel_callback(self, msg):
        with self.lock:
            self.last_cmd_vel = msg

    def monitor_robot_status(self):
        """Monitorear si el robot está atascado"""
        if self.last_cmd_vel is None:
            return
        
        linear_vel = abs(self.last_cmd_vel.linear.x)
        angular_vel = abs(self.last_cmd_vel.angular.z)
        
        # Verificar si el robot está intentando moverse pero no se mueve
        if linear_vel < 0.01 and angular_vel < 0.01:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        
        # Si está atascado por mucho tiempo, limpiar costmaps
        if self.stuck_counter >= self.max_stuck_count:
            self.get_logger().warning(
                f'🚫 Robot detectado como atascado ({self.stuck_counter} checks). Limpiando costmaps...'
            )
            self.clean_costmaps(reason="robot_stuck")
            self.stuck_counter = 0  # Reset counter

    def clean_costmaps(self, reason="manual"):
        """Limpiar tanto el costmap local como el global"""
        self.clean_counter += 1
        
        self.get_logger().info(
            f'🧹 Iniciando limpieza de costmaps #{self.clean_counter} (Razón: {reason})'
        )
        
        success_count = 0
        
        # Limpiar costmap local
        if self.clear_local_costmap_client.wait_for_service(timeout_sec=2.0):
            try:
                future = self.clear_local_costmap_client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    self.get_logger().info('✅ Costmap local limpiado exitosamente')
                    success_count += 1
                else:
                    self.get_logger().warning('⚠️ Fallo en limpieza de costmap local')
            except Exception as e:
                self.get_logger().error(f'❌ Error limpiando costmap local: {e}')
        else:
            self.get_logger().warning('⚠️ Servicio de limpieza de costmap local no disponible')
        
        # Pausa breve entre limpiezas
        time.sleep(0.5)
        
        # Limpiar costmap global
        if self.clear_global_costmap_client.wait_for_service(timeout_sec=2.0):
            try:
                future = self.clear_global_costmap_client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    self.get_logger().info('✅ Costmap global limpiado exitosamente')
                    success_count += 1
                else:
                    self.get_logger().warning('⚠️ Fallo en limpieza de costmap global')
            except Exception as e:
                self.get_logger().error(f'❌ Error limpiando costmap global: {e}')
        else:
            self.get_logger().warning('⚠️ Servicio de limpieza de costmap global no disponible')
        
        # Resumen de la limpieza
        if success_count == 2:
            self.get_logger().info('🎉 Limpieza completa exitosa')
        elif success_count == 1:
            self.get_logger().warning('⚠️ Limpieza parcial (solo un costmap limpiado)')
        else:
            self.get_logger().error('❌ Limpieza fallida en ambos costmaps')

    def periodic_clean(self):
        """Limpieza periódica preventiva"""
        self.get_logger().info('⏰ Ejecutando limpieza periódica preventiva')
        self.clean_costmaps(reason="periodic_maintenance")

    def force_clean(self):
        """Método para limpieza forzada (puede ser llamado externamente)"""
        self.get_logger().info('🔧 Limpieza forzada solicitada')
        self.clean_costmaps(reason="forced")

def main(args=None):
    rclpy.init(args=args)
    
    cleaner = CostmapCleaner()
    
    try:
        rclpy.spin(cleaner)
    except KeyboardInterrupt:
        cleaner.get_logger().info('🛑 Deteniendo limpiador de costmap...')
    finally:
        cleaner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
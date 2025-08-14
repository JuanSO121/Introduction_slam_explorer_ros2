#!/usr/bin/env python3
#!/usr/bin/env python3
"""
Nodo para realizar mapeo inicial rotando el robot
para generar un mapa base antes de la exploración automática.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import time
import math

class InitialMapper(Node):
    def __init__(self):
        super().__init__('initial_mapper')
        
        # Publicador de velocidad
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscriptor del mapa
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        
        # Variables de estado
        self.rotation_complete = False
        self.start_time = None
        self.total_rotation = 0.0
        self.target_rotation = 2 * math.pi  # Una rotación completa
        self.angular_velocity = 0.5  # rad/s - velocidad de rotación lenta
        
        # Timer para controlar la rotación
        self.timer = self.create_timer(0.1, self.rotation_timer_callback)
        
        self.get_logger().info('Initial Mapper iniciado - comenzando rotación para mapeo...')

    def map_callback(self, msg):
        """Callback del mapa - no necesario para esta función pero útil para monitoring"""
        pass

    def rotation_timer_callback(self):
        """Controla la rotación inicial del robot"""
        if self.rotation_complete:
            return
            
        # Inicializar tiempo de inicio
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('🔄 Iniciando rotación para mapeo inicial...')
        
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Calcular rotación total realizada
        self.total_rotation = self.angular_velocity * elapsed_time
        
        # Verificar si hemos completado la rotación
        if self.total_rotation >= self.target_rotation:
            # Detener el robot
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
            
            self.rotation_complete = True
            self.get_logger().info(
                f'✅ Rotación inicial completada en {elapsed_time:.1f}s - '
                'mapa base generado, listo para exploración automática'
            )
            
            # Esperar un poco más y luego terminar este nodo
            self.create_timer(2.0, self.shutdown_node, once=True)
            return
        
        # Continuar rotando
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_velocity
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Log de progreso cada 2 segundos
        if int(elapsed_time) % 2 == 0 and elapsed_time > 0:
            progress = (self.total_rotation / self.target_rotation) * 100
            self.get_logger().info(
                f'Progreso de mapeo inicial: {progress:.0f}% '
                f'({self.total_rotation:.1f}/{self.target_rotation:.1f} rad)'
            )

    def shutdown_node(self):
        """Termina este nodo después de completar el mapeo inicial"""
        self.get_logger().info('🏁 Initial Mapper terminando - mapeo inicial completo')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    initial_mapper = InitialMapper()
    
    try:
        rclpy.spin(initial_mapper)
    except KeyboardInterrupt:
        pass
    
    initial_mapper.destroy_node()

if __name__ == '__main__':
    main()
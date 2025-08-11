#exploration_monitor.py
"""
Monitor de exploración que verifica el progreso y reinicia
la exploración si se detiene prematuramente.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import time

class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')
        
        # Suscriptores
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/explore/goal',
            self.goal_callback,
            10
        )
        
        # Publicador para reiniciar exploración
        self.explore_publisher = self.create_publisher(
            String,
            '/explore/resume',
            10
        )
        
        # Variables de estado
        self.last_map_update = time.time()
        self.last_goal_time = time.time()
        self.map_area = 0
        self.unknown_area = 0
        self.free_area = 0
        self.occupied_area = 0
        
        # Contadores
        self.stuck_counter = 0
        self.total_goals = 0
        
        # Timer para verificación periódica
        self.timer = self.create_timer(10.0, self.check_exploration_progress)
        
        self.get_logger().info('Exploration Monitor iniciado')

    def map_callback(self, msg):
        """Callback para analizar el mapa"""
        self.last_map_update = time.time()
        
        # Convertir mapa a numpy array
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        # Calcular estadísticas
        total_cells = map_data.size
        unknown_cells = np.count_nonzero(map_data == -1)
        free_cells = np.count_nonzero(map_data == 0)
        occupied_cells = np.count_nonzero(map_data == 100)
        
        # Calcular áreas (en m²)
        cell_area = msg.info.resolution ** 2
        self.map_area = total_cells * cell_area
        self.unknown_area = unknown_cells * cell_area
        self.free_area = free_cells * cell_area
        self.occupied_area = occupied_cells * cell_area
        
        # Calcular porcentaje explorado
        explored_percentage = ((free_cells + occupied_cells) / total_cells) * 100
        
        # Log de progreso cada 30 segundos
        current_time = time.time()
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = current_time
        
        if current_time - self.last_log_time > 30.0:
            self.get_logger().info(
                f'Progreso de exploración: {explored_percentage:.1f}% '
                f'(Libre: {self.free_area:.1f}m², Ocupado: {self.occupied_area:.1f}m², '
                f'Desconocido: {self.unknown_area:.1f}m²)'
            )
            self.last_log_time = current_time

    def goal_callback(self, msg):
        """Callback para objetivos de exploración"""
        self.last_goal_time = time.time()
        self.total_goals += 1
        self.stuck_counter = 0  # Reset contador si hay nueva meta
        
        self.get_logger().info(
            f'Nueva meta de exploración #{self.total_goals}: '
            f'x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
        )

    def check_exploration_progress(self):
        """Verifica el progreso de exploración y toma acciones correctivas"""
        current_time = time.time()
        
        # Verificar si ha pasado mucho tiempo sin nuevas metas
        time_since_last_goal = current_time - self.last_goal_time
        
        if time_since_last_goal > 60.0:  # 1 minuto sin metas
            self.stuck_counter += 1
            
            # Calcular porcentaje explorado
            if self.map_area > 0:
                explored_percentage = ((self.free_area + self.occupied_area) / self.map_area) * 100
                
                self.get_logger().warn(
                    f'Sin nuevas metas por {time_since_last_goal:.0f}s. '
                    f'Explorado: {explored_percentage:.1f}%'
                )
                
                # Si aún hay área desconocida significativa, reintentar
                if explored_percentage < 85.0 and self.unknown_area > 2.0:  # Menos del 85% y >2m² desconocido
                    if self.stuck_counter >= 2:  # Después de 2 minutos sin metas
                        self.get_logger().warn('Reintentando exploración...')
                        self.restart_exploration()
                        self.stuck_counter = 0
                else:
                    self.get_logger().info(
                        f'Exploración completada: {explored_percentage:.1f}% '
                        f'(área desconocida restante: {self.unknown_area:.1f}m²)'
                    )
        else:
            self.stuck_counter = 0

    def restart_exploration(self):
        """Reinicia el proceso de exploración"""
        try:
            # Publicar comando de reinicio (si explore_lite lo soporta)
            msg = String()
            msg.data = "restart"
            self.explore_publisher.publish(msg)
            
            self.get_logger().info('Comando de reinicio de exploración enviado')
            
        except Exception as e:
            self.get_logger().error(f'Error al reiniciar exploración: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    exploration_monitor = ExplorationMonitor()
    
    try:
        rclpy.spin(exploration_monitor)
    except KeyboardInterrupt:
        pass
    
    exploration_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
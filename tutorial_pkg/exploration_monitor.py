# Archivo: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/exploration_monitor.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import time
import random
import math

class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')
        
        # Publishers
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Variables de estado
        self.last_movement_time = time.time()
        self.last_map_size = 0
        self.stuck_threshold = 30.0  # segundos sin movimiento
        self.exploration_points = []
        
        # Timer para monitoreo
        self.timer = self.create_timer(5.0, self.check_exploration_status)
        
        self.get_logger().info("Monitor de exploración iniciado")

    def map_callback(self, msg):
        current_map_size = len([cell for cell in msg.data if cell != -1])
        
        # Si el mapa creció, hay progreso
        if current_map_size > self.last_map_size:
            self.last_movement_time = time.time()
            self.last_map_size = current_map_size
            self.get_logger().info(f"Mapa expandido: {current_map_size} celdas conocidas")

    def check_exploration_status(self):
        current_time = time.time()
        time_since_progress = current_time - self.last_movement_time
        
        if time_since_progress > self.stuck_threshold:
            self.get_logger().warn("Robot detenido por mucho tiempo. Enviando nueva meta...")
            self.send_exploration_goal()
            self.last_movement_time = current_time

    def send_exploration_goal(self):
        # Crear objetivo aleatorio en área no explorada
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Coordenadas aleatorias (ajustar según tu salón)
        goal.pose.position.x = random.uniform(-5.0, 5.0)
        goal.pose.position.y = random.uniform(-5.0, 5.0)
        goal.pose.position.z = 0.0
        
        # Orientación aleatoria
        yaw = random.uniform(0, 2 * math.pi)
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        
        self.goal_publisher.publish(goal)
        self.get_logger().info(f"Nueva meta enviada: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")

    def rotate_robot(self):
        # Rotar el robot para encontrar nuevas áreas
        twist = Twist()
        twist.angular.z = 0.5
        
        for _ in range(20):  # Rotar por 2 segundos
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        
        # Detener rotación
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

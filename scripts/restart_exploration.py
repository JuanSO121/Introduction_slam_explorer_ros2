#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class ExplorationRestart(Node):
    def __init__(self):
        super().__init__('exploration_restart')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal_and_restart(self):
        # Enviar robot a una posición específica antes de reiniciar
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
        
        # Reiniciar explore_lite después de 5 segundos
        time.sleep(5)
        import subprocess
        subprocess.run(['ros2', 'lifecycle', 'set', '/explore', 'activate'])

def main():
    rclpy.init()
    restart_node = ExplorationRestart()
    restart_node.send_goal_and_restart()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Nodo de filtrado para el laser scan que elimina lecturas del suelo
y estabiliza las mediciones durante el bamboleo del robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LaserFilter(Node):
    def __init__(self):
        super().__init__('laser_filter')
        
        # Suscriptor al laser original
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  # Tema original del laser
            self.laser_callback,
            10
        )
        
        # Publicador del laser filtrado
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',  # Tema filtrado que usa navigation
            10
        )
        
        # Parámetros de filtrado
        self.min_range_filter = 0.15  # Mínimo rango para evitar suelo
        self.max_range_filter = 3.5   # Máximo rango útil
        self.angle_filter_min = -math.pi  # Ángulo mínimo
        self.angle_filter_max = math.pi   # Ángulo máximo
        
        # Buffer para suavizado temporal
        self.scan_buffer = []
        self.buffer_size = 3
        
        self.get_logger().info('Laser Filter Node iniciado')

    def laser_callback(self, msg):
        """Callback que procesa y filtra los datos del laser"""
        
        # Crear mensaje filtrado
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = self.min_range_filter
        filtered_msg.range_max = self.max_range_filter
        
        # Procesar rangos
        filtered_ranges = []
        
        for i, range_val in enumerate(msg.ranges):
            # Calcular ángulo actual
            angle = msg.angle_min + i * msg.angle_increment
            
            # Filtrar por ángulo (opcional: eliminar lecturas hacia abajo)
            if angle < self.angle_filter_min or angle > self.angle_filter_max:
                filtered_ranges.append(float('inf'))
                continue
            
            # Filtrar rangos inválidos
            if math.isnan(range_val) or math.isinf(range_val):
                filtered_ranges.append(float('inf'))
                continue
            
            # Filtrar rangos demasiado cortos (posible suelo)
            if range_val < self.min_range_filter:
                filtered_ranges.append(float('inf'))
                continue
                
            # Filtrar rangos demasiado largos
            if range_val > self.max_range_filter:
                filtered_ranges.append(float('inf'))
                continue
            
            # Filtro adicional: eliminar picos anómalos
            if self.is_anomalous_reading(range_val, i, msg.ranges):
                filtered_ranges.append(float('inf'))
                continue
            
            # Rango válido
            filtered_ranges.append(range_val)
        
        # Aplicar suavizado temporal
        filtered_ranges = self.temporal_smoothing(filtered_ranges)
        
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = msg.intensities if len(msg.intensities) > 0 else []
        
        # Publicar mensaje filtrado
        self.publisher.publish(filtered_msg)

    def is_anomalous_reading(self, current_range, index, all_ranges):
        """Detecta lecturas anómalas comparando con vecinos"""
        window_size = 3
        
        # Obtener rangos vecinos
        start_idx = max(0, index - window_size)
        end_idx = min(len(all_ranges), index + window_size + 1)
        
        neighbors = []
        for i in range(start_idx, end_idx):
            if i != index and not math.isnan(all_ranges[i]) and not math.isinf(all_ranges[i]):
                if self.min_range_filter <= all_ranges[i] <= self.max_range_filter:
                    neighbors.append(all_ranges[i])
        
        if len(neighbors) < 2:
            return False
        
        # Calcular diferencia con vecinos
        neighbor_mean = np.mean(neighbors)
        difference = abs(current_range - neighbor_mean)
        
        # Si la diferencia es muy grande, considerar anómalo
        threshold = 0.5  # metros
        return difference > threshold and current_range < neighbor_mean * 0.5

    def temporal_smoothing(self, ranges):
        """Aplica suavizado temporal usando buffer de scans anteriores"""
        # Agregar al buffer
        self.scan_buffer.append(ranges.copy())
        
        # Mantener tamaño del buffer
        if len(self.scan_buffer) > self.buffer_size:
            self.scan_buffer.pop(0)
        
        # Si no hay suficientes scans, retornar sin suavizar
        if len(self.scan_buffer) < 2:
            return ranges
        
        # Suavizar usando media móvil
        smoothed_ranges = []
        for i in range(len(ranges)):
            valid_values = []
            for scan in self.scan_buffer:
                if i < len(scan) and not math.isinf(scan[i]):
                    valid_values.append(scan[i])
            
            if len(valid_values) > 0:
                smoothed_ranges.append(np.mean(valid_values))
            else:
                smoothed_ranges.append(float('inf'))
        
        return smoothed_ranges

def main(args=None):
    rclpy.init(args=args)
    
    laser_filter = LaserFilter()
    
    try:
        rclpy.spin(laser_filter)
    except KeyboardInterrupt:
        pass
    
    laser_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Sistema de Arbitraje Central para Control de Exploración
Coordina exploración automática y comandos por voz sin conflictos
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/exploration_arbiter.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import time
from enum import Enum
from typing import Dict, Optional


class ControlMode(Enum):
    """Estados de control del robot"""
    AUTONOMOUS_EXPLORATION = "autonomous_exploration"
    VOICE_CONTROL = "voice_control" 
    MANUAL_CONTROL = "manual_control"
    EMERGENCY_STOP = "emergency_stop"
    IDLE = "idle"


class ExplorationArbiter(Node):
    def __init__(self):
        super().__init__('exploration_arbiter')
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Estado del sistema
        self.current_mode = ControlMode.IDLE
        self.previous_mode = ControlMode.IDLE
        self.mode_change_time = time.time()
        self.last_voice_command_time = 0
        self.last_exploration_command_time = 0
        self.voice_command_timeout = 10.0  # 10 segundos
        self.command_queue = []
        
        # Configuración de prioridades
        self.priority_map = {
            ControlMode.EMERGENCY_STOP: 4,
            ControlMode.VOICE_CONTROL: 3,
            ControlMode.MANUAL_CONTROL: 2,
            ControlMode.AUTONOMOUS_EXPLORATION: 1,
            ControlMode.IDLE: 0
        }
        
        # === SUSCRIPTORES ===
        
        # Control por voz
        self.voice_command_sub = self.create_subscription(
            String, '/voice_commands', self.voice_command_callback, qos_reliable)
        
        self.voice_feedback_sub = self.create_subscription(
            String, '/voice_feedback', self.voice_feedback_callback, qos_reliable)
        
        # Control de exploración
        self.exploration_control_sub = self.create_subscription(
            String, '/exploration_control', self.exploration_control_callback, qos_reliable)
        
        # Estado del monitor de exploración
        self.monitor_status_sub = self.create_subscription(
            String, '/monitor_status', self.monitor_status_callback, qos_reliable)
        
        # Comandos de velocidad (para detectar control manual directo)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_input', self.cmd_vel_input_callback, qos_reliable)
        
        # === PUBLICADORES ===
        
        # Control de exploración automática
        self.exploration_enable_pub = self.create_publisher(
            Bool, '/exploration_enabled', qos_reliable)
        
        # Control de comandos por voz
        self.voice_control_enable_pub = self.create_publisher(
            Bool, '/voice_control_enabled', qos_reliable)
        
        # Estado del arbitraje
        self.arbiter_status_pub = self.create_publisher(
            String, '/arbiter_status', qos_reliable)
        
        # Control centralizado de cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', qos_reliable)
        
        # Objetivos de exploración filtrados
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, '/goal_pose', qos_reliable)
        
        # === TIMERS ===
        self.arbiter_timer = self.create_timer(0.5, self.arbitration_cycle)
        self.status_timer = self.create_timer(2.0, self.publish_status)
        self.timeout_timer = self.create_timer(1.0, self.check_timeouts)
        
        self.get_logger().info("🎯 Sistema de Arbitraje iniciado")
        self.get_logger().info("📋 Coordinando exploración automática y control por voz")
        
    def voice_command_callback(self, msg: String):
        """Procesar comandos de voz y determinar prioridad"""
        command = msg.data.lower().strip()
        current_time = time.time()
        
        self.get_logger().info(f"🎤 Comando de voz recibido: {command}")
        
        # Detectar tipo de comando
        if self.is_emergency_command(command):
            self.request_mode_change(ControlMode.EMERGENCY_STOP, "voice_emergency")
        elif self.is_movement_command(command):
            self.request_mode_change(ControlMode.VOICE_CONTROL, "voice_movement")
            self.last_voice_command_time = current_time
        elif self.is_exploration_command(command):
            self.handle_exploration_voice_command(command)
        else:
            # Comando conversacional o de información
            self.get_logger().info(f"💬 Comando conversacional: {command}")
    
    def voice_feedback_callback(self, msg: String):
        """Procesar feedback de comandos de voz"""
        feedback = msg.data.lower()
        current_time = time.time()
        
        # Detectar indicadores de control activo por voz
        if any(indicator in feedback for indicator in [
            "moviendo", "girando", "avanzando", "retrocediendo", "ejecutando"
        ]):
            self.last_voice_command_time = current_time
            if self.current_mode != ControlMode.VOICE_CONTROL:
                self.request_mode_change(ControlMode.VOICE_CONTROL, "voice_active")
    
    def exploration_control_callback(self, msg: String):
        """Manejar comandos de control de exploración"""
        command = msg.data
        current_time = time.time()
        
        self.get_logger().info(f"🗺️ Comando de exploración: {command}")
        
        if command == "start_exploration":
            self.request_mode_change(ControlMode.AUTONOMOUS_EXPLORATION, "exploration_start")
        elif command == "pause_exploration":
            self.request_mode_change(ControlMode.IDLE, "exploration_pause")
        elif command == "resume_exploration":
            self.request_mode_change(ControlMode.AUTONOMOUS_EXPLORATION, "exploration_resume")
        elif command == "finish_exploration":
            self.request_mode_change(ControlMode.IDLE, "exploration_finish")
        elif command == "emergency_stop":
            self.request_mode_change(ControlMode.EMERGENCY_STOP, "exploration_emergency")
        
        self.last_exploration_command_time = current_time
    
    def monitor_status_callback(self, msg: String):
        """Procesar estado del monitor de exploración"""
        try:
            parts = msg.data.split('|')
            if len(parts) >= 1:
                status = parts[0]
                self.get_logger().debug(f"📊 Monitor status: {status}")
        except Exception as e:
            self.get_logger().error(f"Error procesando monitor status: {e}")
    
    def cmd_vel_input_callback(self, msg: Twist):
        """Detectar control manual directo"""
        if (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01):
            current_time = time.time()
            
            # Si no es comando por voz reciente, es control manual
            if (current_time - self.last_voice_command_time) > 2.0:
                self.request_mode_change(ControlMode.MANUAL_CONTROL, "direct_input")
    
    def is_emergency_command(self, command: str) -> bool:
        """Detectar comandos de emergencia"""
        emergency_keywords = [
            "emergencia", "emergency", "parar", "stop", "alto", "help", "ayuda"
        ]
        return any(keyword in command for keyword in emergency_keywords)
    
    def is_movement_command(self, command: str) -> bool:
        """Detectar comandos de movimiento"""
        movement_keywords = [
            "adelante", "atras", "izquierda", "derecha", "forward", "backward", 
            "left", "right", "girar", "mover", "avanzar", "retroceder"
        ]
        return any(keyword in command for keyword in movement_keywords)
    
    def is_exploration_command(self, command: str) -> bool:
        """Detectar comandos de exploración"""
        exploration_keywords = [
            "explorar", "mapear", "exploration", "iniciar", "pausar", "continuar",
            "terminar", "resume", "start", "finish"
        ]
        return any(keyword in command for keyword in exploration_keywords)
    
    def handle_exploration_voice_command(self, command: str):
        """Manejar comandos de exploración por voz"""
        if "iniciar" in command or "start" in command or "explorar" in command:
            self.request_mode_change(ControlMode.AUTONOMOUS_EXPLORATION, "voice_start_exploration")
        elif "pausar" in command or "pause" in command:
            self.request_mode_change(ControlMode.IDLE, "voice_pause_exploration")
        elif "continuar" in command or "resume" in command:
            self.request_mode_change(ControlMode.AUTONOMOUS_EXPLORATION, "voice_resume_exploration")
        elif "terminar" in command or "finish" in command:
            self.request_mode_change(ControlMode.IDLE, "voice_finish_exploration")
    
    def request_mode_change(self, requested_mode: ControlMode, reason: str):
        """Solicitar cambio de modo con verificación de prioridad"""
        current_priority = self.priority_map[self.current_mode]
        requested_priority = self.priority_map[requested_mode]
        
        # Permitir cambio si tiene mayor prioridad o si el tiempo de comando ha expirado
        current_time = time.time()
        voice_timeout = (current_time - self.last_voice_command_time) > self.voice_command_timeout
        
        if (requested_priority > current_priority or 
            (requested_mode == ControlMode.AUTONOMOUS_EXPLORATION and voice_timeout) or
            requested_mode == ControlMode.IDLE):
            
            self.change_mode(requested_mode, reason)
            return True
        else:
            self.get_logger().info(f"⚠️ Cambio de modo denegado: {requested_mode.value} (prioridad insuficiente)")
            return False
    
    def change_mode(self, new_mode: ControlMode, reason: str):
        """Cambiar modo de control del sistema"""
        if new_mode != self.current_mode:
            self.previous_mode = self.current_mode
            self.current_mode = new_mode
            self.mode_change_time = time.time()
            
            self.get_logger().info(f"🔄 Cambio de modo: {self.previous_mode.value} -> {new_mode.value} ({reason})")
            
            # Aplicar configuraciones del nuevo modo
            self.apply_mode_settings(new_mode)
            
            # Notificar a otros nodos
            self.notify_mode_change(new_mode, reason)
    
    def apply_mode_settings(self, mode: ControlMode):
        """Aplicar configuraciones específicas del modo"""
        if mode == ControlMode.AUTONOMOUS_EXPLORATION:
            self.enable_exploration(True)
            self.enable_voice_control(False)
            
        elif mode == ControlMode.VOICE_CONTROL:
            self.enable_exploration(False)
            self.enable_voice_control(True)
            
        elif mode == ControlMode.MANUAL_CONTROL:
            self.enable_exploration(False)
            self.enable_voice_control(False)
            
        elif mode == ControlMode.EMERGENCY_STOP:
            self.enable_exploration(False)
            self.enable_voice_control(False)
            self.send_emergency_stop()
            
        elif mode == ControlMode.IDLE:
            self.enable_exploration(False)
            self.enable_voice_control(False)
    
    def enable_exploration(self, enabled: bool):
        """Habilitar/deshabilitar exploración automática"""
        msg = Bool()
        msg.data = enabled
        self.exploration_enable_pub.publish(msg)
        self.get_logger().debug(f"🗺️ Exploración automática: {'ON' if enabled else 'OFF'}")
    
    def enable_voice_control(self, enabled: bool):
        """Habilitar/deshabilitar control por voz"""
        msg = Bool()
        msg.data = enabled
        self.voice_control_enable_pub.publish(msg)
        self.get_logger().debug(f"🎤 Control por voz: {'ON' if enabled else 'OFF'}")
    
    def send_emergency_stop(self):
        """Enviar comando de parada de emergencia"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().error("🚨 PARADA DE EMERGENCIA ACTIVADA")
    
    def notify_mode_change(self, mode: ControlMode, reason: str):
        """Notificar cambio de modo a otros nodos"""
        notification = {
            "event": "mode_change",
            "previous_mode": self.previous_mode.value,
            "current_mode": mode.value,
            "reason": reason,
            "timestamp": time.time()
        }
        
        msg = String()
        msg.data = json.dumps(notification)
        self.arbiter_status_pub.publish(msg)
    
    def check_timeouts(self):
        """Verificar timeouts y transiciones automáticas"""
        current_time = time.time()
        
        # Timeout de control por voz
        if (self.current_mode == ControlMode.VOICE_CONTROL and 
            (current_time - self.last_voice_command_time) > self.voice_command_timeout):
            
            self.get_logger().info("⏱️ Timeout de control por voz - regresando a exploración")
            self.request_mode_change(ControlMode.AUTONOMOUS_EXPLORATION, "voice_timeout")
        
        # Timeout de emergencia (auto-reset después de 30 segundos)
        if (self.current_mode == ControlMode.EMERGENCY_STOP and
            (current_time - self.mode_change_time) > 30.0):
            
            self.get_logger().info("🔄 Reset automático de emergencia")
            self.request_mode_change(ControlMode.IDLE, "emergency_timeout")
    
    def arbitration_cycle(self):
        """Ciclo principal de arbitraje"""
        current_time = time.time()
        
        # Evaluar condiciones para cambios automáticos
        if self.current_mode == ControlMode.IDLE:
            # Verificar si debería iniciar exploración automática
            time_in_idle = current_time - self.mode_change_time
            if (time_in_idle > 5.0 and 
                (current_time - self.last_voice_command_time) > self.voice_command_timeout):
                
                self.request_mode_change(ControlMode.AUTONOMOUS_EXPLORATION, "auto_resume")
        
        # Procesar cola de comandos pendientes
        self.process_command_queue()
    
    def process_command_queue(self):
        """Procesar cola de comandos pendientes"""
        while self.command_queue:
            command = self.command_queue.pop(0)
            self.get_logger().debug(f"📝 Procesando comando en cola: {command}")
            # Aquí se pueden procesar comandos que fueron diferidos
    
    def publish_status(self):
        """Publicar estado actual del arbitraje"""
        current_time = time.time()
        
        status = {
            "current_mode": self.current_mode.value,
            "previous_mode": self.previous_mode.value,
            "time_in_mode": current_time - self.mode_change_time,
            "last_voice_command": current_time - self.last_voice_command_time,
            "exploration_enabled": self.current_mode == ControlMode.AUTONOMOUS_EXPLORATION,
            "voice_control_enabled": self.current_mode == ControlMode.VOICE_CONTROL,
            "system_status": "operational",
            "timestamp": current_time
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.arbiter_status_pub.publish(msg)
        
        # Log periódico resumido
        if int(current_time) % 30 == 0:  # Cada 30 segundos
            self.get_logger().info(
                f"🎯 Estado: {self.current_mode.value}, "
                f"Tiempo: {status['time_in_mode']:.1f}s"
            )
    
    def get_system_summary(self) -> str:
        """Obtener resumen del estado del sistema"""
        return f"""
Sistema de Arbitraje - Estado Actual:
- Modo: {self.current_mode.value}
- Modo anterior: {self.previous_mode.value}
- Tiempo en modo actual: {time.time() - self.mode_change_time:.1f}s
- Último comando de voz: {time.time() - self.last_voice_command_time:.1f}s atrás
- Exploración habilitada: {self.current_mode == ControlMode.AUTONOMOUS_EXPLORATION}
- Control por voz habilitado: {self.current_mode == ControlMode.VOICE_CONTROL}
        """.strip()


def main(args=None):
    rclpy.init(args=args)
    
    exploration_arbiter = ExplorationArbiter()
    
    try:
        exploration_arbiter.get_logger().info("🎯 Sistema de Arbitraje ejecutándose...")
        rclpy.spin(exploration_arbiter)
    except KeyboardInterrupt:
        exploration_arbiter.get_logger().info("🔄 Deteniendo Sistema de Arbitraje...")
    finally:
        exploration_arbiter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
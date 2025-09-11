#!/usr/bin/env python3
"""
Robot Control Coordinator - Sistema Centralizado de Control
Implementa patrón Coordinator + State Machine para evitar conflictos entre servicios
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/robot_control_coordinator.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import json
import time
import threading
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, Callable
from collections import deque
import uuid


class RobotState(Enum):
    """Estados principales del robot"""
    INITIALIZING = auto()
    IDLE = auto()
    EXPLORING_AUTO = auto()
    MANUAL_CONTROL = auto()
    VOICE_CONTROL = auto()
    EMERGENCY_STOP = auto()
    RECOVERING = auto()
    PAUSED = auto()


class ControlSource(Enum):
    """Fuentes de control posibles"""
    SYSTEM = auto()          # Sistema/inicialización
    USER_VOICE = auto()      # Comando de voz del usuario
    USER_MANUAL = auto()     # Control manual directo
    EXPLORATION = auto()     # Sistema de exploración autónoma
    EMERGENCY = auto()       # Sistema de emergencia
    RECOVERY = auto()        # Sistema de recuperación


@dataclass
class ControlRequest:
    """Estructura para solicitudes de control"""
    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    source: ControlSource = ControlSource.SYSTEM
    command_type: str = ""
    data: Dict[str, Any] = field(default_factory=dict)
    priority: int = 0  # 0=más alta, mayor número=menor prioridad
    timestamp: float = field(default_factory=time.time)
    timeout: float = 30.0  # Timeout en segundos
    callback: Optional[Callable] = None


@dataclass
class SystemStatus:
    """Estado completo del sistema"""
    current_state: RobotState = RobotState.INITIALIZING
    active_controller: Optional[ControlSource] = None
    last_command: Optional[ControlRequest] = None
    pending_requests: int = 0
    state_entry_time: float = field(default_factory=time.time)
    emergency_active: bool = False
    voice_control_active: bool = False
    exploration_active: bool = False
    manual_override_active: bool = False


class RobotControlCoordinator(Node):
    """Coordinador central que gestiona todos los controles del robot"""
    
    def __init__(self):
        super().__init__('robot_control_coordinator')
        
        # =====================================================================
        # CONFIGURACIÓN DE PARÁMETROS
        # =====================================================================
        self.declare_parameter('emergency_priority', 0)
        self.declare_parameter('voice_priority', 1)
        self.declare_parameter('manual_priority', 2)
        self.declare_parameter('exploration_priority', 5)
        self.declare_parameter('max_queue_size', 10)
        self.declare_parameter('state_timeout', 300.0)  # 5 minutos
        self.declare_parameter('heartbeat_interval', 2.0)
        
        # Estado del sistema
        self.status = SystemStatus()
        self.request_queue = deque(maxlen=self.get_parameter('max_queue_size').value)
        self.active_subscriptions = {}
        self._lock = threading.RLock()
        
        # Mapeo de prioridades
        self.priority_map = {
            ControlSource.EMERGENCY: self.get_parameter('emergency_priority').value,
            ControlSource.USER_VOICE: self.get_parameter('voice_priority').value,
            ControlSource.USER_MANUAL: self.get_parameter('manual_priority').value,
            ControlSource.EXPLORATION: self.get_parameter('exploration_priority').value,
            ControlSource.RECOVERY: 3,
            ControlSource.SYSTEM: 4
        }
        
        # =====================================================================
        # PUBLISHERS - SALIDAS DEL COORDINADOR
        # =====================================================================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.exploration_control_pub = self.create_publisher(String, '/exploration_control', 10)
        self.system_status_pub = self.create_publisher(String, '/coordinator_status', 10)
        self.control_feedback_pub = self.create_publisher(String, '/control_feedback', 10)
        
        # =====================================================================
        # SUBSCRIBERS - ENTRADAS AL COORDINADOR
        # =====================================================================
        # Control por voz
        self.voice_commands_sub = self.create_subscription(
            String, '/voice_commands', self._handle_voice_command, 10)
        
        # Control manual directo
        self.manual_control_sub = self.create_subscription(
            String, '/manual_control', self._handle_manual_command, 10)
        
        # Solicitudes de exploración
        self.exploration_request_sub = self.create_subscription(
            String, '/exploration_request', self._handle_exploration_request, 10)
        
        # Comandos de emergencia
        self.emergency_sub = self.create_subscription(
            String, '/emergency_command', self._handle_emergency_command, 10)
        
        # Estado de sensores para toma de decisiones
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._handle_odometry, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._handle_laser_scan, 10)
        
        # =====================================================================
        # TIMERS
        # =====================================================================
        self.control_timer = self.create_timer(0.1, self._process_control_queue)
        self.status_timer = self.create_timer(
            self.get_parameter('heartbeat_interval').value, 
            self._publish_status
        )
        self.cleanup_timer = self.create_timer(10.0, self._cleanup_expired_requests)
        
        # =====================================================================
        # MÁQUINA DE ESTADOS - Transiciones válidas
        # =====================================================================
        self.valid_transitions = {
            RobotState.INITIALIZING: [RobotState.IDLE, RobotState.EMERGENCY_STOP],
            RobotState.IDLE: [
                RobotState.EXPLORING_AUTO, RobotState.MANUAL_CONTROL, 
                RobotState.VOICE_CONTROL, RobotState.EMERGENCY_STOP
            ],
            RobotState.EXPLORING_AUTO: [
                RobotState.IDLE, RobotState.VOICE_CONTROL, RobotState.PAUSED,
                RobotState.EMERGENCY_STOP, RobotState.RECOVERING
            ],
            RobotState.MANUAL_CONTROL: [
                RobotState.IDLE, RobotState.VOICE_CONTROL, RobotState.EMERGENCY_STOP
            ],
            RobotState.VOICE_CONTROL: [
                RobotState.IDLE, RobotState.EXPLORING_AUTO, RobotState.MANUAL_CONTROL,
                RobotState.PAUSED, RobotState.EMERGENCY_STOP
            ],
            RobotState.EMERGENCY_STOP: [RobotState.IDLE, RobotState.RECOVERING],
            RobotState.RECOVERING: [RobotState.IDLE, RobotState.EMERGENCY_STOP],
            RobotState.PAUSED: [
                RobotState.IDLE, RobotState.EXPLORING_AUTO, RobotState.VOICE_CONTROL,
                RobotState.EMERGENCY_STOP
            ]
        }
        
        # =====================================================================
        # HANDLERS DE COMANDOS POR TIPO
        # =====================================================================
        self.command_handlers = {
            'move_forward': self._execute_movement_command,
            'move_backward': self._execute_movement_command,
            'turn_left': self._execute_movement_command,
            'turn_right': self._execute_movement_command,
            'stop': self._execute_stop_command,
            'start_exploration': self._execute_exploration_command,
            'pause_exploration': self._execute_exploration_command,
            'resume_exploration': self._execute_exploration_command,
            'finish_exploration': self._execute_exploration_command,
            'emergency_stop': self._execute_emergency_command,
            'recovery': self._execute_recovery_command
        }
        
        self.get_logger().info('🎛️ Robot Control Coordinator iniciado')
        self._transition_to_state(RobotState.IDLE)
    
    # =========================================================================
    # MÉTODOS PÚBLICOS - API DEL COORDINADOR
    # =========================================================================
    
    def submit_control_request(self, source: ControlSource, command_type: str, 
                             data: Dict[str, Any] = None, callback: Callable = None) -> str:
        """API principal para enviar solicitudes de control"""
        with self._lock:
            request = ControlRequest(
                source=source,
                command_type=command_type,
                data=data or {},
                priority=self.priority_map.get(source, 10),
                callback=callback
            )
            
            # Validar que la solicitud sea válida para el estado actual
            if not self._is_request_valid(request):
                self.get_logger().warn(
                    f'❌ Solicitud rechazada: {command_type} de {source.name} '
                    f'no válida en estado {self.status.current_state.name}'
                )
                return None
            
            self.request_queue.append(request)
            self.status.pending_requests = len(self.request_queue)
            
            self.get_logger().info(
                f'📝 Nueva solicitud: {command_type} de {source.name} '
                f'(prioridad: {request.priority}, ID: {request.id})'
            )
            
            return request.id
    
    def cancel_request(self, request_id: str) -> bool:
        """Cancelar una solicitud pendiente"""
        with self._lock:
            for request in list(self.request_queue):
                if request.id == request_id:
                    self.request_queue.remove(request)
                    self.status.pending_requests = len(self.request_queue)
                    return True
            return False
    
    def get_system_status(self) -> Dict[str, Any]:
        """Obtener estado completo del sistema"""
        with self._lock:
            return {
                'state': self.status.current_state.name,
                'active_controller': self.status.active_controller.name if self.status.active_controller else None,
                'pending_requests': self.status.pending_requests,
                'emergency_active': self.status.emergency_active,
                'voice_control_active': self.status.voice_control_active,
                'exploration_active': self.status.exploration_active,
                'manual_override_active': self.status.manual_override_active,
                'last_command': {
                    'type': self.status.last_command.command_type,
                    'source': self.status.last_command.source.name,
                    'timestamp': self.status.last_command.timestamp
                } if self.status.last_command else None,
                'state_duration': time.time() - self.status.state_entry_time
            }
    
    # =========================================================================
    # HANDLERS DE ENTRADA - Procesan comandos externos
    # =========================================================================
    
    def _handle_voice_command(self, msg: String):
        """Manejar comandos de voz - COORDINA con otros sistemas"""
        try:
            command = msg.data.lower().strip()
            
            # Mapear comandos de voz a tipos estándar
            voice_command_map = {
                'adelante': 'move_forward',
                'atras': 'move_backward', 
                'izquierda': 'turn_left',
                'derecha': 'turn_right',
                'parar': 'stop',
                'explorar': 'start_exploration',
                'pausar exploracion': 'pause_exploration',
                'continuar': 'resume_exploration',
                'terminar exploracion': 'finish_exploration',
                'emergencia': 'emergency_stop'
            }
            
            command_type = voice_command_map.get(command)
            if command_type:
                self.submit_control_request(
                    ControlSource.USER_VOICE,
                    command_type,
                    {'original_command': command}
                )
            else:
                self.get_logger().warn(f'❓ Comando de voz no reconocido: {command}')
                
        except Exception as e:
            self.get_logger().error(f'Error procesando comando de voz: {e}')
    
    def _handle_manual_command(self, msg: String):
        """Manejar comandos manuales directos"""
        try:
            data = json.loads(msg.data)
            command_type = data.get('command')
            
            if command_type:
                self.submit_control_request(
                    ControlSource.USER_MANUAL,
                    command_type,
                    data
                )
        except Exception as e:
            self.get_logger().error(f'Error procesando comando manual: {e}')
    
    def _handle_exploration_request(self, msg: String):
        """Manejar solicitudes del sistema de exploración"""
        try:
            data = json.loads(msg.data)
            request_type = data.get('type', 'continue_exploration')
            
            # Solo procesar si NO hay control manual/voz activo
            if not (self.status.voice_control_active or self.status.manual_override_active):
                self.submit_control_request(
                    ControlSource.EXPLORATION,
                    request_type,
                    data
                )
            else:
                self.get_logger().debug(
                    f'🚫 Solicitud de exploración ignorada - Control manual/voz activo'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error procesando solicitud de exploración: {e}')
    
    def _handle_emergency_command(self, msg: String):
        """Manejar comandos de emergencia - MÁXIMA PRIORIDAD"""
        self.submit_control_request(
            ControlSource.EMERGENCY,
            'emergency_stop',
            {'reason': msg.data}
        )
    
    def _handle_odometry(self, msg: Odometry):
        """Procesar odometría para detección de problemas"""
        # Aquí se puede implementar detección de robot atascado
        pass
    
    def _handle_laser_scan(self, msg: LaserScan):
        """Procesar scan láser para detección de obstáculos"""
        # Aquí se puede implementar detección de obstáculos críticos
        pass
    
    # =========================================================================
    # PROCESAMIENTO DE CONTROL - Núcleo del coordinador
    # =========================================================================
    
    def _process_control_queue(self):
        """Procesar cola de solicitudes - EJECUTA EN TIMER DE ALTA FRECUENCIA"""
        if not self.request_queue:
            return
        
        with self._lock:
            # Ordenar por prioridad (0 = más alta)
            sorted_requests = sorted(self.request_queue, key=lambda r: r.priority)
            
            # Procesar la solicitud de mayor prioridad
            request = sorted_requests[0]
            self.request_queue.remove(request)
            self.status.pending_requests = len(self.request_queue)
            
            # Verificar si la solicitud ha expirado
            if time.time() - request.timestamp > request.timeout:
                self.get_logger().warn(f'⏰ Solicitud expirada: {request.id}')
                return
            
            # Ejecutar la solicitud
            success = self._execute_request(request)
            
            if success:
                self.status.last_command = request
                self.status.active_controller = request.source
                self.get_logger().info(
                    f'✅ Ejecutada: {request.command_type} de {request.source.name}'
                )
                
                # Llamar callback si existe
                if request.callback:
                    try:
                        request.callback(True, None)
                    except Exception as e:
                        self.get_logger().error(f'Error en callback: {e}')
            else:
                self.get_logger().warn(
                    f'❌ Falló ejecución: {request.command_type} de {request.source.name}'
                )
                
                if request.callback:
                    try:
                        request.callback(False, "Execution failed")
                    except Exception as e:
                        self.get_logger().error(f'Error en callback: {e}')
    
    def _execute_request(self, request: ControlRequest) -> bool:
        """Ejecutar una solicitud específica"""
        try:
            handler = self.command_handlers.get(request.command_type)
            if handler:
                return handler(request)
            else:
                self.get_logger().warn(f'❓ Handler no encontrado: {request.command_type}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error ejecutando {request.command_type}: {e}')
            return False
    
    def _is_request_valid(self, request: ControlRequest) -> bool:
        """Validar si una solicitud es válida para el estado actual"""
        # Comandos de emergencia siempre son válidos
        if request.source == ControlSource.EMERGENCY:
            return True
        
        # Validaciones por estado
        current_state = self.status.current_state
        
        if current_state == RobotState.EMERGENCY_STOP:
            # Solo comandos de recuperación o sistema son válidos
            return request.source in [ControlSource.SYSTEM, ControlSource.RECOVERY]
        
        # Validaciones por prioridad - rechazar si hay un controlador de mayor prioridad activo
        if self.status.active_controller:
            active_priority = self.priority_map.get(self.status.active_controller, 10)
            request_priority = self.priority_map.get(request.source, 10)
            
            # Si la nueva solicitud tiene menor prioridad, rechazar
            if request_priority > active_priority:
                return False
        
        return True
    
    # =========================================================================
    # EJECUTORES DE COMANDOS - Realizan acciones específicas
    # =========================================================================
    
    def _execute_movement_command(self, request: ControlRequest) -> bool:
        """Ejecutar comandos de movimiento"""
        cmd_vel = Twist()
        command = request.command_type
        data = request.data
        
        # Parámetros de velocidad (configurables)
        linear_speed = data.get('linear_speed', 0.2)
        angular_speed = data.get('angular_speed', 0.3)
        
        if command == 'move_forward':
            cmd_vel.linear.x = linear_speed
            self._transition_to_state(RobotState.MANUAL_CONTROL)
        elif command == 'move_backward':
            cmd_vel.linear.x = -linear_speed
            self._transition_to_state(RobotState.MANUAL_CONTROL)
        elif command == 'turn_left':
            cmd_vel.angular.z = angular_speed
            self._transition_to_state(RobotState.MANUAL_CONTROL)
        elif command == 'turn_right':
            cmd_vel.angular.z = -angular_speed
            self._transition_to_state(RobotState.MANUAL_CONTROL)
        
        # Publicar comando
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Actualizar flags de estado
        if request.source == ControlSource.USER_VOICE:
            self.status.voice_control_active = True
            self.status.manual_override_active = True
            self._transition_to_state(RobotState.VOICE_CONTROL)
        elif request.source == ControlSource.USER_MANUAL:
            self.status.manual_override_active = True
        
        return True
    
    def _execute_stop_command(self, request: ControlRequest) -> bool:
        """Ejecutar comando de parada"""
        cmd_vel = Twist()  # Velocidades en cero
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Actualizar estado según fuente
        if request.source == ControlSource.USER_VOICE:
            self.status.voice_control_active = True
            self.status.manual_override_active = True
            self._transition_to_state(RobotState.VOICE_CONTROL)
        else:
            self._transition_to_state(RobotState.IDLE)
        
        return True
    
    def _execute_exploration_command(self, request: ControlRequest) -> bool:
        """Ejecutar comandos de exploración"""
        command = request.command_type
        
        # Enviar comando al sistema de exploración
        exploration_msg = String()
        
        if command == 'start_exploration':
            exploration_msg.data = 'start_exploration'
            self.status.exploration_active = True
            self.status.manual_override_active = False
            self.status.voice_control_active = False
            self._transition_to_state(RobotState.EXPLORING_AUTO)
            
        elif command == 'pause_exploration':
            exploration_msg.data = 'pause_exploration'
            self.status.exploration_active = False
            if request.source == ControlSource.USER_VOICE:
                self.status.voice_control_active = True
                self.status.manual_override_active = True
                self._transition_to_state(RobotState.VOICE_CONTROL)
            else:
                self._transition_to_state(RobotState.PAUSED)
                
        elif command == 'resume_exploration':
            exploration_msg.data = 'resume_exploration'
            self.status.exploration_active = True
            self.status.manual_override_active = False
            self.status.voice_control_active = False
            self._transition_to_state(RobotState.EXPLORING_AUTO)
            
        elif command == 'finish_exploration':
            exploration_msg.data = 'finish_exploration'
            self.status.exploration_active = False
            self.status.manual_override_active = False
            self.status.voice_control_active = False
            self._transition_to_state(RobotState.IDLE)
        
        self.exploration_control_pub.publish(exploration_msg)
        return True
    
    def _execute_emergency_command(self, request: ControlRequest) -> bool:
        """Ejecutar comando de emergencia"""
        # Parar inmediatamente
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Parar exploración
        exploration_msg = String()
        exploration_msg.data = 'emergency_stop'
        self.exploration_control_pub.publish(exploration_msg)
        
        # Actualizar estado
        self.status.emergency_active = True
        self.status.exploration_active = False
        self.status.voice_control_active = False
        self.status.manual_override_active = False
        
        self._transition_to_state(RobotState.EMERGENCY_STOP)
        
        # Enviar feedback
        feedback = String()
        feedback.data = f"🚨 EMERGENCIA: {request.data.get('reason', 'Activada por comando')}"
        self.control_feedback_pub.publish(feedback)
        
        return True
    
    def _execute_recovery_command(self, request: ControlRequest) -> bool:
        """Ejecutar comando de recuperación"""
        self._transition_to_state(RobotState.RECOVERING)
        
        # Resetear flags de emergencia
        self.status.emergency_active = False
        
        # Limpiar cola de solicitudes pendientes
        with self._lock:
            self.request_queue.clear()
            self.status.pending_requests = 0
        
        # Volver a estado idle tras recuperación
        self.create_timer(2.0, lambda: self._transition_to_state(RobotState.IDLE))
        
        return True
    
    # =========================================================================
    # MÁQUINA DE ESTADOS - Gestión de transiciones
    # =========================================================================
    
    def _transition_to_state(self, new_state: RobotState) -> bool:
        """Ejecutar transición de estado con validación"""
        current_state = self.status.current_state
        
        # Validar transición
        if new_state not in self.valid_transitions.get(current_state, []):
            self.get_logger().warn(
                f'❌ Transición inválida: {current_state.name} -> {new_state.name}'
            )
            return False
        
        # Ejecutar transición
        old_state = current_state
        self.status.current_state = new_state
        self.status.state_entry_time = time.time()
        
        self.get_logger().info(f'🔄 Estado: {old_state.name} -> {new_state.name}')
        
        # Ejecutar acciones de entrada al estado
        self._on_state_entry(new_state, old_state)
        
        return True
    
    def _on_state_entry(self, new_state: RobotState, old_state: RobotState):
        """Ejecutar acciones al entrar a un nuevo estado"""
        if new_state == RobotState.IDLE:
            # Limpiar todas las flags de control
            self.status.voice_control_active = False
            self.status.manual_override_active = False
            self.status.exploration_active = False
            
        elif new_state == RobotState.EXPLORING_AUTO:
            # Asegurarse de que no hay override activo
            self.status.manual_override_active = False
            self.status.voice_control_active = False
            
        elif new_state == RobotState.EMERGENCY_STOP:
            # Activar flag de emergencia
            self.status.emergency_active = True
            
        elif new_state == RobotState.VOICE_CONTROL:
            # Activar flags de control por voz
            self.status.voice_control_active = True
            self.status.manual_override_active = True
    
    # =========================================================================
    # UTILIDADES Y MANTENIMIENTO
    # =========================================================================
    
    def _cleanup_expired_requests(self):
        """Limpiar solicitudes expiradas"""
        current_time = time.time()
        with self._lock:
            original_count = len(self.request_queue)
            self.request_queue = deque(
                (req for req in self.request_queue 
                 if current_time - req.timestamp < req.timeout),
                maxlen=self.request_queue.maxlen
            )
            self.status.pending_requests = len(self.request_queue)
            
            cleaned = original_count - len(self.request_queue)
            if cleaned > 0:
                self.get_logger().info(f'🧹 Limpiadas {cleaned} solicitudes expiradas')
    
    def _publish_status(self):
        """Publicar estado del sistema periódicamente"""
        status_data = self.get_system_status()
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.system_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        coordinator = RobotControlCoordinator()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        print('🔄 Cerrando Robot Control Coordinator...')
    except Exception as e:
        print(f'❌ Error crítico en coordinador: {e}')
    finally:
        try:
            coordinator.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
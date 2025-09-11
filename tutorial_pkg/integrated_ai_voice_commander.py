#!/usr/bin/env python3
"""
AI Voice Commander - VERSIÓN INTEGRADA CON COORDINADOR
Envía comandos al coordinador central en lugar de publicar directamente a /cmd_vel
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/integrated_ai_voice_commander.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
import time
import json
from typing import Dict, List, Tuple, Optional


class IntegratedAIVoiceCommander(Node):
    def __init__(self):
        super().__init__('integrated_ai_voice_commander')
        
        # =====================================================================
        # CONFIGURACIÓN DE PARÁMETROS
        # =====================================================================
        self.declare_parameter('linear_speed_default', 0.2)
        self.declare_parameter('angular_speed_default', 0.3)
        self.declare_parameter('enable_ai_responses', True)
        self.declare_parameter('log_voice_commands', True)
        self.declare_parameter('command_timeout', 10.0)
        
        try:
            self.linear_speed = self.get_parameter('linear_speed_default').value
            self.angular_speed = self.get_parameter('angular_speed_default').value
            self.ai_integration_active = self.get_parameter('enable_ai_responses').value
            self.log_commands = self.get_parameter('log_voice_commands').value
            self.command_timeout = self.get_parameter('command_timeout').value
        except Exception as e:
            self.get_logger().warn(f"Error obteniendo parámetros: {e}")
            self.linear_speed = 0.2
            self.angular_speed = 0.3
            self.ai_integration_active = True
            self.log_commands = True
            self.command_timeout = 10.0
        
        # =====================================================================
        # ESTADO DEL SISTEMA
        # =====================================================================
        self.coordinator_state = {
            'current_state': 'UNKNOWN',
            'exploration_active': False,
            'voice_control_active': False,
            'emergency_active': False
        }
        
        self.command_stats = {
            'total_commands': 0,
            'successful_commands': 0,
            'failed_commands': 0,
            'coordinator_rejections': 0,
            'session_start': time.time()
        }
        
        # =====================================================================
        # PUBLISHERS - Comunicación con coordinador
        # =====================================================================
        # EN LUGAR de /cmd_vel, usamos /voice_commands que el coordinador maneja
        self.voice_commands_pub = self.create_publisher(String, '/voice_commands', 10)
        self.feedback_pub = self.create_publisher(String, '/voice_feedback', 10)
        
        # =====================================================================
        # SUBSCRIBERS
        # =====================================================================
        # Comandos de voz entrantes (desde Flask, Whisper, etc.)
        self.voice_input_sub = self.create_subscription(
            String, '/raw_voice_input', self._voice_input_callback, 10)
        
        # Estado del coordinador para tomar decisiones inteligentes
        self.coordinator_status_sub = self.create_subscription(
            String, '/coordinator_status', self._coordinator_status_callback, 10)
        
        # Feedback de control para monitorear respuestas
        self.control_feedback_sub = self.create_subscription(
            String, '/control_feedback', self._control_feedback_callback, 10)
        
        # =====================================================================
        # MAPEO DE COMANDOS INTELIGENTE
        # =====================================================================
        self.command_patterns = {
            # Comandos de movimiento
            r'\b(adelante|avanza|forward|hacia adelante)\b': {
                'command': 'adelante',
                'type': 'movement',
                'priority': 'medium'
            },
            r'\b(atras|atrás|retrocede|backward|hacia atrás)\b': {
                'command': 'atras',
                'type': 'movement', 
                'priority': 'medium'
            },
            r'\b(izquierda|left|gira izquierda|voltea izquierda)\b': {
                'command': 'izquierda',
                'type': 'movement',
                'priority': 'medium'
            },
            r'\b(derecha|right|gira derecha|voltea derecha)\b': {
                'command': 'derecha',
                'type': 'movement',
                'priority': 'medium'
            },
            
            # Comandos de control
            r'\b(para|parar|stop|detente|alto|halt)\b': {
                'command': 'parar',
                'type': 'control',
                'priority': 'high'
            },
            r'\b(emergencia|emergency|ayuda crítica|parada de emergencia)\b': {
                'command': 'emergencia',
                'type': 'emergency',
                'priority': 'critical'
            },
            
            # Comandos de exploración
            r'\b(explorar|mapear|inicia exploración|empezar a explorar)\b': {
                'command': 'explorar',
                'type': 'exploration',
                'priority': 'medium'
            },
            r'\b(pausar exploración|pausa el mapeo|detener exploración)\b': {
                'command': 'pausar exploracion',
                'type': 'exploration',
                'priority': 'medium'
            },
            r'\b(continuar exploración|reanuda exploración|sigue explorando)\b': {
                'command': 'continuar',
                'type': 'exploration',
                'priority': 'medium'
            },
            r'\b(terminar exploración|finalizar exploración|acabar mapeo)\b': {
                'command': 'terminar exploracion',
                'type': 'exploration',
                'priority': 'medium'
            },
            
            # Comandos de velocidad
            r'\b(más rápido|rapido|acelera|aumenta velocidad)\b': {
                'command': 'rapido',
                'type': 'speed',
                'priority': 'low'
            },
            r'\b(más lento|lento|despacio|reduce velocidad)\b': {
                'command': 'lento',
                'type': 'speed',
                'priority': 'low'
            },
            
            # Comandos de estado
            r'\b(estado|status|cómo estás|información|reporte)\b': {
                'command': 'estado',
                'type': 'query',
                'priority': 'low'
            }
        }
        
        self.get_logger().info('🎤 Integrated AI Voice Commander iniciado')
        self.get_logger().info(f'   - Integrado con coordinador central')
        self.get_logger().info(f'   - IA habilitada: {self.ai_integration_active}')
        self.get_logger().info('👂 Escuchando comandos en /raw_voice_input')
    
    # =========================================================================
    # CALLBACKS PRINCIPALES
    # =========================================================================
    
    def _voice_input_callback(self, msg: String):
        """Procesar comando de voz crudo y enviarlo al coordinador"""
        command_text = msg.data.lower().strip()
        
        if not command_text:
            return
        
        if self.log_commands:
            self.get_logger().info(f'🎤 Procesando: "{command_text}"')
        
        self.command_stats['total_commands'] += 1
        
        # Analizar y procesar comando
        processing_result = self._process_voice_command(command_text)
        
        if processing_result['success']:
            self.command_stats['successful_commands'] += 1
        else:
            self.command_stats['failed_commands'] += 1
        
        # Enviar feedback
        self._send_feedback(processing_result)
    
    def _coordinator_status_callback(self, msg: String):
        """Actualizar estado conocido del coordinador"""
        try:
            status_data = json.loads(msg.data)
            self.coordinator_state.update({
                'current_state': status_data.get('state', 'UNKNOWN'),
                'exploration_active': status_data.get('exploration_active', False),
                'voice_control_active': status_data.get('voice_control_active', False),
                'emergency_active': status_data.get('emergency_active', False)
            })
        except Exception as e:
            self.get_logger().error(f'Error procesando estado coordinador: {e}')
    
    def _control_feedback_callback(self, msg: String):
        """Monitorear feedback de control para detectar rechazos"""
        feedback = msg.data.lower()
        if 'rechazado' in feedback or 'rejected' in feedback or 'invalid' in feedback:
            self.command_stats['coordinator_rejections'] += 1
            self.get_logger().warn('❌ Comando rechazado por coordinador')
    
    # =========================================================================
    # PROCESAMIENTO DE COMANDOS
    # =========================================================================
    
    def _process_voice_command(self, command_text: str) -> Dict:
        """Procesar comando de voz y enviarlo al coordinador"""
        
        # 1. Extraer comando del texto natural
        extracted_command = self._extract_command(command_text)
        
        if not extracted_command:
            return {
                'success': False,
                'error': 'Comando no reconocido',
                'original_text': command_text,
                'suggestions': self._generate_suggestions(command_text)
            }
        
        # 2. Validar comando según estado del coordinador
        validation_result = self._validate_command(extracted_command)
        
        if not validation_result['valid']:
            return {
                'success': False,
                'error': validation_result['reason'],
                'original_text': command_text,
                'extracted_command': extracted_command['command'],
                'coordinator_state': self.coordinator_state['current_state']
            }
        
        # 3. Enviar comando al coordinador
        send_result = self._send_to_coordinator(extracted_command)
        
        return {
            'success': send_result,
            'original_text': command_text,
            'extracted_command': extracted_command['command'],
            'command_type': extracted_command['type'],
            'priority': extracted_command['priority'],
            'coordinator_state': self.coordinator_state['current_state']
        }
    
    def _extract_command(self, text: str) -> Optional[Dict]:
        """Extraer comando específico del lenguaje natural"""
        text_lower = text.lower().strip()
        
        # Buscar patrones y encontrar mejor coincidencia
        best_match = None
        highest_confidence = 0.0
        
        for pattern, command_info in self.command_patterns.items():
            match = re.search(pattern, text_lower, re.IGNORECASE)
            if match:
                # Calcular confianza basada en la longitud del match
                match_length = len(match.group())
                text_length = len(text_lower)
                confidence = min(0.95, (match_length / text_length) + 0.4)
                
                if confidence > highest_confidence:
                    highest_confidence = confidence
                    best_match = {
                        'command': command_info['command'],
                        'type': command_info['type'],
                        'priority': command_info['priority'],
                        'confidence': confidence,
                        'pattern_matched': pattern
                    }
        
        return best_match if highest_confidence > 0.5 else None
    
    def _validate_command(self, extracted_command: Dict) -> Dict:
        """Validar si el comando es apropiado para el estado actual"""
        command_type = extracted_command['type']
        current_state = self.coordinator_state['current_state']
        
        # Comandos de emergencia siempre son válidos
        if command_type == 'emergency':
            return {'valid': True}
        
        # Validaciones por estado del coordinador
        if current_state == 'EMERGENCY_STOP':
            return {
                'valid': False,
                'reason': 'Sistema en emergencia - solo comandos de recuperación permitidos'
            }
        
        # Validaciones por tipo de comando
        if command_type == 'movement':
            # Movimientos no permitidos durante exploración automática activa
            if (self.coordinator_state.get('exploration_active', False) and 
                not self.coordinator_state.get('voice_control_active', False)):
                return {
                    'valid': False,
                    'reason': 'Exploración automática activa - use comandos de pausa primero'
                }
        
        elif command_type == 'exploration':
            command = extracted_command['command']
            if command in ['explorar', 'continuar'] and current_state in ['MANUAL_CONTROL', 'VOICE_CONTROL']:
                return {
                    'valid': False,
                    'reason': 'Control manual activo - detenga el control manual primero'
                }
        
        return {'valid': True}
    
    def _send_to_coordinator(self, extracted_command: Dict) -> bool:
        """Enviar comando al coordinador central"""
        try:
            # El coordinador espera comandos en /voice_commands
            command_msg = String()
            command_msg.data = extracted_command['command']
            
            self.voice_commands_pub.publish(command_msg)
            
            self.get_logger().info(
                f'📤 Enviado al coordinador: "{extracted_command["command"]}" '
                f'(tipo: {extracted_command["type"]}, prioridad: {extracted_command["priority"]})'
            )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error enviando comando al coordinador: {e}')
            return False
    
    def _generate_suggestions(self, text: str) -> List[str]:
        """Generar sugerencias para comandos no reconocidos"""
        suggestions = []
        text_lower = text.lower()
        
        # Sugerencias basadas en palabras clave
        if any(word in text_lower for word in ['mov', 'ir', 'caminar']):
            suggestions.append('Intenta: "adelante", "atrás", "izquierda", "derecha"')
        
        if any(word in text_lower for word in ['map', 'explor', 'buscar']):
            suggestions.append('Intenta: "explorar", "pausar exploración", "continuar"')
        
        if any(word in text_lower for word in ['vel', 'rapid', 'lent']):
            suggestions.append('Intenta: "más rápido", "más lento"')
        
        if any(word in text_lower for word in ['par', 'det', 'stop']):
            suggestions.append('Intenta: "parar", "detente"')
        
        if not suggestions:
            suggestions.append('Comandos disponibles: adelante, atrás, parar, explorar, estado')
        
        return suggestions
    
    def _send_feedback(self, result: Dict):
        """Enviar feedback sobre el procesamiento del comando"""
        try:
            if result['success']:
                feedback_text = f'Comando "{result["extracted_command"]}" procesado correctamente'
                
                # Agregar información contextual
                if result.get('command_type') == 'movement':
                    feedback_text += ' - Iniciando movimiento'
                elif result.get('command_type') == 'exploration':
                    feedback_text += ' - Controlando exploración'
                elif result.get('command_type') == 'emergency':
                    feedback_text += ' - EMERGENCIA ACTIVADA'
            else:
                feedback_text = f'Error: {result["error"]}'
                
                # Agregar sugerencias si están disponibles
                if 'suggestions' in result:
                    feedback_text += '. ' + '. '.join(result['suggestions'])
            
            # Publicar feedback
            feedback_msg = String()
            feedback_msg.data = f'[{time.strftime("%H:%M:%S")}] {feedback_text}'
            self.feedback_pub.publish(feedback_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error enviando feedback: {e}')
    
    # =========================================================================
    # COMANDOS ESPECIALES Y UTILIDADES
    # =========================================================================
    
    def get_command_stats(self) -> Dict:
        """Obtener estadísticas de comandos procesados"""
        total = self.command_stats['total_commands']
        success_rate = (self.command_stats['successful_commands'] / max(1, total)) * 100
        rejection_rate = (self.command_stats['coordinator_rejections'] / max(1, total)) * 100
        
        return {
            'total_commands': total,
            'successful_commands': self.command_stats['successful_commands'],
            'failed_commands': self.command_stats['failed_commands'],
            'coordinator_rejections': self.command_stats['coordinator_rejections'],
            'success_rate': round(success_rate, 1),
            'rejection_rate': round(rejection_rate, 1),
            'session_duration': time.time() - self.command_stats['session_start']
        }
    
    def _handle_status_request(self):
        """Manejar solicitud de estado"""
        stats = self.get_command_stats()
        coordinator_state = self.coordinator_state['current_state']
        
        status_msg = (
            f'Estado del sistema: {coordinator_state}, '
            f'Comandos procesados: {stats["successful_commands"]}/{stats["total_commands"]}, '
            f'Tasa de éxito: {stats["success_rate"]}%'
        )
        
        feedback_msg = String()
        feedback_msg.data = f'[{time.strftime("%H:%M:%S")}] {status_msg}'
        self.feedback_pub.publish(feedback_msg)
        
        self.get_logger().info(f'Estado reportado: {status_msg}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        ai_voice_commander = IntegratedAIVoiceCommander()
        rclpy.spin(ai_voice_commander)
    except KeyboardInterrupt:
        print('Cerrando Integrated AI Voice Commander...')
    except Exception as e:
        print(f'Error crítico: {e}')
    finally:
        try:
            ai_voice_commander.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
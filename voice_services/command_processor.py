#!/usr/bin/env python3
"""
Procesador de comandos para tutorial_pkg
"""
import logging
import re
from typing import Dict, Optional

class CommandProcessor:
    def __init__(self, gemini_service=None, ros_bridge=None):
        self.gemini_service = gemini_service
        self.ros_bridge = ros_bridge
        self.logger = logging.getLogger(__name__)
    
    def process_voice_input(self, user_input: str) -> Dict:
        """Procesar entrada de voz y generar respuesta"""
        try:
            # Clasificar tipo de comando
            intent = self._classify_intent(user_input)
            
            # Generar respuesta según el intent
            if intent == "robot_command":
                return self._handle_robot_command(user_input)
            elif intent == "conversation":
                return self._handle_conversation(user_input)
            else:
                return self._handle_unknown(user_input)
        
        except Exception as e:
            self.logger.error(f"Error procesando comando: {e}")
            return {
                'ai_response': 'Error procesando comando',
                'type': 'error',
                'confidence': 0.0,
                'ros_sent': False
            }
    
    def _classify_intent(self, text: str) -> str:
        """Clasificar intención del usuario"""
        text_lower = text.lower()
        
        # Comandos de robot
        robot_patterns = [
            r'\b(adelante|avanzar|forward|move)\b',
            r'\b(atras|atrás|backward|retroceder)\b',
            r'\b(izquierda|derecha|left|right|girar|turn)\b',
            r'\b(parar|stop|detener|halt)\b',
            r'\b(explorar|mapear|explore|mapping)\b'
        ]
        
        for pattern in robot_patterns:
            if re.search(pattern, text_lower):
                return "robot_command"
        
        # Conversación
        conv_patterns = [
            r'\b(hola|hello|hi|buenos)\b',
            r'\b(gracias|thank|thanks)\b',
            r'\b(como|how|que tal)\b'
        ]
        
        for pattern in conv_patterns:
            if re.search(pattern, text_lower):
                return "conversation"
        
        return "unknown"
    
    def _handle_robot_command(self, text: str) -> Dict:
        """Manejar comando de robot"""
        response = "Comando de robot procesado"
        if self.gemini_service:
            try:
                response = self.gemini_service.generate_response(
                    text, {"type": "robot_command"}
                )
            except:
                pass
        
        return {
            'ai_response': response,
            'type': 'robot_command',
            'confidence': 0.8,
            'ros_sent': True
        }
    
    def _handle_conversation(self, text: str) -> Dict:
        """Manejar conversación"""
        response = "¡Hola! Soy tu robot explorador TurtleBot3"
        if self.gemini_service:
            try:
                response = self.gemini_service.generate_response(
                    text, {"type": "conversation"}
                )
            except:
                pass
        
        return {
            'ai_response': response,
            'type': 'conversation',
            'confidence': 0.9,
            'ros_sent': False
        }
    
    def _handle_unknown(self, text: str) -> Dict:
        """Manejar comando desconocido"""
        return {
            'ai_response': 'No entendí ese comando. Prueba: avanzar, girar, parar, explorar',
            'type': 'unknown',
            'confidence': 0.3,
            'ros_sent': False
        }

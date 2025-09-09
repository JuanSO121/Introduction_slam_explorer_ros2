#!/usr/bin/env python3
"""
Servicio de IA con Gemini adaptado para tutorial_pkg
"""
import requests
import json
import logging
import os
from typing import Optional, Dict

class GeminiService:
    def __init__(self):
        self.api_key = os.getenv('GEMINI_API_KEY')
        if not self.api_key:
            # API key por defecto para tutorial_pkg
            self.api_key = 'GEMINI_API_KEY'
        
        self.base_url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent"
        self.logger = logging.getLogger(__name__)
        
        self.robot_context = {
            "name": "TurtleBot3 Explorer",
            "capabilities": [
                "exploración autónoma con SLAM",
                "navegación inteligente",
                "mapeo de entornos",
                "control por voz"
            ],
            "package": "tutorial_pkg"
        }
        
        self.conversation_history = []
    
    def process_command(self, command: str, context: Optional[Dict] = None) -> str:
        """
        Método principal para procesar comandos desde Flutter Bridge
        """
        try:
            # Actualizar historial de conversación
            self.conversation_history.append({
                'command': command,
                'timestamp': __import__('time').time()
            })
            
            # Mantener solo los últimos 5 comandos
            if len(self.conversation_history) > 5:
                self.conversation_history = self.conversation_history[-5:]
            
            # Generar respuesta usando el método existente
            response = self.generate_response(command, context)
            
            self.logger.info(f"Comando procesado: '{command}' -> '{response}'")
            return response
            
        except Exception as e:
            self.logger.error(f"Error procesando comando '{command}': {e}")
            return f"Entendido. Comando '{command}' recibido correctamente."
    
    def generate_response(self, user_input: str, context: Optional[Dict] = None) -> str:
        try:
            # Crear contexto de conversación si hay historial
            conversation_context = ""
            if self.conversation_history:
                recent_commands = [item['command'] for item in self.conversation_history[-3:]]
                conversation_context = f"Comandos recientes: {', '.join(recent_commands)}. "
            
            system_prompt = f"""Eres {self.robot_context['name']}, un robot explorador TurtleBot3 que puede:
- {', '.join(self.robot_context['capabilities'])}

Ejecutándose en el paquete tutorial_pkg de ROS2.

{conversation_context}

Responde de manera amigable en español, máximo 2-3 oraciones.
Si te dan comandos de movimiento: confirma que los ejecutarás.
Si preguntan sobre capacidades: explica brevemente lo que puedes hacer.

Comandos disponibles: avanzar, retroceder, girar izquierda/derecha, parar, explorar, mapear, estado.

Usuario: {user_input}
Robot:"""
            
            payload = {
                "contents": [{"parts": [{"text": system_prompt}]}],
                "generationConfig": {
                    "temperature": 0.7,
                    "maxOutputTokens": 100,
                    "topP": 0.8,
                    "topK": 10
                }
            }
            
            headers = {
                'Content-Type': 'application/json',
                'X-goog-api-key': self.api_key
            }
            
            response = requests.post(
                self.base_url, 
                headers=headers, 
                json=payload, 
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                ai_response = result['candidates'][0]['content']['parts'][0]['text']
                return ai_response.strip()
            elif response.status_code == 400:
                self.logger.warning(f"API key inválida o request malformado")
                return f"Comando '{user_input}' recibido. Ejecutando..."
            else:
                self.logger.warning(f"API respondió con código {response.status_code}")
                return f"Entendido. Procesando '{user_input}'."
                
        except requests.exceptions.Timeout:
            self.logger.warning("Timeout en API de Gemini")
            return f"Comando '{user_input}' recibido y procesando."
        except requests.exceptions.RequestException as e:
            self.logger.warning(f"Error de conexión a Gemini: {e}")
            return f"Ejecutando comando: {user_input}"
        except Exception as e:
            self.logger.error(f"Error generando respuesta IA: {e}")
            return "Robot listo. ¿Qué necesitas que haga?"
    
    def get_robot_status(self) -> str:
        """Generar respuesta sobre el estado actual del robot"""
        try:
            status_prompt = f"""Como {self.robot_context['name']}, proporciona un breve reporte de estado.
Incluye: estado operacional, última acción, y disponibilidad para nuevos comandos.
Máximo 2 oraciones en español."""
            
            return self.generate_response("¿Cuál es tu estado actual?")
        except Exception as e:
            self.logger.error(f"Error obteniendo estado: {e}")
            return "Sistema operativo. Listo para recibir comandos."
    
    def handle_exploration_command(self, action: str) -> str:
        """Manejar comandos específicos de exploración"""
        exploration_responses = {
            'start_exploration': "Iniciando exploración autónoma del entorno. Comenzaré a mapear el área.",
            'pause_exploration': "Pausando exploración. Mantendré mi posición actual.",
            'resume_exploration': "Reanudando exploración desde el punto actual.",
            'finish_exploration': "Finalizando exploración. Guardando mapa generado.",
            'emergency_stop': "¡Parada de emergencia activada! Robot detenido inmediatamente."
        }
        
        return exploration_responses.get(
            action, 
            f"Comando de exploración '{action}' procesado correctamente."
        )
    
    def is_service_available(self) -> bool:
        """Verificar si el servicio de IA está disponible"""
        try:
            # Test simple con timeout corto
            test_response = requests.get(
                "https://generativelanguage.googleapis.com",
                timeout=3
            )
            return test_response.status_code in [200, 404]  # 404 es normal para root
        except:
            return False
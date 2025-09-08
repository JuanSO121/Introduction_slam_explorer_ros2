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
            self.api_key = 'AIzaSyABBoS19JdslBti21hcRT3tT7ASrc8EWsE'
        
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
    
    def generate_response(self, user_input: str, context: Optional[Dict] = None) -> str:
        try:
            system_prompt = f"""Eres {self.robot_context['name']}, un robot explorador TurtleBot3 que puede:
- {', '.join(self.robot_context['capabilities'])}

Ejecutándose en el paquete tutorial_pkg de ROS2.

Responde de manera amigable en español, máximo 2-3 oraciones.
Si te preguntan sobre comandos, puedes: avanzar, retroceder, girar, parar, explorar, mapear.

Usuario: {user_input}
Robot:"""
            
            payload = {
                "contents": [{"parts": [{"text": system_prompt}]}]
            }
            
            headers = {
                'Content-Type': 'application/json',
                'X-goog-api-key': self.api_key
            }
            
            response = requests.post(self.base_url, headers=headers, json=payload, timeout=10)
            
            if response.status_code == 200:
                result = response.json()
                ai_response = result['candidates'][0]['content']['parts'][0]['text']
                return ai_response.strip()
            else:
                return "Lo siento, tengo problemas para procesar tu mensaje."
                
        except Exception as e:
            self.logger.error(f"Error generando respuesta IA: {e}")
            return "Disculpa, algo salió mal con la IA."

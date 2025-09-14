#!/usr/bin/env python3
"""
Servicio FastAPI para recibir transcripciones de voz y publicarlas a ROS2
Ubicación: ~/ros2_ws/src/tutorial_pkg/voice_services/fastapi_voice_service.py
Integración directa con el sistema tutorial_pkg existente
"""

import asyncio
import logging
import time
import json
from pathlib import Path
from typing import Optional, Dict, Any
import uvicorn
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import threading

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String


class VoiceTranscription(BaseModel):
    """Modelo para recibir transcripciones de voz"""
    transcription: str
    confidence: Optional[float] = None
    language: Optional[str] = "es"
    timestamp: Optional[float] = None
    device_id: Optional[str] = None


class TextCommand(BaseModel):
    """Modelo para comandos de texto directo"""
    command: str
    source: Optional[str] = "flutter_app"
    timestamp: Optional[float] = None


class HealthResponse(BaseModel):
    """Modelo para respuesta de salud"""
    status: str
    services: Dict[str, bool]
    ros_node_active: bool
    uptime: float


class VoiceROSPublisher(Node):
    """Nodo ROS2 para publicar comandos de voz al sistema existente"""
    
    def __init__(self):
        super().__init__('fastapi_voice_publisher')
        
        # Publicador principal que se integra con el sistema existente
        self.voice_command_publisher = self.create_publisher(
            String, 
            '/voice_commands',  # Topic que ya maneja tu sistema
            10
        )
        
        # Publicador de contexto para IA
        self.ai_context_publisher = self.create_publisher(
            String,
            '/ai_context',
            10
        )
        
        # Suscriptor para respuestas del sistema
        self.voice_feedback_subscription = self.create_subscription(
            String,
            '/voice_feedback',
            self.voice_feedback_callback,
            10
        )
        
        self.ai_status_subscription = self.create_subscription(
            String,
            '/ai_status', 
            self.ai_status_callback,
            10
        )
        
        # Estado interno
        self.last_response = ""
        self.last_ai_status = ""
        self.commands_sent = 0
        self.start_time = time.time()
        self.is_ready = True
        
        self.get_logger().info('FastAPI Voice Publisher iniciado - integrado con tutorial_pkg')
    
    def publish_voice_command(self, transcription: str, metadata: Dict[str, Any] = None):
        """Publicar comando de voz al sistema ROS2 existente"""
        try:
            # Crear mensaje con metadatos
            command_data = {
                'command': transcription.strip(),
                'source': 'fastapi_voice_service',
                'timestamp': time.time(),
                'confidence': metadata.get('confidence', 0.0) if metadata else 0.0,
                'language': metadata.get('language', 'es') if metadata else 'es',
                'device_id': metadata.get('device_id', 'unknown') if metadata else 'unknown'
            }
            
            # Publicar al topic principal
            msg = String()
            msg.data = json.dumps(command_data)
            self.voice_command_publisher.publish(msg)
            
            # Publicar contexto para IA
            ai_context = {
                'type': 'voice_command',
                'content': transcription,
                'metadata': metadata or {}
            }
            context_msg = String()
            context_msg.data = json.dumps(ai_context)
            self.ai_context_publisher.publish(context_msg)
            
            self.commands_sent += 1
            self.get_logger().info(f'Comando publicado: "{transcription}" (Total: {self.commands_sent})')
            
        except Exception as e:
            self.get_logger().error(f'Error publicando comando: {e}')
            raise
    
    def voice_feedback_callback(self, msg: String):
        """Recibir respuestas del sistema de voz existente"""
        self.last_response = msg.data
        self.get_logger().debug(f'Respuesta recibida: {msg.data[:100]}...')
    
    def ai_status_callback(self, msg: String):
        """Recibir estado del sistema IA"""
        self.last_ai_status = msg.data
        self.get_logger().debug('Estado IA actualizado')
    
    def get_system_status(self) -> Dict[str, Any]:
        """Obtener estado del sistema"""
        return {
            'node_active': self.is_ready,
            'commands_sent': self.commands_sent,
            'last_response': self.last_response,
            'last_ai_status': self.last_ai_status,
            'uptime': time.time() - self.start_time
        }


# Instancia global del nodo ROS
ros_node: Optional[VoiceROSPublisher] = None
ros_executor: Optional[MultiThreadedExecutor] = None
ros_thread: Optional[threading.Thread] = None

def init_ros():
    """Inicializar ROS2 en thread separado"""
    global ros_node, ros_executor, ros_thread
    
    try:
        rclpy.init()
        ros_node = VoiceROSPublisher()
        ros_executor = MultiThreadedExecutor()
        ros_executor.add_node(ros_node)
        
        def spin_ros():
            try:
                ros_executor.spin()
            except Exception as e:
                print(f"Error en ROS thread: {e}")
        
        ros_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_thread.start()
        
        print("ROS2 inicializado correctamente")
        return True
        
    except Exception as e:
        print(f"Error inicializando ROS2: {e}")
        return False

def shutdown_ros():
    """Cerrar ROS2 limpiamente"""
    global ros_node, ros_executor, ros_thread
    
    try:
        if ros_executor:
            ros_executor.shutdown()
        if ros_node:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS2 cerrado correctamente")
    except Exception as e:
        print(f"Error cerrando ROS2: {e}")


# Crear aplicación FastAPI
app = FastAPI(
    title="Tutorial PKG Voice Service",
    description="Servicio de voz integrado con ROS2 para tutorial_pkg",
    version="1.0.0"
)

# Configurar CORS para Flutter
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # En producción, especificar origen de Flutter
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Variables globales
service_start_time = time.time()


@app.on_event("startup")
async def startup_event():
    """Inicializar servicios al arrancar"""
    print("Iniciando FastAPI Voice Service para tutorial_pkg...")
    success = init_ros()
    if not success:
        print("ADVERTENCIA: ROS2 no se pudo inicializar")


@app.on_event("shutdown") 
async def shutdown_event():
    """Limpiar al cerrar"""
    print("Cerrando FastAPI Voice Service...")
    shutdown_ros()


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Endpoint de salud del servicio"""
    global ros_node
    
    ros_active = ros_node is not None and ros_node.is_ready
    
    return HealthResponse(
        status="healthy" if ros_active else "degraded",
        services={
            "fastapi": True,
            "ros2_node": ros_active,
            "voice_publisher": ros_active
        },
        ros_node_active=ros_active,
        uptime=time.time() - service_start_time
    )


@app.post("/process_transcription")
async def process_voice_transcription(transcription: VoiceTranscription):
    """
    Procesar transcripción de voz y publicarla a ROS2
    Este endpoint reemplaza el procesamiento de audio, ya que Flutter hace la transcripción
    """
    global ros_node
    
    if not ros_node or not ros_node.is_ready:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    
    if not transcription.transcription or len(transcription.transcription.strip()) < 2:
        raise HTTPException(status_code=400, detail="Transcripción vacía o muy corta")
    
    try:
        # Preparar metadatos
        metadata = {
            'confidence': transcription.confidence,
            'language': transcription.language, 
            'timestamp': transcription.timestamp or time.time(),
            'device_id': transcription.device_id
        }
        
        # Publicar al sistema ROS2
        ros_node.publish_voice_command(transcription.transcription, metadata)
        
        # Esperar un momento para la respuesta
        await asyncio.sleep(0.1)
        
        # Obtener estado del sistema
        system_status = ros_node.get_system_status()
        
        return JSONResponse({
            "success": True,
            "transcription": transcription.transcription,
            "ai_response": system_status.get('last_response', 'Comando enviado al robot'),
            "command_type": "voice_transcription",
            "confidence": transcription.confidence,
            "robot_action": True,
            "timestamp": time.time(),
            "system_status": {
                "commands_processed": system_status['commands_sent'],
                "ros_active": system_status['node_active']
            }
        })
        
    except Exception as e:
        logging.error(f"Error procesando transcripción: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing transcription: {str(e)}")


@app.post("/send_text_command")
async def send_text_command(command: TextCommand):
    """
    Enviar comando de texto directo (mantiene compatibilidad con Flutter existente)
    """
    global ros_node
    
    if not ros_node or not ros_node.is_ready:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
        
    if not command.command or len(command.command.strip()) < 2:
        raise HTTPException(status_code=400, detail="Comando vacío")
    
    try:
        metadata = {
            'source': command.source,
            'timestamp': command.timestamp or time.time(),
            'confidence': 1.0,  # Texto tiene confianza máxima
            'language': 'es'
        }
        
        ros_node.publish_voice_command(command.command, metadata)
        
        await asyncio.sleep(0.1)
        
        system_status = ros_node.get_system_status()
        
        return JSONResponse({
            "success": True,
            "transcription": command.command,
            "ai_response": system_status.get('last_response', 'Comando de texto enviado'),
            "command_type": "text_command", 
            "confidence": 1.0,
            "robot_action": True,
            "timestamp": time.time()
        })
        
    except Exception as e:
        logging.error(f"Error enviando comando de texto: {e}")
        raise HTTPException(status_code=500, detail=f"Error sending text command: {str(e)}")


@app.get("/robot_status")
async def get_robot_status():
    """Obtener estado completo del robot"""
    global ros_node
    
    if not ros_node:
        return JSONResponse({
            "status": "ROS node not initialized",
            "active": False
        })
    
    try:
        system_status = ros_node.get_system_status()
        
        return JSONResponse({
            "status": "active" if system_status['node_active'] else "inactive",
            "active": system_status['node_active'],
            "commands_processed": system_status['commands_sent'],
            "last_response": system_status['last_response'],
            "uptime": system_status['uptime'],
            "service_info": {
                "name": "tutorial_pkg_voice_service",
                "version": "1.0.0",
                "type": "fastapi_ros2_bridge"
            }
        })
        
    except Exception as e:
        return JSONResponse({
            "status": f"error: {e}",
            "active": False
        })


@app.get("/")
async def root():
    """Endpoint raíz con información del servicio"""
    return {
        "service": "Tutorial PKG Voice Service",
        "status": "running",
        "version": "1.0.0",
        "description": "FastAPI service for voice command integration with ROS2 tutorial_pkg",
        "endpoints": {
            "/health": "Service health check",
            "/process_transcription": "Process voice transcription from Flutter app",
            "/send_text_command": "Send text command directly", 
            "/robot_status": "Get robot status"
        },
        "uptime": time.time() - service_start_time
    }


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="FastAPI Voice Service para tutorial_pkg")
    parser.add_argument("--host", default="0.0.0.0", help="Host address")
    parser.add_argument("--port", type=int, default=8000, help="Port number") 
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    
    args = parser.parse_args()
    
    # Configurar logging
    log_level = "debug" if args.debug else "info"
    
    print(f"""
    🚀 Iniciando FastAPI Voice Service para tutorial_pkg
    📡 Host: {args.host}:{args.port}
    🔧 Debug: {args.debug}
    🤖 Integración ROS2: tutorial_pkg
    """)
    
    try:
        uvicorn.run(
            "fastapi_voice_service:app",
            host=args.host,
            port=args.port,
            log_level=log_level,
            reload=args.debug,
            access_log=True
        )
    except KeyboardInterrupt:
        print("\n🛑 Servicio detenido por el usuario")
    except Exception as e:
        print(f"❌ Error ejecutando servicio: {e}")
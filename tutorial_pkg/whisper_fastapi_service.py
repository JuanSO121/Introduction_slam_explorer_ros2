#!/usr/bin/env python3
"""
Servicio FastAPI con Whisper para transcripción de voz - tutorial_pkg
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/whisper_fastapi_service.py

Sistema optimizado que recibe audio del móvil y publica directamente a ROS2
"""

import asyncio
import logging
import os
import tempfile
import time
from pathlib import Path
from typing import Optional
import uvicorn
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import torch
from transformers import WhisperProcessor, WhisperForConditionalGeneration
import librosa
import warnings

# Suprimir warnings de transformers
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=FutureWarning)
os.environ["TOKENIZERS_PARALLELISM"] = "false"

class WhisperROSNode(Node):
    """Nodo ROS2 que maneja la transcripción y publicación"""
    
    def __init__(self):
        super().__init__('whisper_transcription_service')
        
        # Configuración
        self.declare_parameter('whisper_model', 'openai/whisper-small')
        self.declare_parameter('voice_commands_topic', '/voice_commands')
        self.declare_parameter('transcription_feedback_topic', '/transcription_feedback')
        
        model_name = self.get_parameter('whisper_model').value
        voice_topic = self.get_parameter('voice_commands_topic').value
        feedback_topic = self.get_parameter('transcription_feedback_topic').value
        
        # Publishers
        self.voice_publisher = self.create_publisher(String, voice_topic, 10)
        self.feedback_publisher = self.create_publisher(String, feedback_topic, 10)
        
        # Whisper Service
        self.whisper_service = WhisperTranscriptionService(model_name)
        self.whisper_ready = False
        
        # Estadísticas
        self.transcription_count = 0
        self.error_count = 0
        self.startup_time = time.time()
        
        # Timer para estado
        self.create_timer(30.0, self.publish_status)
        
        self.get_logger().info('🎤 Whisper ROS Node iniciado')
        self.get_logger().info(f'📡 Publicando comandos en: {voice_topic}')
        self.get_logger().info(f'📡 Feedback en: {feedback_topic}')
        
        # Cargar modelo en hilo separado
        threading.Thread(target=self._load_whisper_async, daemon=True).start()
    
    def _load_whisper_async(self):
        """Cargar Whisper de forma asíncrona"""
        try:
            self.get_logger().info('🧠 Cargando modelo Whisper...')
            self.whisper_service.initialize()
            self.whisper_ready = True
            self.get_logger().info('✅ Whisper listo para transcripciones')
            
            # Notificar que el servicio está listo
            status_msg = String()
            status_msg.data = "whisper_ready"
            self.feedback_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando Whisper: {e}')
            self.whisper_ready = False
    
    async def process_audio(self, audio_file: UploadFile) -> dict:
        """Procesar archivo de audio y publicar transcripción"""
        if not self.whisper_ready:
            raise RuntimeError("Whisper no está listo aún")
        
        start_time = time.time()
        temp_file = None
        
        try:
            # Guardar archivo temporal
            with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as temp_file:
                content = await audio_file.read()
                temp_file.write(content)
                temp_path = temp_file.name
            
            # Transcribir
            transcription = await asyncio.get_event_loop().run_in_executor(
                None, self.whisper_service.transcribe_audio, temp_path
            )
            
            processing_time = time.time() - start_time
            
            if transcription and len(transcription.strip()) > 2:
                # Publicar comando a ROS2
                command_msg = String()
                command_msg.data = transcription.strip()
                self.voice_publisher.publish(command_msg)
                
                # Publicar feedback
                feedback_data = f"transcribed:{transcription.strip()}"
                feedback_msg = String()
                feedback_msg.data = feedback_data
                self.feedback_publisher.publish(feedback_msg)
                
                self.transcription_count += 1
                self.get_logger().info(f'🗣️ Transcrito: "{transcription.strip()}" ({processing_time:.2f}s)')
                
                return {
                    "success": True,
                    "transcription": transcription.strip(),
                    "processing_time": processing_time,
                    "confidence": 0.85,  # Placeholder
                    "published_to_ros": True
                }
            else:
                self.get_logger().warning('⚠️ Transcripción vacía o muy corta')
                return {
                    "success": False,
                    "error": "No se pudo transcribir el audio o transcripción muy corta",
                    "processing_time": processing_time
                }
                
        except Exception as e:
            self.error_count += 1
            error_msg = f"Error procesando audio: {str(e)}"
            self.get_logger().error(f'❌ {error_msg}')
            
            return {
                "success": False,
                "error": error_msg,
                "processing_time": time.time() - start_time
            }
        finally:
            # Limpiar archivo temporal
            if temp_file and os.path.exists(temp_path):
                try:
                    os.unlink(temp_path)
                except:
                    pass
    
    def publish_status(self):
        """Publicar estado del servicio"""
        uptime = time.time() - self.startup_time
        status_data = f"whisper_status:ready={self.whisper_ready},transcriptions={self.transcription_count},errors={self.error_count},uptime={uptime:.0f}s"
        
        status_msg = String()
        status_msg.data = status_data
        self.feedback_publisher.publish(status_msg)
        
        self.get_logger().debug(f'📊 Estado: {self.transcription_count} transcripciones, {self.error_count} errores')
    
    def get_stats(self) -> dict:
        """Obtener estadísticas del servicio"""
        return {
            "whisper_ready": self.whisper_ready,
            "transcription_count": self.transcription_count,
            "error_count": self.error_count,
            "uptime_seconds": time.time() - self.startup_time,
            "model_loaded": self.whisper_service.is_ready() if hasattr(self.whisper_service, 'is_ready') else False
        }


class WhisperTranscriptionService:
    """Servicio de transcripción usando Whisper optimizado"""
    
    def __init__(self, model_name: str = "openai/whisper-small"):
        self.model_name = model_name
        self.model = None
        self.processor = None
        self._ready = False
        self.device = "cpu"  # Forzar CPU para estabilidad
        self.logger = logging.getLogger(__name__)
    
    def initialize(self):
        """Inicializar modelo Whisper"""
        try:
            self.logger.info(f"Cargando {self.model_name}")
            
            # Cargar processor y model
            self.processor = WhisperProcessor.from_pretrained(self.model_name)
            self.model = WhisperForConditionalGeneration.from_pretrained(
                self.model_name,
                torch_dtype=torch.float32,
                low_cpu_mem_usage=True
            )
            
            self.model.to(self.device)
            self.model.eval()
            
            # Configurar para español
            self.forced_decoder_ids = self.processor.get_decoder_prompt_ids(
                language="spanish", task="transcribe"
            )
            
            self._ready = True
            self.logger.info("✅ Whisper inicializado correctamente")
            
        except Exception as e:
            self.logger.error(f"Error inicializando Whisper: {e}")
            self._ready = False
            raise
    
    def transcribe_audio(self, audio_path: str) -> str:
        """Transcribir archivo de audio"""
        if not self._ready:
            raise RuntimeError("Whisper no inicializado")
        
        try:
            # Cargar y procesar audio
            audio_array, _ = librosa.load(audio_path, sr=16000, duration=30)
            
            if len(audio_array) == 0:
                return ""
            
            # Normalizar audio
            if audio_array.max() > 0:
                audio_array = audio_array / audio_array.max() * 0.9
            
            # Transcribir
            with torch.no_grad():
                input_features = self.processor(
                    audio_array, 
                    sampling_rate=16000, 
                    return_tensors="pt"
                ).input_features.to(self.device)
                
                predicted_ids = self.model.generate(
                    input_features,
                    max_new_tokens=200,
                    do_sample=False,
                    forced_decoder_ids=self.forced_decoder_ids,
                    pad_token_id=self.processor.tokenizer.eos_token_id
                )
                
                transcription = self.processor.batch_decode(
                    predicted_ids, skip_special_tokens=True
                )[0]
            
            # Limpiar transcripción
            cleaned_transcription = " ".join(transcription.split()).strip()
            return cleaned_transcription
            
        except Exception as e:
            self.logger.error(f"Error transcribiendo: {e}")
            return ""
    
    def is_ready(self) -> bool:
        return self._ready


# Instancia global del nodo ROS
ros_node: Optional[WhisperROSNode] = None

# Crear aplicación FastAPI
app = FastAPI(
    title="Whisper Voice Transcription Service",
    description="Servicio de transcripción de voz integrado con ROS2 para tutorial_pkg",
    version="1.0.0"
)

# Configurar CORS para Flutter
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # En producción, especificar dominios exactos
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["*"],
)

@app.on_event("startup")
async def startup_event():
    """Inicializar ROS2 al arrancar FastAPI"""
    global ros_node
    
    # Inicializar ROS2 en hilo separado
    def init_ros():
        global ros_node
        rclpy.init()
        ros_node = WhisperROSNode()
        rclpy.spin(ros_node)
    
    ros_thread = threading.Thread(target=init_ros, daemon=True)
    ros_thread.start()
    
    # Esperar un poco para que ROS se inicialice
    await asyncio.sleep(1.0)

@app.on_event("shutdown")
async def shutdown_event():
    """Limpiar recursos al cerrar"""
    global ros_node
    if ros_node:
        ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

@app.get("/health")
async def health_check():
    """Endpoint de salud"""
    global ros_node
    
    if not ros_node:
        return JSONResponse(
            status_code=503,
            content={"status": "unhealthy", "reason": "ROS node not initialized"}
        )
    
    stats = ros_node.get_stats()
    return {
        "status": "healthy",
        "whisper_ready": stats["whisper_ready"],
        "ros_connected": True,
        "transcription_count": stats["transcription_count"],
        "uptime_seconds": stats["uptime_seconds"]
    }

@app.get("/status")
async def get_status():
    """Obtener estado detallado del servicio"""
    global ros_node
    
    if not ros_node:
        return {"error": "ROS node not initialized"}
    
    return ros_node.get_stats()

@app.post("/transcribe")
async def transcribe_audio(audio: UploadFile = File(...)):
    """
    Transcribir archivo de audio y publicar a ROS2
    
    - **audio**: Archivo de audio (WAV, MP3, etc.)
    
    Returns:
    - transcription: Texto transcrito
    - success: Si la transcripción fue exitosa
    - published_to_ros: Si se publicó al topic ROS2
    """
    global ros_node
    
    if not ros_node:
        raise HTTPException(status_code=503, detail="Servicio ROS no disponible")
    
    if not ros_node.whisper_ready:
        raise HTTPException(status_code=503, detail="Whisper no está listo aún")
    
    # Validar archivo
    if not audio.filename:
        raise HTTPException(status_code=400, detail="No se proporcionó archivo")
    
    # Validar tipo de archivo (opcional pero recomendado)
    allowed_types = ['audio/wav', 'audio/mpeg', 'audio/mp4', 'audio/webm', 'audio/ogg']
    if audio.content_type and audio.content_type not in allowed_types:
        # Log warning pero continuar (algunos clientes envían content_type incorrecto)
        ros_node.get_logger().warning(f"Tipo de archivo inusual: {audio.content_type}")
    
    try:
        result = await ros_node.process_audio(audio)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/")
async def root():
    """Información básica de la API"""
    return {
        "service": "Whisper Voice Transcription",
        "version": "1.0.0",
        "description": "Servicio de transcripción integrado con ROS2 tutorial_pkg",
        "endpoints": {
            "transcribe": "POST /transcribe - Transcribir archivo de audio",
            "health": "GET /health - Estado de salud del servicio",
            "status": "GET /status - Estado detallado"
        }
    }

def main():
    """Función principal para ejecutar el servicio"""
    # Configurar logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Configuración del servidor
    config = uvicorn.Config(
        app=app,
        host="0.0.0.0",  # Escuchar en todas las interfaces
        port=8000,       # Puerto estándar
        log_level="info",
        access_log=True,
        reload=False     # No recargar en producción
    )
    
    server = uvicorn.Server(config)
    
    print("🚀 Iniciando Whisper FastAPI Service")
    print("🎤 Transcripción de voz para tutorial_pkg")
    print("🌐 Servidor disponible en: http://0.0.0.0:8000")
    print("📡 Publicando comandos de voz a ROS2")
    print("💡 Endpoint principal: POST /transcribe")
    
    try:
        server.run()
    except KeyboardInterrupt:
        print("🛑 Cerrando servicio Whisper...")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Servicio FastAPI con Whisper para tutorial_pkg - Versión Thread-Safe
Soluciona el problema de "double free detected in tcache 2"
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/whisper_fastapi_service_cpu.py
"""

import os
import sys
import time
import tempfile
import uvicorn
import asyncio
import threading
from pathlib import Path
from typing import Optional, Dict, Any
import multiprocessing as mp

# FastAPI y componentes web
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import logging

# CONFIGURACIONES CRÍTICAS PARA EVITAR DOUBLE FREE
os.environ["TORCH_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["TOKENIZERS_PARALLELISM"] = "false"
os.environ["PYTHONUNBUFFERED"] = "1"

# Whisper para transcripción
try:
    import whisper
    import torch
    # FORZAR CPU SIEMPRE para evitar conflictos CUDA
    torch.set_num_threads(1)
    if hasattr(torch.backends, 'openmp'):
        torch.backends.openmp.is_available = lambda: False
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    whisper = None
    torch = None

# ROS2 para publicar comandos
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None
    Node = None
    String = None

# Audio processing
try:
    import librosa
    import soundfile as sf
    AUDIO_PROCESSING_AVAILABLE = True
except ImportError:
    AUDIO_PROCESSING_AVAILABLE = False
    librosa = None
    sf = None

# Análisis espectral mejorado
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    np = None

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# LOCK GLOBAL PARA OPERACIONES THREAD-SAFE
whisper_lock = threading.Lock()

class WhisperROS2Bridge(Node):
    """Nodo ROS2 para publicar comandos transcritos por Whisper"""
    
    def __init__(self):
        super().__init__('whisper_ros2_bridge')
        
        # Publicador para comandos de voz (conecta con el sistema existente)
        self.voice_commands_pub = self.create_publisher(
            String, '/voice_commands', 10)
        
        # Estadísticas
        self.commands_sent = 0
        self.start_time = time.time()
        
        self.get_logger().info('Bridge Whisper-ROS2 iniciado')
        self.get_logger().info('Publicando en /voice_commands')
    
    def publish_voice_command(self, command: str) -> bool:
        """Publicar comando de voz al sistema ROS2 existente"""
        try:
            msg = String()
            msg.data = command.strip()
            
            self.voice_commands_pub.publish(msg)
            self.commands_sent += 1
            
            self.get_logger().info(f'Comando enviado: "{command}"')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error publicando comando: {e}')
            return False
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtener estadísticas del bridge"""
        uptime = time.time() - self.start_time
        return {
            'commands_sent': self.commands_sent,
            'uptime_seconds': round(uptime, 1),
            'node_active': True
        }

class WhisperService:
    """Servicio de transcripción con Whisper optimizado y thread-safe"""
    
    def __init__(self):
        self.model = None
        self.model_name = "base"  # Modelo más liviano para evitar problemas
        self.device = "cpu"  # FORZAR CPU SIEMPRE
        
        # Variables para evitar race conditions
        self._model_loading = False
        self._transcription_in_progress = False
        
        self.load_model()
        
        # Estadísticas
        self.transcriptions_count = 0
        self.total_processing_time = 0.0
    
    def load_model(self):
        """Cargar modelo Whisper de forma thread-safe"""
        if not WHISPER_AVAILABLE:
            logger.error("Whisper no está disponible")
            return
        
        with whisper_lock:  # THREAD SAFETY CRÍTICO
            if self._model_loading:
                logger.warning("Modelo ya se está cargando...")
                return
                
            self._model_loading = True
            
            try:
                logger.info(f"Cargando Whisper modelo '{self.model_name}' en CPU")
                
                # CONFIGURACIÓN CRÍTICA: Cargar siempre en CPU
                self.model = whisper.load_model(self.model_name, device="cpu")
                
                # Verificar que está en CPU
                if hasattr(self.model, 'device'):
                    logger.info(f"Modelo cargado en dispositivo: {self.model.device}")
                
                logger.info("Modelo Whisper cargado exitosamente")
                
            except Exception as e:
                logger.error(f"Error cargando modelo Whisper: {e}")
                self.model = None
            finally:
                self._model_loading = False
    
    def transcribe_audio(self, audio_path: str) -> Dict[str, Any]:
        """Transcribir archivo de audio de forma thread-safe"""
        start_time = time.time()
        
        # VERIFICACIÓN THREAD-SAFE
        with whisper_lock:
            if self._transcription_in_progress:
                return {
                    'success': False,
                    'error': 'Transcripción ya en progreso - servicio ocupado',
                    'transcription': None,
                    'confidence': 0.0,
                    'processing_time': 0.0
                }
            
            if not self.model:
                return {
                    'success': False,
                    'error': 'Modelo Whisper no disponible',
                    'transcription': None,
                    'confidence': 0.0,
                    'processing_time': 0.0
                }
            
            self._transcription_in_progress = True
        
        try:
            logger.info(f"Transcribiendo: {audio_path}")
            
            # Verificar que el archivo existe
            if not os.path.exists(audio_path):
                return {
                    'success': False,
                    'error': f'Archivo no encontrado: {audio_path}',
                    'transcription': None,
                    'confidence': 0.0,
                    'processing_time': 0.0
                }
            
            # Preprocesar audio
            processed_audio_path = self._preprocess_audio_safe(audio_path)
            
            # TRANSCRIPCIÓN CON CONFIGURACIÓN SEGURA
            with whisper_lock:  # LOCK durante transcripción
                result = self.model.transcribe(
                    processed_audio_path,
                    language='es',
                    task='transcribe',
                    verbose=False,
                    # CONFIGURACIÓN CONSERVADORA PARA ESTABILIDAD
                    beam_size=1,  # Reducido para evitar problemas de memoria
                    best_of=1,    # Simplificado
                    temperature=0.0,
                    word_timestamps=False,  # Deshabilitado para reducir complejidad
                    fp16=False,   # CRÍTICO: Deshabilitar FP16 para CPU
                )
            
            transcription = result.get('text', '').strip()
            confidence = self._calculate_confidence_safe(result)
            transcription = self._post_process_spanish_text(transcription)
            
            processing_time = time.time() - start_time
            
            # Actualizar estadísticas
            self.transcriptions_count += 1
            self.total_processing_time += processing_time
            
            logger.info(f"Transcripción exitosa: '{transcription}' (confianza: {confidence:.2f}, tiempo: {processing_time:.2f}s)")
            
            # Limpiar archivo procesado si es diferente al original
            if processed_audio_path != audio_path:
                try:
                    os.remove(processed_audio_path)
                except:
                    pass
            
            return {
                'success': True,
                'error': None,
                'transcription': transcription,
                'confidence': confidence,
                'processing_time': processing_time,
                'language': result.get('language', 'es'),
                'word_count': len(transcription.split()) if transcription else 0,
                'device_used': 'cpu'
            }
            
        except Exception as e:
            processing_time = time.time() - start_time
            logger.error(f"Error transcribiendo audio: {e}")
            
            return {
                'success': False,
                'error': str(e),
                'transcription': None,
                'confidence': 0.0,
                'processing_time': processing_time
            }
        finally:
            # SIEMPRE liberar el lock
            with whisper_lock:
                self._transcription_in_progress = False
    
    def _calculate_confidence_safe(self, whisper_result) -> float:
        """Cálculo de confianza simplificado para evitar errores"""
        try:
            segments = whisper_result.get('segments', [])
            if not segments:
                return 0.5  # Confianza neutral por defecto
            
            # Solo usar avg_logprob para simplicidad
            avg_logprob = sum(seg.get('avg_logprob', -1.0) for seg in segments) / len(segments)
            confidence = max(0.0, min(1.0, (avg_logprob + 1.0)))
            
            return confidence
        except:
            return 0.5  # Fallback seguro
    
    def _post_process_spanish_text(self, text: str) -> str:
        """Post-procesamiento específico para español"""
        if not text:
            return text
        
        # Correcciones básicas
        corrections = {
            'muévete': 'mueve',
            'avanza': 'adelante',
            'retrocede': 'atrás',
            'hacia delante': 'adelante',
            'hacia atrás': 'atrás',
            'hacia la derecha': 'derecha',
            'hacia la izquierda': 'izquierda'
        }
        
        processed_text = text.lower()
        for wrong, correct in corrections.items():
            processed_text = processed_text.replace(wrong, correct)
        
        return ' '.join(processed_text.split()).strip()
    
    def _preprocess_audio_safe(self, audio_path: str) -> str:
        """Preprocesamiento de audio simplificado y seguro"""
        if not AUDIO_PROCESSING_AVAILABLE:
            return audio_path
        
        try:
            # Cargar con sampling rate óptimo para Whisper
            audio, sr = librosa.load(audio_path, sr=16000, mono=True)
            
            # Verificar duración mínima
            if len(audio) < 0.1 * sr:  # menos de 100ms
                return audio_path
            
            # Procesamiento básico y seguro
            audio = librosa.util.normalize(audio)
            
            # Guardar audio procesado
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                processed_path = temp_file.name
            
            sf.write(processed_path, audio, sr)
            return processed_path
            
        except Exception as e:
            logger.warning(f"Error en preprocesamiento: {e}, usando original")
            return audio_path
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtener estadísticas del servicio"""
        avg_processing_time = (
            self.total_processing_time / self.transcriptions_count
            if self.transcriptions_count > 0 else 0.0
        )
        
        return {
            'transcriptions_count': self.transcriptions_count,
            'avg_processing_time': round(avg_processing_time, 3),
            'model_name': self.model_name,
            'device': self.device,
            'model_loaded': self.model is not None,
            'thread_safe': True
        }

# Variables globales
whisper_service: Optional[WhisperService] = None
ros2_bridge: Optional[WhisperROS2Bridge] = None
ros2_executor = None
ros2_thread = None

# Crear aplicación FastAPI
app = FastAPI(
    title="Whisper Voice Command Service - CPU Safe",
    description="Servicio de transcripción de voz thread-safe para tutorial_pkg",
    version="1.1.0"
)

# Configurar CORS para permitir conexiones desde Flutter
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

def init_ros2_bridge():
    """Inicializar bridge ROS2 en hilo separado"""
    global ros2_bridge, ros2_executor, ros2_thread
    
    if not ROS2_AVAILABLE:
        logger.error("ROS2 no está disponible")
        return False
    
    try:
        rclpy.init()
        ros2_bridge = WhisperROS2Bridge()
        ros2_executor = rclpy.executors.SingleThreadedExecutor()
        ros2_executor.add_node(ros2_bridge)
        
        # Ejecutar en hilo separado
        def spin_ros2():
            try:
                ros2_executor.spin()
            except Exception as e:
                logger.error(f"Error en executor ROS2: {e}")
        
        ros2_thread = threading.Thread(target=spin_ros2, daemon=True)
        ros2_thread.start()
        
        logger.info("Bridge ROS2 iniciado correctamente")
        return True
        
    except Exception as e:
        logger.error(f"Error inicializando ROS2: {e}")
        return False

# USAR LIFESPAN EN LUGAR DE ON_EVENT (corrige deprecation warning)
from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    global whisper_service
    
    logger.info("Iniciando Whisper FastAPI Service")
    
    # Verificar dependencias
    missing_deps = []
    if not WHISPER_AVAILABLE:
        missing_deps.append("whisper")
    if not ROS2_AVAILABLE:
        missing_deps.append("ROS2")
    if not AUDIO_PROCESSING_AVAILABLE:
        missing_deps.append("librosa/soundfile")
    if not NUMPY_AVAILABLE:
        missing_deps.append("numpy")
    
    if missing_deps:
        logger.warning(f"Dependencias faltantes: {', '.join(missing_deps)}")
    
    # Inicializar servicios
    if WHISPER_AVAILABLE:
        whisper_service = WhisperService()
    
    if ROS2_AVAILABLE:
        init_ros2_bridge()
    
    logger.info("Servicios iniciados correctamente")
    
    yield
    
    # Shutdown
    global ros2_executor, ros2_bridge
    
    logger.info("Cerrando servicios...")
    
    if ros2_executor:
        ros2_executor.shutdown()
    
    if ros2_bridge:
        ros2_bridge.destroy_node()
    
    if ROS2_AVAILABLE and rclpy.ok():
        rclpy.shutdown()

# Asignar lifespan a la app
app.router.lifespan_context = lifespan

@app.get("/health")
async def health_check():
    """Endpoint de salud para verificar estado de servicios"""
    services = {
        'whisper': whisper_service is not None and whisper_service.model is not None,
        'ros2': ros2_bridge is not None,
        'audio_processing': AUDIO_PROCESSING_AVAILABLE,
        'numpy': NUMPY_AVAILABLE
    }
    
    status = "healthy" if services['whisper'] else "partial"
    
    return {
        'status': status,
        'services': services,
        'whisper_available': services['whisper'],
        'ros2_connected': services['ros2'],
        'timestamp': time.time(),
        'uptime': time.time() - (ros2_bridge.start_time if ros2_bridge else time.time()),
        'whisper_model': whisper_service.model_name if whisper_service else 'unknown',
        'device': 'cpu',
        'thread_safe': True
    }

@app.post("/transcribe")
async def transcribe_audio(audio: UploadFile = File(...)):
    """Transcribir audio y opcionalmente publicar a ROS2"""
    
    if not whisper_service or not whisper_service.model:
        raise HTTPException(
            status_code=503,
            detail="Servicio Whisper no disponible"
        )
    
    # Validar archivo
    if not audio.filename:
        raise HTTPException(status_code=400, detail="Nombre de archivo requerido")
    
    # Crear archivo temporal
    temp_file = None
    try:
        with tempfile.NamedTemporaryFile(
            suffix=Path(audio.filename).suffix or '.wav',
            delete=False
        ) as temp_file:
            # Leer y guardar archivo
            content = await audio.read()
            temp_file.write(content)
            temp_file.flush()
            temp_audio_path = temp_file.name
        
        logger.info(f"Audio recibido: {audio.filename} ({len(content)} bytes)")
        
        # Transcribir de forma asíncrona pero thread-safe
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(
            None, 
            whisper_service.transcribe_audio, 
            temp_audio_path
        )
        
        if result['success'] and result['transcription']:
            transcription = result['transcription']
            
            # Publicar a ROS2 si está disponible
            ros2_published = False
            if ros2_bridge:
                ros2_published = ros2_bridge.publish_voice_command(transcription)
            
            return {
                'success': True,
                'transcription': transcription,
                'confidence': result['confidence'],
                'processing_time': result['processing_time'],
                'language': result.get('language', 'es'),
                'ros2_published': ros2_published,
                'ai_response': f'Comando "{transcription}" procesado correctamente',
                'timestamp': time.time(),
                'word_count': result.get('word_count', 0),
                'device_used': result.get('device_used', 'cpu')
            }
        else:
            return {
                'success': False,
                'error': result['error'],
                'transcription': None,
                'confidence': 0.0,
                'processing_time': result['processing_time'],
                'ros2_published': False,
                'ai_response': f'Error transcribiendo: {result["error"]}',
                'timestamp': time.time()
            }
    
    except Exception as e:
        logger.error(f"Error procesando audio: {e}")
        raise HTTPException(status_code=500, detail=str(e))
    
    finally:
        # Limpiar archivo temporal
        if temp_file:
            try:
                os.unlink(temp_audio_path)
            except:
                pass

@app.post("/send_text_command")
async def send_text_command(request: dict):
    """Enviar comando de texto directamente a ROS2 (sin Whisper)"""
    
    command = request.get('command', '').strip()
    if not command:
        raise HTTPException(status_code=400, detail="Comando requerido")
    
    if not ros2_bridge:
        raise HTTPException(status_code=503, detail="Bridge ROS2 no disponible")
    
    try:
        success = ros2_bridge.publish_voice_command(command)
        
        return {
            'success': success,
            'command_sent': command,
            'ai_response': f'Comando "{command}" enviado al robot',
            'ros2_published': success,
            'timestamp': time.time()
        }
    
    except Exception as e:
        logger.error(f"Error enviando comando: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/stats")
async def get_statistics():
    """Obtener estadísticas de los servicios"""
    stats = {
        'service_name': 'Whisper FastAPI Service - Thread Safe',
        'version': '1.1.0',
        'timestamp': time.time()
    }
    
    if whisper_service:
        stats['whisper'] = whisper_service.get_stats()
    
    if ros2_bridge:
        stats['ros2_bridge'] = ros2_bridge.get_stats()
    
    return stats

@app.get("/")
async def root():
    """Página de inicio con información del servicio"""
    return {
        'message': 'Whisper Voice Command Service - Thread Safe para tutorial_pkg',
        'version': '1.1.0',
        'endpoints': {
            '/health': 'Estado de servicios',
            '/transcribe': 'Transcribir audio (POST)',
            '/send_text_command': 'Enviar comando de texto (POST)',
            '/stats': 'Estadísticas del servicio'
        },
        'whisper_available': whisper_service is not None,
        'ros2_connected': ros2_bridge is not None,
        'whisper_model': whisper_service.model_name if whisper_service else 'unknown',
        'device': 'cpu',
        'thread_safe': True,
        'timestamp': time.time()
    }

def main():
    """Función principal para ejecutar el servicio"""
    
    # Configuración del servidor
    host = "0.0.0.0"
    port = 8000
    
    print("Whisper FastAPI Service - Thread Safe para tutorial_pkg")
    print("=" * 60)
    print(f"Servidor: http://{host}:{port}")
    print(f"Health check: http://{host}:{port}/health")
    print(f"Estadísticas: http://{host}:{port}/stats")
    print("MODO: CPU-only thread-safe")
    print("=" * 60)
    
    # Verificar dependencias críticas
    if not WHISPER_AVAILABLE:
        print("ADVERTENCIA: Whisper no está disponible")
        print("   Instalar con: pip install openai-whisper")
    
    if not ROS2_AVAILABLE:
        print("ADVERTENCIA: ROS2 no está disponible")
        print("   Ejecutar: source /opt/ros/humble/setup.bash")
    
    # Ejecutar servidor
    try:
        uvicorn.run(
            app,
            host=host,
            port=port,
            log_level="info",
            reload=False,
            access_log=True,
            workers=1  # CRÍTICO: Solo 1 worker para evitar conflictos
        )
    except KeyboardInterrupt:
        print("\nServicio detenido por el usuario")
    except Exception as e:
        print(f"Error ejecutando servidor: {e}")

if __name__ == "__main__":
    main()
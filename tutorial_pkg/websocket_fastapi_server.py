#!/usr/bin/env python3
"""
Servidor FastAPI WebSocket optimizado para comandos de voz en tiempo real
Ubicación: ~/ros2_ws/src/tutorial_pkg/tutorial_pkg/websocket_fastapi_server.py
"""

import asyncio
import json
import time
import logging
import threading
from datetime import datetime
from typing import Dict, List, Optional, Set
from contextlib import asynccontextmanager

# FastAPI y WebSocket
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# ROS2 integración
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = None

# Whisper para transcripción en tiempo real
try:
    import whisper
    import torch
    import numpy as np
    import tempfile
    import base64
    import wave
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False

# Configurar logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ===============================================
# SINGLETON ROS2 BRIDGE
# ===============================================

class ROS2WebSocketBridge(Node):
    """Singleton ROS2 Bridge para WebSocket"""
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if hasattr(self, 'initialized'):
            return
        
        super().__init__('websocket_ros2_bridge')
        
        # Publishers
        self.voice_commands_pub = self.create_publisher(String, '/voice_commands', 10)
        self.text_commands_pub = self.create_publisher(String, '/text_commands', 10)
        self.emergency_pub = self.create_publisher(String, '/emergency_stop', 10)
        
        # Subscribers para feedback
        self.feedback_sub = self.create_subscription(
            String, '/voice_feedback', self.feedback_callback, 10
        )
        
        # Estadísticas
        self.commands_sent = 0
        self.start_time = time.time()
        self.feedback_callbacks: Set[callable] = set()
        
        self.initialized = True
        logger.info("🌉 ROS2 WebSocket Bridge singleton initialized")
    
    def publish_command(self, command_type: str, content: str, priority: str = 'normal') -> bool:
        """Publicar comando a ROS2"""
        try:
            msg = String()
            msg.data = content.strip()
            
            if command_type == 'emergency':
                self.emergency_pub.publish(msg)
                logger.critical(f"🚨 Emergency command: {content}")
            elif command_type == 'voice':
                self.voice_commands_pub.publish(msg)
                logger.info(f"🎤 Voice command: {content}")
            else:
                self.text_commands_pub.publish(msg)
                logger.info(f"💬 Text command: {content}")
            
            self.commands_sent += 1
            return True
            
        except Exception as e:
            logger.error(f"❌ Error publishing command: {e}")
            return False
    
    def feedback_callback(self, msg: String):
        """Callback para feedback de ROS2"""
        for callback in self.feedback_callbacks:
            try:
                callback(msg.data)
            except Exception as e:
                logger.error(f"Error in feedback callback: {e}")
    
    def add_feedback_listener(self, callback: callable):
        """Agregar listener para feedback"""
        self.feedback_callbacks.add(callback)
    
    def remove_feedback_listener(self, callback: callable):
        """Remover listener para feedback"""
        self.feedback_callbacks.discard(callback)

# ===============================================
# SINGLETON WHISPER SERVICE
# ===============================================

class WhisperStreamingService:
    """Singleton service para transcripción en streaming"""
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if hasattr(self, 'initialized'):
            return
            
        if not WHISPER_AVAILABLE:
            logger.error("❌ Whisper not available")
            self.model = None
            self.initialized = True
            return
        
        # OPTIMIZACIÓN: Modelo small para mejor precisión
        self.model_name = "small"
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        
        try:
            logger.info(f"🤖 Loading Whisper {self.model_name} on {self.device}")
            self.model = whisper.load_model(self.model_name, device=self.device)
            logger.info("✅ Whisper model loaded successfully")
        except Exception as e:
            logger.error(f"❌ Failed to load Whisper: {e}")
            self.model = None
        
        # Estadísticas
        self.transcriptions_count = 0
        self.total_processing_time = 0.0
        self.initialized = True
    
    async def transcribe_audio_data(self, audio_data: bytes, format: str = 'wav') -> Dict:
        """Transcribir datos de audio en tiempo real"""
        if not self.model:
            return {
                'success': False,
                'error': 'Whisper model not available',
                'transcription': '',
                'confidence': 0.0,
                'processing_time': 0.0
            }
        
        start_time = time.time()
        
        try:
            # Crear archivo temporal desde bytes
            with tempfile.NamedTemporaryFile(suffix=f'.{format}', delete=False) as temp_file:
                temp_file.write(audio_data)
                temp_path = temp_file.name
            
            # Transcripción optimizada para español
            result = await asyncio.get_event_loop().run_in_executor(
                None, 
                lambda: self.model.transcribe(
                    temp_path,
                    language='es',
                    task='transcribe',
                    beam_size=5,
                    best_of=3,
                    temperature=0.0,
                    word_timestamps=True,
                    verbose=False
                )
            )
            
            transcription = result.get('text', '').strip()
            confidence = self._calculate_confidence(result)
            processing_time = time.time() - start_time
            
            # Actualizar estadísticas
            self.transcriptions_count += 1
            self.total_processing_time += processing_time
            
            # Limpiar archivo temporal
            import os
            try:
                os.unlink(temp_path)
            except:
                pass
            
            return {
                'success': True,
                'transcription': transcription,
                'confidence': confidence,
                'processing_time': processing_time,
                'language': result.get('language', 'es')
            }
            
        except Exception as e:
            processing_time = time.time() - start_time
            logger.error(f"Transcription error: {e}")
            return {
                'success': False,
                'error': str(e),
                'transcription': '',
                'confidence': 0.0,
                'processing_time': processing_time
            }
    
    def _calculate_confidence(self, whisper_result) -> float:
        """Calcular confianza basada en logprobs y compresión"""
        segments = whisper_result.get('segments', [])
        if not segments:
            return 0.0
        
        avg_logprob = sum(seg.get('avg_logprob', -10) for seg in segments) / len(segments)
        logprob_confidence = max(0.0, min(1.0, (avg_logprob + 1.0) * 0.8))
        
        # Penalizar por alta compresión (audio problemático)
        compression_ratios = [seg.get('compression_ratio', 2.0) for seg in segments]
        avg_compression = sum(compression_ratios) / len(compression_ratios)
        compression_penalty = max(0.0, min(1.0, (4.0 - avg_compression) / 2.0))
        
        return (logprob_confidence * 0.7) + (compression_penalty * 0.3)

# ===============================================
# CONNECTION MANAGER - SINGLETON
# ===============================================

class WebSocketConnectionManager:
    """Singleton para manejar todas las conexiones WebSocket"""
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if hasattr(self, 'initialized'):
            return
        
        self.active_connections: Dict[str, WebSocket] = {}
        self.connection_metadata: Dict[str, Dict] = {}
        self.message_queue: Dict[str, List] = {}
        self.performance_metrics: Dict[str, List] = {}
        self.initialized = True
        logger.info("📡 WebSocket Connection Manager singleton initialized")
    
    async def connect(self, websocket: WebSocket, client_id: str):
        """Conectar cliente WebSocket"""
        await websocket.accept(subprotocol='voice-command')
        self.active_connections[client_id] = websocket
        self.connection_metadata[client_id] = {
            'connected_at': datetime.now(),
            'messages_sent': 0,
            'messages_received': 0,
            'last_activity': datetime.now()
        }
        self.message_queue[client_id] = []
        self.performance_metrics[client_id] = []
        
        logger.info(f"🔗 Client {client_id} connected. Active connections: {len(self.active_connections)}")
        
        # Enviar confirmación de handshake
        await self.send_message(client_id, {
            'type': 'handshake_response',
            'id': f'handshake_{int(time.time())}',
            'data': {
                'status': 'connected',
                'server_time': datetime.now().isoformat(),
                'client_id': client_id
            }
        })
    
    def disconnect(self, client_id: str):
        """Desconectar cliente"""
        if client_id in self.active_connections:
            del self.active_connections[client_id]
            del self.connection_metadata[client_id]
            del self.message_queue[client_id]
            del self.performance_metrics[client_id]
            logger.info(f"🔌 Client {client_id} disconnected. Active connections: {len(self.active_connections)}")
    
    async def send_message(self, client_id: str, message: Dict):
        """Enviar mensaje a cliente específico"""
        if client_id not in self.active_connections:
            return False
        
        try:
            websocket = self.active_connections[client_id]
            await websocket.send_text(json.dumps(message))
            
            # Actualizar métricas
            self.connection_metadata[client_id]['messages_sent'] += 1
            self.connection_metadata[client_id]['last_activity'] = datetime.now()
            
            return True
            
        except Exception as e:
            logger.error(f"Error sending message to {client_id}: {e}")
            self.disconnect(client_id)
            return False
    
    async def broadcast_message(self, message: Dict, exclude_client: str = None):
        """Broadcast mensaje a todos los clientes conectados"""
        disconnected_clients = []
        
        for client_id, websocket in self.active_connections.items():
            if exclude_client and client_id == exclude_client:
                continue
            
            try:
                await websocket.send_text(json.dumps(message))
                self.connection_metadata[client_id]['messages_sent'] += 1
            except Exception as e:
                logger.error(f"Error broadcasting to {client_id}: {e}")
                disconnected_clients.append(client_id)
        
        # Limpiar conexiones desconectadas
        for client_id in disconnected_clients:
            self.disconnect(client_id)
    
    def record_latency(self, client_id: str, latency_ms: float):
        """Registrar latencia para métricas"""
        if client_id in self.performance_metrics:
            metrics = self.performance_metrics[client_id]
            metrics.append(latency_ms)
            if len(metrics) > 100:  # Mantener solo las últimas 100
                metrics.pop(0)
    
    def get_connection_stats(self) -> Dict:
        """Obtener estadísticas de conexiones"""
        return {
            'active_connections': len(self.active_connections),
            'total_messages_sent': sum(meta['messages_sent'] for meta in self.connection_metadata.values()),
            'total_messages_received': sum(meta['messages_received'] for meta in self.connection_metadata.values()),
            'connections_metadata': self.connection_metadata
        }

# ===============================================
# INICIALIZACIÓN DE SERVICIOS SINGLETON
# ===============================================

# Instancias globales singleton
connection_manager = WebSocketConnectionManager()
ros2_bridge = None
whisper_service = WhisperStreamingService()
ros2_executor = None
ros2_thread = None

def init_ros2():
    """Inicializar ROS2 en hilo separado"""
    global ros2_bridge, ros2_executor, ros2_thread
    
    if not ROS2_AVAILABLE:
        logger.error("❌ ROS2 not available")
        return False
    
    try:
        rclpy.init()
        ros2_bridge = ROS2WebSocketBridge()
        ros2_executor = SingleThreadedExecutor()
        ros2_executor.add_node(ros2_bridge)
        
        def spin_ros2():
            try:
                ros2_executor.spin()
            except Exception as e:
                logger.error(f"ROS2 executor error: {e}")
        
        ros2_thread = threading.Thread(target=spin_ros2, daemon=True)
        ros2_thread.start()
        
        logger.info("✅ ROS2 bridge initialized successfully")
        return True
        
    except Exception as e:
        logger.error(f"Failed to initialize ROS2: {e}")
        return False

# ===============================================
# FASTAPI APPLICATION
# ===============================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifecycle manager para FastAPI"""
    # Startup
    logger.info("🚀 Starting WebSocket FastAPI Server")
    
    # Inicializar ROS2
    if ROS2_AVAILABLE:
        init_ros2()
    else:
        logger.warning("⚠️ ROS2 not available - running in standalone mode")
    
    yield
    
    # Shutdown
    logger.info("🛑 Shutting down WebSocket FastAPI Server")
    if ros2_executor:
        ros2_executor.shutdown()
    if ROS2_AVAILABLE and rclpy.ok():
        rclpy.shutdown()

app = FastAPI(
    title="Real-Time Voice Command WebSocket Service",
    description="WebSocket service optimizado para comandos de voz en tiempo real",
    version="2.0.0",
    lifespan=lifespan
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ===============================================
# WEBSOCKET ENDPOINTS
# ===============================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Endpoint principal WebSocket para comandos en tiempo real"""
    client_id = f"client_{int(time.time())}_{id(websocket)}"
    
    try:
        await connection_manager.connect(websocket, client_id)
        
        # Agregar listener para feedback de ROS2
        def feedback_handler(feedback: str):
            asyncio.create_task(connection_manager.send_message(client_id, {
                'type': 'system_notification',
                'id': f'feedback_{int(time.time())}',
                'data': {'message': feedback, 'source': 'ros2'}
            }))
        
        if ros2_bridge:
            ros2_bridge.add_feedback_listener(feedback_handler)
        
        while True:
            try:
                # Recibir mensaje del cliente
                data = await websocket.receive_text()
                message = json.loads(data)
                
                # Actualizar métricas
                connection_manager.connection_metadata[client_id]['messages_received'] += 1
                connection_manager.connection_metadata[client_id]['last_activity'] = datetime.now()
                
                # Procesar mensaje según tipo
                response = await process_websocket_message(client_id, message)
                
                # Enviar respuesta si existe
                if response:
                    await connection_manager.send_message(client_id, response)
                
            except WebSocketDisconnect:
                break
            except json.JSONDecodeError:
                await connection_manager.send_message(client_id, {
                    'type': 'error',
                    'id': f'error_{int(time.time())}',
                    'data': {'message': 'Invalid JSON format'}
                })
            except Exception as e:
                logger.error(f"Error processing message from {client_id}: {e}")
                await connection_manager.send_message(client_id, {
                    'type': 'error',
                    'id': f'error_{int(time.time())}',
                    'data': {'message': f'Processing error: {str(e)}'}
                })
    
    finally:
        # Cleanup
        if ros2_bridge:
            ros2_bridge.remove_feedback_listener(feedback_handler)
        connection_manager.disconnect(client_id)

async def process_websocket_message(client_id: str, message: Dict) -> Optional[Dict]:
    """Procesar mensaje WebSocket y generar respuesta"""
    message_type = message.get('type')
    message_id = message.get('id', f'msg_{int(time.time())}')
    start_time = time.time()
    
    try:
        if message_type == 'handshake':
            return {
                'type': 'handshake_response',
                'id': f'handshake_resp_{int(time.time())}',
                'response_to_id': message_id,
                'data': {
                    'status': 'connected',
                    'server_capabilities': ['voice_commands', 'text_commands', 'audio_streaming'],
                    'ros2_available': ros2_bridge is not None,
                    'whisper_available': whisper_service.model is not None
                }
            }
        
        elif message_type == 'command':
            return await process_command_message(client_id, message, start_time)
        
        elif message_type == 'audio_stream':
            return await process_audio_stream(client_id, message, start_time)
        
        elif message_type == 'heartbeat':
            return {
                'type': 'heartbeat',
                'id': f'heartbeat_resp_{int(time.time())}',
                'response_to_id': message_id,
                'data': {'server_time': datetime.now().isoformat()}
            }
        
        else:
            return {
                'type': 'error',
                'id': f'error_{int(time.time())}',
                'response_to_id': message_id,
                'data': {'message': f'Unknown message type: {message_type}'}
            }
    
    except Exception as e:
        logger.error(f"Error processing {message_type} message: {e}")
        return {
            'type': 'error',
            'id': f'error_{int(time.time())}',
            'response_to_id': message_id,
            'data': {'message': f'Processing error: {str(e)}'}
        }

async def process_command_message(client_id: str, message: Dict, start_time: float) -> Dict:
    """Procesar mensaje de comando"""
    command_data = message.get('command', {})
    command_type = command_data.get('type', 'text')
    content = command_data.get('content', '').strip()
    priority = command_data.get('priority', 'normal')
    
    if not content:
        return {
            'type': 'command_response',
            'id': f'cmd_resp_{int(time.time())}',
            'response_to_id': message.get('id'),
            'data': {
                'success': False,
                'message': 'Empty command content',
                'processing_time_ms': (time.time() - start_time) * 1000
            }
        }
    
    # Enviar comando a ROS2
    success = False
    if ros2_bridge:
        success = ros2_bridge.publish_command(command_type, content, priority)
    
    processing_time_ms = (time.time() - start_time) * 1000
    connection_manager.record_latency(client_id, processing_time_ms)
    
    return {
        'type': 'command_response',
        'id': f'cmd_resp_{int(time.time())}',
        'response_to_id': message.get('id'),
        'data': {
            'success': success,
            'message': f'Command {"executed" if success else "failed"}',
            'command_type': command_type,
            'command_content': content,
            'processing_time_ms': processing_time_ms,
            'ros2_published': success
        }
    }

async def process_audio_stream(client_id: str, message: Dict, start_time: float) -> Dict:
    """Procesar stream de audio para transcripción"""
    if not whisper_service.model:
        return {
            'type': 'transcription_result',
            'id': f'transcr_{int(time.time())}',
            'response_to_id': message.get('id'),
            'data': {
                'success': False,
                'error': 'Whisper service not available',
                'processing_time_ms': (time.time() - start_time) * 1000
            }
        }
    
    try:
        # Decodificar audio base64
        audio_base64 = message.get('audio_data', '')
        audio_format = message.get('format', 'wav')
        
        if not audio_base64:
            raise ValueError('No audio data provided')
        
        audio_bytes = base64.b64decode(audio_base64)
        
        # Transcribir audio
        result = await whisper_service.transcribe_audio_data(audio_bytes, audio_format)
        
        processing_time_ms = (time.time() - start_time) * 1000
        connection_manager.record_latency(client_id, processing_time_ms)
        
        # Si la transcripción es exitosa y confianza alta, enviar como comando automáticamente
        if result['success'] and result.get('confidence', 0) > 0.8:
            transcription = result['transcription']
            if transcription and ros2_bridge:
                ros2_bridge.publish_command('voice', transcription, 'high')
        
        return {
            'type': 'transcription_result',
            'id': f'transcr_{int(time.time())}',
            'response_to_id': message.get('id'),
            'data': {
                **result,
                'processing_time_ms': processing_time_ms,
                'auto_command_sent': result['success'] and result.get('confidence', 0) > 0.8
            }
        }
    
    except Exception as e:
        return {
            'type': 'transcription_result',
            'id': f'transcr_{int(time.time())}',
            'response_to_id': message.get('id'),
            'data': {
                'success': False,
                'error': str(e),
                'processing_time_ms': (time.time() - start_time) * 1000
            }
        }

# ===============================================
# HTTP ENDPOINTS (compatibilidad)
# ===============================================

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        'status': 'healthy',
        'websocket_available': True,
        'ros2_connected': ros2_bridge is not None,
        'whisper_available': whisper_service.model is not None,
        'active_connections': len(connection_manager.active_connections),
        'timestamp': datetime.now().isoformat()
    }

@app.get("/stats")
async def get_stats():
    """Estadísticas del servidor"""
    connection_stats = connection_manager.get_connection_stats()
    
    whisper_stats = {
        'transcriptions_count': whisper_service.transcriptions_count,
        'avg_processing_time': (
            whisper_service.total_processing_time / whisper_service.transcriptions_count
            if whisper_service.transcriptions_count > 0 else 0.0
        ),
        'model_available': whisper_service.model is not None
    }
    
    ros2_stats = {}
    if ros2_bridge:
        ros2_stats = {
            'commands_sent': ros2_bridge.commands_sent,
            'uptime_seconds': time.time() - ros2_bridge.start_time,
            'active_feedback_listeners': len(ros2_bridge.feedback_callbacks)
        }
    
    return {
        'server': {
            'name': 'Real-Time Voice Command WebSocket Service',
            'version': '2.0.0',
            'uptime': time.time() - (ros2_bridge.start_time if ros2_bridge else time.time())
        },
        'connections': connection_stats,
        'whisper': whisper_stats,
        'ros2': ros2_stats,
        'timestamp': datetime.now().isoformat()
    }

@app.get("/")
async def root():
    """Root endpoint con información del servicio"""
    return {
        'message': 'Real-Time Voice Command WebSocket Service',
        'version': '2.0.0',
        'websocket_endpoint': '/ws',
        'documentation': '/docs',
        'health_check': '/health',
        'statistics': '/stats',
        'features': [
            'WebSocket real-time communication',
            'Voice command transcription',
            'ROS2 integration',
            'Low-latency audio streaming',
            'Singleton architecture',
            'Performance metrics'
        ],
        'connection_info': {
            'active_connections': len(connection_manager.active_connections),
            'supported_protocols': ['voice-command'],
            'max_message_size': '10MB'
        }
    }

# ===============================================
# MAIN EXECUTION
# ===============================================

def main():
    """Función principal"""
    host = "0.0.0.0"
    port = 8000
    
    print("🔥 Real-Time Voice Command WebSocket Service")
    print("=" * 60)
    print(f"🌐 WebSocket Server: ws://{host}:{port}/ws")
    print(f"📊 Health Check: http://{host}:{port}/health")
    print(f"📈 Statistics: http://{host}:{port}/stats")
    print(f"📚 API Docs: http://{host}:{port}/docs")
    print("=" * 60)
    print("✨ Features:")
    print("  - WebSocket real-time communication")
    print("  - Singleton pattern for zero redundancy") 
    print("  - Ultra-low latency audio streaming")
    print("  - Automatic ROS2 command publishing")
    print("  - Real-time voice transcription")
    print("  - Performance metrics and monitoring")
    print("=" * 60)
    
    # Verificar dependencias
    missing_deps = []
    if not WHISPER_AVAILABLE:
        missing_deps.append("Whisper (pip install openai-whisper torch)")
    if not ROS2_AVAILABLE:
        missing_deps.append("ROS2 (source /opt/ros/humble/setup.bash)")
    
    if missing_deps:
        print("⚠️  Missing dependencies:")
        for dep in missing_deps:
            print(f"   - {dep}")
        print("")
    
    try:
        uvicorn.run(
            "tutorial_pkg.websocket_fastapi_server:app",
            host=host,
            port=port,
            log_level="info",
            reload=False,  # Disabled para evitar problemas con singletons
            access_log=True,
            ws_ping_interval=30,
            ws_ping_timeout=10,
            ws_max_size=10 * 1024 * 1024,  # 10MB max message size
        )
    except KeyboardInterrupt:
        print("\n🛑 Server stopped by user")
    except Exception as e:
        print(f"❌ Server error: {e}")

if __name__ == "__main__":
    main()
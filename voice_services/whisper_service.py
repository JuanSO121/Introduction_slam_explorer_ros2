#!/usr/bin/env python3
"""
Servicio de transcripción con Whisper adaptado para tutorial_pkg
"""
import torch
from transformers import WhisperProcessor, WhisperForConditionalGeneration
import librosa
import logging
import warnings
import time
from pathlib import Path
import os

warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=FutureWarning)

class WhisperService:
    def __init__(self, model_name="openai/whisper-small"):
        self.model = None
        self.processor = None
        self._ready = False
        self.logger = logging.getLogger(__name__)
        self.model_name = model_name
        self.device = "cpu"  # Forzar CPU para estabilidad en tutorial_pkg
        
        self._load_model()
    
    def _load_model(self):
        """Cargar modelo Whisper de forma segura"""
        try:
            self.logger.info(f"🧠 Cargando Whisper modelo: {self.model_name}")
            
            os.environ["TOKENIZERS_PARALLELISM"] = "false"
            
            self.processor = WhisperProcessor.from_pretrained(self.model_name)
            self.model = WhisperForConditionalGeneration.from_pretrained(
                self.model_name,
                torch_dtype=torch.float32,
                low_cpu_mem_usage=True
            )
            
            self.model.to(self.device)
            self.model.eval()
            
            # Configurar español
            self.forced_decoder_ids = self.processor.get_decoder_prompt_ids(
                language="spanish", task="transcribe"
            )
            
            self._ready = True
            self.logger.info("✅ Whisper cargado correctamente")
            
        except Exception as e:
            self.logger.error(f"❌ Error cargando Whisper: {e}")
            self._ready = False
    
    def transcribe_audio(self, audio_path: str) -> str:
        """Transcribir archivo de audio"""
        if not self._ready:
            raise RuntimeError("Whisper no está listo")
        
        try:
            # Cargar audio
            audio_array, sample_rate = librosa.load(audio_path, sr=16000, duration=30)
            
            if len(audio_array) == 0:
                return ""
            
            # Normalizar
            if audio_array.max() > 0:
                audio_array = audio_array / audio_array.max() * 0.9
            
            # Procesar con Whisper
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
            transcription = " ".join(transcription.split()).strip()
            
            if len(transcription) < 3:
                return ""
            
            return transcription
            
        except Exception as e:
            self.logger.error(f"❌ Error transcribiendo: {e}")
            return ""
    
    def is_ready(self) -> bool:
        return self._ready
    
    def clear_cache(self):
        try:
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            import gc
            gc.collect()
        except:
            pass

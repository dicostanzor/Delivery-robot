# medbot_gui.py
# ELARA CareGo ATR — Touchscreen HMI for Raspberry Pi Touch Display
# Runs on Raspberry Pi 4 with the official 7" DSI touch display
# Handles voice input, room confirmation, platform controls,
# and sends confirmed room number to Jetson via TCP socket

import sounddevice as sd
from scipy.io.wavfile import write
from faster_whisper import WhisperModel
import subprocess
import socket
import serial
import re
import time
import tkinter as tk
import threading

# --- Hardware Config ---

# Find ReSpeaker mic by name rather than hardcoded index
# Device index can change at boot so name lookup is more reliable
def find_mic():
    devices = sd.query_devices()
    for i, d in enumerate(devices):
        if 'ReSpeaker' in d['name']:
            return i
    return None

MIC_DEVICE = find_mic()
SAMPLE_RATE = 16000     # 16kHz required by Whisper
SECONDS = 10            # Recording duration increased to 10 seconds
PIPER_MODEL = "/home/pi/piper/models/en_US-lessac-medium.onnx"
JETSON_IP = '192.168.137.2'
JETSON_PORT = 5005      # room_receiver.py
PLATFORM_PORT = 5007    # platform_receiver.py
ACTUATOR_PORT = 5006    # actuator_receiver.py
RS485_PORT = '/dev/ttyUSB0'
RS485_BAUD = 38400

model = WhisperModel("tiny.en", device="cpu", compute_type="int8")


# --- Audio Functions ---

def record_and_transcribe(filename="command.wav"):
    """Records audio and transcribes using Whisper. Returns text string."""
    if MIC_DEVICE is None:
        print("  [!] Microphone not found")
        return ""
    audio = sd.rec(
        int(SECONDS * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="int16",
        device=MIC_DEVICE
    )
    sd.wait()
    write(filename, SAMPLE_RATE, audio)
    segments, info = model.transcribe(filename)
    return " ".join(segment.text.strip() for segment in segments)


def speak(message):
    """Converts text to speech using Piper TTS and plays through speaker."""
    subprocess.run(
        f'echo "{message}" | /home/pi/piper/piper --model {PIPER_MODEL} --output_file response.wav',
        shell=True

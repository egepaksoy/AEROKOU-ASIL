#! RASP
import socket
import time
from picamera2 import Picamera2
import cv2
import struct
import json
import threading
import time
from math import ceil


config = json.load(open("./config.json", "r"))
stop_event = threading.Event()

UDP_IP, UDP_PORT = config["ip"], config["port"]
CHUNK_SIZE = 1400                      # ~ MTU altı
HEADER_FMT = '<LHB'                   # frame_id:uint32, chunk_id:uint16, is_last:uint8

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# PiCamera2'yi başlat ve yapılandır
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()
time.sleep(2)  # Kamera başlatma süresi için bekle

frame_id = 0
try:
    print(f"{UDP_IP} adresine gönderim başladı")
    while not stop_event.is_set():
        frame = picam2.capture_array()
        _, buf = cv2.imencode('.jpg', frame)
        data = buf.tobytes()
        total_chunks = ceil(len(data) / CHUNK_SIZE)

        for chunk_id in range(total_chunks):
            start = chunk_id * CHUNK_SIZE
            end = start + CHUNK_SIZE
            chunk = data[start:end]
            is_last = 1 if chunk_id == total_chunks - 1 else 0
            header = struct.pack(HEADER_FMT, frame_id, chunk_id, is_last)
            sock.sendto(header + chunk, (UDP_IP, UDP_PORT))

        frame_id = (frame_id + 1) & 0xFFFFFFFF
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Ctrl+C ile çıkıldı.")

finally:
    # Kamera ve soketi kapat
    print("Program sonlandırıldı.")
    picam2.stop()
    sock.close()
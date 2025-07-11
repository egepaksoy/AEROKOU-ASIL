import time
import sys
import threading
from picamera2 import Picamera2
import cv2
import numpy as np
import struct
import socket
import queue
from math import ceil

def is_equilateral(approx, tolerance=0.15):
    if len(approx) < 3:
        return False
    sides = []
    for i in range(len(approx)):
        pt1 = approx[i][0]
        pt2 = approx[(i + 1) % len(approx)][0]
        dist = np.linalg.norm(pt1 - pt2)
        sides.append(dist)
    mean = np.mean(sides)
    return all(abs(s - mean) / mean < tolerance for s in sides)

stop_event = threading.Event()
broadcast_started = threading.Event()
shared_state = {"last_object": None, "timestamp": 0}
shared_state_lock = threading.Lock()

dropped_objects = []

# PiCamera2'yi başlat ve yapılandır
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()
time.sleep(2)  # Kamera başlatma süresi için bekle

UDP_IP, UDP_PORT = sys.argv[1], int(sys.argv[2])
CHUNK_SIZE = 1400                      # ~ MTU altı
HEADER_FMT = '<LHB'                   # frame_id:uint32, chunk_id:uint16, is_last:uint8

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# HSV renk aralıkları
lower_red1 = np.array([0, 70, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 70, 50])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 50])
upper_blue = np.array([140, 255, 255])


detected_obj = ""
frame_id = 0
try:
    print(f"{UDP_IP} adresine gönderim başladı")
    broadcast_started.set()
    while not stop_event.is_set():
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)

        ####### GORUNTU ISLEME BASLADI ########
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        for color_mask, shape_name, target_sides, color in [
            (red_mask, "Ucgen", 3, (0, 0, 255)),
            (blue_mask, "Altigen", 6, (255, 0, 0))
        ]:
            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                if len(approx) == target_sides and is_equilateral(approx):
                    cv2.drawContours(frame, [approx], 0, color, 2)
                    x, y = approx[0][0]
                    cv2.putText(frame, shape_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    detected_obj = shape_name
                    print(detected_obj)

        if detected_obj:
            with shared_state_lock:
                shared_state["last_object"] = detected_obj
                shared_state["timestamp"] = time.time()

        _, buf = cv2.imencode('.jpg', frame)
        data = buf.tobytes()
        total_chunks = ceil(len(data) / CHUNK_SIZE)
        ###### GORUNTU ISLEME BITTI ###########

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


if len(sys.argv) != 3:
    print("Usage: python main.py <ip> <port>")
    sys.exit(1)

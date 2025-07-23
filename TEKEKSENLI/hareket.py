import libs.gimbal_controller as gimbal_controller
import libs.tcp_handler as tcp_handler
import libs.image_processing_handler as image_processing_handler

import time
import json
import threading


config = json.load(open("./test-config.json"))
stop_event = threading.Event()

server = tcp_handler.TCPServer(config["TCP"]["port"], stop_event)
print("Server bağlantısı bekleniyor")
while len(server.connected_clients) == 0:
    time.sleep(0.5)

image_handler = image_processing_handler.Handler(stop_event)
threading.Thread(target=image_handler.udp_camera_new, args=(config["UDP"]["ip"], config["UDP"]["port"]), daemon=True).start()

try:
    start_time = time.time()
    i = 1
    while not stop_event.is_set():
        if time.time() - start_time >= 5:
            i *= -1
            start_time = time.time()
        server.send_data(f"{i}\n")
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("Cikildi")
    if not stop_event.is_set():
        stop_event.set()

finally:
    if not stop_event.is_set():
        stop_event.set()
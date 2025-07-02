import threading
import json
import time
import sys
import math

import YKI.libs.tcp_handler as tcp_handler
import YKI.libs.image_processing_handler as image_processing_handler
import YKI.libs.gimbal_controller as gimbal_controller

#? Gerekliler
config = json.load(open("./config.json"))
stop_event = threading.Event()

#? Kütüphaneler
server = tcp_handler.TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
camera_handler = image_processing_handler.Handler(stop_event=stop_event)
gimbal_handler = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)

#? Threadler
threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()
threading.Thread(target=gimbal_handler.keyboard_controller, daemon=True).start()

try:
    # target_id, target_clss, target_drone, target_loc
    targets = {}
    start_time = time.time()
    while not stop_event.is_set():
        
            
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Exiting...")
    
    if not stop_event.is_set():
        stop_event.set()

except Exception as e:
    if not stop_event.is_set():
        stop_event.set()
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()


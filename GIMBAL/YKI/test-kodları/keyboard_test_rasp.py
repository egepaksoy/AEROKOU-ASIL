import threading
import time
import json

import libs.gimbal_controller as gimbal_controller
import libs.tcp_handler as tcp_handler
import libs.image_processing_handler as image_processing_handler


config = json.load(open("./suru-config.json"))
stop_event = threading.Event()
server = tcp_handler.TCPServer(port=config["TCP"]["port"], stop_event=stop_event)

camera_handler = image_processing_handler.Handler(stop_event=stop_event)
threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()

gimbal_handler = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)
keyboard_thrd = threading.Thread(target=gimbal_handler.keyboard_controller, daemon=True)
keyboard_thrd.start()

try:
    while keyboard_thrd.is_alive():
        time.sleep(2)
        
    print("Kod bitti")

except KeyboardInterrupt:
    print("Durduruldu")
    if stop_event.is_set():
        stop_event.set()

finally:
    print("Bitti")
    if stop_event.is_set():
        stop_event.set()

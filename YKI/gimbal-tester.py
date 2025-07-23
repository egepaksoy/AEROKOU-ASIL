import threading
import json
import time

from libs.yki_handler import YKIMonitor
from libs.gimbal_controller import GimbalHandler
from libs.tcp_handler import TCPServer
from libs.image_processing_handler import Handler

config = json.load(open("./suru-config.json"))
stop_event = threading.Event()

# Raspberry ile tcp baglantisi
server = TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
# Arduino ile yki baglantisi
yki_monitor = YKIMonitor(config=config, stop_event=stop_event)
yki_monitor.start()

# Gimbal haberlesmesi
gimbal_handler = GimbalHandler(server=server, stop_event=stop_event)
threading.Thread(target=gimbal_handler.joystick_controller, args=(yki_monitor, ), daemon=True).start()

# Gimbal kamera verisi alma
camera_handler = Handler(stop_event=stop_event)
threading.Thread(target=camera_handler.udp_camera_new, args=(config["UDP"]["ip"], config["UDP"]["port"]), daemon=True).start()

try:
    start_time = time.time()
    while not stop_event.is_set():
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Cikildi")
    if not stop_event.is_set():
        stop_event.set()

except Exception as e:
    print(e)
    if not stop_event.is_set():
        stop_event.set()

finally:
    if not stop_event.is_set():
        stop_event.set()
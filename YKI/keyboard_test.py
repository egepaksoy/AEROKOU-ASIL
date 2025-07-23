import threading
import time
import json

import libs.gimbal_controller as gimbal_controller
import libs.tcp_handler as tcp_handler
import libs.image_processing_handler as image_processing_handler
import libs.yki_handler as yki_handler


config = json.load(open("./suru-config.json"))
stop_event = threading.Event()
server = tcp_handler.TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
camera_handler = image_processing_handler.Handler(stop_event=stop_event)
gimbal_handler = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)
yki_monitor = yki_handler.YKIMonitor(config=config, stop_event=stop_event)
yki_monitor.start()

#? Threadler
threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()
#threading.Thread(target=gimbal_handler.keyboard_controller, daemon=True).start()
threading.Thread(target=gimbal_handler.joystick_controller, args=(yki_monitor, ), daemon=True).start()

start_time = 0
while not stop_event.is_set():
    if yki_monitor.get_system_status():
        break
    if time.time() - start_time >= 5:
        print("\rYKI Başlatılması Bekleniyor", end="")
        start_time = time.time()

    time.sleep(0.5)

print("\nYKI Baslatildi")

try:
    while not stop_event.is_set():
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


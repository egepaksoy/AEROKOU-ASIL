import json
import keyboard
import time
import threading
import sys

sys.path.append("./YKI/libs")
import tcp_handler as tcp_handler
import calc_loc as calc_loc
import image_processing_handler as image_processing_handler
import gimbal_controller as gimbal_controller

class Vehicle:
    def __init__(self):
        self.drone_ids = (1, 2)
    
    def get_pos(self, drone_id):
        if drone_id == 1:
            return (0,0,10)
        else:
            return(1,1,10)
    
    def get_yaw(self, drone_id):
        return 0

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

vehicle = Vehicle()
target_locker = threading.Lock()
targets = {}

#? Seçme threadi
gimbal_handler = gimbal_controller.GimbalHandler(server, stop_event)
selector_thrd = threading.Thread(target=gimbal_handler.gimbal_selecter, args=(stop_event, vehicle, 1, server, targets, target_locker))
selector_thrd.start()

try:
    start_time = time.time()
    while True:
        if time.time() - start_time >= 5:
            print(targets)
            start_time = time.time()
        if not selector_thrd.is_alive():
            break
        time.sleep(1)

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
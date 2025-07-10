import threading
import time
import sys
import json

sys.path.append("../pymavlink_custom/")
from pymavlink_custom import Vehicle

from libs.tcp_handler import TCPServer
from libs import gimbal_controller
from libs.calc_loc import calc_location_geopy, get_dist
from libs import new_image_processing_handler
import keyboard


def select_thread(config: dict, server: TCPServer, vehicle: Vehicle, DRONE_ID: int):
    global hedefler

    while not stop_event.is_set():
        select_pressed = keyboard.is_pressed(config["KEYS"]["select"])
        delete_pressed = keyboard.is_pressed(config["KEYS"]["delete"])

        if select_pressed:
            print("Hedef konumu aliniyor...")
            server.send_data("2|2\n")

            tcp_data = server.get_data()
            while tcp_data == None:
                tcp_data = server.get_data()
                time.sleep(0.5)

            obj_name = input("Obje adi giriniz: ")

            if obj_name != None:
                if obj_name not in hedefler:
                    current_yaw = vehicle.get_yaw(drone_id=DRONE_ID)
                    current_pos = vehicle.get_pos(drone_id=DRONE_ID)

                    target_loc = calc_location_geopy(current_loc=current_pos, yaw_angle=current_yaw, tcp_data=tcp_data)

                    min_dist = None
                    min_dist_drone_id = None
                    for drone_id in vehicle.drone_ids:
                        if any(value[0] == drone_id for value in hedefler.values()):
                            continue

                        drone_pos = vehicle.get_pos(drone_id=drone_id)
                        
                        if min_dist == None:
                            min_dist = get_dist(drone_pos, target_loc)
                            min_dist_drone_id = drone_id
                            continue

                        if get_dist(drone_pos, target_loc) < min_dist:
                            min_dist = get_dist(drone_pos, target_loc)
                            min_dist_drone_id = drone_id

                    hedefler[obj_name] = [min_dist_drone_id, target_loc]
                    print(f"Hedef {obj_name} eklendi.\nKonumu: {target_loc}\nDronu: {min_dist_drone_id}")
                    print(f"current_yaw: {current_yaw}")
                    print(f"tcp_data: {tcp_data}")
                    print(f"distance: {get_dist(target_loc, current_pos)}")
                    print(f"Hedefler: {hedefler}")

        if delete_pressed:
            delete_hedef = input("Silinecek hedef adi girin: ")
            if delete_hedef in hedefler:
                hedefler.pop(delete_hedef)

        time.sleep(0.05)

config = json.load(open("./config.json", "r"))
stop_event = threading.Event()
hedefler = {}

server = TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
gimbal_controller = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)

threading.Thread(target=gimbal_controller.keyboard_controller, daemon=True).start()

# Kamera başlatılıyor
camera_handler = new_image_processing_handler.Handler(stop_event)
camera_handler.start_proccessing(config["model"])

# Kamera işleme thread’i
threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()

DRONE_ID = config["GOZLEMCI"]["id"]

# Drone Uçuş İçin
vehicle = Vehicle(config["CONN-PORT"])
threading.Thread(target=select_thread, args=(config, server, vehicle, DRONE_ID), daemon=True).start()

try:
    while not stop_event.is_set():
        print(hedefler)
        time.sleep(1)

except KeyboardInterrupt:
    print("Çıkıldı")

except Exception as e:
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()
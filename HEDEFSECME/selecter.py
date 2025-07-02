import threading
import json
import time
from queue import Queue
import keyboard

import libs.tcp_handler as tcp_handler
import libs.calc_loc as calc_loc
import libs.image_processing_handler as image_processing_handler
import libs.gimbal_controller as gimbal_controller


def hedef_sec(hedefler, objects_queue):
    old_key = None

    while not stop_event.is_set():
        if keyboard.is_pressed("x"):
            check_key, objects = objects_queue.get()
            if check_key != old_key:
                old_key = check_key

                min_dist = objects[0]
                
                for obj in objects:
                    if obj[1] < min_dist[1]:
                        min_dist = obj

                if min_dist[0] in hedefler:
                    hedefler.pop(min_dist[0])
                    print(f"{min_dist[0]} hedefi silindi")
                else:
                    print(f"{min_dist[0]} hedefi zaten yok")

        if keyboard.is_pressed("c"):
            check_key, objects = objects_queue.get()
            if check_key != old_key:
                old_key = check_key

                min_obj = objects[0]
                
                for obj in objects:
                    if obj[1] > min_obj[1]:
                        min_obj = obj

                if min_obj[0] not in hedefler:
                    #tcp_data = server.get_data()
                    tcp_data = 1

                    if tcp_data != None:
                        """
                        tcp_data = tcp_data.strip()
                        #! Mesafe verisi gelmedi ise devam ediyor
                        if len(tcp_data.split("|")) != 3:
                            continue
                    
                        if float(tcp_data.split("|")[0]) <= 0:
                            continue

                        print(tcp_data.split("|"))
                        if calc_loc.check_data(tcp_data) == False:
                            print("Hedef bozuk alındı yeni hedef bekleniyor")
                            tcp_data = ""
                            continue
                        """
                        
                        current_loc = (0,0,0)
                        current_yaw = 0

                        #target_loc = calc_loc.calc_location_geopy(current_loc=current_loc, yaw_angle=current_yaw, tcp_data=tcp_data)
                        target_loc = (1,1,1)

                        hedefler[min_dist[0]] = target_loc
                        print(f"{min_dist[0]} hedefi eklendi, konumu: {min_dist[2]}")
                    else:
                        print(f"TCP Verisi alınmadı")
                
                else:
                    print(f"{min_dist[0]} hedefi zaten var, konumu: {hedefler[min_dist[0]]}")
        

        time.sleep(0.01)

#? Gerekliler
#config = json.load(open("./config.json"))
stop_event = threading.Event()

objects_queue = Queue()

# hedef_id, hedef_konumu
hedefler = {}

#? Kütüphaneler
#server = tcp_handler.TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
camera_handler = image_processing_handler.Handler(stop_event=stop_event, objects_queue=objects_queue)
camera_handler.start_proccessing("./models/kullanilcak.pt")
#gimbal_handler = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)

#? Threadler
#threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()
threading.Thread(target=camera_handler.local_camera, args=((0, )), daemon=True).start()
#threading.Thread(target=gimbal_handler.keyboard_controller, daemon=True).start()
threading.Thread(target=hedef_sec, args=(hedefler, objects_queue), daemon=True).start()

try:
    while not stop_event.is_set():
        if keyboard.is_pressed("q"):
            print("Çıkılıyor...")
            if not stop_event.is_set():
                stop_event.set()
            break
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
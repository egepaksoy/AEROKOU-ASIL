import time
import sys
import threading
from ultralytics import YOLO
import math
import json
import queue
sys.path.append('../pymavlink_custom')

from pymavlink_custom import Vehicle
from video_handler import Handler


def failsafe(vehicle, home_pos=None, config: json=None):
    def failsafe_drone_id(vehicle, drone_id, home_pos=None):
        vehicle.move_drone(rota=(0,0,0), drone_id=drone_id)
        if home_pos == None:
            print(f"{drone_id}>> Failsafe alıyor")
            vehicle.set_mode(mode="RTL", drone_id=drone_id)

        # guıdedli rtl
        else:
            print(f"{drone_id}>> Failsafe alıyor")
            vehicle.set_mode(mode="GUIDED", drone_id=drone_id)

            alt = vehicle.get_pos(drone_id=drone_id)[2]
            if config != None:
                if "DRONE" in config:
                    if "rtl-alt" in config["DRONE"]:
                        alt = config["DRONE"]["rtl-alt"]
            
            vehicle.go_to(loc=home_pos, alt=alt, drone_id=DRONE_ID)

            start_time = time.time()
            while True:
                if time.time() - start_time > 5:
                    print(f"{drone_id}>> Manuel RTL Alıyor...")
                    start_time = time.time()

                if vehicle.on_location(loc=home_pos, seq=0, sapma=1, drone_id=DRONE_ID):
                    print(f"{DRONE_ID}>> iniş gerçekleşiyor")
                    vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
                    break

    thraeds = []
    for d_id in vehicle.drone_ids:
        args = (vehicle, d_id)
        if home_pos != None:
            args = (vehicle, d_id, home_pos)

        thrd = threading.Thread(target=failsafe_drone_id, args=args)
        thrd.start()
        thraeds.append(thrd)


    for t in thraeds:
        t.join()

    print(f"{vehicle.drone_ids} id'li Drone(lar) Failsafe aldi")

#! ORTALAMA THREADİ YAZ
def ortalama_gorevi(stop_event, tarama, vehicle: Vehicle):
    while not stop_event.is_set():
        distance_yaw = video_handler.get_pixel_distance_yaw()
        if distance_yaw != None:
            tarama.set()
            break
            
        time.sleep(0.05)

    if not stop_event.is_set():
        print(f"{DRONE_ID}>> Hedefi tespit etti")
        vehicle.move_drone(rota=(0,0,0), drone_id=DRONE_ID)
        print(f"{DRONE_ID}>> Drone durduruldu")

    while int(vehicle.get_speed(drone_id=DRONE_ID)) != 0 and not stop_event.is_set():
        time.sleep(0.05)

    while not stop_event.is_set():
        distance_yaw = video_handler.get_pixel_distance_yaw()
        
        if distance_yaw != None:
            distance, yaw = distance_yaw

            vehicle.turn_way(turn_angle=yaw)
            abs_distance = math.sqrt(distance[0] ** 2 + distance[1] ** 2)
            dist = abs_distance / 400

            rota = (0, dist, 0)

        vehicle.move_drone(rota=rota, drone_id=DRONE_ID)

        if rota[0] == 0 and rota[1] == 0:
            break
        time.sleep(0.01)
    
    if not stop_event.is_set():
        print(f"{DRONE_ID}>> Hedefi ortaladı iniş gerçekleştiriyor")
        vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
    

config = json.load(open("./config.json", "r"))
stop_event = threading.Event()
udp_connected = threading.Event()
yuk_birakildi_event = threading.Event()

distance_yaw = queue.Queue()

#########GORUNTU ISLEME KISMI##############
model_name = config["UDP"]["model-path"]
model = YOLO(model_name)

video_handler = Handler(model=model, stop_event=stop_event, distance_yaw=distance_yaw, proccessing=True)
video_handler.show_image()

#threading.Thread(target=video_handler.ortalama_udp, args=(config["UDP"]["ip"], config["UDP"]["port"]), daemon=True).start()
threading.Thread(target=video_handler.local_camera, args=(0, ), daemon=True).start()

timer = time.time()
while not video_handler.udp_connected.is_set():
    if time.time() - timer >= 2:
        print("Kamera Bağlantısı Bekleniyor...")
        timer = time.time()

#########GOREV##########
DRONE_ID = config["DRONE"]["id"]
ALT = config["DRONE"]["alt"]

locs = config["DRONE"]["direk-locs"]
current_loc = locs[0]

tarama = threading.Event()

vehicle = Vehicle(config["DRONE"]["path"])
ortalama_thread = threading.Thread(target=ortalama_gorevi, args=(stop_event, tarama, vehicle), daemon=True)
try:
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.takeoff(alt=ALT, drone_id=DRONE_ID)
    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> {ALT} metreye takeoff yaptı")

    print(f"{DRONE_ID}>> Göreve Başlıyor")
    ortalama_thread.start()

    vehicle.go_to(loc=current_loc, alt=ALT, drone_id=DRONE_ID)

    timer = time.time()
    while not stop_event.is_set() and not tarama.is_set():
        if time.time() - timer >= 5:
            print(f"{DRONE_ID}>> Göreve Devam Ediyor")
            timer = time.time()
        
        if vehicle.on_location(loc=current_loc, seq=0, drone_id=DRONE_ID):
            if current_loc == locs[1]:
                break
            else:
                current_loc = locs[1]
                vehicle.go_to(loc=current_loc, alt=ALT, drone_id=DRONE_ID)
        
        time.sleep(0.01)
    
    if not tarama.is_set():
        if home_pos != None:
            vehicle.rtl(takeoff_pos=home_pos, alt=ALT, drone_id=DRONE_ID)
        
        else:
            vehicle.set_mode(mode="RTL", drone_id=DRONE_ID)
    
        print("Görev Tamamlandı")

    #! ortalama threadinin bitmesini bekle
    while ortalama_thread.is_alive():
        time.sleep(0.05)


except KeyboardInterrupt:
    print("Exiting...")
    if "home_pos" in locals():
        failsafe(vehicle, home_pos, config)
    else:
        failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()

except Exception as e:
    if "home_pos" in locals():
        failsafe(vehicle, home_pos, config)
    else:
        failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()
    vehicle.vehicle.close()
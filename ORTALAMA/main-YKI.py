import time
import sys
import threading
from ultralytics import YOLO
import cv2
import json
import queue
sys.path.append('../pymavlink_custom')

from pymavlink_custom import Vehicle
from video_handler import Handler


def failsafe(vehicle, home_pos=None, config: json=None):
    def failsafe_drone_id(vehicle, drone_id, home_pos=None):
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
                if time.time() - start_time > 3:
                    print(f"{drone_id}>> RTL Alıyor...")
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


config = json.load(open("./config.json", "r"))
stop_event = threading.Event()
udp_connected = threading.Event()
yuk_birakildi_event = threading.Event()

#########GORUNTU ISLEME KISMI##############
model_name = config["UDP"]["model-path"]
model = YOLO(model_name)

video_handler = Handler(model=model, processing=True)
video_handler.show_image()

image_queue = queue.Queue()
img_processing_thread = threading.Thread(target=video_handler.ortalama_udp, args=(config, image_queue), daemon=True)
img_processing_thread.start()

timer = time.time()
while not video_handler.udp_connected.is_set():
    if time.time() - timer >= 2:
        print("Bağlantı bekleniyor")
        timer = time.time()

#########GOREV##########
on_mission = False
yuk_birakildi = False

vehicle = Vehicle(config["DRONE"]["path"])

start_time = time.time()

pid_val = 0.5

ALT = config["DRONE"]["alt"]
DRONE_ID = config["DRONE"]["id"] # drone id
direk_locs = config["DRONE"]["direk-locs"]

try:
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.takeoff(ALT, drone_id=DRONE_ID)
    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    
    print(f"{DRONE_ID}>> takeoff yaptı")

    # TODO: daire cizme kodu ekle
    vehicle.go_to(loc=direk_locs[0], alt=ALT, drone_id=DRONE_ID)
    direk_count = 0

    start_time = time.time()
    on_miss_time = 0
    while not stop_event.is_set():
        if time.time() - start_time >= 3:
            print("Taranıyor..")
            start_time = time.time()
        
        if not image_queue.empty() and yuk_birakildi == False:
            class_name, (x_uzaklik, y_uzaklik) = image_queue.get_nowait()
            #! UDP'de duz gidiyor local'de ters oluyor
            y_uzaklik *= -1
            print(f"\n\n{DRONE_ID}>> DRONE {class_name} hedefini algıladı")
            print(f"x: {x_uzaklik}, y: {y_uzaklik}\n\n")

            if x_uzaklik < 0:
                x_rota = -1 * pid_val
            elif x_uzaklik > 0:
                x_rota = 1 * pid_val
            else:
                x_rota = 0
            if y_uzaklik < 0:
                y_rota = -1 * pid_val
            elif y_uzaklik > 0:
                y_rota = 1 * pid_val
            else:
                y_rota = 0
            
            print(f"{x_rota}|{y_rota}")
            
            on_mission = True

            if on_miss_time == 0 and abs(x_rota) + abs(y_rota) == 0:
                on_miss_time = time.time()
                if pid_val > 0.07:
                    pid_val /= 2

            if abs(x_rota) + abs(y_rota) != 0:
                on_miss_time = 0
        
        if on_mission:
            rota = (y_rota, x_rota, 0)
            vehicle.move_drone(rota, drone_id=DRONE_ID)
            
            if time.time() - on_miss_time > 4 and on_miss_time != 0:
                print("Yük bırakıldı")
                on_mission = False
                yuk_birakildi = True
                yuk_birakildi_event.set()
        
        if on_mission == False:
            if vehicle.on_location(direk_locs[0], seq=0, drone_id=DRONE_ID) and direk_count == 0:
                vehicle.go_to(loc=direk_locs[1], alt=ALT, drone_id=DRONE_ID)
                direk_count = 1

            if vehicle.on_location(direk_locs[1], seq=0, drone_id=DRONE_ID) and direk_count == 1:
                break
        
        if yuk_birakildi:
            print("Yük bırakıldı")
            break
        
    if yuk_birakildi:
        vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
    else:
        vehicle.rtl(takeoff_pos=home_pos, alt=config["DRONE"]["rtl-alt"], drone_id=DRONE_ID)
    
    print("Görev tamamlandı")

except KeyboardInterrupt:
    print("Exiting...")
    if "home_pos" in locals():
        failsafe(vehicle, home_pos, config)
    else:
        failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()
    if "video_handler" in locals():
        video_handler.is_running = False

except Exception as e:
    if "home_pos" in locals():
        failsafe(vehicle, home_pos, config)
    else:
        failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()
    if "video_handler" in locals():
        video_handler.is_running = False
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()
        print("Stop event set edildi")
    if "video_handler" in locals():
        video_handler.is_running = False
    vehicle.vehicle.close()
    cv2.destroyAllWindows()
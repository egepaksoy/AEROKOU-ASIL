import time
import cv2
import json
import threading
from flask import Flask
from picamera2 import Picamera2
import sys

from pymavlink_custom.pymavlink_custom import Vehicle
from libs.mqtt_controller import magnet_control, rotate_servo, cleanup
from libs.image_handler import image_recog_flask
from libs.calculater import get_distance


def failsafe(vehicle, home_pos=None):
    def failsafe_drone_id(vehicle, drone_id, home_pos=None):
        if home_pos == None:
            print(f"{drone_id}>> Failsafe alıyor")
            vehicle.set_mode(mode="RTL", drone_id=drone_id)

        # guıdedli rtl
        else:
            print(f"{drone_id}>> Failsafe alıyor")
            vehicle.set_mode(mode="GUIDED", drone_id=drone_id)

            alt = vehicle.get_pos(drone_id=drone_id)[2]
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

def drop_obj(obj, dropped_objects, miknatis, location, ALT, DRONE_ID, shared_state_lock, shared_state):
    vehicle.go_to(loc=location, alt=ALT, drone_id=DRONE_ID)
    while not stop_event.is_set() and not vehicle.on_location(loc=location, seq=0, sapma=1, drone_id=DRONE_ID):
        time.sleep(0.5)

    time.sleep(3)
    if miknatis == 2:
        magnet_control(True, False)
    else:
        magnet_control(False, True)
    print(f"mıknatıs {miknatis} kapatıldı")
    time.sleep(2)

    dropped_objects.append(obj)

    with shared_state_lock:
        shared_state["last_object"] = None  # tekrar tetiklenmesini engelle


stop_event = threading.Event()
config = json.load(open("./config.json", "r"))

# Algilananlari kullanma
shared_state = {"last_object": None, "object_pos": None}
shared_state_lock = threading.Lock()
objects = config["OBJECTS"]

dropped_objects = []
sonra_birakilcak_obj = None
sonra_birakilcak_pos = None

# PiCamera2'yi başlat ve yapılandır
'''
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()
time.sleep(2)  # Kamera başlatma süresi için bekle
'''
cap = cv2.VideoCapture(0)

# Görüntü işleme
app = Flask(__name__)
broadcast_started = threading.Event()
port = config["UDP-PORT"]
# Raspberry ile
threading.Thread(target=image_recog_flask, args=(cap, port, broadcast_started, stop_event, shared_state, shared_state_lock), daemon=True).start()
# Windows ile
#threading.Thread(target=image_recog_flask, args=(cap, port, broadcast_started, stop_event, shared_state, shared_state_lock), daemon=True).start()

# Erkenden miknatisi calistirma
magnet_control(True, True)
input("Mıknatıslar bağlandığında ENTER tuşuna basın")

# Drone ayarlamalari
vehicle = Vehicle(config["CONN-STR"])
DRONE_ID = config["DRONE"]["id"]
ALT = config["DRONE"]["alt"]
merkez = (config["DRONE"]["loc"][0], config["DRONE"]["loc"][1], ALT)
tarama_sayisi = config["DRONE"]["tarama_sayisi"]
yapilan_tarama_sayisi = 0
area_meter = config["DRONE"]["area_meter"]
distance_meter = config["DRONE"]["distance_meter"]

# Tarama ayarlama
drone_locs = vehicle.scan_area_wpler(center_loc=merkez, alt=ALT, area_meter=area_meter, distance_meter=distance_meter)
print(f"{yapilan_tarama_sayisi + 1}. tarama wp sayısı: {len(drone_locs)}")
print(f"{yapilan_tarama_sayisi + 1}. tarama alanı: {area_meter}")
print(f"{yapilan_tarama_sayisi + 1}. tarama aralık mesafesi: {distance_meter}")

try:
    # Görüntü gelmesini bekle
    while not stop_event.is_set() and not broadcast_started.is_set():
        time.sleep(0.5)
    
    # Tarama konumu dogrulugu kontrol
    pos = vehicle.get_pos(drone_id=DRONE_ID)
    pos = (pos[0], pos[1], ALT)
    merkez_dist = get_distance(pos, merkez)

    if merkez_dist > 80:
        print(f"Tarama merkezi dronedan {merkez_dist} metre uzakta konumu kontrol et")
        raise ValueError("Merkez WP Drondan çok uzak")
    else:
        print(merkez_dist)
    
    for loc in drone_locs:
        dist = get_distance(pos, loc)
        if dist > 100:
            print(f"Tarama waypoint'i dronedan {dist} metre uzakta konumu kontrol et")
            raise ValueError("WP Drondan çok uzak")

    # Drone ALT doğruluğu kontrol ediliyor        
    if ALT > 15 or ALT < 3:
        print(f"Drone yükseklik verisi doğru mu: {ALT}")
        raise ValueError("Drone Yuksekligi kontrol et")


    rotate_servo(0)
    print("servo duruyor")

    # Takeoff
    vehicle.set_mode("GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(True, drone_id=DRONE_ID)
    vehicle.takeoff(ALT, drone_id=DRONE_ID)

    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> Kalkış tamamlandı")

    # Tarama başlangıcı
    current_loc = 0
    vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)

    with shared_state_lock:
        shared_state["last_object"] = None  # tekrar tetiklenmesini engelle

    start_time = time.time()
    while not stop_event.is_set():
        with shared_state_lock:
            obj = shared_state["last_object"]
            obj_pos = shared_state_lock["object_pos"]

        if obj:
            if obj not in dropped_objects and obj != sonra_birakilcak_obj:
                print(obj)
                location = vehicle.get_pos(drone_id=DRONE_ID)

                sira = objects[obj]["sira"]
                miknatis = objects[obj]["miknatis"]
                
                if (sira == 1 or (len(dropped_objects) != 0 and obj not in dropped_objects)):
                    drop_obj(obj, dropped_objects, miknatis, location, ALT, DRONE_ID, shared_state_lock, shared_state)
                    rotate_servo(0)
                    
                    if len(dropped_objects) == 2:
                        print(f"{DRONE_ID}>> Drone hedeflere yük bıraktı")
                        break

                    else:
                        print(f"Yük {sira} bırakıldı tarama devam ediyor...")
                        vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)
                        start_time = time.time()

                else:
                    print(f"{obj} bulundu sonradan bırakılcak")
                    sonra_birakilcak_obj = obj
                    sonra_birakilcak_pos = location

            if sonra_birakilcak_obj != None and len(dropped_objects) != 0:
                if sonra_birakilcak_obj not in dropped_objects:
                    obj = sonra_birakilcak_obj
                    location = sonra_birakilcak_pos
                    print(obj)

                    miknatis = objects[obj]["miknatis"]
                    sira = objects[obj]["sira"]

                    drop_obj(obj, dropped_objects, miknatis, location, ALT, DRONE_ID, shared_state_lock, shared_state)
                    rotate_servo(0)
                    
                    if len(dropped_objects) == 2:
                        print(f"{DRONE_ID}>> Drone hedeflere yük bıraktı")
                        break

                    else:
                        sonra_birakilcak_obj = None
                        sonra_birakilcak_pos = None

                        print(f"Yük {sira} bırakıldı tarama devam ediyor...")
                        vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)
                        start_time = time.time()
        
        if vehicle.on_location(loc=drone_locs[current_loc], seq=0, sapma=1, drone_id=DRONE_ID):
            rotate_servo(0)

            print(f"{DRONE_ID}>> wp: {current_loc + 1}/{len(drone_locs)} ulasildi")

            if len(dropped_objects) == 2:
                print(f"{DRONE_ID}>> Yükler birakildi")
                break

            # Tarama bittiyse
            if current_loc + 1 == len(drone_locs):
                yapilan_tarama_sayisi += 1

                # Tarama bittiyse diger taramaya gec
                if tarama_sayisi - yapilan_tarama_sayisi > 0:
                    print(f"{yapilan_tarama_sayisi + 1}. tarama baslıyor")

                    new_distance_meter = distance_meter / (yapilan_tarama_sayisi + 1)
                    drone_locs = vehicle.scan_area_wpler(center_loc=merkez, alt=ALT, area_meter=area_meter, distance_meter=new_distance_meter)
                    
                    current_loc = 0

                    print(f"{yapilan_tarama_sayisi + 1}. tarama wp sayısı: {len(drone_locs)}")
                    print(f"{yapilan_tarama_sayisi + 1}. tarama alanı: {area_meter}")
                    print(f"{yapilan_tarama_sayisi + 1}. tarama aralık mesafesi: {new_distance_meter}")

                    vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)

                # Tarama bittiyse ve atilcak siradaki yuk kalmadiysa gorevi bitir
                elif sonra_birakilcak_obj == None and sonra_birakilcak_pos == None:
                    print(f"{DRONE_ID}>> Drone taramayı bitirdi")
                    break

            # Tarama bitmediyse sonraki wp'ye gec
            else:
                current_loc += 1
                vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)
                print(f"{DRONE_ID}>> {current_loc + 1}/{len(drone_locs)}. konuma gidiyor...")

        time.sleep(0.02)

    print(f"Algilanan objeler: {dropped_objects}")

    rotate_servo(0)
    print(f"{DRONE_ID}>> Kalkış konumuna gidiyor")
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.go_to(loc=home_pos, alt=ALT, drone_id=DRONE_ID)

    while not stop_event.is_set() and not vehicle.on_location(loc=home_pos, seq=0, sapma=1, drone_id=DRONE_ID):
        time.sleep(0.5)

    vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
    print("Görev tamamlandı")

except KeyboardInterrupt:
    print("Klavye ile çıkış yapıldı")
    if "home_pos" in locals():
        failsafe(vehicle, home_pos)
    else:
        failsafe(vehicle)

except Exception as e:
    if "home_pos" in locals():
        failsafe(vehicle, home_pos)
    else:
        failsafe(vehicle)
    print("Hata:", e)

finally:
    vehicle.vehicle.close()
    input("Servoyu kapatmak için Enter'a basın")
    cleanup()
    print("GPIO temizlendi, bağlantı kapatıldı")

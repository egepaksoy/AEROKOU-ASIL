import math
import time
import cv2
import json
import threading
from flask import Flask
#from picamera2 import Picamera2
import sys

from pymavlink_custom.pymavlink_custom import Vehicle
#from libs.mqtt_controller import magnet_control, rotate_servo, cleanup
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
    
def camera_distance(pos: list, screen_res: list, oran: float=0.3):
    orta_x1 = (screen_res[0] - (screen_res[0] * oran)) / 2
    orta_x2 = screen_res[0] - orta_x1
    orta_y1 = (screen_res[1] - (screen_res[1] * oran)) / 2
    orta_y2 = screen_res[1] - orta_y1

    pos_x, pos_y = pos

    x_dist = 0
    y_dist = 0
    
    if pos_x < orta_x1:
        x_dist = pos_x - orta_x1
    elif pos_x > orta_x2:
        x_dist = orta_x2 - pos_x
    if pos_y < orta_y1:
        y_dist = orta_y1 - pos_y
    elif pos_y > orta_y2:
        y_dist = orta_y2 - pos_y
    
    return x_dist, y_dist

def center_distance(pos: list, screen_res: list):
    return pos[0] - screen_res[0] / 2, screen_res[1] / 2 - pos[1]

def turn_angle_calculate(dist, fov, screen_res):
    return (fov * dist) / screen_res


def angle_from_center(pos, screen_res):
    cx, cy = screen_res[0] / 2.0, screen_res[1] / 2.0
    dx = pos[0] - cx
    dy = pos[1] - cy   # ekran koordinatında aşağı pozitif

    if dx == 0 and dy == 0:
        return 0.0  # merkezdeyse 0 derece diyelim

    # Normal atan2 kullanımı
    raw_angle = math.degrees(math.atan2(dy, dx))  # -180..180 (0° sağ)
    
    # Dönüştür: 0° yukarı olacak şekilde kaydır
    angle = (raw_angle + 90) % 360
    return angle


def drop_obj(obj_origin, dropped_objects, miknatis, ALT, DRONE_ID, shared_state, shared_state_lock, orta_oran):
    x_fov, y_fov = config["FOV"]
    
    print(f"{DRONE_ID}>> BREAK Yapiyor")
    vehicle.set_mode(mode="BREAK", drone_id=DRONE_ID)

    while vehicle.get_speed(drone_id=DRONE_ID) > 0.1 and not stop_event.is_set():
        time.sleep(0.05)
    
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)

    yukselicek = 5
    total_alt = ALT

    while not stop_event.is_set():
        with shared_state_lock:
            obj = shared_state["last_object"]
                    
        start_time = time.time()
        while obj == None and time.time() - start_time <= 1:
            with shared_state_lock:
                obj = shared_state["last_object"]
            
            time.sleep(0.05)
        
        if obj != None:
            if obj == obj_origin:
                print(f"{DRONE_ID}>> {obj} bulundu")
                break

        total_alt += yukselicek

        if total_alt >= 15:
            print(f"{DRONE_ID}>> Cok yukseldi alcalip taramaya devam ediyor")
            total_alt = ALT
            vehicle.go_to(loc=vehicle.get_pos(drone_id=DRONE_ID), alt=total_alt, drone_id=DRONE_ID)
            while vehicle.get_pos(drone_id=DRONE_ID)[2] < total_alt * 0.85 and not stop_event.is_set():
                time.sleep(0.5)
            return False

        vehicle.go_to(loc=vehicle.get_pos(drone_id=DRONE_ID), alt=total_alt, drone_id=DRONE_ID)
        
        print(f"{DRONE_ID}>> Daha genis arama icin {total_alt} metreye yukseliyor...")
        while vehicle.get_pos(drone_id=DRONE_ID)[2] < total_alt * 0.85 and not stop_event.is_set():
            time.sleep(0.5)
        
        print(f"{DRONE_ID}>> {total_alt} metreye yukseldi")
    
    with shared_state_lock:
        obj = shared_state["last_object"]

    start_time = time.time()
    while obj == None and time.time() - start_time <= 1:
        with shared_state_lock:
            obj = shared_state["last_object"]

    if obj == None:
        return False

    print(f"{DRONE_ID}>> {obj} ortalaniyor")
    # kamera ortasina hareket ettirme
    if obj == obj_origin:
        while not stop_event.is_set():
            with shared_state_lock:
                obj = shared_state["last_object"]
                obj_pos = shared_state["object_pos"]
                screen_res = shared_state["screen_res"]

            start_time = time.time()
            while obj == None and time.time() - start_time <= 1:
                with shared_state_lock:
                    obj = shared_state["last_object"]
                    obj_pos = shared_state["object_pos"]
                    screen_res = shared_state["screen_res"]
                
                time.sleep(0.05)
            
            if obj == None:
                print(f"{DRONE_ID}>> OBJE KAYBOLDU")
                return False

            if camera_distance(obj_pos, screen_res, orta_oran) != (0, 0):
                break

            # XDe hizalama
            while not stop_event.is_set():
                print(f"{DRONE_ID}>> Hedefe donuyor...")

                with shared_state_lock:
                    obj = shared_state["last_object"]
                    obj_pos = shared_state["object_pos"]
                    screen_res = shared_state["screen_res"]

                start_time = time.time()
                while obj == None and time.time() - start_time <= 1:
                    with shared_state_lock:
                        obj = shared_state["last_object"]
                        obj_pos = shared_state["object_pos"]
                        screen_res = shared_state["screen_res"]
                    
                    time.sleep(0.05)
                
                if obj == None:
                    print(f"{DRONE_ID}>> OBJE KAYBOLDU")
                    return False

                if camera_distance(obj_pos, screen_res, orta_oran)[0] == 0:
                    print(f"{DRONE_ID}>> Drone {obj} tarafina donduruldu")
                    break
                
                if turn_angle > 180:
                    turn_angle -= 360

                turn_angle = angle_from_center(obj_pos, screen_res)
                print("Donus acisi: ", turn_angle)

                vehicle.turn_way(turn_angle=turn_angle, drone_id=DRONE_ID)

                while vehicle.yaw_speed(drone_id=DRONE_ID) < 0.02:
                    time.sleep(0.05)

                while vehicle.yaw_speed(drone_id=DRONE_ID) >= 0.02:
                    time.sleep(0.05)
                
                print(f"{DRONE_ID}>> Nesneye donuldu")

            # YDe hizalama
            carpan = 0.5
            while not stop_event.is_set():
                print(f"{DRONE_ID}>> Y'de ortalaniyor")
                with shared_state_lock:
                    obj = shared_state["last_object"]
                    obj_pos = shared_state["object_pos"]
                    screen_res = shared_state["screen_res"]

                start_time = time.time()
                while obj == None and time.time() - start_time <= 1:
                    with shared_state_lock:
                        obj = shared_state["last_object"]
                        obj_pos = shared_state["object_pos"]
                        screen_res = shared_state["screen_res"]
                    
                    time.sleep(0.05)
                
                if obj == None:
                    print(f"{DRONE_ID}>> OBJE KAYBOLDU")
                    return False
            
                if camera_distance(obj_pos, screen_res, orta_oran)[1] == 0:
                    print(f"{DRONE_ID}>> Drone Y'De ortalandi")
                    break
                
                last_seen = vehicle.get_pos(drone_id=DRONE_ID)

                y_dist = center_distance(obj_pos, screen_res)[1]
                y_angle = turn_angle_calculate(y_dist, y_fov, screen_res[1])

                y_dist_meter = math.tan(math.radians(y_angle)) * total_alt

                print(f"{DRONE_ID}>> Hedef {obj} {y_dist_meter} metre uzakta")

                # buraya ilerleme eklenilcek
                vehicle.move_drone_body(rota=(y_dist_meter * carpan, 0, 0), drone_id=DRONE_ID)

                #! buraya daha duzgun cozum ekle
                start_time = time.time()
                while time.time() - start_time <= 3 and not stop_event.is_set():
                    time.sleep(0.05)

                print(f"{DRONE_ID}>> İleri gitti")
                
                if carpan <= 0.2:
                    carpan /= 1.5

                with shared_state_lock:
                    obj = shared_state["last_object"]
                    obj_pos = shared_state["object_pos"]
                    screen_res = shared_state["screen_res"]
                
                start_time = time.time()
                while obj == None and time.time() - start_time <= 1:
                    with shared_state_lock:
                        obj = shared_state["last_object"]
                        obj_pos = shared_state["object_pos"]
                        screen_res = shared_state["screen_res"]
                    
                    time.sleep(0.05)
                
                if obj == None:
                    print(f"{DRONE_ID}>> Nesne ortalama esnasinda kayboldu son gorunen konuma gidiliyor...")
                    
                    vehicle.go_to(loc=last_seen, alt=last_seen[2], drone_id=DRONE_ID)
                    while not vehicle.on_location(loc=last_seen, seq=0, drone_id=DRONE_ID):
                        time.sleep(0.5)
                    
                    print(f"{DRONE_ID}>> ilk gorunen noktaya geldi")
                
    else:
        return False
    
    print(f"{DRONE_ID}>> {obj} ortalandi")

    time.sleep(3)
    '''
    #! Test
    if miknatis == 2:
        magnet_control(True, False)
    else:
        magnet_control(False, True)
    '''
    print(f"mıknatıs {miknatis} kapatıldı")
    time.sleep(2)

    dropped_objects.append(obj_origin)

    with shared_state_lock:
        shared_state["last_object"] = None  # tekrar tetiklenmesini engelle
        shared_state["object_pos"] = None
        shared_state["screen_res"] = None
    
    return True


stop_event = threading.Event()
config = json.load(open("./config.json", "r"))

# Algilananlari kullanma
shared_state = {"last_object": None, "object_pos": None, "screen_res": None}
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

orta_oran = config["ORTA"]

# Görüntü işleme
app = Flask(__name__)
broadcast_started = threading.Event()
port = config["UDP-PORT"]
# Raspberry ile
threading.Thread(target=image_recog_flask, args=(cap, port, broadcast_started, stop_event, shared_state, shared_state_lock, orta_oran), daemon=True).start()
# Windows ile
#threading.Thread(target=image_recog_flask, args=(picam2, port, broadcast_started, stop_event, shared_state, shared_state_lock), daemon=True).start()

# Erkenden miknatisi calistirma
#!magnet_control(True, True)
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


    #!rotate_servo(0)
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

    while not stop_event.is_set():
        with shared_state_lock:
            obj = shared_state["last_object"]
            obj_pos = shared_state["object_pos"]

        if obj:
            if obj not in dropped_objects and obj != sonra_birakilcak_obj:
                print(obj)
                sira = objects[obj]["sira"]
                miknatis = objects[obj]["miknatis"]
                
                if ((sira == 1 or len(dropped_objects) != 0) and obj not in dropped_objects):
                    dropped = drop_obj(obj, dropped_objects, miknatis, ALT, DRONE_ID, shared_state, shared_state_lock, orta_oran)
                    if dropped == False:
                        print(f"{DRONE_ID}>> Hedefe yuk birakilmadi")
                    #!rotate_servo(0)
                    
                    if len(dropped_objects) == 2:
                        print(f"{DRONE_ID}>> Drone hedeflere yük bıraktı")
                        break

                    else:
                        print(f"Yük {sira} bırakıldı tarama devam ediyor...")
                        vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)

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

                    drop_obj(obj, dropped_objects, miknatis, ALT, DRONE_ID, shared_state, shared_state_lock, orta_oran)
                    if dropped == False:
                        print(f"{DRONE_ID}>> Hedefe yuk birakilmadi")
                    #!rotate_servo(0)
                    
                    if len(dropped_objects) == 2:
                        print(f"{DRONE_ID}>> Drone hedeflere yük bıraktı")
                        break

                    else:
                        sonra_birakilcak_obj = None
                        sonra_birakilcak_pos = None

                        print(f"Yük {sira} bırakıldı tarama devam ediyor...")
                        vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)
        
        if vehicle.on_location(loc=drone_locs[current_loc], seq=0, sapma=1, drone_id=DRONE_ID):
            #!rotate_servo(0)

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

    #!rotate_servo(0)
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
    #!cleanup()
    print("GPIO temizlendi, bağlantı kapatıldı")

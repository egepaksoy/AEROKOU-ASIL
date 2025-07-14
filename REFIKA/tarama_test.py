import time
import sys
import threading
import cv2
import numpy as np
import struct
import socket
from math import ceil
from flask import Flask, Response
import math
from geopy.distance import geodesic
from geopy.point import Point

sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle


def calculate_ground_distance(
    drone_height,
    xy_center,
    xy_screen,
    xy_fov
):
    fov_x_deg, fov_y_deg = xy_fov
    image_width, image_height = xy_screen
    pixel_x, pixel_y = xy_center

    # Piksel başına düşen açı (derece cinsinden)
    angle_per_pixel_x = fov_x_deg / image_width
    angle_per_pixel_y = fov_y_deg / image_height

    # Görüntünün merkezine göre fark
    dx = pixel_x - (image_width / 2)
    dy = pixel_y - (image_height / 2)

    # Merkezden sapma açıları (derece)
    angle_x_deg = dx * angle_per_pixel_x
    angle_y_deg = dy * angle_per_pixel_y

    # Açıları radyana çevir
    angle_x_rad = math.radians(angle_x_deg)
    angle_y_rad = math.radians(angle_y_deg)

    # Yere olan yatay uzaklıkları hesapla (tan(θ) = karşı/komşu => karşı = tan(θ) * komşu)
    ground_x = math.tan(angle_x_rad) * drone_height
    ground_y = math.tan(angle_y_rad) * drone_height

    # Öklidyen uzaklık (drone altından olan yatay mesafe)
    ground_distance = math.sqrt(ground_x**2 + ground_y**2)

    return ground_distance

def get_position(camera_distance, total_yaw, current_loc):
    start_loc = Point(current_loc[0], current_loc[1])
    hedef = geodesic(meters=camera_distance).destination(start_loc, bearing=total_yaw)

    return hedef.latitude, hedef.longitude

def failsafe(vehicle):
    def failsafe_drone_id(vehicle, drone_id):
        print(f"{drone_id}>> Failsafe alıyor")
        vehicle.set_mode(mode="RTL", drone_id=drone_id)
    threads = []
    for d_id in vehicle.drone_ids:
        t = threading.Thread(target=failsafe_drone_id, args=(vehicle, d_id))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()
    print(f"Dronlar {vehicle.drone_ids} Failsafe aldı")

def image_recog_flask(cap, port, broadcast_started, stop_event, shared_state: dict, shared_state_lock: threading.Lock):
    app = Flask(__name__)

    def is_equilateral(approx, tolerance=0.15):
        if len(approx) < 3:
            return False
        sides = []
        for i in range(len(approx)):
            pt1 = approx[i][0]
            pt2 = approx[(i + 1) % len(approx)][0]
            dist = np.linalg.norm(pt1 - pt2)
            sides.append(dist)
        mean = np.mean(sides)
        return all(abs(s - mean) / mean < tolerance for s in sides)

    # Renk aralıkları
    lower_red1 = np.array([100, 100, 0])
    upper_red1 = np.array([255, 255, 10])
    lower_red2 = np.array([100, 100, 160])
    upper_red2 = np.array([255, 255, 179])
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([255, 255, 130])

    @app.route('/')
    def video():
        def gen_frames():
            global last_frame
            frame_lock = threading.Lock()

            broadcast_started.set()
            while not stop_event.is_set():
                # 1. Kamera görüntüsü al ve çevir
                _, frame = cap.read()

                detected_obj = ""
                obj_pos = ()

                # 2. Görüntü işleme başlasın
                blurred = cv2.GaussianBlur(frame, (5, 5), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

                red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

                for color_mask, shape_name, target_sides, color in [
                    (red_mask, "Ucgen", 3, (255, 0, 0)),
                    (blue_mask, "Altigen", 6, (0, 0, 255))
                ]:
                    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for cnt in contours:
                        epsilon = 0.02 * cv2.arcLength(cnt, True)
                        approx = cv2.approxPolyDP(cnt, epsilon, True)
                        if len(approx) == target_sides and is_equilateral(approx):
                            cv2.drawContours(frame, [approx], 0, color, 2)
                            x, y = approx[0][0]
                            cv2.putText(frame, shape_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                            detected_obj = shape_name
                            # TODO: x, y'nin nereyi konumlandırdığını bul
                            obj_pos = (x, y)
                            print(detected_obj)

                if detected_obj != "":
                    with shared_state_lock:
                        shared_state["last_object"] = detected_obj
                        shared_state["obj_pos"] = obj_pos

                # 3. Görüntüyü paylaş
                with frame_lock:
                    last_frame = frame.copy()

                _, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    app.run(host='0.0.0.0', port=port)

def drop_obj(obj, dropped_objects, miknatis, location, ALT, DRONE_ID, shared_state_lock, shared_state):
    vehicle.go_to(loc=location, alt=ALT, drone_id=DRONE_ID)
    while not stop_event.is_set() and not vehicle.on_location(loc=location, seq=0, sapma=1, drone_id=DRONE_ID):
        time.sleep(0.5)

    time.sleep(3)
    print(f"mıknatıs {miknatis} kapatıldı")
    time.sleep(2)

    dropped_objects.append(obj)

    with shared_state_lock:
        shared_state["last_object"] = None  # tekrar tetiklenmesini engelle


if len(sys.argv) != 4:
    print("Usage: python main.py <connection_string> <drone_id> <port>")
    sys.exit(1)


stop_event = threading.Event()
shared_state = {"last_object": None, "obj_pos": None}
shared_state_lock = threading.Lock()

dropped_objects = []

# PiCamera2'yi başlat ve yapılandır
cap = cv2.VideoCapture(0)

#threading.Thread(target=image_recog_new, args=(picam2, sys.argv[3], int(sys.argv[4]), stop_event, broadcast_started, shared_state, shared_state_lock), daemon=True).start()
app = Flask(__name__)
last_frame = None
broadcast_started = threading.Event()
threading.Thread(target=image_recog_flask, args=(cap, int(sys.argv[3]), broadcast_started, stop_event, shared_state, shared_state_lock), daemon=True).start()

vehicle = Vehicle(sys.argv[1])
DRONE_ID = int(sys.argv[2])
ALT = 6

merkez = (-35.36298498, 149.16517906, ALT)

objects = {"Altigen": {"sira": 1, "miknatis": 2}, "Ucgen": {"sira": 2, "miknatis": 1}}

sonra_birakilcak_obj = None
sonra_birakilcak_pos = None

tarama_sayisi = 1
yapilan_tarama_sayisi = 0

area_meter = 10
distance_meter = 2

drone_locs = vehicle.scan_area_wpler(center_loc=merkez, alt=ALT, area_meter=area_meter, distance_meter=distance_meter)
print(f"{yapilan_tarama_sayisi + 1}. tarama wp sayısı: {len(drone_locs)}")
print(f"{yapilan_tarama_sayisi + 1}. tarama alanı: {area_meter}")
print(f"{yapilan_tarama_sayisi + 1}. tarama aralık mesafesi: {distance_meter}")

try:
    while not stop_event.is_set() and not broadcast_started.is_set():
        time.sleep(0.5)

    print("servo duruyor")

    vehicle.set_mode("GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(True, drone_id=DRONE_ID)
    vehicle.takeoff(ALT, drone_id=DRONE_ID)

    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> Kalkış tamamlandı")

    current_loc = 0
    vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)

    start_time = time.time()
    while not stop_event.is_set():
        with shared_state_lock:
            obj = shared_state["last_object"]
            obj_pos = shared_state["obj_pos"]

        if obj:
            if obj not in dropped_objects and sonra_birakilcak_obj != obj:
                print(f"{obj} algilandi")
                pos = vehicle.get_pos(drone_id=DRONE_ID)
                drone_yaw = vehicle.get_yaw(drone_id=DRONE_ID)

                uzaklik = calculate_ground_distance(drone_height=pos[2], xy_center=obj_pos, xy_screen=(640, 480), xy_fov=(62.2, 48.8))
                print("Kameradan uzaklik: ", uzaklik)
                location = get_position(camera_distance=uzaklik, total_yaw=drone_yaw, current_loc=pos)

                sira = objects[obj]["sira"]
                miknatis = objects[obj]["miknatis"]
                
                if sira == 1 or (len(dropped_objects) != 0 and obj not in dropped_objects):
                    drop_obj(obj, dropped_objects, miknatis, location, ALT, DRONE_ID, shared_state_lock, shared_state)
                
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
                miknatis = objects[obj]["miknatis"]

                drop_obj(obj, dropped_objects, miknatis, location, ALT, DRONE_ID, shared_state_lock, shared_state)
            
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
            start_time = time.time()
            print(f"{DRONE_ID}>> drone {current_loc + 1} ulasti")
            if current_loc + 1 == len(drone_locs):
                yapilan_tarama_sayisi += 1
                if tarama_sayisi - yapilan_tarama_sayisi > 0:
                    print(f"{yapilan_tarama_sayisi + 1}. tarama baslıyor")
                    new_distance_meter = distance_meter / (yapilan_tarama_sayisi + 1)
                    drone_locs = vehicle.scan_area_wpler(center_loc=merkez, alt=ALT, area_meter=area_meter, distance_meter=new_distance_meter)
                    current_loc = 0

                    print(f"{yapilan_tarama_sayisi + 1}. tarama wp sayısı: {len(drone_locs)}")
                    print(f"{yapilan_tarama_sayisi + 1}. tarama alanı: {area_meter}")
                    print(f"{yapilan_tarama_sayisi + 1}. tarama aralık mesafesi: {new_distance_meter}")

                    vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)
                elif sonra_birakilcak_obj == None and sonra_birakilcak_pos == None:
                    print(f"{DRONE_ID}>> Drone taramayı bitirdi")
                    break
            else:
                current_loc += 1
                vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)
                print(f"{DRONE_ID}>> {current_loc + 1}/{len(drone_locs)}. konuma gidiyor...")

        time.sleep(0.05)

    print(f"Algilanan objeler: {dropped_objects}")
    print(f"{DRONE_ID}>> Kalkış konumuna gidiyor")
    vehicle.go_to(loc=home_pos, alt=ALT, drone_id=DRONE_ID)

    while not stop_event.is_set() and not vehicle.on_location(loc=home_pos, seq=0, sapma=1, drone_id=DRONE_ID):
        time.sleep(0.5)

    vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
    print("Görev tamamlandı")

except KeyboardInterrupt:
    print("Klavye ile çıkış yapıldı")
    failsafe(vehicle)

except Exception as e:
    failsafe(vehicle)
    print("Hata:", e)

finally:
    vehicle.vehicle.close()
    input("Servoyu kapatmak için Enter'a basın")
    print("GPIO temizlendi, bağlantı kapatıldı")

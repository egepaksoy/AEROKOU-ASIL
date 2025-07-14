import time
import sys
import threading
import cv2
import numpy as np
import struct
import socket
from math import ceil
from flask import Flask, Response

sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle


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
def image_recog_flask(cap: cv2.VideoCapture, port, stop_event):
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
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])

    shared_state = {"last_object": None, "timestamp": 0}
    shared_state_lock = threading.Lock()

    @app.route('/')
    def video():
        def gen_frames():
            global last_frame
            frame_lock = threading.Lock()

            broadcast_started.set()
            while not stop_event.is_set():
                # 1. Kamera görüntüsü al ve çevir
                ret, frame = cap.read()
                frame = cv2.flip(frame, -1)
                detected_obj = ""

                # 2. Görüntü işleme başlasın
                blurred = cv2.GaussianBlur(frame, (5, 5), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

                for color_mask, shape_name, target_sides, color in [
                    (red_mask, "Ucgen", 3, (0, 0, 255)),
                    (blue_mask, "Altigen", 6, (255, 0, 0))
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
                            print(detected_obj)

                if detected_obj != "":
                    with shared_state_lock:
                        shared_state["last_object"] = detected_obj
                        shared_state["timestamp"] = time.time()

                # 3. Görüntüyü paylaş
                with frame_lock:
                    last_frame = frame.copy()

                _, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    app.run(host='0.0.0.0', port=port)

if len(sys.argv) != 4:
    print("Usage: python main.py <connection_string> <drone_id> <port>")
    sys.exit(1)


stop_event = threading.Event()
shared_state = {"last_object": None, "timestamp": 0}
shared_state_lock = threading.Lock()

dropped_objects = []

cap = cv2.VideoCapture(0)

#threading.Thread(target=image_recog_new, args=(picam2, sys.argv[3], int(sys.argv[4]), stop_event, broadcast_started, shared_state, shared_state_lock), daemon=True).start()
app = Flask(__name__)
last_frame = None
broadcast_started = threading.Event()
threading.Thread(target=image_recog_flask, args=(cap, int(sys.argv[3]), stop_event), daemon=True).start()

vehicle = Vehicle(sys.argv[1])
DRONE_ID = int(sys.argv[2])
ALT = 6

merkez = (-35.36291919, 149.16513516, ALT)

objects = {"Altigen": {"sira": 1, "miknatis": 2}, "Ucgen": {"sira": 2, "miknatis": 1}}

sonra_birakilcak_obj = None
sonra_birakilcak_pos = None

tarama_sayisi = 1
yapilan_tarama_sayisi = 0

area_meter = 10
distance_meter = 3

drone_locs = vehicle.scan_area_wpler(center_loc=merkez, alt=ALT, area_meter=area_meter, distance_meter=distance_meter)
print(f"{yapilan_tarama_sayisi + 1}. tarama wp sayısı: {len(drone_locs)}")
print(f"{yapilan_tarama_sayisi + 1}. tarama alanı: {area_meter}")
print(f"{yapilan_tarama_sayisi + 1}. tarama aralık mesafesi: {distance_meter}")

try:
    while not stop_event.is_set() and not broadcast_started.is_set():
        time.sleep(0.5)

    vehicle.set_mode("GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(True, drone_id=DRONE_ID)
    vehicle.takeoff(ALT, drone_id=DRONE_ID)

    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> Kalkış tamamlandı")

    current_loc = 0
    vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)

    with shared_state_lock:
        shared_state["last_object"] = None  # tekrar tetiklenmesini engelle

    start_time = time.time()
    while not stop_event.is_set():
        with shared_state_lock:
            obj = shared_state["last_object"]
            ts = shared_state["timestamp"]

        if obj:
            print(obj)
            if dropped_objects != []:
                print(dropped_objects)

            if obj not in dropped_objects and sonra_birakilcak_obj != obj:
                pos = vehicle.get_pos(drone_id=DRONE_ID)

                sira = objects[obj]["sira"]
                miknatis = objects[obj]["miknatis"]
                
                if sira == 1 or (len(dropped_objects) != 0 and obj not in dropped_objects):
                    print(f"{obj} bulundu")

                    vehicle.go_to(loc=pos, alt=ALT, drone_id=DRONE_ID)
                    while not stop_event.is_set() and not vehicle.on_location(loc=pos, seq=0, sapma=1, drone_id=DRONE_ID):
                        time.sleep(0.5)

                    time.sleep(3)
                    print(f"mıknatıs {miknatis} kapatıldı")
                    time.sleep(2)

                    print(f"Yük {sira} bırakıldı tarama devam ediyor...")
                    vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)

                    dropped_objects.append(obj)

                    with shared_state_lock:
                        shared_state["last_object"] = None  # tekrar tetiklenmesini engelle
                    start_time = time.time()

                else:
                    print(f"{obj} bulundu sonradan bırakılcak")
                    sonra_birakilcak_obj = obj
                    sonra_birakilcak_pos = pos

        if sonra_birakilcak_obj != None and len(dropped_objects) != 0:
            if sonra_birakilcak_obj not in dropped_objects:
                obj = sonra_birakilcak_obj
                pos = sonra_birakilcak_pos

                miknatis = objects[obj]["miknatis"]

                print(f"{obj} {pos} konumuna bırakılmaya gidiyor...")
                
                vehicle.go_to(loc=pos, alt=ALT, drone_id=DRONE_ID)
                while not stop_event.is_set() and not vehicle.on_location(loc=pos, seq=0, sapma=1, drone_id=DRONE_ID):
                    time.sleep(0.5)

                time.sleep(3)
                print(f"mıknatıs {miknatis} kapatıldı")
                time.sleep(2)
                
                print(f"Yük {objects[obj]['sira']} bırakıldı tarama devam ediyor...")
                vehicle.go_to(loc=drone_locs[current_loc], alt=ALT, drone_id=DRONE_ID)

                dropped_objects.append(obj)

                sonra_birakilcak_obj = None
                sonra_birakilcak_pos = None

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

import time
import sys
import threading
from picamera2 import Picamera2
import cv2
import numpy as np
import struct
import socket
import queue
from math import ceil

sys.path.append('./pymavlink_custom')
from pymavlink_custom import Vehicle
from mqtt_controller import magnet_control, rotate_servo, cleanup


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

# Açı hesaplama fonksiyonu
def calculate_angles(approx):
    """ İç açıları hesaplayan fonksiyon """
    angles = []
    for i in range(len(approx)):
        p1 = approx[i - 2][0]  # Önceki nokta
        p2 = approx[i - 1][0]  # Şu anki nokta
        p3 = approx[i][0]      # Sonraki nokta

        v1 = p1 - p2  # İlk vektör
        v2 = p3 - p2  # İkinci vektör

        # Kosinüs teoremi ile açıyı hesapla
        dot_product = np.dot(v1, v2)
        magnitude = np.linalg.norm(v1) * np.linalg.norm(v2)
        angle = np.arccos(dot_product / magnitude) * (180.0 / np.pi)  # Açıyı dereceye çevir
        angles.append(angle)

    return angles

def image_recog_new(ip: str, port: int, stop_event, shared_state: dict, shared_state_lock: threading.Lock):
    UDP_IP, UDP_PORT = ip, port
    CHUNK_SIZE = 1400                      # ~ MTU altı
    HEADER_FMT = '<LHB'                   # frame_id:uint32, chunk_id:uint16, is_last:uint8

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # PiCamera2'yi başlat ve yapılandır
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
    picam2.start()
    time.sleep(2)  # Kamera başlatma süresi için bekle

    frame_id = 0
    try:
        print(f"{UDP_IP} adresine gönderim başladı")
        while not stop_event.is_set():
            frame = picam2.capture_array()

            #### GORUNTU ISLEME #####
            detected_obj = None

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (9, 9), 0)
            edges = cv2.Canny(blurred, 100, 200)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Kırmızı üçgen için HSV aralığı
            lower_red = np.array([0, 120, 70])
            upper_red = np.array([10, 255, 255])
            mask_red = cv2.inRange(hsv, lower_red, upper_red)

            # Mavi altıgen için HSV aralığı
            lower_blue = np.array([100, 100, 100])
            upper_blue = np.array([130, 255, 255])
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            final_mask = cv2.bitwise_or(mask_red, mask_blue)
            result = cv2.bitwise_and(frame, frame, mask=final_mask)

            # ucgen
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_blue:
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if len(approx) == 3:
                    angles = calculate_angles(approx)
                    if all(30 <= angle <= 90 for angle in angles):
                        cv2.drawContours(frame, [approx], 0, (0, 0, 255), -1)
                        x, y = approx[0][0]
                        cv2.putText(frame, "Ucgen", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        #print("Ucgen tespit edildi")
                        detected_obj = "Ucgen"
                    
            # altıgen
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_blue:
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 6:
                    angles = calculate_angles(approx)
                    if all(110 <= angle <= 130 for angle in angles):
                        cv2.drawContours(frame, [approx], 0, (255, 0, 0), -1)
                        x, y = approx[0][0]  # İlk köşeye yazıyı ekle
                        cv2.putText(frame, "Altigen", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        #print("Altigen tespit edildi")
                        detected_obj = "Altigen"

            if detected_obj:
                with shared_state_lock:
                    shared_state["last_object"] = detected_obj
                    shared_state["timestamp"] = time.time()

            _, buf = cv2.imencode('.jpg', frame)
            data = buf.tobytes()
            total_chunks = ceil(len(data) / CHUNK_SIZE)
            ###### GORUNTU ISLEME BITTI ###########

            for chunk_id in range(total_chunks):
                start = chunk_id * CHUNK_SIZE
                end = start + CHUNK_SIZE
                chunk = data[start:end]
                is_last = 1 if chunk_id == total_chunks - 1 else 0
                header = struct.pack(HEADER_FMT, frame_id, chunk_id, is_last)
                sock.sendto(header + chunk, (UDP_IP, UDP_PORT))

            frame_id = (frame_id + 1) & 0xFFFFFFFF
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Ctrl+C ile çıkıldı.")

    finally:
        # Kamera ve soketi kapat
        print("Program sonlandırıldı.")
        picam2.stop()
        sock.close()


if len(sys.argv) != 5:
    print("Usage: python main.py <connection_string> <drone_id> <ip> <port>")
    sys.exit(1)


stop_event = threading.Event()
shared_state = {"last_object": None, "timestamp": 0}
shared_state_lock = threading.Lock()

dropped_objects = []

threading.Thread(target=image_recog_new, args=(sys.argv[3], int(sys.argv[4]), stop_event, shared_state, shared_state_lock), daemon=True).start()

magnet_control(True, True)
print("mıknatıslar açık")

vehicle = Vehicle(sys.argv[1])
DRONE_ID = int(sys.argv[2])
ALT = 5

loc1 = (40.7121241, 30.0245068, ALT)  # konum 1
loc2 = (40.7120884, 30.0245936, ALT)  # konum 2
locs = [loc1, loc2]

try:        
    rotate_servo(0)
    print("servo duruyor")

    vehicle.set_mode("GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(True, drone_id=DRONE_ID)
    vehicle.takeoff(ALT, drone_id=DRONE_ID)

    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> Kalkış tamamlandı")

    current_loc = 0
    vehicle.go_to(loc=locs[current_loc], alt=ALT, drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> Hedef 1'e gidiyor")
    
    with shared_state_lock:
        shared_state["last_object"] = None  # tekrar tetiklenmesini engelle

    start_time = time.time()
    while not stop_event.is_set():
        with shared_state_lock:
            obj = shared_state["last_object"]
            ts = shared_state["timestamp"]

        if obj and time.time() - ts <= 1:
            if obj == "Ucgen" and obj not in dropped_objects:
                #! stop kodu ekle
                print("Üçgen Tespit edildi")
                ucgen_loc = vehicle.get_pos(drone_id=DRONE_ID)
                vehicle.go_to(loc=ucgen_loc, drone_id=DRONE_ID)
                time.sleep(3)
                magnet_control(True, False)
                print("mıknatıs2 kapatıldı")
                time.sleep(2)
                print("Yük 2 bırakıldı tarama devam ediyor...")
                vehicle.go_to(loc=locs[current_loc], alt=ALT, drone_id=DRONE_ID)

                dropped_objects.append(obj)

                with shared_state_lock:
                    shared_state["last_object"] = None  # tekrar tetiklenmesini engelle
            
            if obj == "Altigen" and obj not in dropped_objects:
                #! stop kodu ekle
                print("Altıgen Tespit edildi")
                altigen_loc = vehicle.get_pos(drone_id=DRONE_ID)
                vehicle.go_to(loc=altigen_loc, drone_id=DRONE_ID)
                time.sleep(3)
                magnet_control(False, True)
                print("mıknatıs1 kapatıldı")
                time.sleep(2)
                print("Yük 1 bırakıldı tarama devam ediyor...")
                vehicle.go_to(loc=locs[current_loc], alt=ALT, drone_id=DRONE_ID)

                dropped_objects.append(obj)

                with shared_state_lock:
                    shared_state["last_object"] = None  # tekrar tetiklenmesini engelle
        
        if vehicle.on_location(loc=locs[current_loc], seq=0, sapma=1, drone_id=DRONE_ID):
            print(f"{DRONE_ID}>> Hedef {current_loc+1} ulaştı")
            if current_loc + 1 == len(locs):
                print("Son konuma gelindi kalkışa donuyor...")
                break
            current_loc += 1
            vehicle.go_to(loc=locs[current_loc], alt=ALT, drone_id=DRONE_ID)

        if time.time() - start_time >= 5:
            print("Nesneleri arıyor...")
            start_time = time.time()

        time.sleep(0.05)
    
    vehicle.rtl(takeoff_pos=home_pos, alt=ALT, drone_id=DRONE_ID)

except KeyboardInterrupt:
    print("Klavye ile çıkış yapıldı")
    failsafe(vehicle)

except Exception as e:
    failsafe(vehicle)
    print("Hata:", e)

finally:
    vehicle.vehicle.close()
    cleanup()
    print("GPIO temizlendi, bağlantı kapatıldı")

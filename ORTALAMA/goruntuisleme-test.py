import time
import sys
sys.path.append('./pymavlink_custom')
from pymavlink_custom import Vehicle
import threading
import cv2
import numpy as np
import struct
import json
import socket
import queue
from picamera2 import Picamera2

def is_centered(xyxy, ekran_orani, orta_pozisyon_orani: float = 0.4):
    if len(xyxy) >= 4:
        x, y = (xyxy[0] + (xyxy[2] - xyxy[0]) / 2), (xyxy[1] + (xyxy[3] - xyxy[1]) / 2)
    elif len(xyxy) == 2:
        x, y = xyxy
    ekran_genislik, ekran_yukseklik = ekran_orani
    # Merkezdeki alanın sınırlarını belirle     baslangıc, bitis
    x_orta_genisligi = ekran_genislik * orta_pozisyon_orani
    y_orta_genisligi = ekran_yukseklik * orta_pozisyon_orani

    x_uzaklik = 0
    y_uzaklik = 0

    if x > ekran_genislik * (0.5 + orta_pozisyon_orani / 2):
        x_uzaklik = x - ekran_genislik * (0.5 + orta_pozisyon_orani / 2)
    elif x < ekran_genislik * (0.5 - orta_pozisyon_orani / 2):
        x_uzaklik = x - ekran_genislik * (0.5 - orta_pozisyon_orani / 2)
    
    if y > ekran_yukseklik * (0.5 + orta_pozisyon_orani / 2):
        y_uzaklik = y - ekran_yukseklik * (0.5 + orta_pozisyon_orani / 2)
    elif y < ekran_yukseklik * (0.5 - orta_pozisyon_orani / 2):
        y_uzaklik = y - ekran_yukseklik * (0.5 - orta_pozisyon_orani / 2)
    
    
    return float(x_uzaklik), float(y_uzaklik)

def show_proccesed_image(config):
    #!!! zehranın son yaptıgı kod tamamı gelcek !!!#
    ekran_orani = (640, 480)
    orta_pozisyon_orani = 0.4

    def calculate_angles(approx):
        angles = []
        for i in range(len(approx)):
            p1 = approx[i - 2][0]  
            p2 = approx[i - 1][0]  
            p3 = approx[i][0]     

            v1 = p1 - p2  
            v2 = p3 - p2  

            
            dot_product = np.dot(v1, v2)
            magnitude = np.linalg.norm(v1) * np.linalg.norm(v2)
            angle = np.arccos(dot_product / magnitude) * (180.0 / np.pi) 
            angles.append(angle)

        return angles

    UDP_IP = config["UDP"]["ip"]  # Alıcı bilgisayarın IP adresi
    UDP_PORT = config["UDP"]["port"]  # Alıcı bilgisayarın port numarası
    # 61440
    PACKET_SIZE = 60000  # UDP paket boyutu, 60 KB

    # UDP soketi oluştur
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # PiCamera2'yi başlat ve yapılandır
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": ekran_orani}))
    picam2.start()

    #!cap = cv2.VideoCapture(0) 
    time.sleep(2)  # Kamera başlatma süresi için bekle

    #!if not cap.isOpened():
        #!print("Kamera açılamadı!")
        #!exit(1)

    frame_counter = 0  # Çerçeve sayacını başlat

    total_frame = 0
    try:
        while not stop_event.is_set():
            # Kamera görüntüsünü yakala
            #!ret, frame = cap.read()
            frame = picam2.capture_array()  # Kameradan görüntü al

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


            # kırmızı üçgen algılama
            # mavi altıgen algılama 
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

                        x_uzaklik, y_uzaklik = is_centered((x, y), ekran_orani)
                        image_queue.put(("Ucgen", (x_uzaklik, y_uzaklik)))
                
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_red:
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)# Mavi altıgenleri algıla
        
                if len(approx) == 6:
                    angles = calculate_angles(approx)
                    if all(90 <= angle <= 150 for angle in angles):
                        cv2.drawContours(frame, [approx], 0, (255, 0, 0), -1)
                        x, y = approx[0][0]  # İlk köşeye yazıyı ekle
                        cv2.putText(frame, "Altigen", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        #print("Altigen tespit edildi")
        
                        x_uzaklik, y_uzaklik = is_centered((x, y), ekran_orani)
                        image_queue.put(("Altigen", (x_uzaklik, y_uzaklik)))
            
            cv2.rectangle(frame, (int(ekran_orani[0] * (0.5 - orta_pozisyon_orani / 2)), int(ekran_orani[1] * (0.5 - orta_pozisyon_orani / 2))), (int(ekran_orani[0] * (0.5 + orta_pozisyon_orani / 2)), int(ekran_orani[1] * (0.5 + orta_pozisyon_orani / 2))), (255, 0, 0), 2)

            # Görüntüyü JPEG formatında sıkıştır
            _, buffer = cv2.imencode('.jpg', frame)
            buffer = buffer.tobytes()  # Veriyi baytlara dönüştür
            buffer_size = len(buffer)  # Verinin boyutunu al

            # Veriyi parçalara böl ve gönder
            for i in range(0, buffer_size, PACKET_SIZE):
                end = i + PACKET_SIZE
                if end > buffer_size:
                    end = buffer_size
                packet = struct.pack('<L', frame_counter) + buffer[i:end]
                sock.sendto(packet, (UDP_IP, UDP_PORT))

            # Son paketle bir "son" işareti gönder
            sock.sendto(struct.pack('<L', frame_counter) + b'END', (UDP_IP, UDP_PORT))
            
            if frame_counter >= 20000:
                frame_counter = 0
            frame_counter += 1  # Çerçeve sayacını artır
            total_frame += 1

                        
            frame_queue.put(frame)
            result_queue.put(result)


    finally:
        # Kamera ve soketi kapat
        print("Program sonlandırıldı.")
        picam2.stop()
        cv2.destroyAllWindows()  # Tüm OpenCV pencerelerini kapat
        #!cap.release()  # Kamerayı serbest bırak
        sock.close()


def failsafe(vehicle, home_pos=None, config: json=None):
    def failsafe_drone_id(vehicle, drone_id, home_pos=None):
        if home_pos == None:
            print(f"{drone_id}>> Failsafe alıyor")
            vehicle.set_mode(mode="RTL", drone_id=drone_id)

        # guıdedli rtl
        else:
            print(f"{drone_id}>> Failsafe alıyor")
            vehicle.set_mode(mode="GUIDED", drone_id=drone_id)

            alt = 5
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


############### GÖREV ###############
config = json.load(open("./config.json", "r"))
stop_event = threading.Event()

image_queue = queue.Queue()
frame_queue = queue.Queue()
result_queue = queue.Queue()

img_processing_thread = threading.Thread(target=show_proccesed_image, args=(config,), daemon=True)
img_processing_thread.start()

on_mission = False
yuk_birakildi = False

vehicle = Vehicle(config["DRONE"]["path"])

start_time = time.time()

ALT = config["DRONE"]["alt"]
DRONE_ID = config["DRONE"]["id"]
loc = config["DRONE"]["loc"]

try:
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.takeoff(ALT, drone_id=DRONE_ID)
    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    
    print(f"{DRONE_ID}>> takeoff yaptı")

    vehicle.go_to(loc=loc, alt=ALT, drone_id=DRONE_ID)

    start_time = time.time()
    on_miss_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time >= 3:
            print("Taranıyor..")
            start_time = time.time()
        
        if not image_queue.empty() and yuk_birakildi == False:
            class_name, (x_uzaklik, y_uzaklik) = image_queue.get()
            print(f"{DRONE_ID}>> DRONE {class_name} hedefini algıladı")
            print(f"x: {x_uzaklik}, y: {y_uzaklik}")

            if x_uzaklik < 0:
                x_rota = -1
            elif x_uzaklik > 0:
                x_rota = 1
            else:
                x_rota = 0
            if y_uzaklik < 0:
                y_rota = -1
            elif y_uzaklik > 0:
                y_rota = 1
            else:
                y_rota = 0
            
            on_mission = True

            if on_miss_time == 0 and abs(x_rota) + abs(y_rota) == 0:
                on_miss_time = time.time()
        
        if on_mission:
            rota = (x_rota, y_rota, 0)
            vehicle.move_drone(rota, drone_id=DRONE_ID)
            
            if time.time() - on_miss_time > 2 and on_miss_time != 0:
                print(f"{DRONE_ID}>> Alçalıyor")
                vehicle.go_to(loc=vehicle.get_pos(drone_id=DRONE_ID), alt=config["DRONE"]["top-birakma-alt"], drone_id=DRONE_ID)
                
                start_time = time.time()
                while not stop_event.is_set():
                    drone_alt = vehicle.get_pos(drone_id=DRONE_ID)[2]

                    if time.time() - start_time > 2:
                        print("Alçalıyor...")
                        start_time = time.time()
                    
                    if drone_alt <= config["DRONE"]["top-birakma-alt"] * 1.1:
                        break

                on_mission = False
                yuk_birakildi = True
                print("Top bırakıldı")
                break
        
        if on_mission == False:
            if vehicle.on_location(loc, seq=0, drone_id=DRONE_ID) or yuk_birakildi == True:
                break
        
    print("Kalkış konumunua dönülüyor")
    vehicle.rtl(takeoff_pos=home_pos, alt=config["DRONE"]["rtl-alt"], drone_id=DRONE_ID)
    
    print("Görev tamamlandı")

except KeyboardInterrupt:
    print("Exiting...")
    if "home_pos" in locals():
        failsafe(vehicle, home_pos, config)
        if "stop_event" in locals():
            stop_event.set()
    else:
        failsafe(vehicle)
        if "stop_event" in locals():
            stop_event.set()

except Exception as e:
    if "home_pos" in locals():
        failsafe(vehicle, home_pos, config)
        if "stop_event" in locals():
            stop_event.set()
    else:
        failsafe(vehicle)
        if "stop_event" in locals():
            stop_event.set()
    print(e)

finally:
    print("Program tamamen kapatiliyor...")
    if "stop_event" in locals():
        if not stop_event.is_set():
            stop_event.set()
    vehicle.vehicle.close()
    cv2.destroyAllWindows()
    img_processing_thread.join()
    print("Program tamamen kapatıldı")
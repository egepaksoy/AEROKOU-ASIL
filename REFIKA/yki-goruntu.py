import time
import sys
import threading
import cv2
import numpy as np
import socket

sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle
#from mqtt_controller import magnet_control, rotate_servo, cleanup


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

def image_recog(cap, stop_event, shared_state, shared_state_lock):
    # Renk aralıklarını HSV formatında tanımla
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])

    while not stop_event.is_set():
        _, frame = cap.read()
        detected_obj = None

        ############### TESPİT ################
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Maske oluştur
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Kontur bul
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

        if detected_obj:
            with shared_state_lock:
                shared_state["last_object"] = detected_obj
                shared_state["timestamp"] = time.time()

        cv2.imshow("frame", frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break


if len(sys.argv) != 3:
    print("Usage: python main.py <connection_string> <drone_id>")
    sys.exit(1)


stop_event = threading.Event()
shared_state = {"last_object": None, "timestamp": 0}
shared_state_lock = threading.Lock()

dropped_objects = []

cap = cv2.VideoCapture(0)

threading.Thread(target=image_recog, args=(cap, stop_event, shared_state, shared_state_lock), daemon=True).start()

print("mıknatıslar açık")

vehicle = Vehicle(sys.argv[1])
DRONE_ID = int(sys.argv[2])
ALT = 5

loc1 = (-35.36311920, 149.16510876, ALT)  # konum 1
loc2 = (-35.36299716, 149.16532272 , ALT)  # konum 2
locs = [loc1, loc2]

try:        
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
                print("Üçgen Tespit edildi")

                pos = vehicle.get_pos(drone_id=DRONE_ID)

                vehicle.go_to(loc=pos, alt=ALT, drone_id=DRONE_ID)
                while not stop_event.is_set() and not vehicle.on_location(loc=pos, seq=0, sapma=0.5, drone_id=DRONE_ID):
                    time.sleep(0.5)

                time.sleep(3)
                print("mıknatıs2 kapatıldı")
                time.sleep(2)

                vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
                print("Yük 2 bırakıldı tarama devam ediyor...")
                vehicle.go_to(loc=locs[current_loc], alt=ALT, drone_id=DRONE_ID)

                dropped_objects.append(obj)

                with shared_state_lock:
                    shared_state["last_object"] = None  # tekrar tetiklenmesini engelle
            
            if obj == "Altigen" and obj not in dropped_objects:
                print("Altıgen Tespit edildi")

                pos = vehicle.get_pos(drone_id=DRONE_ID)

                vehicle.go_to(loc=pos, alt=ALT, drone_id=DRONE_ID)
                while not stop_event.is_set() and not vehicle.on_location(loc=pos, seq=0, sapma=0.5, drone_id=DRONE_ID):
                    time.sleep(0.5)

                time.sleep(3)
                print("mıknatıs1 kapatıldı")
                time.sleep(2)
                
                vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
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
    print("GPIO temizlendi, bağlantı kapatıldı")

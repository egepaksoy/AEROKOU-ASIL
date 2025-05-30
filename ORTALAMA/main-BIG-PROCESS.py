import time
import sys
import threading
from ultralytics import YOLO
import json
import queue
import cv2
import math
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


def visualise(screen_res, middle_rect_range, frame, box=None, model=None, object_color=(255, 0, 0), middle_rect_color = (0, 0, 255)):
    y, x = screen_res
    middle_x1 = int((x - x*middle_rect_range) / 2)
    middle_x2 = int((x + x*middle_rect_range) / 2)

    middle_y1 = int((y - y*middle_rect_range) / 2)
    middle_y2 = int((y + y*middle_rect_range) / 2)


    cv2.rectangle(frame, (middle_x1, middle_y1), (middle_x2, middle_y2), middle_rect_color, 2)

    if box != None:
        # Sınıf ve güven skorunu al
        cls = int(box.cls[0].item())
        conf = box.conf[0].item()

        # Sınırlayıcı kutu koordinatlarını al
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        konumu = (int(x1 + (x2-x1)/2), int(y1 + (y2-y1)/2))

        # Sınıf adını al
        class_name = model.names[cls]
    
        cv2.rectangle(frame, (x1, y1), (x2, y2), object_color, 2)
        cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 10), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.9, object_color, 2)
        
        object_center = int((x1 + x2) / 2), int((y1 + y2) / 2)
        cv2.circle(frame, konumu, 5, object_color, 2)

        cv2.line(frame, object_center, (int(screen_res[1] / 2), int(screen_res[0] / 2)), (0, 255, 0), 1)

        return class_name, conf, (x1, y1, x2, y2)

def is_centered(screen_res, object_center):
    res_x, res_y = 1, 1
    obj_x, obj_y = object_center
    x, y = screen_res
    middle_x1 = int((x - x*middle_range) / 2)
    middle_x2 = int((x + x*middle_range) / 2)

    middle_y1 = int((y - y*middle_range) / 2)
    middle_y2 = int((y + y*middle_range) / 2)

    if obj_x <= middle_x2 and obj_x >= middle_x1:
        res_x = 0
    if obj_y <= middle_y2 and obj_y >= middle_y1:
        res_y = 0
    
    return res_x, res_y

def center_object_pixel_distance(screen_res, object_size):
    screen_center = screen_res[0] / 2, screen_res[1] / 2
    object_center = (int(object_size[0]) + int(object_size[2])) / 2, (int(object_size[1]) + int(object_size[3])) / 2

    if is_centered(screen_res, object_center) == (0,0):
        return (0,0)
    return (screen_center[1] - object_center[1]), (screen_center[0] - object_center[0]) * -1

def get_yaw(distance):
    return math.atan(distance[0] / distance[1])


def local_camera(stop_event, camera_path):
    cap = cv2.VideoCapture(camera_path)

    if not cap.isOpened():
        print(f"Dahili kamera {camera_path} açılamadı")
    
    while not stop_event.is_set():
        visualised = False
        ret, frame = cap.read()
        screen_res = frame.shape[:2]

        results = model(frame, verbose=False)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                if box.conf[0] < 0.90:
                    continue
                
                visualised = True
                class_name, conf, (x1, y1, x2, y2) = visualise(screen_res, middle_range, frame, box, model)
                distance = center_object_pixel_distance((frame.shape[:2]), (x1, y1, x2, y2))
                yaw = get_yaw(distance)

                print(f"Sınıf: {class_name}, Güven: {conf:.2f}, Konum: ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}")

                # Nesneyi çerçeve içine al ve etiketle
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    
        if not visualised:
            visualise(screen_res, middle_range, frame)

        cv2.imshow("local image", frame)

        # Çıkış için 'q' tuşuna basılması beklenir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break




config = json.load(open("./config.json", "r"))
stop_event = threading.Event()
udp_connected = threading.Event()
yuk_birakildi_event = threading.Event()

distance_yaw = queue.Queue()

#########GORUNTU ISLEME KISMI##############
model_name = config["UDP"]["model-path"]
model = YOLO(model_name)

#########GOREV##########
DRONE_ID = config["DRONE"]["id"]
ALT = config["DRONE"]["alt"]

locs = config["DRONE"]["direk-locs"]
current_loc = locs[0]

middle_range = 0.2

vehicle = Vehicle(config["DRONE"]["path"])
try:
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.takeoff(alt=ALT, drone_id=DRONE_ID)
    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> {ALT} metreye takeoff yaptı")

    print(f"{DRONE_ID}>> Göreve Başlıyor")

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
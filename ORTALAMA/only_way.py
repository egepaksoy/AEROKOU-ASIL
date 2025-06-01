import cv2
import time
import threading
import queue
import math
from ultralytics import YOLO
import sys
import json
import socket
import struct
import numpy as np
sys.path.append('../pymavlink_custom')

from pymavlink_custom import Vehicle



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





def visualise(screen_res, frame, middle_size, box=None, model=None, object_color=(255, 0, 0), middle_rect_color = (0, 0, 255)):
    x, y = screen_res
    middle_x1 = int((x - x*middle_size) / 2)
    middle_x2 = int((x + x*middle_size) / 2)

    middle_y1 = int((y - y*middle_size) / 2)
    middle_y2 = int((y + y*middle_size) / 2)


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

        cv2.line(frame, object_center, (int(x / 2), int(y / 2)), (0, 255, 0), 1)
        
        #print(f"Sınıf: {class_name}, Güven: {conf:.2f}")

        return class_name, conf, (x1, y1, x2, y2)

def get_yaw(xy_center, screen_center):
    x_center, y_center = xy_center
    screen_x, screen_y = screen_center
    x, y = x_center - screen_x, y_center - screen_y
    if x % 90 == 0:
        x += 1
    if y % 90 == 0:
        y += 1


    if x > 0 and y < 0:
        deg = math.degrees(math.atan(x/(y*-1)))
        
    elif x > 0 and y > 0:
        deg = math.degrees(math.atan(y/x)) + 90
    
    elif x < 0 and y < 0:
        deg = math.degrees(math.atan(x/y)) * -1
    
    elif x < 0 and y > 0:
        deg = (math.degrees(math.atan(y/(x*-1))) + 90) * -1
    
    return deg

def get_distance(xy_center, screen, middle_range):
    screen_x, screen_y = screen
    obj_center_x, obj_center_y = xy_center

    middle_x1 = int((screen_x - screen_x*middle_range) / 2)
    middle_x2 = int((screen_x + screen_x*middle_range) / 2)

    middle_y1 = int((screen_y - screen_y*middle_range) / 2)
    middle_y2 = int((screen_y + screen_y*middle_range) / 2)


    dist_x = abs(screen_x / 2 - obj_center_x)
    dist_y = abs(screen_y / 2 - obj_center_y)
    if obj_center_x >= middle_x1 and obj_center_x <= middle_x2:
        dist_x = 0
    if obj_center_y >= middle_y1 and obj_center_y <= middle_y2:
        dist_y = 0

    return math.sqrt(dist_x**2 + dist_y**2)

def local_camera(camera_path, model, stop_event, algilandi, yaw, distance, camera_connected, middle_range=0.2):
    cap = cv2.VideoCapture(camera_path)

    if not cap.isOpened():
        print(f"Dahili kamera {camera_path} açılamadı")
    
    camera_connected.set()
    while not stop_event.is_set():
        visualised = False
        _, frame = cap.read()
        screen_res = frame.shape[:2]
        screen_res = (int(screen_res[1]), int(screen_res[0]))
        
        screen_center = screen_res[0] / 2, screen_res[1] / 2

        results = model(frame, verbose=False)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                if box.conf[0] < 0.90:
                    continue
                
                visualised = True
                _, _, xyxy = visualise(screen_res, frame, middle_range, box, model)
                if not algilandi.is_set():
                    algilandi.set()
                
                center_xy = (xyxy[2] + xyxy[0]) / 2, (xyxy[3] + xyxy[1]) / 2
                yaw.put_nowait(get_yaw(center_xy, screen_center))
                distance.put_nowait(get_distance(center_xy, screen_res, middle_range))

    
        if not visualised:
            visualise(screen_res, frame, middle_range)

        cv2.imshow("local image", frame)

        # Çıkış için 'q' tuşuna basılması beklenir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def udp_camera(ip, port, model, stop_event, algilandi, yaw, distance, camera_connected, middle_range=0.2):
    BUFFER_SIZE = 65536
    HEADER_FMT = '<LHB'
    HEADER_SIZE = struct.calcsize(HEADER_FMT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))

    buffers = {}  # {frame_id: {chunk_id: bytes, …}, …}
    expected_counts = {}  # {frame_id: total_chunks, …}

    camera_connected.set()
    while not stop_event.is_set():
        visualised = False
        packet, _ = sock.recvfrom(BUFFER_SIZE)
        frame_id, chunk_id, is_last = struct.unpack(HEADER_FMT, packet[:HEADER_SIZE])
        chunk_data = packet[HEADER_SIZE:]
        
        # Kaydet
        if frame_id not in buffers:
            buffers[frame_id] = {}
        buffers[frame_id][chunk_id] = chunk_data
        
        # Toplam parça sayısını son pakette işaretle
        if is_last:
            expected_counts[frame_id] = chunk_id + 1

        # Hepsi geldiyse işle
        if frame_id in expected_counts and len(buffers[frame_id]) == expected_counts[frame_id]:
            # Birleştir
            data = b''.join(buffers[frame_id][i] for i in range(expected_counts[frame_id]))
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            
            if frame is not None:
                screen_res = frame.shape[:2]
                screen_res = (int(screen_res[1]), int(screen_res[0]))

                screen_center = screen_res[0] / 2, screen_res[1] / 2

                results = model(frame, verbose=False)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        if box.conf[0] < 0.90:
                            continue
                        
                        visualised = True
                        _, _, xyxy = visualise(screen_res, frame, middle_range, box, model)
                        if not algilandi.is_set():
                            algilandi.set()
                        
                        center_xy = (xyxy[2] + xyxy[0]) / 2, (xyxy[3] + xyxy[1]) / 2
                        yaw.put_nowait(get_yaw(center_xy, screen_center))
                        distance.put_nowait(get_distance(center_xy, screen_res, middle_range))

            
                if not visualised:
                    visualise(screen_res, frame, middle_range)

                cv2.imshow("udp image", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    # Temizlik
    if frame_id in buffers:
        del buffers[frame_id]
    if frame_id in expected_counts:
        del expected_counts[frame_id]


config = json.load(open("./config.json", "r"))

algilandi = threading.Event()
stop_event = threading.Event()
camera_connected = threading.Event()

yaw = queue.Queue()
distance = queue.Queue()

#########GORUNTU ISLEME KISMI##############
model_name = config["UDP"]["model-path"]
model = YOLO(model_name)

threading.Thread(target=local_camera, args=(0, model, stop_event, algilandi, yaw, distance, camera_connected), daemon=True).start()
#threading.Thread(target=udp_camera, args=(config["UDP"]["ip"], config["UDP"]["port"], model, stop_event, algilandi, yaw, distance, camera_connected), daemon=True).start()

while not stop_event.is_set() and not camera_connected.is_set():
    time.sleep(0.05)

#########GOREV##########
DRONE_ID = config["DRONE"]["id"]
ALT = config["DRONE"]["alt"]

locs = config["DRONE"]["direk-locs"]
current_loc = locs[0]

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
    while not stop_event.is_set():
        if time.time() - timer >= 5:
            print(f"{DRONE_ID}>> Göreve Devam Ediyor")
            timer = time.time()
        
        if vehicle.on_location(loc=current_loc, seq=0, drone_id=DRONE_ID):
            if current_loc == locs[1]:
                break
            else:
                current_loc = locs[1]
                vehicle.go_to(loc=current_loc, alt=ALT, drone_id=DRONE_ID)
        
        if algilandi.is_set():
            print(f"{DRONE_ID}>> Objeyi algıladı")
            obj_loc = vehicle.get_pos(drone_id=DRONE_ID)
            break
        
        time.sleep(0.01)
    
    if algilandi.is_set():
        # DUR
        while not stop_event.is_set() and vehicle.get_speed(drone_id=DRONE_ID) > 0.1:
            time.sleep(0.05)

        time.sleep(3)
        print("Objenin konumuna gidiyor...")
        vehicle.go_to(loc=obj_loc, drone_id=DRONE_ID)
        while not stop_event.is_set() and not vehicle.on_location(loc=obj_loc, seq=0, sapma=0.7, drone_id=DRONE_ID):
            time.sleep(0.05)
        
        time.sleep(4)
        
        # DON
        print("Yaw açısı çekiliyor...")
        while yaw.empty():
            time.sleep(0.05)
        yaw = yaw.get()
        print(f"yaw: {yaw}")
        vehicle.turn_way(turn_angle=yaw, drone_id=DRONE_ID)
        time.sleep(1)
        while not stop_event.is_set() and vehicle.yaw_speed(drone_id=DRONE_ID) >= 0.1:
            time.sleep(0.01)
        
        time.sleep(1)
        # GIT
        print("İleri gidiyor...")
        while distance.empty():
            time.sleep(0.01)
        distance_flt = distance.get()
        while not stop_event.is_set():
            distance_flt = distance.get()
            if time.time() - timer >= 0.25:
                vehicle.move_drone_body(rota=(distance_flt/640, 0, 0), drone_id=DRONE_ID)
                timer = time.time()

            if distance_flt == 0:
                print(f"{DRONE_ID}>> ortaladı iniş gerçekleştiriyor")
                vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
                break
            time.sleep(0.01)

    else:
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
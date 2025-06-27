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
from geopy.distance import geodesic
from geopy.point import Point
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


def get_position(camera_distance, camera_yaw, current_loc):
    start_loc = Point(current_loc[0], current_loc[1])
    hedef = geodesic(meters=camera_distance).destination(start_loc, bearing=camera_yaw)

    return hedef.latitude, hedef.longitude

def get_location_distance(loc1, loc2):
    return geodesic(loc1, loc2).meters

    
def local_camera(vehicle, camera_path, model, stop_event, algilandi, yaw, distance, camera_connected, middle_range=0.2, xy_fov=(62.2, 48.8)):
    detected = False
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
                if detected == False:
                    uzaklik = calculate_ground_distance(drone_height=vehicle.get_pos()[2], xy_center=center_xy, xy_screen=screen_res, xy_fov=xy_fov)
                    print("Kameradan uzaklik: ", uzaklik)
                    distance.put_nowait(uzaklik)

                detected = True

    
        if not visualised:
            visualise(screen_res, frame, middle_range)

        cv2.imshow("local image", frame)

        # Çıkış için 'q' tuşuna basılması beklenir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def udp_camera(vehicle, ip, port, model, stop_event, algilandi, yaw, distance, camera_connected, middle_range=0.2, xy_fov=(62.2, 48.8)):
    detected = False

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
                        if detected == False:
                            uzaklik = calculate_ground_distance(drone_height=vehicle.get_pos()[2], xy_center=center_xy, xy_screen=screen_res, xy_fov=xy_fov)
                            print("Kameradan uzaklik: ", uzaklik)
                            distance.put_nowait(uzaklik)

                        detected = True

            
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

#########GOREV##########
DRONE_ID = config["DRONE"]["id"]
ALT = config["DRONE"]["alt"]

locs = config["DRONE"]["direk-locs"]
current_loc = locs[0]

vehicle = Vehicle(config["DRONE"]["path"])

threading.Thread(target=local_camera, args=(vehicle, 0, model, stop_event, algilandi, yaw, distance, camera_connected), daemon=True).start()
#threading.Thread(target=udp_camera, args=(vehicle, config["UDP"]["ip"], config["UDP"]["port"], model, stop_event, algilandi, yaw, distance, camera_connected), daemon=True).start()

while not stop_event.is_set() and not camera_connected.is_set():
    time.sleep(0.05)

try:
    print(f"{DRONE_ID}>> Göreve Başlıyor")

    timer = time.time()
    while not stop_event.is_set():
        if time.time() - timer >= 5:
            print(f"{DRONE_ID}>> Göreve Devam Ediyor")
            timer = time.time()
        
        if algilandi.is_set():
            camera_yaw = yaw.get()

            print(f"{DRONE_ID}>> Objeyi algıladı")
            drone_loc = vehicle.get_pos(drone_id=DRONE_ID)
            #obj_pos = get_position(camera_distance=distance.get(), camera_yaw=yaw.get(), current_loc=drone_loc)
            obj_pos = get_position(camera_distance=distance.get(), camera_yaw=((camera_yaw + vehicle.get_yaw()) % 360), current_loc=drone_loc)
            print("Drondan uzaklik: ", get_location_distance(drone_loc[:2], obj_pos))
            print("Kamera yaw'ı: ", camera_yaw)
            break

        time.sleep(0.01)
    
    if algilandi.is_set():
        vehicle.set_mode(mode="GUIDED")
        vehicle.go_to(loc=obj_pos, drone_id=DRONE_ID)
        print("Objenin konumuna gidiyor...")
        while not stop_event.is_set() and not vehicle.on_location(loc=obj_pos, seq=0, sapma=0.7, drone_id=DRONE_ID):
            time.sleep(0.05)

        vehicle.set_mode(mode="LAND")

    else:
        vehicle.set_mode(mode="RTL", drone_id=DRONE_ID)

    print("Görev Tamamlandı")


except KeyboardInterrupt:
    print("Exiting...")
    failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()

except Exception as e:
    failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()
    vehicle.vehicle.close()


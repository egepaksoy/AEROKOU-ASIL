from picamera2 import Picamera2
import cv2
import time
import threading
import queue
import sys
import json
import struct
import socket
from math import ceil
sys.path.append('./pymavlink_custom')

from pymavlink_custom_nonblock import Vehicle
from tcp_handler import TCPServer



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


def goruntu_aktarimi(config, camera_connected, stop_event):
    UDP_IP, UDP_PORT = config["UDP"]["ip"], config["UDP"]["port"]
    CHUNK_SIZE = 1400                      # ~ MTU altı
    HEADER_FMT = '<LHB'                   # frame_id:uint32, chunk_id:uint16, is_last:uint8

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # PiCamera2'yi başlat ve yapılandır
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
    picam2.start()
    time.sleep(2)  # Kamera başlatma süresi için bekle

    frame_id = 0

    print(f"{UDP_IP} adresine gönderim başladı")
    while not stop_event.is_set():
        frame = picam2.capture_array()
        _, buf = cv2.imencode('.jpg', frame)
        data = buf.tobytes()
        total_chunks = ceil(len(data) / CHUNK_SIZE)

        for chunk_id in range(total_chunks):
            start = chunk_id * CHUNK_SIZE
            end = start + CHUNK_SIZE
            chunk = data[start:end]
            is_last = 1 if chunk_id == total_chunks - 1 else 0
            header = struct.pack(HEADER_FMT, frame_id, chunk_id, is_last)
            sock.sendto(header + chunk, (UDP_IP, UDP_PORT))

        if not camera_connected.is_set():
            camera_connected.set()

        frame_id = (frame_id + 1) & 0xFFFFFFFF
        
        time.sleep(0.01)

def mavlink_sender(vehicle, DRONE_ID, server, obj_pos, algilandi, stop_event):
    while not stop_event.is_set():
        recieved_data = server.get_data()
        
        if recieved_data != None:
            if recieved_data.strip() == "get_data":
                drone_loc = vehicle.get_pos(drone_id=DRONE_ID)
                drone_yaw = vehicle.get_yaw(drone_id=DRONE_ID)

                server.send_data(f"{drone_loc}|{drone_yaw}\n")
            
            else:
                obj_loc = [float(x) for x in recieved_data.strip().strip("()").split(",")]
                obj_pos.put(obj_loc)
                if not algilandi.is_set():
                    algilandi.set()
                break


config = json.load(open("./config_ucus.json", "r"))

algilandi = threading.Event()
stop_event = threading.Event()
camera_connected = threading.Event()

obj_pos_queue = queue.Queue()

#########GOREV##########
DRONE_ID = config["DRONE"]["id"]
ALT = config["DRONE"]["alt"]

locs = config["DRONE"]["direk-locs"]
current_loc = locs[0]

vehicle = Vehicle(config["DRONE"]["path"])
server = TCPServer(config["TCP"]["port"], stop_event)

threading.Thread(target=mavlink_sender, args=(vehicle, DRONE_ID, server, obj_pos_queue, algilandi, stop_event), daemon=True).start()
threading.Thread(target=goruntu_aktarimi, args=(config, camera_connected, stop_event), daemon=True).start()

while not stop_event.is_set() and not camera_connected.is_set():
    time.sleep(0.05)

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
            obj_pos = obj_pos_queue.get()
            print("Hedef konumu algılandı gidiliyor...")
            break
        
        time.sleep(0.01)
    
    if algilandi.is_set():
        vehicle.go_to(loc=obj_pos, drone_id=DRONE_ID)
        print("Objenin konumuna gidiyor...")
        while not stop_event.is_set() and not vehicle.on_location(loc=obj_pos, seq=0, sapma=0.7, drone_id=DRONE_ID):
            time.sleep(0.05)

        vehicle.set_mode(mode="LAND")

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



import threading
import json
import time, sys
import keyboard

sys.path.append("../pymavlink_custom/")
from pymavlink_custom import Vehicle
from libs import new_image_processing_handler
from libs.gimbal_controller import GimbalHandler
from libs.tcp_handler import TCPServer
from libs.calc_loc import calc_location_geopy, get_dist
from libs.serial_handler import Serial_Control

# En güncel nesne verisini tutan shared değişken
latest_objects = []
latest_lock = threading.Lock()

# Hedefler sözlüğü
hedefler = {}

# Stop sinyali
stop_event = threading.Event()


def failsafe(vehicle):
    def failsafe_drone_id(vehicle, drone_id):
        print(f"{drone_id}>> Failsafe alıyor")
        vehicle.set_mode(mode="RTL", drone_id=drone_id)

    thraeds = []
    for d_id in vehicle.drone_ids:
        args = (vehicle, d_id)

        thrd = threading.Thread(target=failsafe_drone_id, args=args)
        thrd.start()
        thraeds.append(thrd)


    for t in thraeds:
        t.join()

    print(f"{vehicle.drone_ids} id'li Drone(lar) Failsafe aldi")

def arduino_listener(arduino: Serial_Control, stop_event: threading.Event):
    global rtl1, rtl2, rtl3, abort_miss
    global target_select, target_delete

    while not stop_event.is_set():
        try:
            arduino_data = arduino.read_value()

            if arduino_data != None:
                arduino_data = {k: int(v) for k, v in (pair.split(":") for pair in arduino_data.strip().split())}

                abort_miss = int(arduino_data["SYSTEM"])
                i = 1
                for rtl in rtls:
                    rtls[rtl] = int(arduino_data[f"ARM{i}"])
                    i+=1

                target_select, target_delete = int(arduino_data["TARGET_SELECT"]), int(arduino_data["TARGET_DELETE"])
            time.sleep(0.05)
        except Exception:
            continue

def miss_controller(vehicle, stop_event):
    global rtls, abort_miss

    while not stop_event.is_set():
        if abort_miss:
            if not stop_event.is_set():
                stop_event.set()
            failsafe(vehicle)
        else:
            for rtl in rtls.items():
                if rtl[1]:
                    if rtl[0] in vehicle.drone_ids:
                        vehicle.set_mode(mode="RTL", drone_id=rtl[0])

def select_thread_arduino(stop_event: threading.Event, server: TCPServer, vehicle: Vehicle, DRONE_ID: int, arduino: Serial_Control):
    global hedefler
    global target_select, target_delete

    while not stop_event.is_set():
        arduino_val = arduino.read_value()
        if arduino_val == None:
            continue

        if target_select:
            with latest_lock:
                current_objects = latest_objects.copy()
                
            print("Hedef konumu aliniyor...")
            server.send_data("2|2\n")

            tcp_data = server.get_data()
            while tcp_data == None:
                tcp_data = server.get_data()
                time.sleep(0.5)

            obj_name = None
            if current_objects:
                obj_name = min(current_objects, key=lambda x: x[1])[0]
            else:
                obj_name = input("Obje adi giriniz: ")

            if obj_name != None:
                if obj_name not in hedefler:
                    
                    target_loc = calc_location_geopy(current_loc=vehicle.get_pos(drone_id=DRONE_ID), yaw_angle=vehicle.get_yaw(drone_id=DRONE_ID), tcp_data=tcp_data)

                    min_dist = None
                    min_dist_drone_id = None
                    for drone_id in vehicle.drone_ids:
                        if drone_id == DRONE_ID:
                            continue
                        
                        if any(value[0] == drone_id for value in hedefler.values()):
                            continue

                        drone_pos = vehicle.get_pos(drone_id=drone_id)
                        
                        if min_dist == None:
                            min_dist = get_dist(drone_pos, target_loc)
                            min_dist_drone_id = drone_id
                            continue

                        if get_dist(drone_pos, target_loc) < min_dist:
                            min_dist = get_dist(drone_pos, target_loc)
                            min_dist_drone_id = drone_id

                    hedefler[obj_name] = [min_dist_drone_id, target_loc]
                    print(f"Hedef {obj_name} eklendi.\nKonumu: {target_loc}\nDronu: {min_dist_drone_id}")
            
            target_select = 0

        elif target_delete:
            with latest_lock:
                current_objects = latest_objects.copy()
            delete_hedef = None
            if current_objects:
                delete_hedef = min(current_objects, key=lambda x: x[1])[0]

            else:
                delete_hedef = input("Silinecek hedef adi girin: ")
                if delete_hedef in hedefler:
                    hedefler.pop(delete_hedef)
            if delete_hedef != None:
                if delete_hedef in hedefler:
                    hedefler.pop(delete_hedef)
            
            target_delete = 0

        time.sleep(0.05)

# Kamera görüntüsünden gelen en güncel nesne verisini güncelleyen thread
def update_latest_objects(camera_handler):
    while not stop_event.is_set():
        with latest_lock:
            latest_objects.clear()
            latest_objects.extend(camera_handler.current_objects)
        time.sleep(0.01)


# UCUS FONKSİYONLARI
def second_miss(stop_event, vehicle: Vehicle, DRONE_ID, ALT, target_loc):
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.multiple_takeoff(alt=ALT, drone_id=DRONE_ID)
    
    current_alt = 0
    start_time = time.time()
    while current_alt < ALT * 0.9 and not stop_event.is_set() and rtls[DRONE_ID] == 0:
        current_alt = vehicle.get_pos(drone_id=DRONE_ID)[2]
        if time.time() - start_time > 2:
            print(f"{DRONE_ID}>> Anlık irtifa: {current_alt} metre")
            start_time = time.time()
        
        time.sleep(0.05)
    
    if not stop_event.is_set() and rtls[DRONE_ID] == 0:
        print(f"{DRONE_ID}>> takeoff yaptı")
            
        vehicle.go_to(loc=target_loc, alt=ALT, drone_id=DRONE_ID)

        start_time = time.time()

    while not stop_event.is_set() and rtls[DRONE_ID] == 0:
        if time.time() - start_time > 5:
            print(f"{DRONE_ID}>> hedefe gidiyor...")
            print(f"Kalan mesafe: {get_dist(vehicle.get_pos(drone_id=DRONE_ID), target_loc)}m")
            start_time = time.time()
            
        if vehicle.on_location(loc=target_loc, seq=0, sapma=1, drone_id=DRONE_ID):
            print(f"{DRONE_ID}>> hedefe ulaştı")
            print(f"{DRONE_ID}>> iniş gerçekleştiriyor")
            vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
            break
    
        time.sleep(0.05)


def go_home(stop_event, vehicle: Vehicle, home_pos, DRONE_ID):
    vehicle.go_to(loc=home_pos, drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> kalkış konumuna dönüyor...")

    start_time = time.time()
    while not stop_event.is_set() and rtls[DRONE_ID] == 0:
        if time.time() - start_time > 5:
            print(f"{DRONE_ID}>> kalkış konumuna dönüyor...")
            start_time = time.time()
        
        if vehicle.on_location(loc=home_pos, seq=0, sapma=1, drone_id=DRONE_ID):
            print(f"{DRONE_ID}>> iniş gerçekleşiyor")
            vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
            break
            
        time.sleep(0.05)


config = json.load(open("./config.json", "r"))

target_delete, target_select = 0,0
rtls = {}
abort_miss = 0

server = TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
arduino = Serial_Control(config["ARDUINO"]["port"])
gimbal_controller = GimbalHandler(server=server, stop_event=stop_event)

threading.Thread(target=gimbal_controller.keyboard_controller, daemon=True).start()

# Kamera başlatılıyor
camera_handler = new_image_processing_handler.Handler(stop_event)
camera_handler.start_proccessing(config["model"])

# Kamera işleme thread’i
threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()

# Drone Uçuş İçin
vehicle = Vehicle(config["CONN-PORT"])

#GOZLEMCI
GOZLEMCI_ID = config["GOZLEMCI"]["id"]
GOZLEMCI_ALT = config["GOZLEMCI"]["alt"]
#SALDIRI 1
SALDIRI1_ID = config["SALDIRI1"]["id"]
SALDIRI_ALT = config["SALDIRI_ALT"]
#SALDIRI 2
'''SALDIRI2_ID = config["SALDIRI2"]["id"]
SALDIRI_ALT = config["SALDIRI_ALT"]'''

# Verileri güncelleyen thread
threading.Thread(target=update_latest_objects, args=(camera_handler,), daemon=True).start()

# Tuş kontrol thread’i
#threading.Thread(target=select_thread, args=(config, server, vehicle, GOZLEMCI_ID), daemon=True).start()
threading.Thread(target=select_thread_arduino, args=(stop_event, server, vehicle, GOZLEMCI_ID, arduino), daemon=True).start()
threading.Thread(target=arduino_listener, args=(arduino, stop_event), daemon=True).start()
threading.Thread(target=miss_controller, args=(vehicle, stop_event), daemon=True).start()

saldiri_threadleri = []

# Ana döngü
try:
    vehicle.set_mode(mode="GUIDED", drone_id=GOZLEMCI_ID)
    vehicle.arm_disarm(arm=True, drone_id=GOZLEMCI_ID)
    vehicle.takeoff(alt=GOZLEMCI_ALT, drone_id=GOZLEMCI_ID)

    home_pos_gozlemci = vehicle.get_pos(drone_id=GOZLEMCI_ID)

    print(f"{GOZLEMCI_ID}>> Kalkış yaptı")

    start_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time >= 5:
            print(f"{GOZLEMCI_ID}>> Hedefleri Arıyor")
            if hedefler:
                print(hedefler)
            start_time = time.time()
        
        time.sleep(0.05)

    go_home_1 = threading.Thread(target=go_home, args=(stop_event, vehicle, home_pos_gozlemci, GOZLEMCI_ID), daemon=True)
    go_home_1.start()

    #? 2. aşama
    '''if len(vehicle.drone_ids) >= 2:
        for drone_id in vehicle.drone_ids:
            if drone_id == GOZLEMCI_ID:
                continue

            saldiri_hedef = {key: value for key, value in hedefler.items() if value[0] == drone_id}
            saldiri_hedef_name = list(saldiri_hedef.keys())[0]
            saldiri_hedef_drone_id = list(saldiri_hedef.values())[0][0]
            saldiri_hedef_loc = list(saldiri_hedef.values())[0][1]

            if saldiri_hedef:
                saldiri_thrd = threading.Thread(target=second_miss, args=(stop_event, vehicle, saldiri_hedef_drone_id, SALDIRI_ALT, saldiri_hedef_loc), daemon=True)
                saldiri_thrd.start()
                saldiri_threadleri.append(saldiri_thrd)
    else:
        print("Saldırı dronları sistemde değiller")'''
    saldiri_hedef = {key: value for key, value in hedefler.items() if value[0] == SALDIRI1_ID}
    saldiri_hedef_name = list(saldiri_hedef.keys())[0]
    saldiri_hedef_drone_id = list(saldiri_hedef.values())[0][0]
    saldiri_hedef_loc = list(saldiri_hedef.values())[0][1]

    saldiri_thrd = threading.Thread(target=second_miss, args=(stop_event, vehicle, saldiri_hedef_drone_id, SALDIRI_ALT, saldiri_hedef_loc), daemon=True)
    saldiri_thrd.start()
    saldiri_threadleri.append(saldiri_thrd)
    
    while not stop_event.is_set():
        saldiri_bitti = True
        for saldiri_thrd in saldiri_threadleri:
            if saldiri_thrd.is_alive():
                saldiri_bitti = False
                break
        
        if saldiri_bitti and not go_home_1.is_alive():
            break
            
        time.sleep(0.5)
    
    print("Görev Tamamlandi")
    


except KeyboardInterrupt:
    failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()
    print("Durduruldu.")

except Exception as e:
    failsafe(vehicle)
    if not stop_event.is_set():
        stop_event.set()
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()


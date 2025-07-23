import threading
import json
import time
import sys
sys.path.append('../../pymavlink_custom/')
sys.path.append("./libs/")

from pymavlink_custom import Vehicle
import libs.tcp_handler as tcp_handler
import libs.calc_loc as calc_loc
import libs.image_processing_handler as image_processing_handler
import libs.gimbal_controller as gimbal_controller

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


def second_miss(stop_event: threading.Event, vehicle: Vehicle, config: dict, targets: dict, target_locker: threading.Lock):
    def second_miss(stop_event, vehicle: Vehicle, config: dict, target_loc, target_cls: str):
        drone_id = config["id"]
        alt = config["alt"]
        alcalma_alt = config["alcalma_alt"]

        # UPD Verileri
        ip = config["UDP"]["ip"]
        port = config["UDP"]["port"]

        # Saldiri iha goruntu isleme
        detected_obj = {"cls": None, "pos": None}
        detected_obj_lock = threading.Lock()
        saldiri_camera_handler = image_processing_handler.Handler(stop_event=stop_event)
        threading.Thread(target=saldiri_camera_handler.udp_camera_new, args=(ip, port, detected_obj, detected_obj_lock), daemon=True).start()
        
        # Ucus
        vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
        vehicle.arm_disarm(arm=True, drone_id=drone_id)
        vehicle.multiple_takeoff(alt=alt, drone_id=drone_id)
        
        current_alt = 0
        start_time = time.time()
        while current_alt < alt * 0.9 and not stop_event.is_set():
            current_alt = vehicle.get_pos(drone_id=drone_id)[2]
            if time.time() - start_time > 2:
                print(f"{drone_id}>> Anlık irtifa: {current_alt} metre")
                start_time = time.time()

        home_pos = vehicle.get_pos(drone_id=drone_id)
        print(f"{drone_id}>> takeoff yaptı")
            
        vehicle.go_to(loc=target_loc, alt=alt, drone_id=drone_id)

        start_time = time.time()
        while not stop_event.is_set():
            if time.time() - start_time > 5:
                print(f"{drone_id}>> hedefe gidiyor...")
                start_time = time.time()
            
            with detected_obj_lock:
                if detected_obj["cls"] != None:
                    algilanan_obj = detected_obj["cls"]
                    print(f"{drone_id}>> Algilanan obj: {algilanan_obj}")
                    
                    if algilanan_obj == target_cls or algilanan_obj.lower() == target_cls.lower() or algilanan_obj.upper() == target_cls.upper():
                        obj_location = vehicle.get_pos(drone_id=drone_id)

                        vehicle.go_to(loc=target_loc, alt=alt, drone_id=drone_id)
                        while not vehicle.on_location(loc=target_loc, seq=0, sapma=1, drone_id=drone_id) and not stop_event.is_set():
                            time.sleep(0.1)
                        
                        print(f"{drone_id}>> Nesnenin konumuna alcaliyor")
                        vehicle.go_to(loc=obj_location, alt=alcalma_alt, drone_id=drone_id)
                        drone_alt = alt
                        while drone_alt < alcalma_alt * 0.9 and not stop_event.is_set():
                            time.sleep(0.1)
                        
                        print(f"{drone_id}>> Nesnenin konumuna alcaldi")
                        time.sleep(5)
                        print(f"{drone_id}>> Hedefe gidis bitti")
                        
                        break

                    else:
                        detected_obj["cls"] = None
                        detected_obj["pos"] = None

            if vehicle.on_location(loc=target_loc, seq=0, sapma=1, drone_id=drone_id):
                print(f"{drone_id}>> hedefe ulaştı")
                break
            
            time.sleep(0.01)
        
        go_home(stop_event, vehicle, home_pos, drone_id)
    
    miss_thrds = []
    with target_locker:
        for drone_id in targets:
            target_loc = targets[drone_id]["loc"]
            target_cls = targets[drone_id]["cls"]

            for saldiri_iha_config in config["SALDIRILAR"]:
                if saldiri_iha_config["id"] == drone_id:
                    break

            if saldiri_iha_config["id"] == drone_id:
                thrd = threading.Thread(target=second_miss, args=(stop_event, vehicle, saldiri_iha_config, target_loc, target_cls))
                thrd.start()
                miss_thrds.append(thrd)
    
    if len(miss_thrds) != 0:
        for miss_thrd in miss_thrds:
            miss_thrd.join()


def go_home(stop_event, vehicle: Vehicle, home_pos, DRONE_ID):
    vehicle.go_to(loc=home_pos, drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> kalkış konumuna dönüyor...")

    start_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time > 5:
            print(f"{DRONE_ID}>> kalkış konumuna dönüyor...")
            start_time = time.time()
        
        if vehicle.on_location(loc=home_pos, seq=0, sapma=1, drone_id=DRONE_ID):
            print(f"{DRONE_ID}>> iniş gerçekleşiyor")
            vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
            break


#? Gerekliler
config = json.load(open("./suru-config.json"))
stop_event = threading.Event()

#? Kütüphaneler
server = tcp_handler.TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
gozlemci_camera_handler = image_processing_handler.Handler(stop_event=stop_event)
gimbal_handler = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)

#? Threadler
threading.Thread(target=gozlemci_camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start() # Gözlemci iha kamera
threading.Thread(target=gimbal_handler.keyboard_controller, daemon=True).start()

#? Uçuş hazırlıkları
ALT_1 = config["GOZLEMCI"]["alt"]
ALT_2 = config["SALDIRI1"]["alt"]
DRONE_1_ID = int(config["GOZLEMCI"]["id"])
DRONE_2_ID = int(config["SALDIRI1"]["id"])
target_loc = []

vehicle = Vehicle(config["CONN-PORT"])

#? Hedef seçme silme threadi
targets = {}
target_locker = threading.Lock()
selecter_started = threading.Event()
selecter_thrd = threading.Thread(target=gimbal_handler.gimbal_selecter, args=(stop_event, vehicle, DRONE_1_ID, server, targets, target_locker, selecter_started), daemon=True)
selecter_thrd.start()

while not selecter_started.is_set():
    time.sleep(1)

try:
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_1_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_1_ID)
    vehicle.takeoff(ALT_1, drone_id=DRONE_1_ID)

    home_pos = vehicle.get_pos(drone_id=DRONE_1_ID)
    
    print(f"{DRONE_1_ID}>> takeoff yaptı")
    print(f"{DRONE_1_ID}>> hedefleri arıyor...")

    #? 1. aşama (tcp verisi bekleniyor)
    start_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time > 5:
            if targets != {}:
                print(targets)
            print(f"{DRONE_1_ID}>> hedefler aranıyor...")
            start_time = time.time()
        
        if not selecter_thrd.is_alive():
            go_home_1 = threading.Thread(target=go_home, args=(stop_event, vehicle, home_pos, DRONE_1_ID), daemon=True)
            go_home_1.start()
            print("İHA'lar hedeflere saldiriya gecti")
            break
        
        time.sleep(0.01)
        
    #? 2. aşama (tcp verisine gidiliyor)
    second_miss_thrd = threading.Thread(target=second_miss, args=(stop_event, vehicle, config, targets, target_locker), daemon=True)
    second_miss_thrd.start()
    
    while go_home_1.is_alive() or second_miss_thrd.is_alive():
        time.sleep(1)
    
    print("Görev tamamlandı")

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



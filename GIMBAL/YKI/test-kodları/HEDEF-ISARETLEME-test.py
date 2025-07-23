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


def second_miss(stop_event, vehicle, config, targets, target_locker):
    def second_miss(stop_event, vehicle: Vehicle, DRONE_ID, ALT, target_loc):
        print(f"{DRONE_ID}>> takeoff yaptı")
            
        print(f"{DRONE_ID}>> hedefe gidiyor...")
        print(f"Kalan mesafe: {calc_loc.get_dist(vehicle.get_pos(drone_id=DRONE_ID), target_loc)}m")
    
    miss_thrds = []
    with target_locker:
        for drone_id in targets:
            target_loc = targets[drone_id]["loc"]

            if calc_loc.get_dist(vehicle.get_pos(drone_id=drone_id), target_loc) > 50:
                print("Hedef konumu yanlis: ", targets[drone_id])
                continue

            thrd = threading.Thread(target=second_miss, args=(stop_event, vehicle, drone_id, 5, target_loc))
            thrd.start()
            miss_thrds.append(thrd)
    
    if len(miss_thrds) != 0:
        for miss_thrd in miss_thrds:
            miss_thrd.join()


#? Gerekliler
config = json.load(open("./suru-config.json"))
stop_event = threading.Event()

#? Kütüphaneler
server = tcp_handler.TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
camera_handler = image_processing_handler.Handler(stop_event=stop_event)
gimbal_handler = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)

#? Threadler
threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()
threading.Thread(target=gimbal_handler.keyboard_controller, daemon=True).start()

#? Uçuş hazırlıkları
ALT_1 = config["GOZLEMCI"]["alt"]
DRONE_1_ID = int(config["GOZLEMCI"]["id"])

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
            print("İHA'lar hedeflere saldiriya gecti")
            break
        
        time.sleep(0.01)

    #? 2. aşama (tcp verisine gidiliyor)
    second_miss_thrd = threading.Thread(target=second_miss, args=(stop_event, vehicle, config, targets, target_locker), daemon=True)
    second_miss_thrd.start()
    
    while second_miss_thrd.is_alive():
        time.sleep(1)
    
    print("Görev tamamlandı")

except KeyboardInterrupt:
    print("Exiting...")
    if not stop_event.is_set():
        stop_event.set()

except Exception as e:
    if not stop_event.is_set():
        stop_event.set()
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()
    vehicle.vehicle.close()


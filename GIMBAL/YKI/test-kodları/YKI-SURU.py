import threading
import json
import time
import sys
sys.path.append('../../pymavlink_custom')

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


def second_miss(stop_event, vehicle: Vehicle, DRONE_ID, ALT, target_loc):
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.multiple_takeoff(alt=ALT, drone_id=DRONE_ID)
    
    current_alt = 0
    start_time = time.time()
    while current_alt < ALT * 0.9 and not stop_event.is_set():
        current_alt = vehicle.get_pos(drone_id=DRONE_ID)[2]
        if time.time() - start_time > 2:
            print(f"{DRONE_ID}>> Anlık irtifa: {current_alt} metre")
            start_time = time.time()

    print(f"{DRONE_ID}>> takeoff yaptı")
        
    vehicle.go_to(loc=target_loc, alt=ALT, drone_id=DRONE_ID)

    start_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time > 5:
            print(f"{DRONE_ID}>> hedefe gidiyor...")
            print(f"Kalan mesafe: {calc_loc.get_dist(vehicle.get_pos(drone_id=DRONE_ID), target_loc)}m")
            start_time = time.time()
            
        if vehicle.on_location(loc=target_loc, seq=0, sapma=1, drone_id=DRONE_ID):
            print(f"{DRONE_ID}>> hedefe ulaştı")
            print(f"{DRONE_ID}>> iniş gerçekleştiriyor")
            vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
            break


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
camera_handler = image_processing_handler.Handler(stop_event=stop_event)
gimbal_handler = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)

#? Threadler
threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()
threading.Thread(target=gimbal_handler.keyboard_controller, daemon=True).start()

#? Uçuş hazırlıkları
ALT_1 = config["GOZLEMCI"]["alt"]
ALT_2 = config["SALDIRI1"]["alt"]
DRONE_1_ID = int(config["GOZLEMCI"]["id"])
DRONE_2_ID = int(config["SALDIRI1"]["id"])
target_loc = []

vehicle = Vehicle(config["CONN-PORT"])

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
        tcp_data = server.get_data()
        if time.time() - start_time > 5:
            print(f"{DRONE_1_ID}>> hedefler aranıyor...")
            start_time = time.time()
        
        if tcp_data != None:
            tcp_data = tcp_data.strip()
            #! Mesafe verisi gelmedi ise devam ediyor
            if len(tcp_data.split("|")) != 3:
                continue
        
            if float(tcp_data.split("|")[0]) <= 0:
                continue

            print(tcp_data.split("|"))
            if calc_loc.check_data(tcp_data) == False:
                print("Hedef bozuk alındı yeni hedef bekleniyor")
                tcp_data = ""
                continue
            
            pos_timer = time.time()
            current_loc = vehicle.get_pos(drone_id=DRONE_1_ID)
            current_yaw = vehicle.get_yaw(drone_id=DRONE_1_ID)

            calc_timer = time.time()
            target_loc = calc_loc.calc_location_geopy(current_loc=current_loc, yaw_angle=current_yaw, tcp_data=tcp_data)
            finish_timer = time.time()

            print(f"{DRONE_1_ID}>> hedef bulundu: {target_loc}")
            print(f"Hedef konumu {finish_timer - calc_timer} surede hesaplandi. Drone bilgisi {calc_timer - pos_timer} surede cekildi")
            print(f"Hedefin uzakligi: {calc_loc.get_dist(target_loc, current_loc)} metre")
            if calc_loc.get_dist(target_loc, current_loc) > 20:
                raise ValueError("Hedef konumu yanlis hesaplandi")
            break
            
        time.sleep(0.01)
    
    go_home_1 = threading.Thread(target=go_home, args=(stop_event, vehicle, home_pos, DRONE_1_ID), daemon=True)
    go_home_1.start()
        
    #? 2. aşama (tcp verisine gidiliyor)
    second_miss_thrd = threading.Thread(target=second_miss, args=(stop_event, vehicle, DRONE_2_ID, ALT_2, target_loc), daemon=True)
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


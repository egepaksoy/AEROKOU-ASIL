import threading
import json
import time
import sys
import math
sys.path.append('../../pymavlink_custom')

from pymavlink_custom import Vehicle
import YKI.libs.tcp_handler as tcp_handler
import YKI.libs.calc_loc as calc_loc
import YKI.libs.image_processing_handler as image_processing_handler
import YKI.libs.gimbal_controller as gimbal_controller

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


#? Gerekliler
config = json.load(open("./config.json"))
stop_event = threading.Event()

#? Kütüphaneler
server = tcp_handler.TCPServer(port=config["TCP"]["port"], stop_event=stop_event)
camera_handler = image_processing_handler.Handler(stop_event=stop_event)
gimbal_handler = gimbal_controller.GimbalHandler(server=server, stop_event=stop_event)

#? Threadler
threading.Thread(target=camera_handler.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True).start()
threading.Thread(target=gimbal_handler.keyboard_controller, daemon=True).start()

#? Uçuş hazırlıkları
drone_config = config["DRONE"]
ALT = drone_config["alt"]
DRONE_ID = int(drone_config["id"])
target_loc = []
home_pos = []

vehicle = Vehicle(drone_config["port"])

try:
    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.takeoff(ALT, drone_id=DRONE_ID)

    home_pos = vehicle.get_pos(drone_id=DRONE_ID)    
    
    print(f"{DRONE_ID}>> takeoff yaptı")
    print(f"{DRONE_ID}>> hedefleri arıyor...")

    #? 1. aşama (tcp verisi bekleniyor)
    start_time = time.time()
    while not stop_event.is_set():
        tcp_data = server.get_data()
        if time.time() - start_time > 5:
            print(f"{DRONE_ID}>> hedefler aranıyor...")
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
            
            current_loc = vehicle.get_pos(drone_id=DRONE_ID)
            current_yaw = vehicle.get_yaw(drone_id=DRONE_ID)

            target_loc = calc_loc.calc_location_geopy(current_loc=current_loc, yaw_angle=current_yaw, tcp_data=tcp_data, DEG=vehicle.DEG)

            print(f"{DRONE_ID}>> hedef bulundu: {target_loc}")
            print(f"Hedefe olan mesafe: {calc_loc.get_dist(current_loc, target_loc)}m")
            break
            
        time.sleep(0.01)

    #? 2. aşama (tcp verisine gidiliyor)
    if target_loc != []:
        vehicle.go_to(loc=target_loc, alt=ALT, drone_id=DRONE_ID)
        print(f"{DRONE_ID}>> hedefe gidiyor")

        start_time = time.time()
        while not stop_event.is_set():
            if time.time() - start_time > 5:
                print(f"{DRONE_ID}>> hedefe gidiyor...")
                print(f"Kalan mesafe: {calc_loc.get_dist(current_loc, target_loc)}m")
                start_time = time.time()
            
            if vehicle.on_location(loc=target_loc, seq=0, sapma=1, drone_id=DRONE_ID):
                print(f"{DRONE_ID}>> hedefe ulaştı")
                print(f"{DRONE_ID}>> iniş gerçekleştiriyor")
                vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
                break
    
    else:
        vehicle.go_to(loc=home_pos, alt=ALT, drone_id=DRONE_ID)
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

    print("Görev tamamlandı")

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

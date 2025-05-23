import time
import sys
import threading
import json
import math

sys.path.append('../pymavlink_custom')
from pymavlink_custom import Vehicle


def failsafe(vehicle):
    def failsafe_drone_id(vehicle, drone_id):
        print(f"{drone_id}>> Failsafe alıyor")
        vehicle.set_mode(mode="RTL", drone_id=drone_id)

    thraeds = []
    for d_id in vehicle.drone_ids:
        thrd = threading.Thread(target=failsafe_drone_id, args=(vehicle, d_id))
        thrd.start()
        thraeds.append(thrd)

    for t in thraeds:
        t.join()

    print(f"Dronlar {vehicle.drone_ids} Failsafe aldi")

def calc_loc(pos, yaw_angle, distance, DEG):
    return pos[0] + math.cos(math.radians(yaw_angle)) * distance * DEG, pos[1] + math.sin(math.radians(yaw_angle)) * distance * DEG

def mission(vehicle, alt, distance, drone_id):
    DRONE_ID = drone_id # drone id

    loc = calc_loc(pos=vehicle.get_pos(drone_id=drone_id), yaw_angle=vehicle.get_yaw(drone_id=drone_id), distance=distance, DEG=vehicle.DEG)

    print(f"{DRONE_ID}>> Göreve Başladı")

    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.takeoff(alt=alt, drone_id=DRONE_ID)
    
    print(f"{DRONE_ID}>> takeoff yaptı")
    vehicle.go_to(loc=loc, alt=alt, drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> {distance} metre ileri gidiyor...")

    try:
        start_time = time.time()
        while not stop_event.is_set():
            if time.time() - start_time > 5:
                print(f"{DRONE_ID}>> ucus devam ediyor...")
                start_time = time.time()
            
            if vehicle.on_location(loc=loc, seq=0, sapma=1, drone_id=DRONE_ID):
                print(f"{DRONE_ID}>> drone hedefe ulasti")
                vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
                break
        
        print(f"{DRONE_ID}>> Görevini tamamladı")

    finally:
        pass

config = json.load(open("config.json", "r"))

drone_ids = config["DRONE"]["ids"]
alt = config["DRONE"]["alt"]
distance = config["DRONE"]["distance"]

stop_event = threading.Event()
vehicle = Vehicle(config["DRONE"]["path"])

threads = []
try:
    for drone_id in drone_ids:
        thrd = threading.Thread(target=mission, args=(vehicle, alt, distance, drone_id), daemon=True)
        thrd.start()
        threads.append(thrd)

    for t in threads:
        t.join()        
    
    print("Görev tamamlandı")

except KeyboardInterrupt:
    if not stop_event.is_set():
        stop_event.set()
    print("Exiting...")
    failsafe(vehicle)

except Exception as e:
    if not stop_event.is_set():
        stop_event.set()
    failsafe(vehicle)
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()
    vehicle.vehicle.close()
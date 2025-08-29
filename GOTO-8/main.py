import sys
sys.path.append("../pymavlink_custom")

from pymavlink_custom import Vehicle
import time
import threading

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

def go_home(vehicle: Vehicle, home_pos, DRONE_ID):
    vehicle.go_to(loc=home_pos, drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> kalkış konumuna dönüyor...")

    start_time = time.time()
    while True:
        if time.time() - start_time > 5:
            print(f"{DRONE_ID}>> kalkış konumuna dönüyor...")
            start_time = time.time()
        
        if vehicle.on_location(loc=home_pos, seq=0, sapma=1, drone_id=DRONE_ID):
            print(f"{DRONE_ID}>> iniş gerçekleşiyor")
            vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
            break

if len(sys.argv) != 3:
    print("Usage: python main.py <connection_string> <drone_id>")
    sys.exit(1)

vehicle = Vehicle(sys.argv[1])

ALT = 6
DRONE_ID = int(sys.argv[2])

wplist = []

try:
    wplist = vehicle.get_wp_list(drone_id=DRONE_ID)
    print(len(wplist))

    vehicle.set_mode(mode="GUIDED", drone_id=DRONE_ID)
    vehicle.arm_disarm(arm=True, drone_id=DRONE_ID)
    vehicle.takeoff(ALT, drone_id=DRONE_ID)
    
    home_pos = vehicle.get_pos(drone_id=DRONE_ID)
    print(f"{DRONE_ID}>> takeoff yaptı")

    start_time = time.time()
    current_wp = 0
    vehicle.go_to(loc=wplist[current_wp], alt=wplist[current_wp][2], drone_id=DRONE_ID)
    while True:
        if time.time() - start_time > 5:
            print(f"{DRONE_ID}>> ucus devam ediyor...")
            start_time = time.time()
        
        if vehicle.on_location(loc=wplist[current_wp], seq=0, sapma=1, drone_id=DRONE_ID):
            print(f"{DRONE_ID}>> wp: {current_wp + 1}/{len(wplist)}")
            current_wp += 1

            if current_wp == len(wplist):
                print("Gorev bitti")
                go_home(vehicle, home_pos, DRONE_ID)
                break

            vehicle.go_to(loc=wplist[current_wp], alt=wplist[current_wp][2], drone_id=DRONE_ID)
    
    print("Görev tamamlandı")

except KeyboardInterrupt:
    print("Exiting...")
    failsafe(vehicle)

except Exception as e:
    failsafe(vehicle)
    print(e)

finally:
    vehicle.vehicle.close()
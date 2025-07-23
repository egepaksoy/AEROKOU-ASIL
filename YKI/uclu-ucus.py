import threading
import json
import time
import sys
sys.path.append('../pymavlink_custom/')

from pymavlink_custom import Vehicle
from libs.yki_handler import YKIMonitor

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

def stop_event_listener(stop_event, vehicle, yki_monitor):
    global rtl_drones

    while not stop_event.is_set():
        if not yki_monitor.get_system_status():
            break
        time.sleep(0.01)

    rtl_drones = vehicle.drone_ids
    if not stop_event.is_set():
        stop_event.set()

    failsafe(vehicle)

def flight_controller(vehicle, stop_event, config, yki_monitor):
    global rtl_drones
    global arm_drone_status

    while not stop_event.is_set():
        yki_data = yki_monitor.get_arm_status()

        for drone in config["ARM-BUTTONS"]:
            # babaninyo hocaninyo was here
            if yki_data[drone]:
                drone_id = config["ARM-BUTTONS"][drone]
                if drone_id not in rtl_drones and drone_id in vehicle.drone_ids:
                    vehicle.set_mode(mode="LAND", drone_id=drone_id)
                    rtl_drones.append(drone_id)
                    print(f"{drone_id}>> LAND Alıyor")

        time.sleep(0.01)

def takeoff(vehicle, drone_id, alt, stop_event):
    global rtl_drones

    if drone_id in rtl_drones:
        return

    else:
        vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
        vehicle.arm_disarm(arm=True, drone_id=drone_id)
        vehicle.multiple_takeoff(alt=alt, drone_id=drone_id)

        current_alt = vehicle.get_pos(drone_id=drone_id)[2]
        while not stop_event.is_set() and current_alt <= alt * 0.9:
            current_alt = vehicle.get_pos(drone_id=drone_id)[2]
            time.sleep(0.5)

        print(f"{DRONE_ID}>> takeoff yaptı")

        start_time = time.time()
        while not stop_event.is_set():
            if time.time() - start_time >= 3:
                print(f"{drone_id}>> Drone havada")
                start_time = time.time()
            
            if drone_id in rtl_drones:
                break

            time.sleep(0.01)

#? Gerekliler
config = json.load(open("./suru-config.json"))
#config = json.load(open("./test-config.json"))
stop_event = threading.Event()
rtl_drones = []
arm_drone_status = {}

#? Uçuş hazırlıkları
ALT = config["GOZLEMCI"]["alt"]
DRONE_ID = int(config["GOZLEMCI"]["id"])

vehicle = Vehicle(config["CONN-PORT"], stop_event)

#? Threadler
yki_monitor = YKIMonitor(config, stop_event)
yki_monitor.start()

flight_controller_thrd = threading.Thread(target=flight_controller, args=(vehicle, stop_event, config, yki_monitor), daemon=True)
stop_event_thrd = threading.Thread(target=stop_event_listener, args=(stop_event, vehicle, yki_monitor), daemon=True)

start_time = 0
while not stop_event.is_set():
    if yki_monitor.get_system_status():
        break
    if time.time() - start_time >= 5:
        print("\rYKI Başlatılması Bekleniyor", end="")
        start_time = time.time()

    time.sleep(0.5)

print("\nYKI Baslatildi")

flight_controller_thrd.start()
stop_event_thrd.start()

try:
    drone_thrds = []
    for drone_id in vehicle.drone_ids:
        for drone_config in config["DRONES"]:
            if drone_config["id"] == drone_id and drone_id not in rtl_drones:
                thrd = threading.Thread(target=takeoff, args=(vehicle, drone_id, drone_config["alt"], stop_event), daemon=True)
                thrd.start()

    while stop_event_thrd.is_alive() and len(rtl_drones) != vehicle.drone_ids and not stop_event.is_set():
        time.sleep(0.5)
 
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


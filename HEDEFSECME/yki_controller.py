import threading
import json
import time, sys

sys.path.append("../pymavlink_custom/")
from pymavlink_custom import Vehicle
from libs import new_image_processing_handler
from libs.tcp_handler import TCPServer
from libs.serial_handler import Serial_Control


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
            print(arduino_data)

            if arduino_data != None:
                arduino_data = {k: int(v) for k, v in (pair.split(":") for pair in arduino_data.strip().split())}
                print(arduino_data)

                abort_miss = int(arduino_data["SYSTEM"])
                i = 1
                for rtl in rtls:
                    rtls[rtl] = int(arduino_data[f"ARM{i}"])
                    i+=1

                target_select, target_delete = int(arduino_data["TARGET_SELECT"]), int(arduino_data["TARGET_DELETE"])
            time.sleep(0.5)
        except Exception:
            continue

def miss_controller(vehicle, stop_event):
    global rtls, abort_miss

    while not stop_event.is_set():
        if abort_miss == 0:
            if not stop_event.is_set():
                stop_event.set()
            failsafe(vehicle)
        else:
            for rtl in rtls.items():
                if rtl[1]:
                    if rtl[0] in vehicle.drone_ids:
                        vehicle.set_mode(mode="RTL", drone_id=rtl[0])

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

def takeoff(vehicle: Vehicle, ALT: int, drone_id: int):
    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.arm_disarm(arm=True, drone_id=drone_id)

    vehicle.multiple_takeoff(alt=ALT, drone_id=drone_id)
    drone_alt = vehicle.get_pos(drone_id=drone_id)[2]
    while not stop_event.is_set() and drone_alt < ALT * 0.9:
        drone_alt = vehicle.get_pos(drone_id=drone_id)[2]
        time.sleep(0.5)

    print(f"{drone_id}>> Kalkış yaptı")


config = json.load(open("./config.json", "r"))

target_delete, target_select = 0,0
rtls = {}
abort_miss = 1

arduino = Serial_Control(config["ARDUINO"]["port"])

threading.Thread(target=arduino_listener, args=(arduino, stop_event), daemon=True).start()

# Drone Uçuş İçin
vehicle = Vehicle(config["CONN-PORT"])

threading.Thread(target=miss_controller, args=(vehicle, stop_event), daemon=True).start()

# Ana döngü
try:
    if abort_miss == 0:
        print("YKİ'Yi başlatın")
        exit(1)
    takeoff_thrds = []
    for drone_id in vehicle.drone_ids:
        t = threading.Thread(target=takeoff, args=(vehicle, 5, drone_id), daemon=True)
        t.start()
        takeoff_thrds.append(t)
    
    for t in takeoff_thrds:
        t.join()

    start_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time >= 5:
            print("Bekleniyor...")
            start_time = time.time()
        
        time.sleep(0.05)

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



import threading
import queue
import json
import time
import sys
sys.path.append('../../pymavlink_custom')

from pymavlink_custom import Vehicle
import tcp_handler
import calc_loc
import serial_handler
import image_processing_handler
import math
import keyboard

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


def keyboard_controller(server: tcp_handler.TCPServer, config):
    ters = -1

    while not stop_event.is_set():
        ser_data = ""
        ser_x = 0
        ser_y = 0

        if keyboard.is_pressed("x"):
            ser_x = 2
            ser_y = 2
        else:
            if keyboard.is_pressed('right') or keyboard.is_pressed('d'):
                ser_x = -1 * ters

            if keyboard.is_pressed('left') or keyboard.is_pressed('a'):
                ser_x = 1 * ters

            if keyboard.is_pressed('up') or keyboard.is_pressed('w'):
                ser_y = 1 * ters

            if keyboard.is_pressed('down') or keyboard.is_pressed('s'):
                ser_y = -1 * ters

        ser_data = f"{ser_x}|{ser_y}\n"
        server.send_data(ser_data)
        time.sleep(0.05)

def joystick_controller(server: tcp_handler.TCPServer, config):
    arduino = serial_handler.Serial_Control(port=config["ARDUINO"]["port"])

    while not stop_event.is_set():
        joystick_data = arduino.read_value()
        if len(joystick_data.split("|")) == 3:
            server.send_data("iste")
        elif joystick_data != None:
            if "|" in joystick_data and "\n" not in joystick_data:
                joystick_data = joystick_data.strip()
                joystick_data = f"{joystick_data.split('|')[0]}|{joystick_data.split('|')[1]}\n"
            server.send_data(joystick_data)

def distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2) * vehicle.DEG

def takeoff(vehicle, alt, drone_id, home_pos_queue):
    vehicle.set_mode(mode="GUIDED", drone_id=drone_id)
    vehicle.arm_disarm(arm=True, drone_id=drone_id)
    vehicle.takeoff(alt=alt, drone_id=drone_id)

    home_pos_queue.put(vehicle.get_pos(drone_id=drone_id))
    
    print(f"{drone_id} takeoff yaptı")

def manuel_RTL(vehicle, drone_id, home_pos):
    current_alt = vehicle.get_pos(drone_id=drone_id)
    vehicle.go_to(loc=home_pos, alt=current_alt[2], drone_id=drone_id)
    print(f"{drone_id}>> kalkış konumuna dönüyor")

    start_time = time.time()
    while not stop_event.is_set():
        if time.time() - start_time >= 5:
            print(f"{drone_id}>> drone kalkış konumuna dönüyor...")
        if vehicle.on_location(loc=home_pos, seq=0, sapma=1, drone_id=drone_id):
            print(f"{drone_id}>> iniş gerçekleştiriyor...")
            vehicle.set_mode(mode="LAND", drone_id=drone_id)
            break


config = json.load(open("./config.json"))

stop_event = threading.Event()

camera = image_processing_handler.Handler()
#!camera_thread = threading.Thread(target=camera.udp_camera, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True)
camera_thread = threading.Thread(target=camera.udp_camera_new, args=((config["UDP"]["ip"]), config["UDP"]["port"]), daemon=True)
camera_thread.start()

server = tcp_handler.TCPServer(port=config["TCP"]["port"])

threading.Thread(target=keyboard_controller, args=(server, config), daemon=True).start()
#!threading.Thread(target=joystick_controller, args=(server, config), daemon=True).start()

gozlemci_config = config["GOZLEMCI"]
saldiri1_config = config["SALDIRI1"]

vehicle = Vehicle(config["CONN-PORT"])

GOZLEMCI_ID = gozlemci_config["id"]
SALDIRI_ID = saldiri1_config["id"]

target_loc = []
home_pos = []

try:
    ################# GOZLEMCI KALKIS ###############
    gozlemci_home_queue = queue.Queue
    gozlemci_takeoff_thread = threading.Thread(target=takeoff, args=(vehicle, gozlemci_config["alt"], GOZLEMCI_ID, gozlemci_home_queue), daemon=True)
    gozlemci_takeoff_thread.start()

    gozlemci_takeoff_thread.join()
    gozlemci_home_pos = gozlemci_home_queue.get()

    print(f"{GOZLEMCI_ID}>> hedefleri arıyor...")

    #? 1. aşama (tcp verisi bekleniyor)
    start_time = time.time()
    while not stop_event.is_set():
        tcp_data = server.get_data()
        if time.time() - start_time > 5:
            print(f"{GOZLEMCI_ID}>> hedefler aranıyor...")
            start_time = time.time()
        
        if tcp_data != None:
            print(tcp_data.split("|"))
            if calc_loc.check_data(tcp_data) == False:
                print("Hedef bozuk alındı yeni hedef bekleniyor")
                tcp_data = ""
                continue
            
            target_loc = calc_loc.calc_location(current_loc=vehicle.get_pos(drone_id=GOZLEMCI_ID), yaw_angle=vehicle.get_yaw(drone_id=GOZLEMCI_ID), tcp_data=tcp_data, DEG=vehicle.DEG)
            print(f"{GOZLEMCI_ID}>> hedef bulundu: {target_loc}")
            break
    
    ############### GOZLEMCI LAND ###############
    gozlemci_rtl = threading.Thread(target=manuel_RTL, args=(vehicle, GOZLEMCI_ID, gozlemci_home_pos))
    gozlemci_rtl.start()


    #? 2. aşama (tcp verisine gidiliyor)
    if target_loc != []:
        ############### SALDIRI DRONU TAKEOFF #################
        saldiri_home_pos_queue = queue.Queue()
        saldiri_takeoff_thread = threading.Thread(target=takeoff, args=(vehicle, saldiri1_config["alt"], SALDIRI_ID, saldiri_home_pos_queue), daemon=True)
        saldiri_takeoff_thread.start()

        saldiri_takeoff_thread.join()
        saldiri_home_pos = saldiri_home_pos_queue.get()
    
        vehicle.go_to(loc=target_loc, alt=saldiri1_config["alt"], drone_id=SALDIRI_ID)
        print(f"{SALDIRI_ID}>> hedefe gidiyor")

        start_time = time.time()
        while True:
            if time.time() - start_time > 5:
                print(f"{SALDIRI_ID}>> hedefe gidiyor...")
                print(f"Kalan mesafe: {distance(vehicle.get_pos(drone_id=SALDIRI_ID), target_loc)} m")
                start_time = time.time()
            
            if vehicle.on_location(loc=target_loc, seq=0, sapma=1, drone_id=SALDIRI_ID):
                print(f"{SALDIRI_ID}>> hedefe ulaştı")
                print(f"{SALDIRI_ID}>> iniş gerçekleştiriyor")
                vehicle.set_mode(mode="LAND", drone_id=SALDIRI_ID)
                break

    gozlemci_rtl.join()
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


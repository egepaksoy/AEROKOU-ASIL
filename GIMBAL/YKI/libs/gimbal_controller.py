import keyboard
import threading
import time

import tcp_handler
import calc_loc

class GimbalHandler:
    def __init__(self, server, stop_event):
        self.server = server
        self.stop_event = stop_event
    
    def request_data(self):
        self.server.send_data("2|2\n")

    def gimbal_selecter(self, stop_event: threading.Event, vehicle, DRONE_ID, server: tcp_handler.TCPServer, targets: dict, target_locker: threading.Lock, selecter_started: threading.Event):
        print("Hedef Seçimi başlatıldı")
        selecter_started.set()
        while not stop_event.is_set():
            tcp_data = None
            # Hedef secme
            if keyboard.is_pressed("x"):
                self.request_data()

                tcp_data = server.get_data()
                while tcp_data ==  None:
                    tcp_data = server.get_data()
                    time.sleep(0.1)

                target_name = input("Hedef adini giriniz: ")
                target_loc = calc_loc.calc_location_geopy(vehicle.get_pos(drone_id=DRONE_ID), vehicle.get_yaw(drone_id=DRONE_ID), tcp_data=tcp_data)
                
                for drone_id in vehicle.drone_ids:
                    if drone_id != DRONE_ID:
                        min_dist_drone_id = drone_id
                        min_drone_dist = calc_loc.get_dist(vehicle.get_pos(drone_id=drone_id), target_loc)
                        break
                for drone_id in vehicle.drone_ids:
                    if drone_id != DRONE_ID and drone_id not in targets:
                        if calc_loc.get_dist(vehicle.get_pos(drone_id=drone_id), target_loc) < min_drone_dist:
                            min_dist_drone_id = drone_id
                
                new_target = {"cls": target_name, "loc": target_loc}
                with target_locker:
                    targets[min_dist_drone_id] = new_target
                
                print(f"Hedef {target_name} {min_dist_drone_id} dronuna atandi")

            # Heddef silme
            elif keyboard.is_pressed("c"):
                deleted = False

                self.request_data()

                tcp_data = server.get_data()
                while tcp_data ==  None:
                    tcp_data = server.get_data()
                    time.sleep(0.1)

                target_loc = calc_loc.calc_location_geopy(vehicle.get_pos(drone_id=DRONE_ID), vehicle.get_yaw(drone_id=DRONE_ID), tcp_data=tcp_data)
                
                for drone_id in targets:
                    if calc_loc.get_dist(targets[drone_id]["loc"], target_loc) <= 2:
                        yes_no = input(f"Hedef {targets[drone_id]['cls']} silincek emin misiniz?\ny/n")

                        if yes_no == "y":
                            print(f"Hedef {targets[drone_id]['cls']} silindi")
                            targets.pop(drone_id)
                        else:
                            print("Hedef silme iptal edildi")

                        deleted = True
                        break
                
                if not deleted:
                    target_name = input("Silinecek Hedef adini giriniz: ")
                    for drone_id in targets:
                        if target_name == targets[drone_id]["cls"] or target_name.upper() == targets[drone_id]["cls"].upper() or target_name.lower() == targets[drone_id]["cls"].lower():
                            print(f"Hedef {targets[drone_id]['cls']} silindi")
                            targets.pop(drone_id)
                            deleted = True
                            break
                
                if not deleted:
                    print("Hedef silinemedi tekrar deneyin")
            
            # Cikis
            elif keyboard.is_pressed("z"):
                break
        
            time.sleep(0.05)

    def keyboard_controller(self):
        ters = -1

        while not self.stop_event.is_set():
            ser_data = ""
            ser_x = 0
            ser_y = 0

            if keyboard.is_pressed("x"):
                ser_x = 2
                ser_y = 2

            else:
                if keyboard.is_pressed('right'):
                    ser_x = -1 * ters

                if keyboard.is_pressed('left'):
                    ser_x = 1 * ters

                if keyboard.is_pressed('up'):
                    ser_y = 1 * ters

                if keyboard.is_pressed('down'):
                    ser_y = -1 * ters

            ser_data = f"{ser_x}|{ser_y}\n"
            self.server.send_data(ser_data)

            time.sleep(0.01)

    def joystick_controller(self, arduino):
        ters = -1

        while not self.stop_event.is_set():
            write_data = ""
            ser_x = 0
            ser_y = 0

            joystick_data = arduino.read_value()
            if len(joystick_data.split("|")) == 3:
                ser_x = 2
                ser_y = 2

            elif joystick_data != None:
                if "|" in joystick_data:
                    joystick_data = joystick_data.strip()
                
                    if int(joystick_data.split("|")[0]) > 0:
                        ser_x = 1 * ters
                    elif int(joystick_data.split("|")[0]) < 0:
                        ser_x = -1 * ters
                    
                    if int(joystick_data.split("|")[1]) > 0:
                        ser_y = 1 * ters
                    elif int(joystick_data.split("|")[1]) < 0:
                        ser_y = -1 * ters
                    
            
            write_data = f"{ser_x}|{ser_y}\n"            
            self.server.send_data(write_data)

            time.sleep(0.01)
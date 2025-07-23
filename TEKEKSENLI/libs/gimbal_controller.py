import keyboard
import threading
import time

import libs.tcp_handler as tcp_handler
import libs.calc_loc as calc_loc

class GimbalHandler:
    def __init__(self, server, stop_event):
        self.server = server
        self.stop_event = stop_event
        self.movement_command = 0 #Ana koddan degisitirilerek hareket saglanir
    
    def request_data(self):
        self.server.send_data("2|2\n")

    def gimbal_selecter(self, stop_event: threading.Event, vehicle, DRONE_ID, server: tcp_handler.TCPServer, targets: dict, target_locker: threading.Lock, selecter_started: threading.Event, keyboard: bool=True, yki_monitor=None):
        print("Hedef Seçimi başlatıldı")
        while not stop_event.is_set():
            tcp_data = None
            selecter_pressed = False
            deleter_pressed = False
            start_miss = False

            if yki_monitor != None and not keyboard:
                button_states = yki_monitor.get_target_select_status()

                if button_states["SELECT"]:
                    selecter_pressed = True
                elif button_states["DELETE"]:
                    deleter_pressed = True
                elif button_states["START"]:
                    start_miss = True
            else:
                if keyboard.is_pressed("x"):
                    selecter_pressed = True
                elif keyboard.is_pressed("c"):
                    deleter_pressed = True
                elif keyboard.is_pressed("z"):
                    start_miss = True


            # Hedef secme
            if selecter_pressed:
                selecter_pressed = False

                print("Hedef secme tusuna basildi")
                self.request_data()

                tcp_data = server.get_data()
                while tcp_data ==  None:
                    tcp_data = server.get_data()
                    time.sleep(0.1)

                target_name = input("Hedef adini giriniz: ")
                target_loc = calc_loc.calc_location_geopy(vehicle.get_pos(drone_id=DRONE_ID), vehicle.get_yaw(drone_id=DRONE_ID), tcp_data=tcp_data)
 
                min_dist_drone_id = None
                for drone_id in vehicle.drone_ids:
                    if drone_id != DRONE_ID:
                        min_dist_drone_id = drone_id
                        min_drone_dist = calc_loc.get_dist(vehicle.get_pos(drone_id=drone_id), target_loc)
                        break
                for drone_id in vehicle.drone_ids:
                    if drone_id != DRONE_ID and drone_id not in targets:
                        #if calc_loc.get_dist(vehicle.get_pos(drone_id=drone_id), target_loc) < min_drone_dist:
                            #min_dist_drone_id = drone_id
                        min_dist_drone_id = drone_id

                new_target = {"cls": target_name, "loc": target_loc}
                with target_locker:
                    targets[min_dist_drone_id] = new_target

                print(f"Hedef {target_name} {min_dist_drone_id} dronuna atandi")


            # Heddef silme
            elif deleter_pressed:
                deleter_pressed = False

                print("Hedef silme tusuna basildi")
                deleted = False

                #! Burası otomatik hedef silme icin
                #self.request_data()

                #tcp_data = server.get_data()
                #while tcp_data ==  None:
                    #tcp_data = server.get_data()
                    #time.sleep(0.1)

                #target_loc = calc_loc.calc_location_geopy(vehicle.get_pos(drone_id=DRONE_ID), vehicle.get_yaw(drone_id=DRONE_ID), tcp_data=tcp_data)

                #for drone_id in targets:
                    #if calc_loc.get_dist(targets[drone_id]["loc"], target_loc) <= 2:
                        #yes_no = input(f"Hedef {targets[drone_id]['cls']} silincek emin misiniz?\ny/n")

                        #if yes_no == "y":
                            #print(f"Hedef {targets[drone_id]['cls']} silindi")
                            #targets.pop(drone_id)
                        #else:
                            #print("Hedef silme iptal edildi")

                        #deleted = True
                        #break
                #! Bitti

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
            elif start_miss:
                start_miss = False
                print("Hedef secme kapatildi")
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

    def joystick_controller(self, yki_monitor):
        ters = -1

        while not self.stop_event.is_set():
            write_data = ""
            ser_x = 0
            ser_y = 0

            # Hedef seçme veya silme tusuna basildiysa
            if yki_monitor.get_target_select_status()["DELETE"] or yki_monitor.get_target_select_status()["SELECT"]:
                ser_x = 2
                ser_y = 2

            # Joystick hareket ettirildiyse
            else:
                joystick_data = yki_monitor.get_joystick_values()
                if joystick_data != None:
                    # TODO: buradaki > 0 işaretlerindeki 0'ı arduinodan al (arduinoda yarıdan büyük ise 1 küçük ise -1 yarısı kadar ise 0 dondur)
                    if int(joystick_data["JOY_X"]) > 0:
                        ser_x = 1 * ters
                    elif int(joystick_data["JOY_X"]) < 0:
                        ser_x = -1 * ters

                    if int(joystick_data["JOY_Y"]) > 0:
                        ser_y = 1 * ters
                    elif int(joystick_data["JOY_Y"]) < 0:
                        ser_y = -1 * ters

            write_data = f"{ser_x}|{ser_y}\n"
            self.server.send_data(write_data)

            time.sleep(0.01)
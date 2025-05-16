from pymavlink import mavutil, mavwp
import serial.tools.list_ports
from pymavlink.dialects.v10 import ardupilotmega as mavlink
import os
import time
import math
import threading
import requests
from bs4 import BeautifulSoup as bs

class Vehicle():
    def __init__(self, address: str=None, baud: int=57600, autoreconnect: bool=True, drone_id: int=1, on_flight: bool=True):
        try:
            address = self.check_address(address=address)
            self.vehicle = mavutil.mavlink_connection(device=address, baud=baud, autoreconnect=autoreconnect)
            self.vehicle.wait_heartbeat()
            print("Bağlantı başarılı")
            self.drone_id = drone_id
            self.drone_ids = []
            # 1 Metre
            self.DEG = 0.00001172485

            self.TAKEOFF_POS = {}

            if on_flight:
                self.get_all_drone_ids()
                drone_idler = self.get_all_drone_ids()

                if len(self.drone_ids) == 1:
                    self.drone_id = list(drone_idler)[0]
                    print(f"Ucusta tek drone var ve id'si: {self.drone_id}")
                else:
                    print("Ucustaki drone idleri: ", self.drone_ids)

                self.request_message_interval("ATTITUDE", 1, self.drone_ids)
                self.request_message_interval("GLOBAL_POSITION_INT", 2, self.drone_ids)
                self.request_message_interval("MISSION_CURRENT", 3, self.drone_ids)
                self.request_message_interval("VFR_HUD", 4, self.drone_ids)
                print("Mesajlar gonderildi")

                self.wp = mavwp.MAVWPLoader()
        except Exception as e:
            print("Baglanti saglanamadi: ", e)
            exit()
    
    def connect_port(self):
        ports = serial.tools.list_ports.comports()
        if ports:
            for port in ports:
                return port.device
        else:
            print("Hiçbir COM portu bağlı değil.")
            return None
    
    def parse_message(self, message):
        return message, message.get_srcSystem()

    # Baglantidaki tum drone idlerini getirir
    def get_all_drone_ids(self):
        drone_ids = set()

        start_time = time.time()

        try:
            self.vehicle.wait_heartbeat(blocking=True, timeout=2)
            first_drone_ids = self.drone_ids
            while time.time() - start_time < 3:
                msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)
                if msg:
                    drone_ids.add(msg.get_srcSystem())

            self.drone_ids.clear()
            for d_id in drone_ids:
                if d_id != 255:
                    self.drone_ids.append(d_id)

            return drone_ids
        except TimeoutError as e:
            self.drone_ids.clear()
            print("Drone idleri alinamadi: ", e)

        except Exception as e:
            self.drone_ids.clear()
            print("Drone idleri alinamadi: ", e)
            return e

    # Dronun hızını döndürür
    def get_speed(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            start_time = time.time()
            while True:
                message = self.vehicle.recv_match(type='VFR_HUD', blocking=True)

                if self.parse_message(message)[1] == drone_id:
                    return message.airspeed
                
                if time.time() - start_time > 5:
                    print(f"{drone_id}>> UYARI 5 SANIYEDIR HIZ DEGERI CEKILEMEDI!!!")
                    start_time = time.time()
                    
        except Exception as e:
            return e
    
    # Auto modunda drone hızını ayarlar
    def set_auto_speed(self, speed: float, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if speed >= 15:
                print(f"Dikkat drone hızını {speed} m/s olarak ayarladınız çok yüksek bir değer olabilir")
                Exception(f"Drone hızı çok yüksek: {speed} m/s")
            
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            self.vehicle.mav.command_long_send(
                drone_id,
                self.vehicle.target_component, 
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 
                0,  # Confirmation
                1,  # 0 = Hava hızı, 1 = Yer hızı
                speed,  # Yeni hız (m/s)
                0, 0, 0, 0, 0  # Boş parametreler
            )

            print(f"{drone_id}>> auto mod hızı {speed} m/s olarak ayarlandı")

        except Exception as e:
            return e
        
    # Guided modunda dronun hızını ayarlar
    def set_guided_speed(self, speed: float, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if speed >= 15:
                print(f"Dikkat drone hızını {speed} m/s olarak ayarladınız çok yüksek bir değer olabilir")
                Exception(f"Drone hızı çok yüksek: {speed} m/s")
            
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            self.vehicle.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms (kendisi güncelleyecek)
                drone_id,
                self.vehicle.target_component,  # hedef bileşen
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # NED koordinat düzlemi
                0b0000111111000111,  # sadece hız vektörünü kontrol etmek için bayrak
                0, 0, 0,  # hedef konum (kullanılmıyor)
                speed, speed, speed,  # hız (m/s)
                0, 0, 0,  # ivme (kullanılmıyor)
                0, 0  # yaw ve yaw_rate (kullanılmıyor)
            )
        
            print(f"{drone_id}>> guided mod hızı {speed} m/s olarak ayarlandı")

        except Exception as e:
            return e

    # Waypointleri döndürür
    def get_wp_list(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            self.vehicle.mav.mission_request_list_send(drone_id, self.vehicle.target_component)
            msg = self.vehicle.recv_match(type='MISSION_COUNT', blocking=True)
            waypoint_count = msg.count

            waypoints = []
            for i in range(waypoint_count):
                self.vehicle.mav.mission_request_int_send(drone_id, self.vehicle.target_component, i)
                message = self.vehicle.recv_match(type='MISSION_ITEM_INT', blocking=True)

                if self.parse_message(message)[1] == drone_id:
                    waypoints.append((msg.x / 1e7, msg.y / 1e7, msg.z / 1e3))  # Latitude ve Longitude değerleri 1e7 ile ölçeklendirilmiştir

            return waypoints

        except Exception as e:
            return e

    # Cekilecek mesajları ister
    def request_message_interval(self, message_input: str, frequency_hz: float, drone_ids: list=None):
        if drone_ids is None:
            drone_ids = self.drone_ids
        
        try:
            for drone_id in drone_ids:
                message_name = "MAVLINK_MSG_ID_" + message_input
                message_id = getattr(mavutil.mavlink, message_name)
                self.vehicle.mav.command_long_send(
                    drone_id, self.vehicle.target_system,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    message_id,
                    1e6 / frequency_hz,
                    0,
                    0, 0, 0, 0)
                print(f"{drone_id}>> drona {message_input} mesajı basairyla iletildi.")
        except Exception as e:
            return e

    # Dronun home konumunu döndürür
    def get_home_pos(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            return self.TAKEOFF_POS[drone_id]
        except:
            return None

    # Dronun konumunu döndürür
    def get_pos(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            start_time = time.time()
            while True:
                message = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

                if self.parse_message(message)[1] == drone_id:
                    lat = message.lat / 1e7
                    lon = message.lon / 1e7
                    alt = message.relative_alt / 1e3
                    return lat, lon, alt

                if time.time() - start_time > 5:
                    print(f"UYARI 5 SANIYEDIR DRONE {drone_id} DEN KONUM CEKILEMEDI!!!")
                    start_time = time.time()

        except Exception as e:
            return e

    # Anlık waypoint'i döndürür
    def get_miss_wp(self, miss_command: str="ITEM_REACHED", drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            start_time = time.time()
            while True:
                message = self.vehicle.recv_match(type=f'MISSION_{miss_command}', blocking=True)
                
                if self.parse_message(message)[1] == drone_id:
                    return int(message.seq)

                if time.time() - start_time > 5:
                    print(f"{drone_id}>> UYARI 5 SANIYEDIR WAYPOINT MESAJI CEKILMEDI!: {miss_command}")
                    if miss_command == "CURRENT":
                        miss_command = "ITEM_REACHED"
                    elif miss_command == "ITEM_REACHED":
                        miss_command = "CURRENT"
                    start_time = time.time()

        except Exception as e:
            return e

    # Yaw acisini derece cinsinden dondurur
    def get_yaw(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            start_time = time.time()
            while True:
                message = self.vehicle.recv_match(type='ATTITUDE', blocking=True)

                if self.parse_message(message)[1] == drone_id:
                    yaw_deg = math.degrees(message.yaw)
                    if yaw_deg < 0:
                        yaw_deg += 360.0
                    return yaw_deg

                if time.time() - start_time > 5:
                    print(f"{drone_id}>> UYARI 5 SANIYEDIR YAW ACISI CEKILEMEDI!!!")
                    start_time = time.time()

        except Exception as e:
            return e

    # Komut gercekleştirme mesaşlarını bekler
    def ack(self, keyword: str=None, keywords: list=None, drone_id: int=None):
        if keywords is None:
            keywords = [keyword]
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            msg = self.vehicle.recv_match(type=keywords, blocking=True)
            if msg.get_srcSystem() == drone_id and msg:
                print("-- Message Read " + str(msg))
                return True
            return False
        except Exception as e:
            return e
        
    # ID'li drone'nun waypointlerini siler
    def clear_wp_target(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            if self.vehicle.mavlink10():
                self.vehicle.mav.mission_clear_all_send(drone_id, self.vehicle.target_component)

            else:
                self.vehicle.mav.waypoint_clear_all_send(drone_id, self.vehicle.target_component)
            print(f"{drone_id}>> idli drone'nun waypointleri silindi")
        except Exception as e:
            return e

    # Tüm waypointleri gönderir
    def send_all_waypoints(self, wp_list: list, drone_id: int = None):
        if drone_id is None:
            drone_id = self.drone_id
        
        wpler = []

        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            # Tüm waypoint'leri temizle
            self.clear_wp_target(drone_id=drone_id)

            # Waypoint sayısını bildir
            self.vehicle.mav.mission_count_send(drone_id, self.vehicle.target_component, len(wp_list))

            # Tüm waypoint'leri mavlink mesajı olarak ekle
            for seq, waypoint in enumerate(wp_list):
                wpler.append(
                    mavutil.mavlink.MAVLink_mission_item_int_message(
                        drone_id,
                        self.vehicle.target_component,
                        seq,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1, 0, 0, 0, 0,
                        int(waypoint[0] * 1e7),  # Latitude
                        int(waypoint[1] * 1e7),  # Longitude
                        waypoint[2]  # Altitude
                    )
                )

            for i, wp in enumerate(wpler):
                self.vehicle.mav.send(wp)
                print(f"{drone_id}>> Waypoint {i} gönderildi")

            # İlk waypoint'i aktif yap
            self.vehicle.mav.mission_set_current_send(drone_id, self.vehicle.target_component, 0)

            print(f"{drone_id}>> {len(wp_list)} waypoint başarıyla gönderildi")
        except Exception as e:
            print(f"{drone_id}>> Waypoint gönderme sırasında hata oluştu: {e}")

    # Dronu guided modunda hareket ettirir
    def go_to(self, loc, alt, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if len(loc) != 2:
                Exception(f"Geçersiz konum: {loc}")

            lat = loc[0]
            lon = loc[1]

            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, drone_id,
                            self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                            int(0b110111111000),
                            int(lat * 1e7), int(lon * 1e7), alt,
                            0, 0, 0, 0, 0, 0, 0, 0)
                        )
        except Exception as e:
            return e
    
    # Tarama waypointlerini dondurur
    def scan_area_wpler(self, center_loc, alt, area_meter, distance_meter):
        met = -1 * (area_meter / 2)
        sign = 1
        i = 0
        wpler = []

        try:
            if len(center_loc) != 2:
                Exception(f"Geçersiz konum: {center_loc}")
            center_lat = center_loc[0]
            center_lon = center_loc[1]

            while i <= (area_meter / distance_meter):
                last_waypoint = (center_lat + (met + distance_meter * i) * self.DEG, center_lon + (met * sign) * self.DEG)
                sign *= -1
                i += 1
                wpler.append([last_waypoint[0], last_waypoint[1], alt])
            
            return wpler
        except Exception as e:
            return e

    # Drone'u arm eder veya disarm eder
    def arm_disarm(self, arm, drone_id: int=None, force_arm: bool=False):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            if arm == 0 or arm == 1:
                if force_arm:
                    self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 21196, 0, 0, 0, 0)
                else:
                    self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 0, 0, 0, 0, 0)
                if arm == 0:
                    print(f"{drone_id}>> Disarm edildi")
                if arm == 1:
                    print(f"{drone_id}>> ARM edildi")
                    if drone_id not in self.TAKEOFF_POS:
                        current_pos = self.get_pos(drone_id=drone_id)
                        if len(current_pos) != 3:
                            print(f"{drone_id}>> TAKEOFF Pozisyonu alınamadı!!!")
                        else:
                            self.TAKEOFF_POS[drone_id] = current_pos
            else:
                print(f"{drone_id}>> Gecersiz arm kodu: {arm}")
                exit()
        except Exception as e:
            return e
    
    # Manuel RTL
    def rtl(self, takeoff_pos, alt: int=None, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        if alt == None:
            alt = self.get_pos(drone_id=drone_id)[2]

        try:
            self.set_mode(mode="GUIDED", drone_id=drone_id)
            self.go_to(lat=takeoff_pos[0], lon=takeoff_pos[1], alt=alt, drone_id=drone_id)
            print(f"{drone_id}>> kalkış konumuna ({takeoff_pos}) dönüyor...")

            start_time = time.time()
            while self.on_location(loc=takeoff_pos, seq=0, sapma=1, drone_id=drone_id) == False:
                if time.time() - start_time > 2:
                    print(f"{drone_id}>> Kalkış konumuna dönüyor...")
                    start_time = time.time()
            
            self.set_mode(mode="LAND", drone_id=drone_id)
            print(f"{drone_id}>> kalkış konumuna dönüldü ve iniş yapılıyor...")
        
        except Exception as e:
            return e
    
    # Threadler icin takeoff fonksiyonu    
    def multiple_takeoff(self, alt, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"{drone_id}>> bağlı değil")

            self.vehicle.mav.command_long_send(
                drone_id, self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
            
            print(f"{drone_id}>> Takeoff mesaji gonderildi")
            print(f"{drone_id}>> Takeoff alınıyor...")

        except Exception as e:
            return e

    # Drona takeoff verir
    def takeoff(self, alt, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"{drone_id}>> bağlı değil")

            self.vehicle.mav.command_long_send(
                drone_id, self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
            
            print(f"{drone_id}>> Takeoff mesaji gonderildi")
            
            current_alt = 0
            start_time = time.time()

            print(f"{drone_id}>> Takeoff alınıyor...")
            mode = self.get_mode(drone_id=drone_id)
            while current_alt < alt * 0.9:
                current_alt = self.get_pos(drone_id=drone_id)[2]
                current_mode = self.get_mode(drone_id=drone_id)
                if time.time() - start_time > 2:
                    print(f"{drone_id}>> Anlık irtifa: {current_alt} metre")
                    start_time = time.time()
        
                if mode != current_mode:
                    print(f"!!!! UYARI MOD DEGISTI: {mode}->{current_mode}")
                    mode = current_mode
            
            print(f"{drone_id}>> {alt} metreye yükseldi")
        except Exception as e:
            return e

    # Dronun modunu belirler
    def set_mode(self, mode: str, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if drone_id not in self.drone_ids:
                Exception(f"{drone_id}>> bağlı değil")

            self.vehicle.wait_heartbeat(timeout=2)
            if mode == "RTL":
                self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

            elif mode == "AUTO":
                self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

            else:
                mode_id = self.vehicle.mode_mapping()[mode]
                if mode not in self.vehicle.mode_mapping():
                    print(f"{drone_id}>> Mod değiştirilemedi gecersiz mod: ", mode)
                    exit()
                else:
                    self.vehicle.mav.command_long_send(drone_id, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)

            time.sleep(0.1)
            current_mode = self.get_mode(drone_id=drone_id)
            if current_mode != mode:
                self.set_mode(mode=mode, drone_id=drone_id)

            else:
                print(f"{drone_id}>> Mod {mode} yapıldı")
        except Exception as e:
            return e

    # Dronun arm durumunu kontrol eder
    def is_armed(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"{drone_id}>> bağlı değil")
                
            start_time = time.time()
            while True:
                msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True)

                if self.parse_message(msg)[1] == drone_id:
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        return 1
                    else:
                        return 0

                if time.time() - start_time > 5:
                    print(f"{drone_id}>> UYARI!!! 5 saniyedir arm durumu cekilmedi!!!")
                    start_time = time.time()

        except Exception as e:
            return e
            
    # Dronun baglanti yolunu kontrol eder
    def check_address(self, address: str):
        if address is None:
            address = self.connect_port()
            if address is None:
                print("COM portu bağlı değil.")
                exit()

        if "tcp" not in address and "udp" not in address:
            if not os.path.exists(address):
                print("Dosya yolu yanlis yada yok:\n", address)
                exit()
        print("Baglanti yolu onaylandi")
        return address

    # Dronun modunu elde eder
    def get_mode(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            start_time = time.time()
            while True:
                msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True)
                
                if self.parse_message(msg)[1] == drone_id:
                    return mavutil.mode_string_v10(msg)

                if time.time() - start_time > 5:
                    print(f"{drone_id}>> UYARI!!! 5 SANiYEDiR MOD BiLGiSi ALINAMADI!!!")
                    start_time = time.time()

        except Exception as e:
            return e

    # Dronun serbo pwm'ini ayarlar
    def set_servo(self, drone_id: int=None, channel: int=9, pwm: int=1000):
        if drone_id is None:
            drone_id = self.drone_id

        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            self.vehicle.mav.command_long_send(
                drone_id,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm,
                0, 0, 0, 0, 0)
        except Exception as e:
            return e

    # sağa sola ileri geri hareket ettirme
    def move_drone(self, rota, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            vx, vy, vz = rota

            self.vehicle.mav.set_position_target_local_ned_send(
                0,  # Timestamp
                drone_id, self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Koordinat sistemi
                0b0000111111000111,  # Type mask (sadece hız kullanılacak)
                0, 0, 0,  # Pozisyon (kullanılmıyor)
                vx, vy, vz,  # Hız (m/s)
                0, 0, 0,  # İvme (kullanılmıyor)
                0, 0)  # Yaw, yaw_rate (kullanılmıyor)

        except Exception as e:
            print(e)
            return e

    def get_distance(self, loc1, loc2):
        R = 6371000
    
        lat1, lon1 = loc1[0], loc1[1]
        lat2, lon2 = loc2[0], loc2[1]

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi / 2.0) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(delta_lambda / 2.0) ** 2
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

    # Konumda oldugunu kontrol etme
    # pos, seq, sapma (mt), drone_id
    def on_location(self, loc, seq: int=0, sapma: int=1, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            if seq != 0:
                if abs(self.get_pos(drone_id=drone_id)[0] - loc[0]) <= self.DEG*sapma and abs(self.get_pos(drone_id=drone_id)[1] - loc[1]) <= self.DEG*sapma and self.get_miss_wp(drone_id=drone_id) == seq:
                    return True
                return False
            else:
                if abs(self.get_pos(drone_id=drone_id)[0] - loc[0]) <= self.DEG*sapma and abs(self.get_pos(drone_id=drone_id)[1] - loc[1]) <= self.DEG*sapma:
                    return True
                return False
        except Exception as e:
            return e

    # Hata mesajlarını okuma
    def error_messages(self):
        start_time = time.time()
        while True:
            msg = self.vehicle.recv_match(type="STATUSTEXT", blocking=True)
            
            if msg:
                drone_id = self.parse_message(msg)[1]  # Hata mesajının geldiği drone id'si
                error_flag = msg.severity  # Hata mesajının ciddiyet seviyesi
                error_msg = msg.text       # Hata mesajı metni
                
                # Ciddiyet seviyesi açıklamaları
                if error_flag == 0:
                    level = "EMERGENCY"
                elif error_flag == 1:
                    level = "ALERT"
                elif error_flag == 2:
                    level = "CRITICAL"
                elif error_flag == 3:
                    level = "ERROR"
                elif error_flag == 4:
                    level = "WARNING"
                elif error_flag == 5:
                    level = "NOTICE"
                elif error_flag == 6:
                    level = "INFO"
                elif error_flag == 7:
                    level = "DEBUG"
                else:
                    level = "UNKNOWN"

                return drone_id, level, error_msg
            
            if time.time() - start_time > 5:
                return None


    def turn_around(self, default_speed: int=30, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            yaw_angle = (360 + (self.get_yaw(drone_id=drone_id) - 20)) % 360
            clock_wise = 1

            print(f"{drone_id}>> Etrafında dönücek, yaw açısı: {yaw_angle}")
            self.vehicle.mav.command_long_send(
                drone_id,
                self.vehicle.target_component, # Hedef bileşen ID
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Yaw kontrol komutu
                0,                       # Confirmation (0: İlk komut)
                int(yaw_angle),               # Yaw açısı
                default_speed,                      # Dönüş hızı (derece/saniye)
                clock_wise,                       # Yön (1: Saat yönü, -1: Saat tersi)
                0,           # Açı göreceli mi? (0: Global, 1: Relative)
                0, 0, 0                  # Kullanılmayan parametreler
            )

            print(f"{drone_id}>> Etrafında dönülüyor...")

            start_time = time.time()
            while True:
                yaw_speed = self.yaw_speed(drone_id=drone_id)
                if abs(yaw_speed) >= 0.1:
                    not_turning = False
                    
                else:
                    if time.time() - start_time >= 1:
                        if not_turning:
                            break
                        start_time = time.time()
                    not_turning = True
                    
            print(f"{drone_id}>> Dönüş tamamlandı")
            return None

        
        except Exception as e:
            return e
    
    def yaw_speed(self, drone_id: int=None):
        if drone_id is None:
            drone_id = self.drone_id
        
        try:
            if drone_id not in self.drone_ids:
                Exception(f"Drone bağlantısı yok: {drone_id}")

            start_time = time.time()
            while True:
                msg = self.vehicle.recv_match(type='ATTITUDE', blocking=True)
                
                if self.parse_message(msg)[1] == drone_id:
                    return msg.yawspeed

                if time.time() - start_time > 5:
                    print(f"{drone_id}>> UYARI!!! 5 SANiYEDiR YAW BiLGiSi ALINAMADI!!!")
                    start_time = time.time()

        except Exception as e:
            return e


# Kameradan goruntu hesaplama kodu
def calc_hipo_angle(screen_rat_x_y, x_y, alt, yaw, alt_met):
    screen_x = screen_rat_x_y[0]
    screen_y = screen_rat_x_y[1]
    screen_y_mid = screen_y/2
    screen_x_mid = screen_x/2
    x = x_y[0]
    y = x_y[1]

    hipo = math.sqrt(abs(screen_x_mid-x)**2 + abs(screen_y_mid-y)**2)

    angle = math.degrees(math.atan2(abs(screen_x_mid-x),abs(screen_y_mid-y)))

    x_sign = x-screen_x_mid
    y_sign = y-screen_y_mid

    if x_sign < 0 and y_sign < 0:
        angle = 270+angle
    elif x_sign < 0 and y_sign > 0:
        angle = 180 + angle
    elif x_sign > 0 and y_sign < 0:
        angle = angle
    elif x_sign > 0 and y_sign > 0:
        angle = 180 - angle
    elif x_sign == 0 and y_sign < 0:
        angle = 0
    elif x_sign == 0 and y_sign > 0:
        angle = 180
    elif x_sign < 0 and y_sign == 0:
        angle = 270
    elif x_sign > 0 and y_sign == 0:
        angle = 90
    elif x_sign == 0 and y_sign == 0:
        angle = 0

    #       metre       derece
    return hipo*alt/alt_met, (yaw + angle) % 360

def calc_location(uzaklik, aci, lat, lon):
    DEG = 0.00001172485
    
    ates_pos = [uzaklik*math.sin(math.radians(aci)), uzaklik*math.cos(math.radians(aci))]
    return lat + ates_pos[0] * DEG + DEG, lon + ates_pos[1] * DEG + DEG

def get_pixel_pos(line):
    if len(line.split(",")) == 4:
        return ((float(line.split(",")[2])-float(line.split(",")[0])) / 2, (float(line.split(",")[3])-float(line.split(",")[1])) / 2)
    
    elif len(line.split(",")) == 2:
        return (float(line.split(",")[0]), float(line.split(",")[1]))
    
    return float(line.split(",")[0]), float(line.split(",")[1])

def parse_website_data(url):
    try:
        webpage = requests.get(url)
        if webpage.status_code != 200:
            Exception(f"Web sayfası bulunamadı: {url}")

        soup = bs(webpage.content, "html.parser")

        yonu = soup.find("td", {"id": "ruzgar_yonu"}).text.strip()
        hizi = soup.find("td", {"id": "ruzgar_hizi"}).text.strip()
        maks_hizi = soup.find("td", {"id": "maks_ruzgar_hizi"}).text.strip()

        if yonu == "←":
            yonu = 1
        
        else:
            yonu = -1
        
        hizi = 1 + (float(hizi) / float(maks_hizi)) * 2

        return yonu, hizi
    except Exception as e:
        print(e)

        return 1, 1

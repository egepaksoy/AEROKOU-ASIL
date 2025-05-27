#! RASP
import tcp_handler
import sys
import socket
import time
from picamera2 import Picamera2
import cv2
import struct
import json
import threading
import serial_handler
import smbus
import time
from math import ceil


def frame_send_new(config):
    UDP_IP, UDP_PORT = config["UDP"]["ip"], config["UDP"]["port"]
    CHUNK_SIZE = 1400                      # ~ MTU altı
    HEADER_FMT = '<LHB'                   # frame_id:uint32, chunk_id:uint16, is_last:uint8

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # PiCamera2'yi başlat ve yapılandır
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
    picam2.start()
    time.sleep(2)  # Kamera başlatma süresi için bekle

    frame_id = 0
    try:
        print(f"{UDP_IP} adresine gönderim başladı")
        broadcast_started.set()
        while not stop_event.is_set():
            frame = picam2.capture_array()
            _, buf = cv2.imencode('.jpg', frame)
            data = buf.tobytes()
            total_chunks = ceil(len(data) / CHUNK_SIZE)

            for chunk_id in range(total_chunks):
                start = chunk_id * CHUNK_SIZE
                end = start + CHUNK_SIZE
                chunk = data[start:end]
                is_last = 1 if chunk_id == total_chunks - 1 else 0
                header = struct.pack(HEADER_FMT, frame_id, chunk_id, is_last)
                sock.sendto(header + chunk, (UDP_IP, UDP_PORT))

            frame_id = (frame_id + 1) & 0xFFFFFFFF
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Ctrl+C ile çıkıldı.")

    finally:
        # Kamera ve soketi kapat
        print("Program sonlandırıldı.")
        picam2.stop()
        sock.close()

def get_distance(repeat=5, LIDAR_ADDRESS = 0x62):
    """ 
    LIDAR Lite v3'ten mesafe okur, birkaç ölçüm alarak ortalama hesaplar.
    repeat: Ortalama alınacak ölçüm sayısı (Gürültüyü azaltır).
    """

    bus = smbus.SMBus(1)  # Raspberry Pi'de I2C-1 hattı kullanılıyor
    distances = []
    
    for _ in range(repeat):
        try:
            # LIDAR'a ölçüm yapmasını söyle
            bus.write_byte_data(LIDAR_ADDRESS, 0x00, 0x04)
            time.sleep(0.02)  # Ölçüm süresi

            # 16-bit mesafe verisini oku
            high_byte = bus.read_byte_data(LIDAR_ADDRESS, 0x0f)
            low_byte = bus.read_byte_data(LIDAR_ADDRESS, 0x10)
            distance_cm = (high_byte << 8) + low_byte  # Mesafeyi cm olarak hesapla

            if 0 < distance_cm < 4000:  # LIDAR'ın ölçebileceği mesafe aralığı
                distances.append(distance_cm)
            else:
                print("Geçersiz ölçüm alındı, tekrar deneniyor...")
            
        except OSError:
            print("I2C bağlantı hatası! LIDAR bağlı mı?")
            return None  # Bağlantı hatası olursa None döndür

        time.sleep(0.001)  # Sensörün stabilize olması için bekleme süresi
    
    if not distances:
        return None  # Geçerli ölçüm alınamadıysa None döndür

    avg_distance_cm = sum(distances) / len(distances)  # Ölçümleri ortalama alarak hassasiyeti artır
    avg_distance_m = avg_distance_cm / 100  # Metreye çevir

    return round(avg_distance_m, 3)  # Ölçümü metre cinsinden 3 ondalık basamakla döndür


config = json.load(open("./dron_config.json", "r"))
stop_event = threading.Event()

try:
    broadcast_started = threading.Event()

    threading.Thread(target=frame_send_new, args=(config, ), daemon=True).start()

    while not broadcast_started.is_set():
        continue

    if broadcast_started.is_set():
        client = tcp_handler.TCPClient(ip=config["TCP"]["ip"], port=config["TCP"]["port"], stop_event=stop_event)
        
        arduino = serial_handler.Serial_Control(port=config["ARDUINO"]["port"])
        print(f"Arduino {config['ARDUINO']['port']} portundan bağlandı")

        timer = time.time()
        arduino_val = ""
        while not stop_event.is_set():
            data = client.get_data()

            if data == None:
                continue

            if "2|2" in data:
                distance = get_distance()

                if distance == None:
                    distance = get_distance(repeat=15)
                
                if distance == None:
                    print("LiDAR'dan mesafe bilgisi alınamadı")

                else:
                    arduino_val = arduino.read_value().strip()
                    print("arduino derece:", arduino_val)

                    if "|" in arduino_val:
                        arduino_val_split = arduino_val.split("|")
                        client.send_data(data=f"{distance}|{arduino_val.split('|')[0].strip()}|{arduino_val.split('|')[1].strip()}")

            elif "|" in data:
                data = data.strip()
                data = f"{data.split('|')[0]}|{data.split('|')[1]}\n"
                arduino.send_to_arduino(data)

                arduino_val = arduino.read_value().strip()

                if "|" in arduino_val:
                    arduino_val_split = arduino_val.split("|")
                    send = True
                    for i in arduino_val_split:
                        if not i.isdigit():
                            send = False
                    
                    if send:
                        client.send_data(data=f"{arduino_val.split('|')[0].strip()}|{arduino_val.split('|')[1].strip()}")
            
            time.sleep(0.01)

except KeyboardInterrupt:
    print("CTRL+C ile cikldi")

except Exception as e:
    print("Hata: ", e)
    print(e.args)

finally:
    if not stop_event.is_set():
        stop_event.set()
    arduino.ser.close()
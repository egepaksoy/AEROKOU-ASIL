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


def frame_send(config):
    # Kullanıcıdan IP adresi ve port numarasını komut satırından al
    UDP_IP = config["UDP"]["ip"]  # Alıcı bilgisayarın IP adresi
    UDP_PORT = config["UDP"]["port"]  # Alıcı bilgisayarın port numarası

    # 61440
    PACKET_SIZE = 60000  # UDP paket boyutu, 60 KB

    # UDP soketi oluştur
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # PiCamera2'yi başlat ve yapılandır
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
    picam2.start()
    time.sleep(2)  # Kamera başlatma süresi için bekle

    frame_counter = 0  # Çerçeve sayacını başlat

    total_frame = 0
    frame_time = time.time()
    try:
        broadcast_started.set()
        while not stop_event.is_set():
            # Kamera görüntüsünü yakala
            frame = picam2.capture_array()
            # Görüntüyü JPEG formatında sıkıştır
            _, buffer = cv2.imencode('.jpg', frame)
            buffer = buffer.tobytes()  # Veriyi baytlara dönüştür
            buffer_size = len(buffer)  # Verinin boyutunu al

            # Veriyi parçalara böl ve gönder
            for i in range(0, buffer_size, PACKET_SIZE):
                end = i + PACKET_SIZE
                if end > buffer_size:
                    end = buffer_size
                packet = struct.pack('<L', frame_counter) + buffer[i:end]
                sock.sendto(packet, (UDP_IP, UDP_PORT))

            # Son paketle bir "son" işareti gönder
            sock.sendto(struct.pack('<L', frame_counter) + b'END', (UDP_IP, UDP_PORT))

            if frame_counter == 20000:
                frame_counter = 0
            frame_counter += 1  # Çerçeve sayacını artır
            total_frame += 1

            if time.time() - frame_time >= 0.1:
                #print(f"FPS: {total_frame / (time.time() - frame_time)}")
                total_frame = 0
                frame_time = time.time()

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

        time.sleep(0.01)  # Sensörün stabilize olması için bekleme süresi
    
    if not distances:
        return None  # Geçerli ölçüm alınamadıysa None döndür

    avg_distance_cm = sum(distances) / len(distances)  # Ölçümleri ortalama alarak hassasiyeti artır
    avg_distance_m = avg_distance_cm / 100  # Metreye çevir

    return round(avg_distance_m, 3)  # Ölçümü metre cinsinden 3 ondalık basamakla döndür


config = json.load(open("./dron_config.json", "r"))
stop_event = threading.Event()
servo_angles = ""

gonderildi = False

try:
    broadcast_started = threading.Event()

    threading.Thread(target=frame_send, args=(config, ), daemon=True).start()

    while not broadcast_started.is_set():
        continue

    if broadcast_started.is_set():
        client = tcp_handler.TCPClient(ip=config["TCP"]["ip"], port=config["TCP"]["port"])
        
        arduino = serial_handler.Serial_Control(port=config["ARDUINO"]["port"])
        print(f"Arduino {config['ARDUINO']['port']} portundan bağlandı")

        while not stop_event.is_set():
            data = client.get_data()
            if data == None:
                continue

            if "|" in data:
                data = data.strip()
                data = f"{data.split('|')[0]}|{data.split('|')[1]}\n"
                arduino.send_to_arduino(data)

            if "2|2" in data:
                distance = get_distance()

                if distance == None:
                    distance = get_distance(repeat=15)
                
                if distance == None:
                    print("LiDAR'dan mesafe bilgisi alınamadı")

                else:
                    line = arduino.read_value().strip()
                    print("arduino derece:", line)

                    if "|" in line:
                        client.send_data(data=f"{distance}|{line.split('|')[0]}|{line.split('|')[1]}")

except KeyboardInterrupt:
    print("CTRL+C ile cikldi")

except Exception as e:
    print("Hata: ", e)
    print(e.args)

finally:
    if not stop_event.is_set():
        stop_event.set()
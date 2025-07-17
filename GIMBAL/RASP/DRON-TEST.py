#! RASP
import tcp_handler
import time
import json
import threading
import serial_handler
import smbus
import time

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
                print("uzaklik:", distance)

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
                
                #if send:
                    #client.send_data(data=f"{arduino_val.split('|')[0].strip()}|{arduino_val.split('|')[1].strip()}")
        
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

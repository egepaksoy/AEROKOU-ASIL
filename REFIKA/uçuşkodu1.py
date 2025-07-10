import time
import sys
import threading
from math import cos, radians

sys.path.append('./pymavlink_custom')
from pymavlink_custom import Vehicle
from mqtt_controller import magnet_control, rotate_servo, cleanup


def failsafe(vehicle):
    def failsafe_drone_id(vehicle, drone_id):
        print(f"{drone_id}>> Failsafe alıyor")
        vehicle.set_mode(mode="RTL", drone_id=drone_id)
    threads = []
    for d_id in vehicle.drone_ids:
        t = threading.Thread(target=failsafe_drone_id, args=(vehicle, d_id))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()
    print(f"Dronlar {vehicle.drone_ids} Failsafe aldı")


def main():
    if len(sys.argv) != 3:
        print("Usage: python main.py <connection_string> <drone_id>")
        sys.exit(1)

    magnet_control(True, True)
    print("mıknatıslar açık")

    vehicle = Vehicle(sys.argv[1])
    DRONE_ID = int(sys.argv[2])
    ALT = 5

    loc1 = (40.7121258, 30.0245359, ALT)  # konum 1
    loc2 = (40.7121348, 30.0246058, ALT)  # konum 2

    try:        
        rotate_servo(0)
        print("servo duruyor")

        vehicle.set_mode("GUIDED", drone_id=DRONE_ID)
        vehicle.arm_disarm(True, drone_id=DRONE_ID)
        vehicle.takeoff(ALT, drone_id=DRONE_ID)

        print(f"{DRONE_ID}>> Kalkış tamamlandı")

        vehicle.go_to(loc=loc1, alt=ALT, drone_id=DRONE_ID)
        print(f"{DRONE_ID}>> Hedef 1'e gidiyor")

        start_time = time.time()
        while not vehicle.on_location(loc=loc1, seq=0, sapma=1, drone_id=DRONE_ID):
            if time.time() - start_time >= 5:
                print(f"{DRONE_ID}>> Hedef 1'e gidiyor...")
                start_time = time.time()
            time.sleep(0.01)
        
        print(f"{DRONE_ID}>> Hedef konuma ulaşıldı")

        time.sleep(3)
        magnet_control(True, False)
        print("mıknatıs2 kapatıldı")
        time.sleep(3)

        vehicle.go_to(loc=loc2, alt=ALT, drone_id=DRONE_ID)
        print(f"{DRONE_ID}>> Hedef 2'e gidiyor")

        start_time = time.time()
        while not vehicle.on_location(loc=loc2, seq=0, sapma=1, drone_id=DRONE_ID):
            if time.time() - start_time >= 5:
                print(f"{DRONE_ID}>> Hedef 2'ye gidiyor...")
                start_time = time.time()
            time.sleep(0.01)
        print(f"{DRONE_ID}>> İkinci hedef konuma ulaşıldı")

        time.sleep(3)
        magnet_control(False, False)
        print("mıknatıs1 kapatıldı")
        time.sleep(3)

        # LAND başlat
        vehicle.set_mode(mode="LAND", drone_id=DRONE_ID)
        print(f"{DRONE_ID}>> LAND başlatıldı")

        print(f"{DRONE_ID}>> Drone iniş yapıyor, görev tamamlanmak üzere...")

    except KeyboardInterrupt:
        print("Klavye ile çıkış yapıldı")
        failsafe(vehicle)

    except Exception as e:
        failsafe(vehicle)
        print("Hata:", e)

    finally:
        vehicle.vehicle.close()
        cleanup()
        print("GPIO temizlendi, bağlantı kapatıldı")


if __name__ == "__main__":
    main()

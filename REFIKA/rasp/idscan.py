import time
import sys
sys.path.append('./pymavlink_custom')
from pymavlink_custom import Vehicle

vehicle = Vehicle()

start_time = time.time()

try:
    while True:
        print(1)
        for i in vehicle.drone_ids:
            print(f"{i}: {vehicle.get_mode(drone_id=i)}")
            print(f"{i}: {vehicle.get_pos(drone_id=i)}")

except KeyboardInterrupt:
    print("Exiting...")

except Exception as e:
    print(e)

finally:
    vehicle.vehicle.close()
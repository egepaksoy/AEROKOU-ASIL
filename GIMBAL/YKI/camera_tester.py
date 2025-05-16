import threading
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


stop_event = threading.Event()

camera = image_processing_handler.Handler()
camera_thread = threading.Thread(target=camera.local_camera, args=(0, ), daemon=True)
camera_thread.start()

try:
    while not stop_event.is_set():
        continue


except KeyboardInterrupt:
    print("Exiting...")
    if not stop_event.is_set():
        stop_event.set()

except Exception as e:
    if not stop_event.is_set():
        stop_event.set()
    print(e)

finally:
    if not stop_event.is_set():
        stop_event.set()
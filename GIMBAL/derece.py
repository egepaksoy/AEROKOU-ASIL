import keyboard
import time
from YKI.libs import serial_handler

ters = -1

arduino = serial_handler.Serial_Control("com5")

try:
    while True:
        ser_x = 0
        ser_y = 0
        if keyboard.is_pressed('right') or keyboard.is_pressed('d'):
            ser_x = -1 * ters

        if keyboard.is_pressed('left') or keyboard.is_pressed('a'):
            ser_x = 1 * ters

        if keyboard.is_pressed('up') or keyboard.is_pressed('w'):
            ser_y = 1 * ters

        if keyboard.is_pressed('down') or keyboard.is_pressed('s'):
            ser_y = -1 * ters

        data = f"{ser_x}|{ser_y}\n"
        arduino.send_to_arduino(data)
        print(arduino.read_value())

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Durduruldu")
import keyboard
import time


class GimbalHandler:
    def __init__(self, server, stop_event):
        self.server = server
        self.stop_event = stop_event

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
                if keyboard.is_pressed('right') or keyboard.is_pressed('d'):
                    ser_x = -1 * ters

                if keyboard.is_pressed('left') or keyboard.is_pressed('a'):
                    ser_x = 1 * ters

                if keyboard.is_pressed('up') or keyboard.is_pressed('w'):
                    ser_y = 1 * ters

                if keyboard.is_pressed('down') or keyboard.is_pressed('s'):
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

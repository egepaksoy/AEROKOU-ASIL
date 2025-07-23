import libs.serial_handler as serial_handler
import time
import threading

class YKIMonitor(threading.Thread):
    def __init__(self, config: dict, stop_event: threading.Event):
        super().__init__()
        self.daemon = True  # Ana program bittiğinde thread de bitsin
        self.arduino = serial_handler.Serial_Control(config["ARDUINO"]["port"])
        self.lock = threading.Lock()
        self.stop_event = stop_event
        self.ters = -1
        
        # Paylaşılacak durumlar
        self.arm_states = {
            'ARM1': False,
            'ARM2': False,
            'ARM3': False
        }

        self.system_status = True

        self.joystick_values = {
            "JOY_X": 0,
            "JOY_Y": 0
        }

        self.target_select_values = {
            "SELECT": False,
            "DELETE": False
        }

        self.system_started = False

    def run(self):
        while not self.stop_event.is_set():
            try:
                line = self.arduino.read_value()
                if line == "" or line == None:
                    continue

                parts = line.split()

                with self.lock:
                    self.arm_states["ARM1"] = bool(int(parts[0].split(":")[1]))
                    self.arm_states["ARM2"] = bool(int(parts[1].split(":")[1]))
                    self.arm_states["ARM3"] = bool(int(parts[2].split(":")[1]))
                
                    self.system_status = bool(int(parts[3].split(":")[1]))

                    self.joystick_values["JOY_X"] = int(parts[4].split(":")[1]) * self.ters
                    self.joystick_values["JOY_Y"] = int(parts[5].split(":")[1])
                
                    self.target_select_values["SELECT"] = bool(int(parts[7].split(":")[1]))
                    self.target_select_values["DELETE"] = bool(int(parts[8].split(":")[1]))
                    self.target_select_values["START"] = bool(int(parts[6].split(":")[1]))

                    self.system_started = True

            except Exception:
                continue
            
            time.sleep(0.05)

    def get_arm_status(self):
        while not self.system_started:
            time.sleep(0.01)
        with self.lock:
            return self.arm_states

    def get_system_status(self):
        while not self.system_started:
            time.sleep(0.01)
        with self.lock:
            return self.system_status

    def get_joystick_values(self):
        while not self.system_started:
            time.sleep(0.01)
        with self.lock:
            return self.joystick_values

    def get_target_select_status(self):
        while not self.system_started:
            time.sleep(0.01)
        with self.lock:
            return self.target_select_values
    
    def get_data_started(self):
        while not self.system_started:
            time.sleep(0.01)
        return self.system_started

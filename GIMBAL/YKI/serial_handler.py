import serial.tools.list_ports
import sys

class Serial_Control:
    def __init__(self, port=None):
        self.port = port
        self.ser = self.connect_serial()
    
    def connect_serial(self):
        """Seri porta bağlanır."""
        seri_portlar = serial.tools.list_ports.comports()
        if self.port == None:
            seri_port = ""
            print("Seri portlar listelendi.")
            for port in seri_portlar:
                if "ch340" in port.description.lower():
                    seri_port = port.device
                    break
        else:
            seri_port = self.port

        if seri_port == "":
            print("CH340 çevirici bulunamadı!")
            sys.exit(1)
        
        ser = serial.Serial(seri_port, 9600)  # Arduino'nun bağlı olduğu portu ve baud rate'i belirle
        print("Bağlantı sağlandı:", ser.name)
        return ser

    def send_to_arduino(self, data):
        """Arduino'ya seri port üzerinden veri gönderir."""
        try:
            self.ser.write(data.encode('utf-8'))

        except serial.SerialException as e:
            print(f"Seri port hatası: {e}")
        except Exception as e:
            print(f"Hata: {e}")
    
    def read_value(self):
        """Seri porttan veri okur."""
        ser = self.ser
        ser_value = ""
        if ser.in_waiting > 0:
            ser_value = str(ser.readline().decode("utf-8", errors='ignore')).strip()

        return ser_value

    
def is_data_valid(data):
    """Gelen verinin geçerli olup olmadığını kontrol eder."""
    try:
        if "|" in data:
            x_value = data.split("|")[0].strip()
            y_value = data.split("|")[1].strip()

            if len(x_value) == 4 and len(y_value) == 4:
                pass
            else:
                return False

            if x_value.isdigit() and y_value.isdigit():
                return True
            else:
                return False
        else:
            return False
    except:
        return False

def return_normal(value):
    value = str(value)
    for i in str(value):
        if i == "0":
            continue
        else:
            return value[value.index(i):]

    return "0"
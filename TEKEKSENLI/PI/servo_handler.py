import time
import threading
import RPi.GPIO as GPIO

class ServoHandler(threading.Thread):
    def __init__(self, config: dict, stop_event: threading.Event):
        super().__init__()
        self.daemon = True

        self.stop_event = stop_event

        self.current_angle = config["SERVO"]["zero_angle"]
        self.carpan = config["SERVO"]["carpan"]
        self.max_angle = config["SERVO"]["max_angle"]
        self.min_angle = config["SERVO"]["min_angle"]

        # GPIO ayarları
        self.SERVO_PIN = config["SERVO"]["pin"]
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)

        # PWM başlat (50Hz -> SG90 servo için ideal)
        self.pwm = GPIO.PWM(self.SERVO_PIN, 50)
        self.pwm.start(0)

    def run(self):
        while not self.stop_event.is_set():
            self.set_angle(self.current_angle)
            time.sleep(0.05)

    # Açıları duty cycle'a çevirme fonksiyonu
    def set_angle(self, angle):
        duty = 2 + (angle / 18)
        GPIO.output(self.SERVO_PIN, True)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        GPIO.output(self.SERVO_PIN, False)
        self.pwm.ChangeDutyCycle(0)
    
    # Hareket fonksiyonu
    def move_servo(self, direction):
        direction *= self.carpan
        if self.max_angle >= direction + self.current_angle and self.min_angle <= direction + self.current_angle:
            self.current_angle += direction
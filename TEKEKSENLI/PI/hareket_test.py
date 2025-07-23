import RPi.GPIO as GPIO
import time

SERVO_PIN = 17

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM frekansı
pwm.start(0)

def set_angle(angle):
    duty = 2 + (angle / 18)  # 0-180 derece için duty cycle hesaplama
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        set_angle(48)
        time.sleep(2)
        set_angle(140)
        time.sleep(2)
        set_angle(48)
        time.sleep(2)
except KeyboardInterrupt:
    pass
finally:
    pwm.ChangeDutyCycle(0)
    time.sleep(0.5)
    pwm.stop()
    GPIO.cleanup()

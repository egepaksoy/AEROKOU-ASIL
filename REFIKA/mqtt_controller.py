import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

BROKER_ADRESS = "192.168.0.120"
BROKER_PORT = 1883
TOPIC = "kontrol"

client = mqtt.Client()
client.connect(BROKER_ADRESS, BROKER_PORT, 60)

servo_pin = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)


def send_command (command):
        client.publish(TOPIC, command)
        print(f"[MQTT] Komut gönderildi: {command}")

def magnet_control (magnet1: bool, magnet2: bool):
        if magnet1:
                send_command("em1_on")
        else:
                send_command("em1_off")

        if magnet2:
                send_command("em2_on")
        else:
                send_command("em2_off")
'''
def set_servo_angle(angle):
        duty = angle / 18 + 2.5
        GPIO.output(SERVO_PIN, True)
        pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        GPIO.output(SERVO_PIN, False)
        pwm.ChangeDutyCycle(0)
'''
def rotate_servo(direction):
        if direction == 1:
                pwm.ChangeDutyCycle(8.0)

        elif direction == -1:
                pwm.ChangeDutyCycle(6.0)

        elif direction == 0:
                pwm.ChangeDutyCycle(7.5)



def cleanup():
        pwm.stop()
        GPIO.cleanup()
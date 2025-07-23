import time
import json
import sys

sys.path.append("./libs")
from mqtt_controller import magnet_control

def drop_obj(miknatis):
    time.sleep(3)
    if miknatis == 2:
        magnet_control(True, False)
    else:
        magnet_control(False, True)
    print(f"mıknatıs {miknatis} kapatıldı")
    time.sleep(2)


try:
    config = json.load(open("./config.json"))
    miknatis = config["OBJECTS"]["Altigen"]["miknatis"]

    magnet_control(True,True)
    time.sleep(5)

    print("mıknatis: ", miknatis)
    drop_obj(miknatis)
    time.sleep(1000)
finally:
    magnet_control(False, False)
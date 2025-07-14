from flask import Flask, Response
import threading
import numpy as np
import cv2
import time
from picamera2 import Picamera2

app = Flask(__name__)
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()
time.sleep(2)  # Kamera başlatma süresi için bekle


def is_equilateral(approx, tolerance=0.15):
    if len(approx) < 3:
        return False
    sides = []
    for i in range(len(approx)):
        pt1 = approx[i][0]
        pt2 = approx[(i + 1) % len(approx)][0]
        dist = np.linalg.norm(pt1 - pt2)
        sides.append(dist)
    mean = np.mean(sides)
    return all(abs(s - mean) / mean < tolerance for s in sides)

# Renk aralıkları
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])
lower_blue = np.array([100, 100, 100])
upper_blue = np.array([130, 255, 255])

shared_state = {"last_object": None, "timestamp": 0}
shared_state_lock = threading.Lock()

@app.route('/')
def video():
    def gen_frames():
        global last_frame
        frame_lock = threading.Lock()

        while True:
            # 1. Kamera görüntüsü al ve çevir
            frame = picam2.capture_array()

            frame = cv2.flip(frame, -1)
            detected_obj = ""

            # 2. Görüntü işleme başlasın
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

            for color_mask, shape_name, target_sides, color in [
                (red_mask, "Ucgen", 3, (0, 0, 255)),
                (blue_mask, "Altigen", 6, (255, 0, 0))
            ]:
                contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    epsilon = 0.02 * cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, epsilon, True)
                    if len(approx) == target_sides and is_equilateral(approx):
                        cv2.drawContours(frame, [approx], 0, color, 2)
                        x, y = approx[0][0]
                        cv2.putText(frame, shape_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        detected_obj = shape_name
                        print(detected_obj)

            if detected_obj != "":
                with shared_state_lock:
                    shared_state["last_object"] = detected_obj
                    shared_state["timestamp"] = time.time()

            # 3. Görüntüyü paylaş
            with frame_lock:
                last_frame = frame.copy()

            _, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(host='0.0.0.0', port=5000)
import cv2
import threading
import time
import numpy as np
from flask import Flask, Response

last_frame = None

def image_recog_flask(cam, port, broadcast_started, stop_event, shared_state, shared_state_lock, oran=0.3):
    app = Flask(__name__)

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
    if type(cam).__name__ == "VideoCapture":
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([130, 255, 255])
    
        red_bg = (0, 0, 255)
        blue_bg = (255, 0, 0)
    else:
        lower_red1 = np.array([100, 100, 0])
        upper_red1 = np.array([255, 255, 10])
        lower_red2 = np.array([100, 100, 160])
        upper_red2 = np.array([255, 255, 179])
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([255, 255, 130])
        
        red_bg = (0, 0, 255)
        blue_bg = (255, 0, 0)

    @app.route('/')
    def video():
        def gen_frames():
            def visualize_box(frame, screen_res, orta_box_color = (255, 0, 255)):
                x1 = (screen_res[0] - (screen_res[0] * oran)) / 2
                x2 = screen_res[0] - x1
                y1 = (screen_res[1] - (screen_res[1] * oran)) / 2
                y2 = screen_res[1] - y1

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), orta_box_color, 1)

            global last_frame
            frame_lock = threading.Lock()

            broadcast_started.set()

            algilanan_obj = []

            while not stop_event.is_set():
                # 1. Kamera görüntüsü al ve çevir
                if type(cam).__name__ == "VideoCapture":
                    _, frame = cam.read()
                    res = frame.shape[:2]
                    screen_res = (int(res[1]), int(res[0]))
                else:
                    frame = cam.capture_array()
                    frame = cv2.flip(frame, -1)
                    screen_res = (frame.shape[:2][1], frame.shape[:2][0])

                detected_obj = ""
                object_pos = None

                # 2. Görüntü işleme başlasın
                blurred = cv2.GaussianBlur(frame, (5, 5), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

                for color_mask, shape_name, target_sides, color in [
                    (red_mask, "Ucgen", 3, red_bg),
                    (blue_mask, "Altigen", 6, blue_bg)
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
                            object_pos = (x, y)
                            if detected_obj not in algilanan_obj:
                                print(detected_obj)
                                algilanan_obj.append(detected_obj)

                if detected_obj != "":
                    with shared_state_lock:
                        shared_state["last_object"] = detected_obj
                        shared_state["object_pos"] = object_pos
                        shared_state["screen_res"] = screen_res
                else:
                    with shared_state_lock:
                        shared_state["last_object"] = None
                        shared_state["object_pos"] = None
                        shared_state["screen_res"] = None
                
                visualize_box(frame, screen_res, oran)

                # 3. Görüntüyü paylaş
                with frame_lock:
                    last_frame = frame.copy()

                _, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    app.run(host='0.0.0.0', port=port)
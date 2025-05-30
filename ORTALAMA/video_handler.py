#! GIBI
import cv2
import socket
import numpy as np
import struct
from ultralytics import YOLO
import threading
import math

class Handler:
    def __init__(self, model, stop_event, distance_yaw=None, proccessing=True, middle_range = 0.2):
        self.model = model
        self.proccessing = proccessing
        self.stop_event = stop_event
        self.showing_image = True
        self.middle_range = middle_range
        self.distance_yaw = distance_yaw

        self.udp_connected = threading.Event()
    
    def visualise(self, screen_res, middle_rect_range, frame, box=None, model=None, object_color=(255, 0, 0)):
        y, x = screen_res
        middle_x1 = int((x - x*middle_rect_range) / 2)
        middle_x2 = int((x + x*middle_rect_range) / 2)

        middle_y1 = int((y - y*middle_rect_range) / 2)
        middle_y2 = int((y + y*middle_rect_range) / 2)

        middle_rect_color = (0, 0, 255)

        cv2.rectangle(frame, (middle_x1, middle_y1), (middle_x2, middle_y2), middle_rect_color, 2)

        if box != None:
            # Sınıf ve güven skorunu al
            cls = int(box.cls[0].item())
            conf = box.conf[0].item()

            # Sınırlayıcı kutu koordinatlarını al
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            konumu = (int(x1 + (x2-x1)/2), int(y1 + (y2-y1)/2))

            # Sınıf adını al
            class_name = model.names[cls]
        
            cv2.rectangle(frame, (x1, y1), (x2, y2), object_color, 2)
            cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, object_color, 2)
            
            object_center = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cv2.circle(frame, konumu, 5, object_color, 2)

            cv2.line(frame, object_center, (int(screen_res[1] / 2), int(screen_res[0] / 2)), (0, 255, 0), 1)

            return class_name, conf, (x1, y1, x2, y2)
    
    def is_centered(self, screen_res, object_center):
        res_x, res_y = 1, 1
        obj_x, obj_y = object_center
        x, y = screen_res
        middle_x1 = int((x - x*self.middle_range) / 2)
        middle_x2 = int((x + x*self.middle_range) / 2)

        middle_y1 = int((y - y*self.middle_range) / 2)
        middle_y2 = int((y + y*self.middle_range) / 2)

        if obj_x <= middle_x2 and obj_x >= middle_x1:
            res_x = 0
        if obj_y <= middle_y2 and obj_y >= middle_y1:
            res_y = 0
        
        return res_x, res_y
    
    def center_object_pixel_distance(self, screen_res, object_size):
        screen_center = screen_res[0] / 2, screen_res[1] / 2
        object_center = (int(object_size[0]) + int(object_size[2])) / 2, (int(object_size[1]) + int(object_size[3])) / 2

        if self.is_centered(screen_res, object_center) == (0,0):
            return (0,0)
        return (screen_center[1] - object_center[1]), (screen_center[0] - object_center[0]) * -1
    
    def get_yaw(self, distance):
        return math.atan(distance[0] / distance[1])

    def local_camera(self, camera_path):
        cap = cv2.VideoCapture(camera_path)

        if not cap.isOpened():
            print(f"Dahili kamera {camera_path} açılamadı")
        
        self.udp_connected.set()
        while not self.stop_event.is_set():
            visualised = False
            ret, frame = cap.read()
            screen_res = frame.shape[:2]

            if self.proccessing and self.model != None:
                results = self.model(frame, verbose=False)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        if box.conf[0] < 0.90:
                            continue
                        
                        visualised = True
                        class_name, conf, (x1, y1, x2, y2) = self.visualise(screen_res, self.middle_range, frame, box, self.model)
                        distance = self.center_object_pixel_distance((frame.shape[:2]), (x1, y1, x2, y2))
                        yaw = self.get_yaw(distance)
                        if self.distance_yaw != None:
                            self.distance_yaw.put((distance, yaw))

                        print(f"Sınıf: {class_name}, Güven: {conf:.2f}, Konum: ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}")

                        # Nesneyi çerçeve içine al ve etiketle
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            if not visualised:
                self.visualise(screen_res, self.middle_range, frame)

            if self.showing_image:
                cv2.imshow("local image", frame)

            # Çıkış için 'q' tuşuna basılması beklenir
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def ortalama_udp(self, ip, port):
        BUFFER_SIZE = 65536
        HEADER_FMT = '<LHB'
        HEADER_SIZE = struct.calcsize(HEADER_FMT)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))

        buffers = {}  # {frame_id: {chunk_id: bytes, …}, …}
        expected_counts = {}  # {frame_id: total_chunks, …}

        self.udp_connected.set()
        while not self.stop_event.is_set():
            visualised = False
            packet, _ = sock.recvfrom(BUFFER_SIZE)
            frame_id, chunk_id, is_last = struct.unpack(HEADER_FMT, packet[:HEADER_SIZE])
            chunk_data = packet[HEADER_SIZE:]
            
            # Kaydet
            if frame_id not in buffers:
                buffers[frame_id] = {}
            buffers[frame_id][chunk_id] = chunk_data
            
            # Toplam parça sayısını son pakette işaretle
            if is_last:
                expected_counts[frame_id] = chunk_id + 1

            # Hepsi geldiyse işle
            if frame_id in expected_counts and len(buffers[frame_id]) == expected_counts[frame_id]:
                # Birleştir
                data = b''.join(buffers[frame_id][i] for i in range(expected_counts[frame_id]))
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                
                if frame is not None:
                    screen_res = frame.shape[:2]
                    if self.proccessing and self.model != None:
                        results = self.model(frame, verbose=False)
                        for r in results:
                            boxes = r.boxes
                            for box in boxes:
                                if box.conf[0] < 0.90:
                                    continue
                                
                                visualised = True
                                class_name, conf, (x1, y1, x2, y2) = self.visualise(screen_res, self.middle_range, frame, box, self.model)
                                distance = self.center_object_pixel_distance((frame.shape[:2]), (x1, y1, x2, y2))
                                yaw = self.get_yaw(distance)
                                if self.distance_yaw != None:
                                    self.distance_yaw.put((distance, yaw))

                                print(f"Sınıf: {class_name}, Güven: {conf:.2f}, Konum: ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}")

                                # Nesneyi çerçeve içine al ve etiketle
                                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)


                    if not visualised:
                        self.visualise(screen_res, self.middle_range, frame)

                    if self.showing_image:
                        cv2.imshow("udp image", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        # Temizlik
        if frame_id in buffers:
            del buffers[frame_id]
        if frame_id in expected_counts:
            del expected_counts[frame_id]

    def show_image(self, show=True):
        self.showing_image = show
    
    def start_proccessing(self, model_path):
        self.proccessing = True
        if self.model == None:
            self.model = YOLO(model_path)
            print("model yuklendi")
    
    def stop_proccessing(self):
        self.proccessing = False
    
    def get_pixel_distance_yaw(self):
        if not self.distance_yaw.empty():
            return self.distance_yaw.get_nowait()
        return None
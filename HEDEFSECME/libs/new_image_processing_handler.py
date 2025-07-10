import cv2
import numpy as np
import math
from ultralytics import YOLO
import struct
import socket

class Handler:
    def __init__(self, stop_event):
        self.model = None
        self.proccessing = False
        self.stop_event = stop_event
        self.showing_image = True
        self.show_crosshair = True
        self.crosshair_color = (255, 255, 0)
        self.screen_ratio = None
        self.current_objects = []  # En güncel nesneleri burada tutar

    def local_camera_new(self, camera_path):
        cap = cv2.VideoCapture(camera_path)
        if not cap.isOpened():
            print(f"Kamera {camera_path} açılamadı.")
            return

        while not self.stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                continue

            objects = []

            if self.screen_ratio is None:
                self.screen_ratio = (frame.shape[1], frame.shape[0])
                print("Ekran oranı:", self.screen_ratio)

            if self.proccessing:
                results = self.model(frame, verbose=False)
                for r in results:
                    for box in r.boxes:
                        if box.conf[0] < 0.90:
                            continue
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        obj_center = ((x1 + x2) / 2, (y1 + y2) / 2)
                        cls = int(box.cls[0].item())
                        conf = box.conf[0].item()
                        class_name = self.model.names[cls]
                        print(f"{class_name} ({conf:.2f}) @ ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f})")

                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        objects.append([class_name, self.distance(obj_center)])

            # Güncel objeleri dışarıdan erişilebilir hale getir
            self.current_objects = objects

            if self.show_crosshair:
                height, width, _ = frame.shape
                center_x, center_y = width // 2, height // 2
                size = 20
                cv2.line(frame, (center_x - size, center_y), (center_x + size, center_y), self.crosshair_color, 2)
                cv2.line(frame, (center_x, center_y - size), (center_x, center_y + size), self.crosshair_color, 2)

            if self.showing_image:
                cv2.imshow("İşlenen Görüntü", frame)

            if cv2.waitKey(10) & 0xFF == ord("q"):
                break

    def udp_camera_new(self, ip, port):
        BUFFER_SIZE = 65536
        HEADER_FMT = '<LHB'
        HEADER_SIZE = struct.calcsize(HEADER_FMT)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))

        buffers = {}  # {frame_id: {chunk_id: bytes, …}, …}
        expected_counts = {}  # {frame_id: total_chunks, …}

        while not self.stop_event.is_set():
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
                    frame = cv2.flip(frame, -1)

                    objects = []
            
                    if self.screen_ratio is None:
                        self.screen_ratio = (frame.shape[1], frame.shape[0])
                        print("Ekran oranı:", self.screen_ratio)
                    
                    if self.proccessing and self.model != None:
                        results = self.model(frame, verbose=False)
                        for r in results:
                            boxes = r.boxes
                            for box in boxes:
                                if box.conf[0] < 0.90:
                                    continue
                                # Sınırlayıcı kutu koordinatlarını al
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                obj_center = ((x1 + x2) / 2, (y1 + y2) / 2)

                                # Sınıf ve güven skorunu al
                                cls = int(box.cls[0].item())
                                conf = box.conf[0].item()

                                # Sınıf adını al
                                class_name = self.model.names[cls]

                                # Bilgileri ekrana yazdır
                                print(f"Sınıf: {class_name}, Güven: {conf:.2f}, Konum: ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}")

                                # Nesneyi çerçeve içine al ve etiketle
                                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                                objects.append([class_name, self.distance(obj_center)])

                    # Güncel objeleri dışarıdan erişilebilir hale getir
                    self.current_objects = objects

                    if self.show_crosshair:
                        height, width, _ = frame.shape

                        # Ortadaki + işaretinin koordinatları
                        center_x = width // 2
                        center_y = height // 2
                        cross_size = 20  # Artı işaretinin uzunluğu

                        # Yatay çizgi
                        cv2.line(frame, (center_x - cross_size, center_y), (center_x + cross_size, center_y), self.crosshair_color, 2)
                        # Dikey çizgi
                        cv2.line(frame, (center_x, center_y - cross_size), (center_x, center_y + cross_size), self.crosshair_color, 2)

                    if self.showing_image:
                        cv2.imshow("udp image", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        # Temizlik
        if frame_id in buffers:
            del buffers[frame_id]
        if frame_id in expected_counts:
            del expected_counts[frame_id]

    def distance(self, obj_pos):
        if self.screen_ratio is None:
            return float("inf")
        return math.hypot(obj_pos[0] - self.screen_ratio[0]/2,
                          obj_pos[1] - self.screen_ratio[1]/2)

    def show_hide_crosshair(self, show):
        self.show_crosshair = show

    def show_image(self):
        self.showing_image = True

    def hide_image(self):
        self.showing_image = False

    def start_proccessing(self, model_path):
        self.proccessing = True
        if self.model is None:
            self.model = YOLO(model_path)
            print("Model yüklendi.")

    def stop_proccessing(self):
        self.proccessing = False

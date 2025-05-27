#! GIBI
import cv2
import socket
import numpy as np
import struct
from ultralytics import YOLO
import time

class Handler:
    def __init__(self, stop_event):
        self.model = None
        self.proccessing = False
        self.stop_event = stop_event
        self.showing_image = True
        self.show_crosshair = True
        self.crosshair_color = (255, 255, 0)

    def local_camera(self, camera_path):
        cap = cv2.VideoCapture(camera_path)

        if not cap.isOpened():
            print(f"Dahili kamera {camera_path} açılamadı")
        
        start_time = time.time()
        while not self.stop_event.is_set():
            ret, frame = cap.read()

            if time.time() - start_time > 0.1 and self.proccessing:
                results = self.model(frame)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        if box.conf[0] < 0.90:
                            continue
                        # Sınırlayıcı kutu koordinatlarını al
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

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

                start_time = time.time()
            
            if self.show_crosshair:
                height, width, _ = frame.shape

                # Ortadaki + işaretinin koordinatları
                center_x = width // 2
                center_y = height // 2
                cross_size = 20  # Artı işaretinin uzunluğu

                # Yatay çizgi
                cv2.line(frame, (center_x - cross_size, center_y), (center_x + cross_size, center_y), self.crosshair_color, 5)
                # Dikey çizgi
                cv2.line(frame, (center_x, center_y - cross_size), (center_x, center_y + cross_size), self.crosshair_color, 5)

                        
            if self.showing_image:
                cv2.imshow("İşlenen görüntü", frame)

            # Çıkış için 'q' tuşuna basılması beklenir
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def udp_camera(self, ip, port):
        BUFFER_SIZE = 65536
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))

        buffer = b''  # Gelen veri parçalarını depolamak için tampon
        current_frame = -1  # Geçerli çerçeve numarasını takip etmek için sayaç
        start_time = time.time()
        while not self.stop_event.is_set():
            data, addr = sock.recvfrom(BUFFER_SIZE)  # Maksimum UDP paket boyutu kadar veri al
            
            # Çerçeve numarasını çöz
            frame_number = struct.unpack('<L', data[:4])[0]
            packet_data = data[4:]

            if frame_number != current_frame:
                if buffer:
                    # Görüntüyü verinin tamamı alındığında oluştur
                    npdata = np.frombuffer(buffer, dtype=np.uint8)
                    frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        frame = cv2.flip(frame, -1)
                        if time.time() - start_time > 0.1 and self.proccessing:
                            results = self.model(frame)
                            for r in results:
                                boxes = r.boxes
                                for box in boxes:
                                    if box.conf[0] < 0.90:
                                        continue
                                    # Sınırlayıcı kutu koordinatlarını al
                                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

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

                            start_time = time.time()
                        
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
                            cv2.imshow("İşlenen görüntü", frame)

                        # Çıkış için 'q' tuşuna basılması beklenir
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                    buffer = b''  # Yeni görüntü için tamponu sıfırla

                current_frame = frame_number  # Geçerli çerçeve numarasını güncelle

            if packet_data == b'END':
                # Son paket işareti, çerçevenin sonu
                continue
            
            buffer += packet_data  # Gelen veri parçasını tampona ekle
    
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
                    if self.proccessing and self.model != None:
                        results = self.model(frame)
                        for r in results:
                            boxes = r.boxes
                            for box in boxes:
                                if box.conf[0] < 0.90:
                                    continue
                                # Sınırlayıcı kutu koordinatlarını al
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

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


    
    def show_hide_crosshair(self, show):
        self.show_crosshair = show
    
    def show_image(self):
        self.showing_image = True
    
    def hide_image(self):
        self.showing_image = False
    
    def start_proccessing(self, model_path):
        self.proccessing = True
        if self.model == None:
            self.model = YOLO(model_path)
            print("model yuklendi")
    
    def stop_proccessing(self):
        self.proccessing = False
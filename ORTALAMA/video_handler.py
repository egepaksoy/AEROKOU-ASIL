#! GIBI
import sys
import cv2
import socket
import numpy as np
import struct
from ultralytics import YOLO
import time
import queue
import threading

class Handler:
    def __init__(self, model=None, processing=False, showing_image=True, show_crosshair=False):
        self.model = model
        self.proccessing = processing
        self.is_running = True
        self.showing_image = showing_image
        self.show_crosshair = show_crosshair
        self.crosshair_color = (255, 255, 0)

        self.udp_connected = threading.Event()

    def local_camera(self, camera_path):
        cap = cv2.VideoCapture(camera_path)

        if not cap.isOpened():
            print(f"Dahili kamera {camera_path} açılamadı")
        
        start_time = time.time()
        while self.is_running:
            ret, frame = cap.read()

            if time.time() - start_time > 0.1 and self.proccessing:
                results = self.model(frame, verbose=False)
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
    
    def visualise_results(self, frame, box):
        # Sınıf ve güven skorunu al
        cls = int(box.cls[0].item())
        conf = box.conf[0].item()

        # Sınırlayıcı kutu koordinatlarını al
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        konumu = (x1 + (x2-x1)/2, y1 + (y2-y1)/2)

        # Sınıf adını al
        class_name = self.model.names[cls]

        # Nesneyi çerçeve içine al ve etiketle
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,0, 255), 2)
        cv2.circle(frame, (int(konumu[0]), int(konumu[1])), 5, (0, 0, 255), -1)

        cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1 - 10)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
        return class_name, (x1, y1, x2, y2)

    def is_centered(self, xyxy, ekran_orani, orta_pozisyon_orani: float = 0.4):
        x, y = (xyxy[0] + (xyxy[2] - xyxy[0]) / 2), (xyxy[1] + (xyxy[3] - xyxy[1]) / 2)
        ekran_genislik, ekran_yukseklik = ekran_orani
        # Merkezdeki alanın sınırlarını belirle     baslangıc, bitis
        x_orta_genisligi = ekran_genislik * orta_pozisyon_orani
        y_orta_genisligi = ekran_yukseklik * orta_pozisyon_orani

        x_uzaklik = 0
        y_uzaklik = 0

        if x > ekran_genislik * (0.5 + orta_pozisyon_orani / 2):
            x_uzaklik = x - ekran_genislik * (0.5 + orta_pozisyon_orani / 2)
        elif x < ekran_genislik * (0.5 - orta_pozisyon_orani / 2):
            x_uzaklik = x - ekran_genislik * (0.5 - orta_pozisyon_orani / 2)
        
        if y > ekran_yukseklik * (0.5 + orta_pozisyon_orani / 2):
            y_uzaklik = y - ekran_yukseklik * (0.5 + orta_pozisyon_orani / 2)
        elif y < ekran_yukseklik * (0.5 - orta_pozisyon_orani / 2):
            y_uzaklik = y - ekran_yukseklik * (0.5 - orta_pozisyon_orani / 2)
        
        return float(x_uzaklik), float(y_uzaklik)
    
    def ortalama_udp(self, config, image_queue: queue.Queue):
        orta_pozisyon_orani = (0.2)

        BUFFER_SIZE = 65536
        HEADER_FMT = '<LHB'
        HEADER_SIZE = struct.calcsize(HEADER_FMT)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((config["UDP"]["ip"], config["UDP"]["port"]))

            buffers = {}  # {frame_id: {chunk_id: bytes, …}, …}
            expected_counts = {}  # {frame_id: total_chunks, …}

            self.udp_connected.set()
            print("UDP Bağlandı fonksiyon")
            while self.is_running:
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
                        ekran_orani = (frame.shape[1], frame.shape[0])
                        if self.proccessing and self.model != None:
                            results = self.model(frame, verbose=False)
                            for r in results:
                                boxes = r.boxes
                                for box in boxes:
                                    if box.conf[0] < 0.90:
                                        continue
                                
                                    class_name, xyxy = self.visualise_results(frame, box)
                                
                                    cv2.putText(frame, f"center_range: {self.is_centered(xyxy, ekran_orani, orta_pozisyon_orani)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                                    image_queue.put((class_name, self.is_centered(xyxy, ekran_orani, orta_pozisyon_orani)))
                            
                            cv2.rectangle(frame, (int(ekran_orani[0] * (0.5 - orta_pozisyon_orani / 2)), int(ekran_orani[1] * (0.5 - orta_pozisyon_orani / 2))), (int(ekran_orani[0] * (0.5 + orta_pozisyon_orani / 2)), int(ekran_orani[1] * (0.5 + orta_pozisyon_orani / 2))), (255, 0, 0), 2)

                    if self.showing_image:
                        cv2.imshow("Hedef Ortalama", frame)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
            
            # Temizlik
            del buffers[frame_id], expected_counts[frame_id]
        
        finally:
            if "buffers" in locals() and "expected_counts" in locals():
                del buffers[frame_id], expected_counts[frame_id]
            print("Görüntü işleme durduruldu.")
            cv2.destroyAllWindows()
            sock.close()


    def udp_camera_new(self, ip, port):
        BUFFER_SIZE = 65536
        HEADER_FMT = '<LHB'
        HEADER_SIZE = struct.calcsize(HEADER_FMT)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((ip, port))

            buffers = {}  # {frame_id: {chunk_id: bytes, …}, …}
            expected_counts = {}  # {frame_id: total_chunks, …}

            self.udp_connected.set()
            while self.is_running:
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
                            results = self.model(frame, verbose=False)
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
            del buffers[frame_id], expected_counts[frame_id]
        
        finally:
            print("Görüntü işleme durduruldu.")
            cv2.destroyAllWindows()
            sock.close()

    def udp_camera(self, ip, port):
        BUFFER_SIZE = 65536
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))

        buffer = b''  # Gelen veri parçalarını depolamak için tampon
        current_frame = -1  # Geçerli çerçeve numarasını takip etmek için sayaç
        start_time = time.time()

        self.udp_connected.set()
        while self.is_running:
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
                            results = self.model(frame, verbose=False)
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
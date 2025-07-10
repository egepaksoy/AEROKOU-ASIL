import cv2
import socket
import struct
import sys
import numpy as np


if len(sys.argv) != 3:
    print("kod kullanımı: python image_listener.py <ip> <port>")
    exit(1)

ip = sys.argv[1]
port = int(sys.argv[2])

try:
    BUFFER_SIZE = 65536
    HEADER_FMT = '<LHB'
    HEADER_SIZE = struct.calcsize(HEADER_FMT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))

    buffers = {}  # {frame_id: {chunk_id: bytes, …}, …}
    expected_counts = {}  # {frame_id: total_chunks, …}

    while True:
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
                cv2.imshow("udp image", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("Cikildi")

except Exception as e:
    print(e)

finally:
    # Temizlik
    if frame_id in buffers:
        del buffers[frame_id]
    if frame_id in expected_counts:
        del expected_counts[frame_id]
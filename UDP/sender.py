import socket, struct, time, sys, cv2
from math import ceil

UDP_IP, UDP_PORT = sys.argv[1], int(sys.argv[2])
CHUNK_SIZE = 1400                      # ~ MTU altı
HEADER_FMT = '<LHB'                   # frame_id:uint32, chunk_id:uint16, is_last:uint8

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

cap = cv2.VideoCapture(0)

frame_id = 0
try:
    print(f"{UDP_IP} adresine gönderim başladı")
    while True:
        ret, frame = cap.read()
        _, buf = cv2.imencode('.jpg', frame)
        data = buf.tobytes()
        total_chunks = ceil(len(data) / CHUNK_SIZE)

        for chunk_id in range(total_chunks):
            start = chunk_id * CHUNK_SIZE
            end = start + CHUNK_SIZE
            chunk = data[start:end]
            is_last = 1 if chunk_id == total_chunks - 1 else 0
            header = struct.pack(HEADER_FMT, frame_id, chunk_id, is_last)
            sock.sendto(header + chunk, (UDP_IP, UDP_PORT))

        frame_id = (frame_id + 1) & 0xFFFFFFFF
        time.sleep(0.01)  # ~30 fps
except KeyboardInterrupt:
    pass
finally:
    cap.release()
    sock.close()

import cv2
import numpy as np

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

# Renk aralıklarını HSV formatında tanımla
lower_red1 = np.array([0, 70, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 70, 50])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([100, 150, 50])
upper_blue = np.array([140, 255, 255])

# Kamerayı başlat veya bir görsel yükle
cap = cv2.VideoCapture(0)  # Kamera yerine 'resim.jpg' de kullanılabilir

while True:
    ret, frame = cap.read()
    if not ret:
        break

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Maske oluştur
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Kontur bul
    for color_mask, shape_name, target_sides, color in [
        (red_mask, "Üçgen", 3, (0, 0, 255)),
        (blue_mask, "Altıgen", 6, (255, 0, 0))
    ]:
        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx) == target_sides and is_equilateral(approx):
                cv2.drawContours(frame, [approx], 0, color, 2)
                x, y = approx[0][0]
                cv2.putText(frame, shape_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imshow("Sekil Algilama", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

from geopy.distance import geodesic, distance
from geopy.point import Point
import math

def calculate_ground_distance(
    drone_height,
    xy_center,
    xy_screen,
    xy_fov= (62.2, 48.8)
):
    fov_x_deg, fov_y_deg = xy_fov
    image_width, image_height = xy_screen
    pixel_x, pixel_y = xy_center

    # Piksel başına düşen açı (derece cinsinden)
    angle_per_pixel_x = fov_x_deg / image_width
    angle_per_pixel_y = fov_y_deg / image_height

    # Görüntünün merkezine göre fark
    dx = pixel_x - (image_width / 2)
    dy = pixel_y - (image_height / 2)

    # Merkezden sapma açıları (derece)
    angle_x_deg = dx * angle_per_pixel_x
    angle_y_deg = dy * angle_per_pixel_y

    # Açıları radyana çevir
    angle_x_rad = math.radians(angle_x_deg)
    angle_y_rad = math.radians(angle_y_deg)

    # Yere olan yatay uzaklıkları hesapla (tan(θ) = karşı/komşu => karşı = tan(θ) * komşu)
    ground_x = math.tan(angle_x_rad) * drone_height
    ground_y = math.tan(angle_y_rad) * drone_height

    # Öklidyen uzaklık (drone altından olan yatay mesafe)
    ground_distance = math.sqrt(ground_x**2 + ground_y**2)

    return ground_distance

def get_position(camera_distance, total_yaw, current_loc):
    start_loc = Point(current_loc[0], current_loc[1])
    hedef = geodesic(meters=camera_distance).destination(start_loc, bearing=total_yaw)

    return hedef.latitude, hedef.longitude

def get_distance(loc1, loc2):
    return distance(loc1[:2], loc2[:2]).m
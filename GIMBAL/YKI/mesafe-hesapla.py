from geopy.point import Point
from geopy.distance import geodesic
import calc_loc

# Başlangıç konumu
loc = (40.71199, 30.0245)
baslangic = Point(loc[0], loc[1])

# 50m kuzey ve 100m doğu yaklaşık 63.4 derece yön ve 111.8m mesafe
hedef = geodesic(meters=8).destination(baslangic, bearing=63.4)
tcp_data = "8|0|0"
target_loc = calc_loc.calc_location(current_loc=loc, yaw_angle=63.4, tcp_data=tcp_data, DEG=0.00001172485)

print(f"geopy: \n{hedef.latitude}, {hedef.longitude}")
print(f"calc_loc: \n{target_loc}")
print(f"{geodesic((hedef.latitude, hedef.longitude), target_loc).meters}m")

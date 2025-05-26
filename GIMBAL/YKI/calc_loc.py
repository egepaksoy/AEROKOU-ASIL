import math

def calc_location(current_loc, yaw_angle, tcp_data, DEG):
    # Calculate the new location of the gimbal
    # based on the distance from the current location
    # and the angles of the servos
    #! tcp_data_format: distance|servo_x|servo_y

    print("TCP Data Fonk: ", tcp_data)
    abs_distance, servo_x_angle, servo_y_angle = tcp_data.strip().split("|")
    abs_distance = float(abs_distance)
    servo_x_angle = float(servo_x_angle)
    servo_y_angle = abs(float(servo_y_angle))

    current_x_loc = current_loc[0]
    current_y_loc = current_loc[1]

    distance = abs_distance * math.cos(math.radians(servo_y_angle))

    abs_angle = (yaw_angle + servo_x_angle + 360) % 360

    current_x_loc += (DEG * distance * math.cos(math.radians((abs_angle + 360) % 360)))
    current_y_loc += (DEG * distance * math.sin(math.radians((abs_angle + 360) % 360)))

    return current_x_loc, current_y_loc

def check_data(tcp_data: str):
    if "|" not in tcp_data:
        print("| yok")
        return False

    if len(tcp_data.split("|")) != 3:
        print("3 degil")
        return False
    
    if not tcp_data.split("|")[0].isascii() or not tcp_data.split("|")[1].isascii() or not tcp_data.split("|")[2].isascii():
        print(tcp_data.split("|")[0].isascii() , tcp_data.split("|")[1].isascii() ,tcp_data.split("|")[2].isascii())
        return False
    
    return True
def constrain(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def map_calibrated_servo(
    input_cmd,
    home_offset_deg=-8,
    limit_deg=60,
    reverse=False
):
    """
    input_cmd: lệnh logic từ 0 -> 180
            90 là home logic

    home_offset_deg: physical offset của home
                    ví dụ home lệch -8 độ thì servo physical home = 90 + (-8) = 82

    limit_deg: giới hạn vật lý mỗi bên
            ví dụ 60 nghĩa là chỉ chạy từ -60 -> +60 quanh home

    reverse: đảo chiều servo nếu cần
    """

    # Giới hạn input logic
    input_cmd = constrain(input_cmd, 0, 180)

    # Tính home servo thật sau calibration
    home_cmd = 90 + home_offset_deg

    # Đổi input 0..180 thành ratio -1..1
    ratio = (input_cmd - 90) / 90

    # Map ratio ra góc servo thật
    calib_deg = input_cmd - 90
    # Cap error nếu vượt quá limit
    if abs(calib_deg) > limit_deg:
        calib_deg = limit_deg if calib_deg > 0 else -limit_deg
        
    servo_cmd = home_cmd + calib_deg

    # Giới hạn an toàn servo 0..180
    servo_cmd = constrain(servo_cmd, 0, 180)

    return servo_cmd
    
home_offset = -8
limit = 60
cmd = 85
servo = map_calibrated_servo(
        input_cmd=cmd,
        home_offset_deg=home_offset,
        limit_deg=limit
    )
print(f"Input: {cmd:3} -> Servo: {servo:.2f}")
    
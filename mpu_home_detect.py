from mpu6050 import mpu6050
import time

MPU_ADDR = 0x68
STRAIGHT_THRESHOLD_DEG = 5.0
LOOP_HZ = 100

sensor = mpu6050(MPU_ADDR)

yaw_deg = 0.0
home_yaw_deg = 0.0
gyro_z_bias = 0.0


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def calibrate_gyro(samples=500):
    print("Calibrating gyro... giữ IMU đứng yên")
    total_z = 0.0

    for _ in range(samples):
        gyro = sensor.get_gyro_data()
        total_z += gyro["z"]   # deg/s
        time.sleep(0.005)

    bias = total_z / samples
    print(f"gyro_z_bias = {bias:.4f} deg/s")
    return bias


def set_home():
    global home_yaw_deg
    home_yaw_deg = yaw_deg
    print("\n===== HOME SET =====")
    print("Hướng hiện tại = 0 độ")
    print("====================\n")


gyro_z_bias = calibrate_gyro()
last_time = time.time()
set_home()

while True:
    gyro = sensor.get_gyro_data()

    now = time.time()
    dt = now - last_time
    last_time = now

    # gyro['z'] đơn vị deg/s
    gz = gyro["z"] - gyro_z_bias

    # tích phân gyro Z để ra yaw tương đối
    yaw_deg += gz * dt
    yaw_deg = normalize_angle(yaw_deg)

    # lệch so với HOME
    error_deg = normalize_angle(yaw_deg - home_yaw_deg)

    if abs(error_deg) <= STRAIGHT_THRESHOLD_DEG:
        state = "STRAIGHT"
    elif error_deg > 0:
        state = " lệch RIGHT"
    else:
        state = " lệch LEFT"

    print(f"Yaw error: {error_deg:7.2f} deg | {state}")

    time.sleep(1.0 / LOOP_HZ)

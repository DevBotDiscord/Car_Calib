"""MPU6050 IMU heading-hold — gyro integration, home locking, P-controller."""

from __future__ import annotations

import time

from . import config


def _normalize_angle(angle: float) -> float:
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def setup_imu() -> None:
    if not config.IMU_ENABLED or config._mpu6050_class is None:
        print("IMU: disabled (IMU_ENABLED=false or mpu6050 not installed)")
        return

    try:
        config.imu = config._mpu6050_class(0x68)
        _ = config.imu.get_gyro_data()
    except Exception as exc:
        config.imu = None
        print(f"IMU: not found, falling back to MQTT-only steer ({exc})")
        return

    print("IMU: calibrating gyro (keep IMU still)...")
    total_z = 0.0
    for _ in range(config.IMU_GYRO_BIAS_SAMPLES):
        gyro = config.imu.get_gyro_data()
        total_z += gyro["z"]
        time.sleep(0.005)
    config.imu_gyro_z_bias = total_z / config.IMU_GYRO_BIAS_SAMPLES
    config.imu_last_time = time.monotonic()
    config.imu_active = True
    print(f"IMU: ready (gyro_z_bias={config.imu_gyro_z_bias:.4f} deg/s)")


def imu_reset_for_cruise() -> None:
    if not config.imu_active or config.imu is None:
        print("IMU: reset skipped (not active)")
        return

    print("IMU: quick recalibrate + yaw reset...")
    total_z = 0.0
    samples = 200
    for _ in range(samples):
        gyro = config.imu.get_gyro_data()
        total_z += gyro["z"]
        time.sleep(0.005)
    config.imu_gyro_z_bias = total_z / samples
    config.imu_yaw = 0.0
    config.imu_home_yaw = 0.0
    config.imu_home_steer_angle = config.steer_angle
    config.imu_last_time = time.monotonic()
    print(f"IMU: reset OK (bias={config.imu_gyro_z_bias:.4f} deg/s)")


def set_home() -> None:
    if not config.imu_active or config.imu is None:
        return
    config.imu_home_yaw = config.imu_yaw
    config.imu_home_steer_angle = config.steer_angle
    print(f"IMU: HOME SET (yaw={config.imu_yaw:.1f} deg, steer={config.imu_home_steer_angle:.1f} deg)")


def poll_imu() -> tuple[float, float]:
    """Return (yaw_deg, error_from_home_deg). (0,0) when IMU inactive."""
    if not config.imu_active or config.imu is None:
        return 0.0, 0.0

    try:
        gyro = config.imu.get_gyro_data()
    except Exception:
        config.imu_active = False
        print("IMU: read error, disabling heading-hold")
        return 0.0, 0.0

    now = time.monotonic()
    dt = now - config.imu_last_time
    config.imu_last_time = now

    gz = gyro["z"] - config.imu_gyro_z_bias
    config.imu_yaw += gz * dt
    config.imu_yaw = _normalize_angle(config.imu_yaw)

    error_deg = _normalize_angle(config.imu_yaw - config.imu_home_yaw)
    return config.imu_yaw, error_deg

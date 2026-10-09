"""手写运动共用的角度、坐标转换与速度限制计算。"""
import math


def apply_accel_limits(current_v, target_v, max_accel, max_decel, dt):
    accel = abs(max_accel)
    decel = abs(max_decel)

    if target_v * current_v < 0 and current_v != 0:
        max_dv = decel * dt
    elif abs(target_v) > abs(current_v):
        max_dv = accel * dt
    else:
        max_dv = decel * dt

    if target_v > current_v:
        return min(target_v, current_v + max_dv)
    else:
        return max(target_v, current_v - max_dv)


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def snap_yaw_to_nearest_90(yaw_rad):
    current_deg = math.degrees(yaw_rad)
    snapped_deg = round(current_deg / 90.0) * 90.0

    if snapped_deg > 180:
        snapped_deg -= 360

    if snapped_deg <= -180:
        snapped_deg += 360

    return math.radians(snapped_deg)


def global_velocity_to_body(vx_global, vy_global, current_yaw):
    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)

    vx_body = cos_yaw * vx_global + sin_yaw * vy_global
    vy_body = -sin_yaw * vx_global + cos_yaw * vy_global

    return vx_body, vy_body


def compute_cross_track_speed(node, cross_error):
    if abs(cross_error) <= node.ERROR_TOLERANCE_CROSS:
        return 0.0

    speed = -node.KP_CROSS_TRACK * cross_error
    speed = clamp(speed, -node.MAX_CROSS_TRACK_VEL, node.MAX_CROSS_TRACK_VEL)

    if 0.0 < abs(speed) < node.MIN_CROSS_TRACK_VEL:
        speed = math.copysign(node.MIN_CROSS_TRACK_VEL, speed)

    return speed


def compute_uphill_cross_track_speed(node, cross_error):
    """计算上坡的地图 Y 方向纠偏速度；cross_error 为当前 Y 减起始 Y。"""
    if abs(cross_error) <= node.UPHILL_ERROR_TOLERANCE_CROSS:
        return 0.0

    speed = -node.UPHILL_KP_CROSS_TRACK * cross_error
    speed = clamp(speed, -node.UPHILL_MAX_CROSS_TRACK_VEL, node.UPHILL_MAX_CROSS_TRACK_VEL)

    if 0.0 < abs(speed) < node.UPHILL_MIN_CROSS_TRACK_VEL:
        speed = math.copysign(node.UPHILL_MIN_CROSS_TRACK_VEL, speed)

    return speed

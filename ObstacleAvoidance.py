import math
import numpy as np
import time

wheelbase_m = 0.18 # 차량 휠베이스 (m) - 튜링 필요
danger_dist_cm = 60.0 # 장애물 위험 거리 (cm) - 튜링 필요
fov_rad = math.radians(90.0) # LiDAR 시야각 (라디안)
max_steer_deg = 20.0 # 최대 조향각 (도) - 튜링 필요
lookahead_m = 0.5 # 회피 예측 거리 (m) - 튜링 필요
vehicle_width_cm = 25.0 # 차량 폭 (cm) - 튜링 필요
min_clearance_cm = 80.0 # 최소 통로 폭 (cm) - 튜링 필요

# state 값에 넣어야 할 듯
recovery_mode = False # 탈출 모드 플래그
recovery_start = 0.0 # 탈출 시작 시간
recovery_phase = 0 # 탈출 단계: 0=후진, 1=회전 
recovery_duration_back = 1.5 # 후진 시간 (초) - 튜링 필요
recovery_duration_spin = 2.0 # 회전 시간 (초) - 튜링 필요
recovery_turn_dir = 1 # 회전 방향: +1=좌, -1=우

# -----------------------------
# 최대 안전 구간 폭
# -----------------------------
def _max_clear_angle_span(theta, d, threshold_cm):
    safe = d > threshold_cm
    if safe.size == 0:
        return 0.0
    angle_step = np.mean(np.diff(theta)) if theta.size >= 2 else 0.0
    max_span = 0
    cur_len = 0
    for s in safe:
        if s:
            cur_len += 1
        else:
            max_span = max(max_span, cur_len * angle_step)
            cur_len = 0
    max_span = max(max_span, cur_len * angle_step)
    return max_span

# -----------------------------
# 통로 판정
# -----------------------------
def is_path_available(theta, d):
    vehicle_width_m = vehicle_width_cm / 100.0
    required_span = 2.0 * math.atan2(vehicle_width_m / 2.0, max(1e-6, lookahead_m))
    max_clear_span = _max_clear_angle_span(theta, d, min_clearance_cm)
    return max_clear_span >= 0.9 * required_span

# -----------------------------
# 좌우 방향 폭 계산
# -----------------------------
def choose_escape_direction(theta, d):
    """
    후진 회전 방향 선택: 좌 vs 우, 더 넓은 쪽 선택
    """
    mid_idx = len(theta) // 2
    left_span = _max_clear_angle_span(theta[mid_idx:], d[mid_idx:], min_clearance_cm)
    right_span = _max_clear_angle_span(theta[:mid_idx], d[:mid_idx], min_clearance_cm)
    return 1 if left_span >= right_span else -1 

# -----------------------------
# LiDAR 처리 메인
# -----------------------------
def process_lidar(theta, d):
    mask = np.abs(theta) < fov_rad
    theta_f = theta[mask]
    d_f = d[mask]

    # -----------------------------
    # 탈출 모드 진행 중
    # -----------------------------
    if recovery_mode:
        now = time.time()
        elapsed = now - recovery_start

        # 후진 단계
        if recovery_phase == 0:
            rear_mask = (np.abs(theta) > math.radians(120))
            if np.any(d[rear_mask] < 30.0):
                recovery_phase = 1
                recovery_start = now
                return 0, 0

            if elapsed > recovery_duration_back:
                recovery_phase = 1
                recovery_start = now
            return 0, -1.0

        # 회전 단계
        elif recovery_phase == 1:
            steer_deg = recovery_turn_dir * max_steer_deg
            if elapsed > recovery_duration_spin:
                recovery_mode = False
                recovery_phase = 0
            return steer_deg, 0.5

    # -----------------------------
    # 정상 회피 루틴
    # -----------------------------
    if not is_path_available(theta_f, d_f):
        # 막힘 → 탈출 모드 진입
        recovery_mode = True
        recovery_start = time.time()
        recovery_phase = 0
        # 자동 좌우 선택
        recovery_turn_dir = choose_escape_direction(theta_f, d_f)
        print(f"[BLOCKED] 전방 막힘 → 후진 탈출 모드 진입. 회전 방향: {'좌' if recovery_turn_dir>0 else '우'}")
        return 0, 0

    # 위험거리 내 장애물
    danger_mask = d_f < danger_dist_cm
    if not np.any(danger_mask):
        return 0, 3.0

    th_d = theta_f[danger_mask]
    dist_d = d_f[danger_mask]

    ox = np.cos(th_d)
    oy = np.sin(th_d)
    weight = np.clip((danger_dist_cm - dist_d) / danger_dist_cm, 0, 1)
    rep_x = -ox * weight
    rep_y = -oy * weight
    avoid_vec = np.array([rep_x.sum(), rep_y.sum()])
    mag = np.linalg.norm(avoid_vec)

    if mag < 1e-6:
        return 0, 3.0

    avoid_angle_rad = float(np.clip(math.atan2(avoid_vec[1], avoid_vec[0]), -math.radians(60), math.radians(60)))
    if abs(avoid_angle_rad) < 1e-6:
        steer_rad = 0.0
        
    else:
        R = lookahead_m / avoid_angle_rad
        steer_rad = math.atan2(wheelbase_m, R)
        steer_rad = math.copysign(abs(steer_rad), avoid_angle_rad)
    steer_deg = float(np.clip(math.degrees(steer_rad), -max_steer_deg, max_steer_deg))
    mag_norm = min(mag, 1.0)
    speed_mps = max(0.5, 3.0 * (1 - mag_norm))
    return steer_deg, speed_mps

# -----------------------------
# 실시간 루프
# -----------------------------
def run(lidar_callback, interval=0.1):
    while True:
        theta, d = lidar_callback()
        info = process_lidar(theta, d)
        time.sleep(interval)    
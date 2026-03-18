from pathlib import Path
import sys
import math
from typing import Dict, Iterable, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
PARENT_DIR = SCRIPT_DIR.parent
if str(PARENT_DIR) not in sys.path:
    sys.path.insert(0, str(PARENT_DIR))

from MotorController import HiwonderMecanumController, FL, RL, FR, RR, MOTOR_ORDER

def normalize_motor_key_dict(d: dict) -> Dict[int, float]:
    return {int(k): d[k] for k in d}

def chassis_to_wheel_rev_s(
    controller: HiwonderMecanumController,
    v_forward_m_s: float,
    v_left_m_s: float,
    omega_rad_s: float,
) -> Dict[int, float]:
    r = controller.cal.wheel_diameter_m / 2.0
    k = controller.cal.k_geom_m

    fl_rad = (v_forward_m_s - v_left_m_s - k * omega_rad_s) / r
    fr_rad = (v_forward_m_s + v_left_m_s + k * omega_rad_s) / r
    rl_rad = (v_forward_m_s + v_left_m_s - k * omega_rad_s) / r
    rr_rad = (v_forward_m_s - v_left_m_s + k * omega_rad_s) / r

    wheel_rev_s = {
        FL: fl_rad / (2.0 * math.pi),
        FR: fr_rad / (2.0 * math.pi),
        RL: rl_rad / (2.0 * math.pi),
        RR: rr_rad / (2.0 * math.pi),
    }

    return {
        motor_id: wheel_rev_s[motor_id] * controller.cal.motor_direction_signs[motor_id]
        for motor_id in wheel_rev_s
    }

def joystick_to_chassis_and_wheels(
    controller: HiwonderMecanumController,
    left_y: float,
    left_x: float,
    right_x: float,
    max_rev_s: float,
) -> Tuple[float, float, float, Dict[int, float]]:
    v_scale = max_rev_s * controller.cal.wheel_circumference_m
    omega_scale = v_scale / max(controller.cal.k_geom_m, 1e-9)

    v_forward = -left_y * v_scale
    v_left = left_x * v_scale
    omega = right_x * omega_scale

    wheel_cmds = chassis_to_wheel_rev_s(controller, v_forward, v_left, omega)
    return v_forward, v_left, omega, wheel_cmds

def run_wheel_speeds(
    controller: HiwonderMecanumController,
    wheel_cmds: Dict[int, float],
    label: str = "Run wheel speeds",
) -> None:
    controller.run_motors({int(k): float(v) for k, v in wheel_cmds.items()}, label=label)

def counts_dict_from_states(states: Iterable) -> Dict[int, int]:
    return {int(s.motor_id): int(s.count) for s in states}

def tps_dict_from_states(states: Iterable) -> Dict[int, float]:
    return {int(s.motor_id): float(s.tps) for s in states}

def rps_dict_from_states(states: Iterable) -> Dict[int, float]:
    return {int(s.motor_id): float(s.rps) for s in states}

def wheel_counts_error(actual_counts: Dict[int, int], ref_counts: Dict[int, int]) -> Dict[int, int]:
    actual_counts = {int(k): int(v) for k, v in actual_counts.items()}
    ref_counts = {int(k): int(v) for k, v in ref_counts.items()}
    return {motor_id: actual_counts.get(motor_id, 0) - ref_counts.get(motor_id, 0) for motor_id in MOTOR_ORDER}

def wheel_speed_from_count_error(
    controller: HiwonderMecanumController,
    error_counts: int,
    motor_id: int,
    dt_s: float,
    kp: float = 0.15,
    max_correction_rev_s: float = 0.20,
) -> float:
    if dt_s <= 0:
        return 0.0
    target_count_rate = -kp * float(error_counts) / dt_s
    target_rev_s = target_count_rate / controller.cal.counts_per_rev[motor_id]
    if target_rev_s > max_correction_rev_s:
        target_rev_s = max_correction_rev_s
    if target_rev_s < -max_correction_rev_s:
        target_rev_s = -max_correction_rev_s
    return target_rev_s

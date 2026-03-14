from __future__ import annotations

import math
import struct
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import serial

# =========================
# configuration
# =========================
PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SER_TIMEOUT = 0.05

# Motor ID mapping assumption. Change to match your robot.
FL = 0  # front-left
RL = 1  # rear-left
FR = 2  # front-right
RR = 3  # rear-right
MOTOR_ORDER = [FL, RL, FR, RR]

# Sign correction from chassis-positive wheel motion to motor-positive command.
# Change signs if a wheel spins opposite of what the chassis model expects.
# Start with all +1, then calibrate using the included direction test menu.
MOTOR_DIRECTION_SIGNS: Dict[int, int] = {
    FL: +1,
    RL: +1,
    FR: +1,
    RR: +1,
}

# Wheel and robot geometry
WHEEL_DIAMETER_IN = 2.25
WHEEL_DIAMETER_M = WHEEL_DIAMETER_IN * 0.0254
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

# Distance from robot center to wheel contact geometry term for omega.
# Use meters. You need to measure these on your chassis.
# L = half of front-to-back wheel center spacing
# W = half of left-to-right wheel center spacing
HALF_LENGTH_M = 0.2921 # 12in
HALF_WIDTH_M = 0.3048 # 11.5 in
K_GEOM = HALF_LENGTH_M + HALF_WIDTH_M

# Critical calibration value. This must be wheel-output encoder counts per full wheel revolution.
# Your verification output implies this is NOT 3900; that value was for 2 s motion at 0.5 rps.
# You need to measure this. A common starting placeholder is 1320 or whatever the exact motor gives.
TICKS_PER_WHEEL_REV = 1320.0

# Motion defaults
DEFAULT_SPEED_RPS = 0.5
DEFAULT_TIMEOUT_S = 1.0
POLL_INTERVAL_S = 0.05
POSITION_TOLERANCE_M = 0.005
MAX_SAFE_RPS = 1.5

# Protocol constants
HEADER = bytes([0xAA, 0x55])
FUNC_MOTOR = 0x03

CMD_MOTOR_RUN_SINGLE = 0x00
CMD_MOTOR_RUN_MULTI = 0x01
CMD_MOTOR_STOP_SINGLE = 0x02
CMD_MOTOR_STOP_MASK = 0x03
CMD_ENCODER_READ_ONE = 0x10
CMD_ENCODER_READ_ALL = 0x11
CMD_ENCODER_RESET_ONE = 0x12
CMD_ENCODER_RESET_ALL = 0x13


@dataclass
class MotorState:
    motor_id: int
    count: int
    tps: float
    rps: float


@dataclass
class RobotPose:
    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0


class HiwonderMecanumController:
    def __init__(self, port: str = PORT, baud: int = BAUD, timeout: float = SER_TIMEOUT):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.pose = RobotPose()

    # -------------------------
    # Serial / protocol helpers
    # -------------------------
    @staticmethod
    def crc8_maxim(data: bytes) -> int:
        crc = 0x00
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x01:
                    crc = ((crc >> 1) ^ 0x8C) & 0xFF
                else:
                    crc = (crc >> 1) & 0xFF
        return crc

    def build_packet(self, function_code: int, payload: bytes) -> bytes:
        length = len(payload)
        body = bytes([function_code, length]) + payload
        checksum = self.crc8_maxim(body)
        return HEADER + body + bytes([checksum])

    @staticmethod
    def hexdump(data: bytes) -> str:
        return data.hex(" ")

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(0.2)

    def close(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def ensure_open(self) -> serial.Serial:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port is not open")
        return self.ser

    def send_packet(self, packet: bytes, label: str = "") -> None:
        ser = self.ensure_open()
        if label:
            print(f"\n{label}")
        print("TX:", self.hexdump(packet))
        ser.write(packet)
        ser.flush()

    def read_exact_packet(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> Optional[bytes]:
        ser = self.ensure_open()
        deadline = time.time() + timeout_s
        state = 0
        buf = bytearray()

        while time.time() < deadline:
            b = ser.read(1)
            if not b:
                continue

            val = b[0]
            if state == 0:
                if val == 0xAA:
                    buf.clear()
                    buf.append(val)
                    state = 1
                continue

            if state == 1:
                if val == 0x55:
                    buf.append(val)
                    state = 2
                else:
                    state = 0
                    buf.clear()
                continue

            if state == 2:
                buf.append(val)
                state = 3
                continue

            if state == 3:
                buf.append(val)
                payload_len = val
                remaining = payload_len + 1
                tail = ser.read(remaining)
                if len(tail) != remaining:
                    return None
                buf.extend(tail)
                return bytes(buf)

        return None

    def validate_packet(self, packet: bytes) -> Tuple[int, bytes]:
        if len(packet) < 5:
            raise ValueError("Packet too short")
        if packet[0:2] != HEADER:
            raise ValueError("Bad header")

        function_code = packet[2]
        length = packet[3]
        payload = packet[4:4 + length]
        rx_crc = packet[4 + length]
        calc_crc = self.crc8_maxim(packet[2:4 + length])
        if rx_crc != calc_crc:
            raise ValueError(f"Bad CRC: rx=0x{rx_crc:02X}, calc=0x{calc_crc:02X}")
        return function_code, payload

    def transact(self, packet: bytes, timeout_s: float = DEFAULT_TIMEOUT_S, label: str = "") -> Tuple[int, bytes, bytes]:
        ser = self.ensure_open()
        ser.reset_input_buffer()
        self.send_packet(packet, label)
        rx = self.read_exact_packet(timeout_s=timeout_s)
        if rx is None:
            raise TimeoutError("Timed out waiting for response packet")
        print("RX:", self.hexdump(rx))
        func, payload = self.validate_packet(rx)
        return func, payload, rx

    # -------------------------
    # Low-level packet builders
    # -------------------------
    def motor_run_single_packet(self, motor_id: int, speed_rps: float) -> bytes:
        payload = bytes([CMD_MOTOR_RUN_SINGLE, motor_id]) + struct.pack("<f", speed_rps)
        return self.build_packet(FUNC_MOTOR, payload)

    def motor_run_multi_packet(self, motor_speeds: List[Tuple[int, float]]) -> bytes:
        payload = bytes([CMD_MOTOR_RUN_MULTI, len(motor_speeds)])
        for motor_id, speed in motor_speeds:
            payload += bytes([motor_id]) + struct.pack("<f", speed)
        return self.build_packet(FUNC_MOTOR, payload)

    def motor_stop_single_packet(self, motor_id: int) -> bytes:
        payload = bytes([CMD_MOTOR_STOP_SINGLE, motor_id])
        return self.build_packet(FUNC_MOTOR, payload)

    def motor_stop_mask_packet(self, mask: int) -> bytes:
        payload = bytes([CMD_MOTOR_STOP_MASK, mask & 0xFF])
        return self.build_packet(FUNC_MOTOR, payload)

    def encoder_read_one_packet(self, motor_id: int) -> bytes:
        payload = bytes([CMD_ENCODER_READ_ONE, motor_id])
        return self.build_packet(FUNC_MOTOR, payload)

    def encoder_read_all_packet(self) -> bytes:
        payload = bytes([CMD_ENCODER_READ_ALL])
        return self.build_packet(FUNC_MOTOR, payload)

    def encoder_reset_one_packet(self, motor_id: int) -> bytes:
        payload = bytes([CMD_ENCODER_RESET_ONE, motor_id])
        return self.build_packet(FUNC_MOTOR, payload)

    def encoder_reset_all_packet(self) -> bytes:
        payload = bytes([CMD_ENCODER_RESET_ALL])
        return self.build_packet(FUNC_MOTOR, payload)

    # -------------------------
    # Response parsing
    # -------------------------
    def parse_single_motor_payload(self, payload: bytes) -> MotorState:
        expected_len = 18
        if len(payload) != expected_len:
            raise ValueError(f"Single motor response len mismatch: got {len(payload)}, expected {expected_len}")
        cmd = payload[0]
        if cmd not in (CMD_ENCODER_READ_ONE, CMD_ENCODER_RESET_ONE):
            raise ValueError(f"Unexpected single response cmd: 0x{cmd:02X}")
        motor_id = payload[1]
        count = struct.unpack_from("<q", payload, 2)[0]
        tps = struct.unpack_from("<f", payload, 10)[0]
        rps = struct.unpack_from("<f", payload, 14)[0]
        return MotorState(motor_id=motor_id, count=count, tps=tps, rps=rps)

    def parse_all_motor_payload(self, payload: bytes) -> List[MotorState]:
        if len(payload) < 2:
            raise ValueError("All motors response payload too short")
        cmd = payload[0]
        if cmd not in (CMD_ENCODER_READ_ALL, CMD_ENCODER_RESET_ALL):
            raise ValueError(f"Unexpected all response cmd: 0x{cmd:02X}")
        motor_num = payload[1]
        entry_size = 17
        expected_len = 2 + motor_num * entry_size
        if len(payload) != expected_len:
            raise ValueError(f"All motors response len mismatch: got {len(payload)}, expected {expected_len}")

        states: List[MotorState] = []
        offset = 2
        for _ in range(motor_num):
            motor_id = payload[offset]
            count = struct.unpack_from("<q", payload, offset + 1)[0]
            tps = struct.unpack_from("<f", payload, offset + 9)[0]
            rps = struct.unpack_from("<f", payload, offset + 13)[0]
            states.append(MotorState(motor_id=motor_id, count=count, tps=tps, rps=rps))
            offset += entry_size
        return states

    # -------------------------
    # Encoder helpers
    # -------------------------
    def read_encoder_one(self, motor_id: int) -> MotorState:
        func, payload, _ = self.transact(self.encoder_read_one_packet(motor_id), label=f"Read motor {motor_id}")
        if func != FUNC_MOTOR:
            raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
        return self.parse_single_motor_payload(payload)

    def read_encoder_all(self) -> Dict[int, MotorState]:
        func, payload, _ = self.transact(self.encoder_read_all_packet(), label="Read all encoders")
        if func != FUNC_MOTOR:
            raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
        states = self.parse_all_motor_payload(payload)
        return {s.motor_id: s for s in states}

    def reset_all_encoders(self) -> Dict[int, MotorState]:
        func, payload, _ = self.transact(self.encoder_reset_all_packet(), label="Reset all encoders")
        if func != FUNC_MOTOR:
            raise RuntimeError(f"Unexpected function code: 0x{func:02X}")
        states = self.parse_all_motor_payload(payload)
        self.pose = RobotPose()
        return {s.motor_id: s for s in states}

    # -------------------------
    # Motion primitives
    # -------------------------
    def stop_all(self) -> None:
        self.send_packet(self.motor_stop_mask_packet(0x0F), label="Stop all 4 motors")
        time.sleep(0.1)

    def run_wheels_rps(self, wheel_rps: Dict[int, float], label: str = "Run wheels") -> None:
        clamped: List[Tuple[int, float]] = []
        for motor_id in MOTOR_ORDER:
            speed = float(wheel_rps.get(motor_id, 0.0))
            speed = max(-MAX_SAFE_RPS, min(MAX_SAFE_RPS, speed))
            clamped.append((motor_id, speed))
        self.send_packet(self.motor_run_multi_packet(clamped), label=label)

    def chassis_to_wheel_rps(self, vx_mps: float, vy_mps: float, omega_radps: float) -> Dict[int, float]:
        r = WHEEL_DIAMETER_M / 2.0
        k = K_GEOM

        # Ideal mecanum inverse kinematics using chassis-positive wheel directions.
        w_fl = (vx_mps - vy_mps - k * omega_radps) / r
        w_fr = (vx_mps + vy_mps + k * omega_radps) / r
        w_rl = (vx_mps + vy_mps - k * omega_radps) / r
        w_rr = (vx_mps - vy_mps + k * omega_radps) / r

        wheel_rps_model = {
            FL: w_fl / (2.0 * math.pi),
            FR: w_fr / (2.0 * math.pi),
            RL: w_rl / (2.0 * math.pi),
            RR: w_rr / (2.0 * math.pi),
        }

        # Convert from model-positive to motor-positive using per-wheel sign correction.
        return {
            motor_id: MOTOR_DIRECTION_SIGNS[motor_id] * wheel_rps_model[motor_id]
            for motor_id in MOTOR_ORDER
        }

    def command_chassis(self, vx_mps: float, vy_mps: float, omega_radps: float, label: str = "Chassis command") -> None:
        wheel_rps = self.chassis_to_wheel_rps(vx_mps, vy_mps, omega_radps)
        self.run_wheels_rps(wheel_rps, label=label)

    def move_forward(self, speed_mps: float) -> None:
        self.command_chassis(vx_mps=+speed_mps, vy_mps=0.0, omega_radps=0.0, label=f"Move forward at {speed_mps:.3f} m/s")

    def move_reverse(self, speed_mps: float) -> None:
        self.command_chassis(vx_mps=-speed_mps, vy_mps=0.0, omega_radps=0.0, label=f"Move reverse at {speed_mps:.3f} m/s")

    def strafe_left(self, speed_mps: float) -> None:
        self.command_chassis(vx_mps=0.0, vy_mps=+speed_mps, omega_radps=0.0, label=f"Strafe left at {speed_mps:.3f} m/s")

    def strafe_right(self, speed_mps: float) -> None:
        self.command_chassis(vx_mps=0.0, vy_mps=-speed_mps, omega_radps=0.0, label=f"Strafe right at {speed_mps:.3f} m/s")

    def rotate_ccw(self, omega_radps: float) -> None:
        self.command_chassis(vx_mps=0.0, vy_mps=0.0, omega_radps=+omega_radps, label=f"Rotate CCW at {omega_radps:.3f} rad/s")

    def rotate_cw(self, omega_radps: float) -> None:
        self.command_chassis(vx_mps=0.0, vy_mps=0.0, omega_radps=-omega_radps, label=f"Rotate CW at {omega_radps:.3f} rad/s")

    def diagonal_front_left(self, speed_mps: float) -> None:
        self.command_chassis(vx_mps=+speed_mps / math.sqrt(2), vy_mps=+speed_mps / math.sqrt(2), omega_radps=0.0,
                             label=f"Diagonal front-left at {speed_mps:.3f} m/s")

    def diagonal_front_right(self, speed_mps: float) -> None:
        self.command_chassis(vx_mps=+speed_mps / math.sqrt(2), vy_mps=-speed_mps / math.sqrt(2), omega_radps=0.0,
                             label=f"Diagonal front-right at {speed_mps:.3f} m/s")

    def diagonal_rear_left(self, speed_mps: float) -> None:
        self.command_chassis(vx_mps=-speed_mps / math.sqrt(2), vy_mps=+speed_mps / math.sqrt(2), omega_radps=0.0,
                             label=f"Diagonal rear-left at {speed_mps:.3f} m/s")

    def diagonal_rear_right(self, speed_mps: float) -> None:
        self.command_chassis(vx_mps=-speed_mps / math.sqrt(2), vy_mps=-speed_mps / math.sqrt(2), omega_radps=0.0,
                             label=f"Diagonal rear-right at {speed_mps:.3f} m/s")

    # -------------------------
    # Odometry and distance control
    # -------------------------
    def counts_to_wheel_distance_m(self, delta_counts: int) -> float:
        return (float(delta_counts) / TICKS_PER_WHEEL_REV) * WHEEL_CIRCUMFERENCE_M

    def delta_counts_to_body_delta(self, prev_counts: Dict[int, int], new_counts: Dict[int, int]) -> Tuple[float, float, float]:
        # Convert measured motor-positive deltas back into chassis-positive wheel deltas.
        d_fl = self.counts_to_wheel_distance_m((new_counts[FL] - prev_counts[FL]) * MOTOR_DIRECTION_SIGNS[FL])
        d_fr = self.counts_to_wheel_distance_m((new_counts[FR] - prev_counts[FR]) * MOTOR_DIRECTION_SIGNS[FR])
        d_rl = self.counts_to_wheel_distance_m((new_counts[RL] - prev_counts[RL]) * MOTOR_DIRECTION_SIGNS[RL])
        d_rr = self.counts_to_wheel_distance_m((new_counts[RR] - prev_counts[RR]) * MOTOR_DIRECTION_SIGNS[RR])

        dx_body = (d_fl + d_fr + d_rl + d_rr) / 4.0
        dy_body = (-d_fl + d_fr + d_rl - d_rr) / 4.0
        dtheta = (-d_fl + d_fr - d_rl + d_rr) / (4.0 * K_GEOM)
        return dx_body, dy_body, dtheta

    def update_pose_from_counts(self, prev_counts: Dict[int, int], new_counts: Dict[int, int]) -> Tuple[float, float, float]:
        dx_body, dy_body, dtheta = self.delta_counts_to_body_delta(prev_counts, new_counts)
        theta_mid = self.pose.theta_rad + dtheta / 2.0

        dx_world = dx_body * math.cos(theta_mid) - dy_body * math.sin(theta_mid)
        dy_world = dx_body * math.sin(theta_mid) + dy_body * math.cos(theta_mid)

        self.pose.x_m += dx_world
        self.pose.y_m += dy_world
        self.pose.theta_rad += dtheta
        return dx_world, dy_world, dtheta

    def print_pose(self) -> None:
        print(f"Pose: x={self.pose.x_m:.4f} m, y={self.pose.y_m:.4f} m, theta={math.degrees(self.pose.theta_rad):.2f} deg")

    def move_by_displacement(self, dx_target_m: float, dy_target_m: float, speed_mps: float = 0.12) -> None:
        if abs(dx_target_m) < 1e-9 and abs(dy_target_m) < 1e-9:
            print("Requested displacement is zero; nothing to do.")
            return

        distance_target = math.hypot(dx_target_m, dy_target_m)
        unit_x = dx_target_m / distance_target
        unit_y = dy_target_m / distance_target

        self.reset_all_encoders()
        prev_states = self.read_encoder_all()
        prev_counts = {mid: s.count for mid, s in prev_states.items()}
        self.pose = RobotPose()

        vx = unit_x * speed_mps
        vy = unit_y * speed_mps
        self.command_chassis(vx_mps=vx, vy_mps=vy, omega_radps=0.0,
                             label=f"Move by displacement dx={dx_target_m:.3f} m, dy={dy_target_m:.3f} m")

        try:
            while True:
                time.sleep(POLL_INTERVAL_S)
                states = self.read_encoder_all()
                new_counts = {mid: s.count for mid, s in states.items()}
                self.update_pose_from_counts(prev_counts, new_counts)
                prev_counts = new_counts

                traveled = math.hypot(self.pose.x_m, self.pose.y_m)
                remaining = max(0.0, distance_target - traveled)
                self.print_pose()
                print(f"Target distance={distance_target:.4f} m, traveled={traveled:.4f} m, remaining={remaining:.4f} m")

                if remaining <= POSITION_TOLERANCE_M:
                    break
        finally:
            self.stop_all()

    def move_distance_forward(self, distance_m: float, speed_mps: float = 0.12) -> None:
        self.move_by_displacement(dx_target_m=distance_m, dy_target_m=0.0, speed_mps=speed_mps)

    def move_distance_reverse(self, distance_m: float, speed_mps: float = 0.12) -> None:
        self.move_by_displacement(dx_target_m=-distance_m, dy_target_m=0.0, speed_mps=speed_mps)

    def move_distance_left(self, distance_m: float, speed_mps: float = 0.12) -> None:
        self.move_by_displacement(dx_target_m=0.0, dy_target_m=distance_m, speed_mps=speed_mps)

    def move_distance_right(self, distance_m: float, speed_mps: float = 0.12) -> None:
        self.move_by_displacement(dx_target_m=0.0, dy_target_m=-distance_m, speed_mps=speed_mps)

    def move_distance_diagonal(self, dx_m: float, dy_m: float, speed_mps: float = 0.12) -> None:
        self.move_by_displacement(dx_target_m=dx_m, dy_target_m=dy_m, speed_mps=speed_mps)

    # -------------------------
    # Calibration helpers
    # -------------------------
    def direction_test(self, motor_id: int, speed_rps: float = 0.3, seconds: float = 1.0) -> None:
        self.send_packet(self.motor_run_single_packet(motor_id, speed_rps), label=f"Direction test motor {motor_id}")
        time.sleep(seconds)
        self.send_packet(self.motor_stop_single_packet(motor_id), label=f"Stop motor {motor_id}")
        time.sleep(0.25)

    def estimate_ticks_per_rev(self, motor_id: int, wheel_turns: float) -> None:
        if wheel_turns <= 0:
            print("wheel_turns must be > 0")
            return
        self.reset_all_encoders()
        input(f"Manually rotate wheel for motor {motor_id} exactly {wheel_turns} wheel turns, then press Enter... ")
        state = self.read_encoder_one(motor_id)
        estimate = abs(state.count) / wheel_turns
        print(f"Estimated ticks per wheel revolution for motor {motor_id}: {estimate:.3f}")
        print("Repeat several times and average the result.")


def print_help() -> None:
    print(
        """
Menu options
------------
1  - Read all encoders
2  - Reset all encoders
3  - Stop all motors
4  - Move forward continuously
5  - Move reverse continuously
6  - Strafe left continuously
7  - Strafe right continuously
8  - Rotate CCW continuously
9  - Rotate CW continuously
10 - Diagonal front-left continuously
11 - Diagonal front-right continuously
12 - Diagonal rear-left continuously
13 - Diagonal rear-right continuously
14 - Move forward a specified distance
15 - Move reverse a specified distance
16 - Move left a specified distance
17 - Move right a specified distance
18 - Move by arbitrary dx, dy displacement
19 - Single motor direction test
20 - Estimate ticks per wheel revolution
21 - Print current software pose
q  - Quit
"""
    )


def get_float(prompt: str, default: Optional[float] = None) -> float:
    while True:
        raw = input(prompt).strip()
        if raw == "" and default is not None:
            return default
        try:
            return float(raw)
        except ValueError:
            print("Please enter a valid number.")


def get_int(prompt: str, default: Optional[int] = None) -> int:
    while True:
        raw = input(prompt).strip()
        if raw == "" and default is not None:
            return default
        try:
            return int(raw)
        except ValueError:
            print("Please enter a valid integer.")


def print_encoder_states(states: Dict[int, MotorState]) -> None:
    print("\nEncoder states")
    print("--------------")
    for motor_id in MOTOR_ORDER:
        s = states[motor_id]
        print(f"motor {motor_id}: count={s.count:>8d}   tps={s.tps:>10.3f}   rps={s.rps:>10.3f}")


def main() -> int:
    ctrl = HiwonderMecanumController()
    try:
        ctrl.open()
        print(f"Opened {ctrl.port} at {ctrl.baud} bps")
        print_help()

        while True:
            choice = input("\nSelect option: ").strip().lower()

            if choice == "1":
                states = ctrl.read_encoder_all()
                print_encoder_states(states)
            elif choice == "2":
                states = ctrl.reset_all_encoders()
                print_encoder_states(states)
            elif choice == "3":
                ctrl.stop_all()
            elif choice == "4":
                speed = get_float("Forward speed (m/s) [0.12]: ", 0.12)
                ctrl.move_forward(speed)
            elif choice == "5":
                speed = get_float("Reverse speed (m/s) [0.12]: ", 0.12)
                ctrl.move_reverse(speed)
            elif choice == "6":
                speed = get_float("Left strafe speed (m/s) [0.12]: ", 0.12)
                ctrl.strafe_left(speed)
            elif choice == "7":
                speed = get_float("Right strafe speed (m/s) [0.12]: ", 0.12)
                ctrl.strafe_right(speed)
            elif choice == "8":
                omega = get_float("CCW angular speed (rad/s) [0.8]: ", 0.8)
                ctrl.rotate_ccw(omega)
            elif choice == "9":
                omega = get_float("CW angular speed (rad/s) [0.8]: ", 0.8)
                ctrl.rotate_cw(omega)
            elif choice == "10":
                speed = get_float("Diagonal front-left speed (m/s) [0.12]: ", 0.12)
                ctrl.diagonal_front_left(speed)
            elif choice == "11":
                speed = get_float("Diagonal front-right speed (m/s) [0.12]: ", 0.12)
                ctrl.diagonal_front_right(speed)
            elif choice == "12":
                speed = get_float("Diagonal rear-left speed (m/s) [0.12]: ", 0.12)
                ctrl.diagonal_rear_left(speed)
            elif choice == "13":
                speed = get_float("Diagonal rear-right speed (m/s) [0.12]: ", 0.12)
                ctrl.diagonal_rear_right(speed)
            elif choice == "14":
                distance = get_float("Forward distance (m): ")
                speed = get_float("Speed (m/s) [0.12]: ", 0.12)
                ctrl.move_distance_forward(distance, speed)
            elif choice == "15":
                distance = get_float("Reverse distance (m): ")
                speed = get_float("Speed (m/s) [0.12]: ", 0.12)
                ctrl.move_distance_reverse(distance, speed)
            elif choice == "16":
                distance = get_float("Left distance (m): ")
                speed = get_float("Speed (m/s) [0.12]: ", 0.12)
                ctrl.move_distance_left(distance, speed)
            elif choice == "17":
                distance = get_float("Right distance (m): ")
                speed = get_float("Speed (m/s) [0.12]: ", 0.12)
                ctrl.move_distance_right(distance, speed)
            elif choice == "18":
                dx = get_float("Target dx forward (m): ")
                dy = get_float("Target dy left (m): ")
                speed = get_float("Translation speed magnitude (m/s) [0.12]: ", 0.12)
                ctrl.move_distance_diagonal(dx, dy, speed)
            elif choice == "19":
                motor = get_int("Motor ID (0-3): ")
                speed_rps = get_float("Test speed in rps [0.3]: ", 0.3)
                seconds = get_float("Run time seconds [1.0]: ", 1.0)
                ctrl.direction_test(motor, speed_rps, seconds)
            elif choice == "20":
                motor = get_int("Motor ID (0-3): ")
                wheel_turns = get_float("Exact number of manual wheel turns: ")
                ctrl.estimate_ticks_per_rev(motor, wheel_turns)
            elif choice == "21":
                ctrl.print_pose()
            elif choice == "q":
                break
            else:
                print_help()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as exc:
        print(f"\nERROR: {exc}")
        return 2
    finally:
        try:
            ctrl.stop_all()
        except Exception:
            pass
        ctrl.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

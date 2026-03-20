from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import serial

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 1_000_000
SER_TIMEOUT = 0.05
DEFAULT_TIMEOUT_S = 1.0
POLL_INTERVAL_S = 0.05
POSITION_TOLERANCE_IN = 0.2
MAX_SAFE_RPS = 1.5

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

FL = 0
RL = 1
FR = 2
RR = 3
MOTOR_ORDER = [FL, RL, FR, RR]

DEFAULT_CALIBRATION = {
    "wheel": {
        "diameter_in": 2.25,
    },
    "robot_geometry": {
        "half_length_in": 6.0,
        "half_width_in": 6.0,
    },
    "motor_mapping": {
        "0": "front_left",
        "1": "rear_left",
        "2": "front_right",
        "3": "rear_right",
    },
    "motor_direction_signs": {
        "0": +1,
        "1": +1,
        "2": -1,
        "3": -1,
    },
    "counts_per_revolution": {
        "0": 1289.0,
        "1": 1317.0,
        "2": 1287.4,
        "3": 1270.5,
    },
    "straight_line": {
        "counts_per_meter_avg": 6962.57,
        "implied_counts_per_rev": 1250.074,
        "preferred_distance_model": "per_wheel_counts",
    },
}


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


@dataclass
class Calibration:
    wheel_diameter_in: float
    wheel_diameter_m: float
    wheel_circumference_m: float
    half_length_in: float
    half_width_in: float
    half_length_m: float
    half_width_m: float
    k_geom_m: float
    motor_direction_signs: Dict[int, int]
    counts_per_rev: Dict[int, float]
    counts_per_meter_avg: Optional[float] = None
    implied_counts_per_rev: Optional[float] = None
    preferred_distance_model: str = "per_wheel_counts"
    port: Optional[str] = None
    baud: Optional[int] = None

    @classmethod
    def load(cls, path: Optional[str]) -> "Calibration":
        data = DEFAULT_CALIBRATION
        if path:
            p = Path(path)
            data = json.loads(p.read_text(encoding="utf-8"))

        wd_in = float(data["wheel"]["diameter_in"])
        wd_m = wd_in * 0.0254
        half_length_in = float(data["robot_geometry"]["half_length_in"])
        half_width_in = float(data["robot_geometry"]["half_width_in"])
        half_length_m = half_length_in * 0.0254
        half_width_m = half_width_in * 0.0254

        direction_signs = {int(k): int(v) for k, v in data["motor_direction_signs"].items()}
        counts_per_rev = {int(k): float(v) for k, v in data["counts_per_revolution"].items()}

        straight_line = data.get("straight_line", {})
        serial_cfg = data.get("serial", {})

        return cls(
            wheel_diameter_in=wd_in,
            wheel_diameter_m=wd_m,
            wheel_circumference_m=math.pi * wd_m,
            half_length_in=half_length_in,
            half_width_in=half_width_in,
            half_length_m=half_length_m,
            half_width_m=half_width_m,
            k_geom_m=half_length_m + half_width_m,
            motor_direction_signs=direction_signs,
            counts_per_rev=counts_per_rev,
            counts_per_meter_avg=(
                float(straight_line["counts_per_meter_avg"])
                if "counts_per_meter_avg" in straight_line and straight_line["counts_per_meter_avg"] is not None
                else None
            ),
            implied_counts_per_rev=(
                float(straight_line["implied_counts_per_rev"])
                if "implied_counts_per_rev" in straight_line and straight_line["implied_counts_per_rev"] is not None
                else None
            ),
            preferred_distance_model=straight_line.get("preferred_distance_model", "per_wheel_counts"),
            port=serial_cfg.get("port"),
            baud=(
                int(serial_cfg["baud"])
                if "baud" in serial_cfg and serial_cfg["baud"] is not None
                else None
            ),
        )

    def meters_to_counts(self, motor_id: int, meters: float) -> float:
        if self.preferred_distance_model == "counts_per_meter_avg" and self.counts_per_meter_avg:
            return meters * self.counts_per_meter_avg
        wheel_revs = meters / self.wheel_circumference_m
        return wheel_revs * self.counts_per_rev[motor_id]

    def counts_to_meters(self, motor_id: int, counts: float) -> float:
        if self.preferred_distance_model == "counts_per_meter_avg" and self.counts_per_meter_avg:
            return counts / self.counts_per_meter_avg
        wheel_revs = counts / self.counts_per_rev[motor_id]
        return wheel_revs * self.wheel_circumference_m


class HiwonderMecanumController:
    """
    Can be imported and used by other scripts, or run directly via the CLI/menu.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baud: Optional[int] = None,
        timeout: float = SER_TIMEOUT,
        calibration_file: Optional[str] = None,
    ):
        self.cal = Calibration.load(calibration_file)
        self.port = port if port is not None else (self.cal.port if self.cal.port is not None else DEFAULT_PORT)
        self.baud = baud if baud is not None else (self.cal.baud if self.cal.baud is not None else DEFAULT_BAUD)
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
        body = bytes([function_code, len(payload)]) + payload
        return HEADER + body + bytes([self.crc8_maxim(body)])

    @staticmethod
    def hexdump(data: bytes) -> str:
        return data.hex(" ")

    def open(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(0.2)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def ensure_open(self) -> serial.Serial:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port is not open")
        return self.ser

    def send_packet(self, packet: bytes, label: str = "") -> None:
        ser = self.ensure_open()
        if label:
            pass
            # print(f"\n{label}")
        # print("TX:", self.hexdump(packet))
        ser.write(packet)
        ser.flush()

    def read_exact_packet(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> Optional[bytes]:
        """
        Scan the serial stream until a plausible full packet is found.
        More robust against stale bytes / stream misalignment.
        """
        ser = self.ensure_open()
        deadline = time.time() + timeout_s
        buf = bytearray()

        while time.time() < deadline:
            chunk = ser.read(1)
            if not chunk:
                continue

            buf += chunk

            # Keep buffer from growing forever
            if len(buf) > 512:
                buf = buf[-128:]

            # Look for header anywhere in buffer
            while len(buf) >= 4:
                hdr_idx = buf.find(HEADER)
                if hdr_idx < 0:
                    # keep only possible partial header tail
                    buf = buf[-1:]
                    break

                # discard bytes before header
                if hdr_idx > 0:
                    del buf[:hdr_idx]

                # need at least header + func + len
                if len(buf) < 4:
                    break

                function_code = buf[2]
                length = buf[3]

                # Basic sanity checks:
                # known function codes are small values, and payload length
                # should not be absurdly large for this protocol.
                if function_code > 0x20 or length > 128:
                    del buf[0]
                    continue

                total_len = 2 + 1 + 1 + length + 1
                if len(buf) < total_len:
                    # wait for rest of packet
                    more = ser.read(total_len - len(buf))
                    if more:
                        buf += more
                    if len(buf) < total_len:
                        break

                packet = bytes(buf[:total_len])
                del buf[:total_len]

                try:
                    self.validate_packet(packet)
                    return packet
                except Exception:
                    # Bad packet; continue scanning the stream
                    if len(buf) > 0:
                        continue
                    break

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

    def transact(
        self,
        packet: bytes,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        label: str = "",
        expected_func: Optional[int] = None,
    ) -> Tuple[int, bytes]:
        """
        Send one packet and wait for a valid response.
        If expected_func is provided, ignore other valid packets until timeout.
        """
        ser = self.ensure_open()
        ser.reset_input_buffer()
        self.send_packet(packet, label)

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rx = self.read_exact_packet(timeout_s=max(0.01, deadline - time.time()))
            if rx is None:
                break

            try:
                func, payload = self.validate_packet(rx)
            except Exception:
                continue

            # If caller wants a specific function, ignore other packets
            if expected_func is not None and func != expected_func:
                print(f"Ignoring unexpected packet function 0x{func:02X}")
                continue

            return func, payload

        raise TimeoutError("Timed out waiting for expected response packet")

    # -------------------------
    # Packet builders
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
        return self.build_packet(FUNC_MOTOR, bytes([CMD_MOTOR_STOP_SINGLE, motor_id]))

    def motor_stop_mask_packet(self, mask: int) -> bytes:
        return self.build_packet(FUNC_MOTOR, bytes([CMD_MOTOR_STOP_MASK, mask & 0xFF]))

    def encoder_read_one_packet(self, motor_id: int) -> bytes:
        return self.build_packet(FUNC_MOTOR, bytes([CMD_ENCODER_READ_ONE, motor_id]))

    def encoder_read_all_packet(self) -> bytes:
        return self.build_packet(FUNC_MOTOR, bytes([CMD_ENCODER_READ_ALL]))

    def encoder_reset_one_packet(self, motor_id: int) -> bytes:
        return self.build_packet(FUNC_MOTOR, bytes([CMD_ENCODER_RESET_ONE, motor_id]))

    def encoder_reset_all_packet(self) -> bytes:
        return self.build_packet(FUNC_MOTOR, bytes([CMD_ENCODER_RESET_ALL]))

    # -------------------------
    # Response parsing
    # -------------------------
    @staticmethod
    def parse_single_motor_payload(payload: bytes) -> MotorState:
        if len(payload) != 18:
            raise ValueError(f"Single response payload length mismatch: {len(payload)}")
        motor_id = payload[1]
        count = struct.unpack_from("<q", payload, 2)[0]
        tps = struct.unpack_from("<f", payload, 10)[0]
        rps = struct.unpack_from("<f", payload, 14)[0]
        return MotorState(motor_id, count, tps, rps)

    @staticmethod
    def parse_all_motor_payload(payload: bytes) -> List[MotorState]:
        if len(payload) < 2:
            raise ValueError("All response payload too short")
        motor_num = payload[1]
        offset = 2
        states = []
        entry_size = 17
        expected_len = 2 + motor_num * entry_size
        if len(payload) != expected_len:
            raise ValueError(f"All response payload mismatch: got {len(payload)}, expected {expected_len}")
        for _ in range(motor_num):
            motor_id = payload[offset]
            count = struct.unpack_from("<q", payload, offset + 1)[0]
            tps = struct.unpack_from("<f", payload, offset + 9)[0]
            rps = struct.unpack_from("<f", payload, offset + 13)[0]
            states.append(MotorState(motor_id, count, tps, rps))
            offset += entry_size
        return states

    # -------------------------
    # Basic motor / encoder API
    # -------------------------
    def run_motor(self, motor_id: int, speed_rps: float) -> None:
        speed_rps = max(-MAX_SAFE_RPS, min(MAX_SAFE_RPS, speed_rps))
        self.send_packet(self.motor_run_single_packet(motor_id, speed_rps), f"Run motor {motor_id} @ {speed_rps:.3f} rev/s")

    def run_motors(self, motor_speeds: Dict[int, float], label: str = "Run motors") -> None:
        normalized = []
        for motor_id, speed_rps in motor_speeds.items():
            speed_rps = max(-MAX_SAFE_RPS, min(MAX_SAFE_RPS, speed_rps))
            normalized.append((motor_id, speed_rps))
        self.send_packet(self.motor_run_multi_packet(normalized), label)

    def stop_motor(self, motor_id: int) -> None:
        self.send_packet(self.motor_stop_single_packet(motor_id), f"Stop motor {motor_id}")

    def stop_all(self) -> None:
        self.send_packet(self.motor_stop_mask_packet(0x0F), "Stop all 4 motors")
        time.sleep(0.15)

    def read_motor(self, motor_id: int) -> MotorState:
        func, payload = self.transact(
            self.encoder_read_one_packet(motor_id),
            label=f"Read motor {motor_id}",
            expected_func=FUNC_MOTOR,
        )
        return self.parse_single_motor_payload(payload)

    def read_all_motors(self) -> List[MotorState]:
        func, payload = self.transact(
            self.encoder_read_all_packet(),
            label="Read all encoders",
            expected_func=FUNC_MOTOR,
        )
        return self.parse_all_motor_payload(payload)

    def reset_motor(self, motor_id: int) -> MotorState:
        func, payload = self.transact(
            self.encoder_reset_one_packet(motor_id),
            label=f"Reset motor {motor_id}",
            expected_func=FUNC_MOTOR,
        )
        return self.parse_single_motor_payload(payload)


    def reset_all_encoders(self) -> List[MotorState]:
        func, payload = self.transact(
            self.encoder_reset_all_packet(),
            label="Reset all encoders",
            expected_func=FUNC_MOTOR,
        )
        return self.parse_all_motor_payload(payload)

    # -------------------------
    # Chassis motion helpers
    # -------------------------
    def _apply_direction_signs(self, wheel_speeds_rev_s: Dict[int, float]) -> Dict[int, float]:
        return {
            motor_id: wheel_speeds_rev_s[motor_id] * self.cal.motor_direction_signs[motor_id]
            for motor_id in wheel_speeds_rev_s
        }

    def _command_chassis(self, v_forward_m_s: float, v_left_m_s: float, omega_rad_s: float, label: str) -> None:
        """
        Chassis convention:
        - +v_forward_m_s means forward
        - +v_left_m_s means left strafe
        - +omega_rad_s means CCW
        """
        r = self.cal.wheel_diameter_m / 2.0
        k = self.cal.k_geom_m

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

        # Left side should drive in reverse when moving forward:
        # this is handled by motor_direction_signs, which should be negative on left wheels.
        motor_cmds = self._apply_direction_signs(wheel_rev_s)
        self.run_motors(motor_cmds, label)

    def move_forward(self, speed_rev_s: float = 0.5) -> None:
        v = speed_rev_s * self.cal.wheel_circumference_m
        self._command_chassis(v_forward_m_s=+v, v_left_m_s=0.0, omega_rad_s=0.0, label=f"Move forward @ {speed_rev_s:.3f} rev/s")

    def move_reverse(self, speed_rev_s: float = 0.5) -> None:
        v = speed_rev_s * self.cal.wheel_circumference_m
        self._command_chassis(v_forward_m_s=-v, v_left_m_s=0.0, omega_rad_s=0.0, label=f"Move reverse @ {speed_rev_s:.3f} rev/s")

    def strafe_left(self, speed_rev_s: float = 0.5) -> None:
        v = speed_rev_s * self.cal.wheel_circumference_m
        self._command_chassis(v_forward_m_s=0.0, v_left_m_s=+v, omega_rad_s=0.0, label=f"Strafe left @ {speed_rev_s:.3f} rev/s")

    def strafe_right(self, speed_rev_s: float = 0.5) -> None:
        v = speed_rev_s * self.cal.wheel_circumference_m
        self._command_chassis(v_forward_m_s=0.0, v_left_m_s=-v, omega_rad_s=0.0, label=f"Strafe right @ {speed_rev_s:.3f} rev/s")

    def rotate_ccw(self, speed_rev_s: float = 0.4) -> None:
        tangential = speed_rev_s * self.cal.wheel_circumference_m
        omega = tangential / max(self.cal.k_geom_m, 1e-9)
        self._command_chassis(v_forward_m_s=0.0, v_left_m_s=0.0, omega_rad_s=+omega, label=f"Rotate CCW @ {speed_rev_s:.3f} rev/s")

    def rotate_cw(self, speed_rev_s: float = 0.4) -> None:
        tangential = speed_rev_s * self.cal.wheel_circumference_m
        omega = tangential / max(self.cal.k_geom_m, 1e-9)
        self._command_chassis(v_forward_m_s=0.0, v_left_m_s=0.0, omega_rad_s=-omega, label=f"Rotate CW @ {speed_rev_s:.3f} rev/s")

    def diagonal_forward_left(self, speed_rev_s: float = 0.5) -> None:
        v = speed_rev_s * self.cal.wheel_circumference_m / math.sqrt(2.0)
        self._command_chassis(v_forward_m_s=+v, v_left_m_s=+v, omega_rad_s=0.0, label=f"Diagonal forward-left @ {speed_rev_s:.3f} rev/s")

    def diagonal_forward_right(self, speed_rev_s: float = 0.5) -> None:
        v = speed_rev_s * self.cal.wheel_circumference_m / math.sqrt(2.0)
        self._command_chassis(v_forward_m_s=+v, v_left_m_s=-v, omega_rad_s=0.0, label=f"Diagonal forward-right @ {speed_rev_s:.3f} rev/s")

    def diagonal_reverse_left(self, speed_rev_s: float = 0.5) -> None:
        v = speed_rev_s * self.cal.wheel_circumference_m / math.sqrt(2.0)
        self._command_chassis(v_forward_m_s=-v, v_left_m_s=+v, omega_rad_s=0.0, label=f"Diagonal reverse-left @ {speed_rev_s:.3f} rev/s")

    def diagonal_reverse_right(self, speed_rev_s: float = 0.5) -> None:
        v = speed_rev_s * self.cal.wheel_circumference_m / math.sqrt(2.0)
        self._command_chassis(v_forward_m_s=-v, v_left_m_s=-v, omega_rad_s=0.0, label=f"Diagonal reverse-right @ {speed_rev_s:.3f} rev/s")

    # -------------------------
    # Odometry / distance
    # -------------------------
    def counts_to_distance_m(self, motor_id: int, delta_counts: float) -> float:
        return self.cal.counts_to_meters(motor_id, abs(delta_counts))

    def estimate_robot_displacement(self, start_counts: Dict[int, int], end_counts: Dict[int, int]) -> Tuple[float, float, float]:
        """
        Returns dx_m, dy_m, dtheta_rad in robot body frame.
        +dx = forward
        +dy = left
        """
        wheel_m = {}
        for motor_id in MOTOR_ORDER:
            raw_delta = end_counts[motor_id] - start_counts[motor_id]
            wheel_signed = raw_delta / self.cal.motor_direction_signs[motor_id]
            wheel_m[motor_id] = self.cal.counts_to_meters(motor_id, wheel_signed)

        dfl = wheel_m[FL]
        dfr = wheel_m[FR]
        drl = wheel_m[RL]
        drr = wheel_m[RR]
        k = self.cal.k_geom_m

        dx = (dfl + dfr + drl + drr) / 4.0
        dy = (-dfl + dfr + drl - drr) / 4.0
        dtheta = (-dfl + dfr - drl + drr) / (4.0 * max(k, 1e-9))
        return dx, dy, dtheta

    def read_count_dict(self) -> Dict[int, int]:
        states = self.read_all_motors()
        return {s.motor_id: s.count for s in states}

    def drive_distance(self, inches: float, speed_rev_s: float = 0.4) -> Tuple[float, float, float]:
        target_m = inches * 0.0254
        self.reset_all_encoders()
        self.move_forward(speed_rev_s)
        return self._wait_for_displacement(target_dx_m=target_m, target_dy_m=0.0)

    def drive_reverse_distance(self, inches: float, speed_rev_s: float = 0.4) -> Tuple[float, float, float]:
        target_m = inches * 0.0254
        self.reset_all_encoders()
        self.move_reverse(speed_rev_s)
        return self._wait_for_displacement(target_dx_m=-target_m, target_dy_m=0.0)

    def strafe_distance_left(self, inches: float, speed_rev_s: float = 0.4) -> Tuple[float, float, float]:
        target_m = inches * 0.0254
        self.reset_all_encoders()
        self.strafe_left(speed_rev_s)
        return self._wait_for_displacement(target_dx_m=0.0, target_dy_m=+target_m)

    def strafe_distance_right(self, inches: float, speed_rev_s: float = 0.4) -> Tuple[float, float, float]:
        target_m = inches * 0.0254
        self.reset_all_encoders()
        self.strafe_right(speed_rev_s)
        return self._wait_for_displacement(target_dx_m=0.0, target_dy_m=-target_m)

    def drive_diagonal(self, forward_inches: float, left_inches: float, speed_rev_s: float = 0.4) -> Tuple[float, float, float]:
        target_dx_m = forward_inches * 0.0254
        target_dy_m = left_inches * 0.0254
        self.reset_all_encoders()

        mag = math.hypot(target_dx_m, target_dy_m)
        if mag < 1e-9:
            return 0.0, 0.0, 0.0

        base_v = speed_rev_s * self.cal.wheel_circumference_m
        scale = base_v / mag
        self._command_chassis(
            v_forward_m_s=target_dx_m * scale,
            v_left_m_s=target_dy_m * scale,
            omega_rad_s=0.0,
            label=f"Drive diagonal toward dx={forward_inches:.3f} in, dy={left_inches:.3f} in @ {speed_rev_s:.3f} rev/s",
        )
        return self._wait_for_displacement(target_dx_m=target_dx_m, target_dy_m=target_dy_m)

    def move_xy(self, forward_inches: float, left_inches: float, speed_rev_s: float = 0.4) -> Tuple[float, float, float]:
        return self.drive_diagonal(forward_inches, left_inches, speed_rev_s)

    def _wait_for_displacement(self, target_dx_m: float, target_dy_m: float) -> Tuple[float, float, float]:
        tol_m = POSITION_TOLERANCE_IN * 0.0254
        start = {0: 0, 1: 0, 2: 0, 3: 0}
        last_dx = 0.0
        last_dy = 0.0
        last_dtheta = 0.0
        try:
            while True:
                time.sleep(POLL_INTERVAL_S)
                now = self.read_count_dict()
                last_dx, last_dy, last_dtheta = self.estimate_robot_displacement(start, now)
                err = math.hypot(target_dx_m - last_dx, target_dy_m - last_dy)
                est_speed = 0.0
                states = self.read_all_motors()
                est_speed = sum(abs(self.cal.counts_to_meters(s.motor_id, s.tps)) for s in states) / max(len(states), 1)

                print(
                    f"Estimated displacement: dx={last_dx/0.0254:.2f} in, "
                    f"dy={last_dy/0.0254:.2f} in, theta={math.degrees(last_dtheta):.2f} deg, "
                    f"estimated movement speed={est_speed:.3f} m/s"
                )

                if err <= tol_m:
                    break
        finally:
            self.stop_all()
        return last_dx, last_dy, last_dtheta

    def test_motor_direction(self, motor_id: int, speed_rev_s: float = 0.2, run_time_s: float = 1.0) -> None:
        self.reset_motor(motor_id)
        self.run_motor(motor_id, speed_rev_s)
        time.sleep(run_time_s)
        self.stop_motor(motor_id)
        state = self.read_motor(motor_id)
        print(
            f"motor {motor_id}: count={state.count}, tps={state.tps:.3f}, rps={state.rps:.3f}. "
            "If this sign is opposite your intended chassis-positive direction, flip that motor's direction sign in calibration JSON."
        )

    def print_calibration(self) -> None:
        print("\nLoaded calibration")
        print("------------------")
        print(f"Wheel diameter: {self.cal.wheel_diameter_in:.3f} in")
        print(f"Wheel circumference: {self.cal.wheel_circumference_m:.6f} m")
        print(f"Half length: {self.cal.half_length_in:.3f} in ({self.cal.half_length_m:.6f} m)")
        print(f"Half width : {self.cal.half_width_in:.3f} in ({self.cal.half_width_m:.6f} m)")
        print(f"Counts/rev : {self.cal.counts_per_rev}")
        print(f"Direction signs: {self.cal.motor_direction_signs}")
        if self.cal.counts_per_meter_avg is not None:
            print(f"Counts/m avg: {self.cal.counts_per_meter_avg:.3f}")
        print(f"Preferred distance model: {self.cal.preferred_distance_model}")


def prompt_float(prompt: str, min_value: Optional[float] = None) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            val = float(raw)
        except ValueError:
            print("Enter a valid number.")
            continue
        if min_value is not None and val < min_value:
            print(f"Enter a value >= {min_value}.")
            continue
        return val


def prompt_int(prompt: str, valid: Optional[set[int]] = None) -> int:
    while True:
        raw = input(prompt).strip()
        try:
            val = int(raw)
        except ValueError:
            print("Enter a valid integer.")
            continue
        if valid is not None and val not in valid:
            print(f"Enter one of: {sorted(valid)}")
            continue
        return val


def run_interactive_menu(controller: HiwonderMecanumController) -> int:
    controller.print_calibration()
    while True:
        print("\nMenu")
        print("----")
        print(" 1) Read all encoders")
        print(" 2) Reset all encoders")
        print(" 3) Move forward continuously")
        print(" 4) Move reverse continuously")
        print(" 5) Strafe left continuously")
        print(" 6) Strafe right continuously")
        print(" 7) Rotate CW continuously")
        print(" 8) Rotate CCW continuously")
        print(" 9) Diagonal forward-left continuously")
        print("10) Diagonal forward-right continuously")
        print("11) Diagonal reverse-left continuously")
        print("12) Diagonal reverse-right continuously")
        print("13) Drive forward a distance (inches)")
        print("14) Drive reverse a distance (inches)")
        print("15) Strafe left a distance (inches)")
        print("16) Strafe right a distance (inches)")
        print("17) Move by forward/left displacement (inches)")
        print("18) Test one motor direction")
        print("19) Stop all")
        print("20) Show calibration")
        print("21) Exit")

        choice = prompt_int("Choose an option: ", valid=set(range(1, 22)))

        if choice == 1:
            states = controller.read_all_motors()
            for s in states:
                print(f"motor {s.motor_id}: count={s.count}, tps={s.tps:.3f}, rps={s.rps:.3f}")

        elif choice == 2:
            states = controller.reset_all_encoders()
            for s in states:
                print(f"motor {s.motor_id}: count={s.count}, tps={s.tps:.3f}, rps={s.rps:.3f}")

        elif choice in {3,4,5,6,7,8,9,10,11,12}:
            speed = prompt_float("Commanded motor speed in rev/s: ", min_value=0.0)
            if choice == 3:
                controller.move_forward(speed)
            elif choice == 4:
                controller.move_reverse(speed)
            elif choice == 5:
                controller.strafe_left(speed)
            elif choice == 6:
                controller.strafe_right(speed)
            elif choice == 7:
                controller.rotate_cw(speed)
            elif choice == 8:
                controller.rotate_ccw(speed)
            elif choice == 9:
                controller.diagonal_forward_left(speed)
            elif choice == 10:
                controller.diagonal_forward_right(speed)
            elif choice == 11:
                controller.diagonal_reverse_left(speed)
            elif choice == 12:
                controller.diagonal_reverse_right(speed)
            print("Motion started. Choose option 19 to stop all.")

        elif choice in {13,14,15,16,17}:
            speed = prompt_float("Commanded motor speed in rev/s: ", min_value=0.01)
            if choice == 13:
                inches = prompt_float("Forward distance in inches: ", min_value=0.0)
                dx, dy, dtheta = controller.drive_distance(inches, speed)
            elif choice == 14:
                inches = prompt_float("Reverse distance in inches: ", min_value=0.0)
                dx, dy, dtheta = controller.drive_reverse_distance(inches, speed)
            elif choice == 15:
                inches = prompt_float("Left strafe distance in inches: ", min_value=0.0)
                dx, dy, dtheta = controller.strafe_distance_left(inches, speed)
            elif choice == 16:
                inches = prompt_float("Right strafe distance in inches: ", min_value=0.0)
                dx, dy, dtheta = controller.strafe_distance_right(inches, speed)
            else:
                fwd = prompt_float("Forward displacement in inches (+forward, -reverse): ")
                left = prompt_float("Left displacement in inches (+left, -right): ")
                dx, dy, dtheta = controller.move_xy(fwd, left, speed)

            print(
                f"Final estimate: dx={dx/0.0254:.3f} in, dy={dy/0.0254:.3f} in, "
                f"dtheta={math.degrees(dtheta):.3f} deg"
            )

        elif choice == 18:
            motor_id = prompt_int("Motor ID (0-3): ", valid={0,1,2,3})
            speed = prompt_float("Commanded motor speed in rev/s: ", min_value=0.01)
            run_time = prompt_float("Run time in seconds: ", min_value=0.01)
            controller.test_motor_direction(motor_id, speed, run_time)

        elif choice == 19:
            controller.stop_all()

        elif choice == 20:
            controller.print_calibration()

        elif choice == 21:
            controller.stop_all()
            return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Hiwonder mecanum motor controller")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--calibration", default="robot_calibration.json", help="Calibration JSON file")
    parser.add_argument("--menu", action="store_true", help="Run interactive menu")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    controller = HiwonderMecanumController(
        port=args.port,
        baud=args.baud,
        calibration_file=args.calibration,
    )

    try:
        controller.open()
        if args.menu or len(sys.argv) == 1:
            return run_interactive_menu(controller)
        controller.print_calibration()
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130
    finally:
        try:
            controller.stop_all()
        except Exception:
            pass
        controller.close()


if __name__ == "__main__":
    sys.exit(main())

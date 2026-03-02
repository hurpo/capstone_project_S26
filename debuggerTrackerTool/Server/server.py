import socket
import struct
import threading
import time
import cv2
import json

import sys
sys.path.insert(1, '../../capstone_project_S26')
import math
from game import StateMachine
from smbus2 import SMBus, i2c_msg

from StateControllers import State, Command, StateController, ClientController



HOST = "0.0.0.0"
PORT = 12345

device_options = ["/dev/video0", "/dev/video1", "/dev/video2"]
DEVICE = device_options[0]  # USB cam node


TYPE_TEXT = b"T"
TYPE_FRAME = b"F"
TYPE_COMMAND = b"C"
TYPE_ROBOT_DATA = b"R"

state_controller = None

# GAME CONTROLS
def game_thread(cap, cap_lock, conn: socket.socket, send_lock: threading.Lock, stop_evt: threading.Event):
    global state_controller
    
    try:
        sm = StateMachine(state_controller, conn, send_lock, cap, cap_lock)
        sm.run()
    except Exception as e:
        print(f"Game Thread Error: {e}")
    finally:
        stop_evt.set()

# SERVER COMM CONTROLS

def send_packet(conn: socket.socket, send_lock: threading.Lock, mtype: bytes, payload: bytes):
    header = mtype + struct.pack("!I", len(payload))
    with send_lock:
        conn.sendall(header)
        conn.sendall(payload)

def recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed")
        buf += chunk
    return buf

def open_camera():
    print("Opening camera....")
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    print(f"cap: {cap}")
    if not cap.isOpened():
        print(f"Initial device {DEVICE} failed, trying other options...")
        for device in device_options:
            print(f"Trying {device}...")
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if cap.isOpened():
                print(f"Success with {device}!")
                break
            else:
                print(f"Failed with {device}...")
        # raise RuntimeError(f"Could not open {DEVICE}")
    else:
        print(f"Camera opened at {DEVICE}")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(30):
        cap.grab()
        time.sleep(0.01)

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        raise RuntimeError("Camera opened but no frames received.")
    return cap

def camera_send_loop(cap, cap_lock, conn: socket.socket,
                     send_lock: threading.Lock, stop_evt: threading.Event):

    jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), 75]

    try:
        while not stop_evt.is_set():
            with cap_lock:
                ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            ok, jpg = cv2.imencode(".jpg", frame, jpeg_params)
            if not ok:
                continue

            try:
                send_packet(conn, send_lock, TYPE_FRAME, jpg.tobytes())
            except (BrokenPipeError, ConnectionResetError, OSError):
                break
    finally:
        pass
    
# Motor encoder streaming
I2C_BUS = 1
M5_ADDR = 0x24  # default per module docs

REG_ENC_BASE   = 0x30  # 4 bytes each: M1..M4
REG_SPEED_BASE = 0x40  # 1 byte each: M1..M4

class M5Encoders:
    def __init__(self, bus=I2C_BUS, addr=M5_ADDR):
        self.addr = addr
        self.bus = SMBus(bus)

    def close(self):
        self.bus.close()

    def _read_n(self, reg, n):
        w = i2c_msg.write(self.addr, [reg])
        r = i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(w, r)
        return bytes(list(r))

    def read_encoder(self, motor_index: int) -> int:
        reg = REG_ENC_BASE + 4 * (motor_index - 1)
        raw = self._read_n(reg, 4)
        #BIG-ENDIAN
        return struct.unpack(">i", raw)[0]

    def read_speed(self, motor_index: int) -> int:
        reg = REG_SPEED_BASE + (motor_index - 1)
        raw = self._read_n(reg, 1)
        return struct.unpack(">b", raw)[0]

def encoder_send_loop(conn: socket.socket,
                               send_lock: threading.Lock,
                               stop_evt: threading.Event,
                               hz: float = 20.0):
    """
    Reads encoder counts, converts to distance using wheel diameter, and sends JSON dict as TYPE_ROBOT_DATA.
    """

    # Counts per ONE WHEEL REVOLUTION. Measure it by turning the wheel 1 rev by hand after reset.
    COUNTS_PER_REV_OUT = 315 

    # Wheel geometry
    WHEEL_DIAM_IN = 2.25
    WHEEL_CIRC_IN = math.pi * WHEEL_DIAM_IN  # inches per wheel revolution

    IN_TO_FT = 1.0 / 12.0
    IN_TO_M  = 0.0254

    if COUNTS_PER_REV_OUT <= 0:
        raise ValueError("COUNTS_PER_REV_OUT must be set to a positive integer.")

    inches_per_count = WHEEL_CIRC_IN / COUNTS_PER_REV_OUT

    enc = M5Encoders(I2C_BUS, M5_ADDR)

    # tracking
    last_counts = [0, 0, 0, 0]
    total_in = [0.0, 0.0, 0.0, 0.0]
    last_t = time.time()

    period = 1.0 / max(1.0, hz)

    try:
        # initialize counts
        last_counts = [enc.read_encoder(i) for i in (1, 2, 3, 4)]
        last_t = time.time()

        while not stop_evt.is_set():
            t0 = time.time()

            counts = [enc.read_encoder(i) for i in (1, 2, 3, 4)]

            now = time.time()
            dt = max(1e-6, now - last_t)

            deltas = [counts[i] - last_counts[i] for i in range(4)]
            delta_in = [deltas[i] * inches_per_count for i in range(4)]
            for i in range(4):
                total_in[i] += delta_in[i]

            # estimate instantaneous linear speed (in/s) per motor
            speed_in_s = [delta_in[i] / dt for i in range(4)]

            # flat dict 
            robot_data = {
                # "enc_m1": counts[0], "enc_m2": counts[1], "enc_m3": counts[2], "enc_m4": counts[3],
                # "denc_m1": deltas[0], "denc_m2": deltas[1], "denc_m3": deltas[2], "denc_m4": deltas[3],

                "dist_in_m1": round(total_in[0], 3),
                "dist_in_m2": round(total_in[1], 3),
                "dist_in_m3": round(total_in[2], 3),
                "dist_in_m4": round(total_in[3], 3),

                # "dist_ft_m1": round(total_in[0] * IN_TO_FT, 3),
                # "dist_ft_m2": round(total_in[1] * IN_TO_FT, 3),
                # "dist_ft_m3": round(total_in[2] * IN_TO_FT, 3),
                # "dist_ft_m4": round(total_in[3] * IN_TO_FT, 3),

                # "dist_m_m1": round(total_in[0] * IN_TO_M, 4),
                # "dist_m_m2": round(total_in[1] * IN_TO_M, 4),
                # "dist_m_m3": round(total_in[2] * IN_TO_M, 4),
                # "dist_m_m4": round(total_in[3] * IN_TO_M, 4),

                # "speed_in_s_m1": round(speed_in_s[0], 3),
                # "speed_in_s_m2": round(speed_in_s[1], 3),
                # "speed_in_s_m3": round(speed_in_s[2], 3),
                # "speed_in_s_m4": round(speed_in_s[3], 3),

                # "enc_dt_s": round(dt, 4),
                # "wheel_diam_in": WHEEL_DIAM_IN,
                # "counts_per_rev_out": COUNTS_PER_REV_OUT,
                # "ts": round(now, 3),
            }

            payload = json.dumps(robot_data).encode("utf-8")

            try:
                send_packet(conn, send_lock, TYPE_ROBOT_DATA, payload)
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

            last_counts = counts
            last_t = now

            # rate control
            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))

    finally:
        enc.close()

def recv_loop(conn: socket.socket, send_lock: threading.Lock, 
              stop_evt: threading.Event):
    global state_controller
    
    # Receives packets from client (mostly text commands/chat)
    try:
        while not stop_evt.is_set():
            mtype = recv_exact(conn, 1)
            (plen,) = struct.unpack("!I", recv_exact(conn, 4))
            payload = recv_exact(conn, plen)

            if mtype == TYPE_TEXT:
                msg = payload.decode("utf-8", errors="replace")
                print(f"Client> {msg}")


                # echo back / ack
                reply = f"ACK: {msg}".encode("utf-8")
                send_packet(conn, send_lock, TYPE_TEXT, reply)
            
            elif mtype == TYPE_COMMAND:
                try:
                    cmd_data = json.loads(payload.decode("utf-8"))
                    command = cmd_data.get("command")
                    data = cmd_data.get("data")

                    print(f"COMMAND Received: {command} (data: {data})")

                    if command == "GOTO_STATE" and data:
                        try:
                            data = State[data]
                        except KeyError:
                            raise ValueError(f"Invalid State Name: {data}")

                    response = {"status": "ok", "message": f"Command {command} processed!"}

                    cmd_enum = Command(command)
                    state_controller.handle_command(cmd_enum, data)

                    response_payload =  json.dumps(response).encode("utf-8")
                    send_packet(conn, send_lock, TYPE_COMMAND, response_payload)

                except json.JSONDecodeError as e:
                    error_response = json.dumps({
                        "status": "error",
                        "message": f"Invalid JSON: {e}"
                    }).encode("utf-8")
                    send_packet(conn, send_lock, TYPE_COMMAND, error_response)

                except (ValueError, KeyError) as e:
                    error_response = json.dumps({
                        "status": "error",
                        "message": str(e)
                    }).encode("utf-8")
                    send_packet(conn, send_lock, TYPE_COMMAND, error_response)

                except Exception as e:
                    print(f"Command error: {e}")
                    error_response = json.dumps({
                        "status": "error",
                        "message": str(e)
                    }).encode("utf-8")
                    send_packet(conn, send_lock, TYPE_COMMAND, error_response)

    except (ConnectionError, OSError):
        pass
    finally:
        stop_evt.set()

def main():
    global state_controller

    with socket.socket(socket.AF_INET, 
                       socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, 
                     socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"Server listening on {HOST}:{PORT}")

        while True:
            conn, addr = s.accept()
            print(f"Connected by {addr}")

            state_controller = ClientController()
            cap = open_camera()

            with conn:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

                send_lock = threading.Lock()
                stop_evt = threading.Event()
                cap_lock = threading.Lock()

                print("t")
                t_recv = threading.Thread(target=recv_loop, 
                                          args=(conn, send_lock, stop_evt), 
                                          daemon=True)
                t_cam  = threading.Thread(target=camera_send_loop, 
                                          args=(cap, cap_lock, conn, send_lock, stop_evt), 
                                          daemon=True)
                t_game = threading.Thread(target=game_thread, 
                                          args=(cap, cap_lock, conn, send_lock, stop_evt), 
                                          daemon=True)
                t_enc  = threading.Thread(target=encoder_send_loop,
                                          args=(conn, send_lock, stop_evt), 
                                          kwargs={"hz": 20.0}, daemon=True)
               
                t_enc.start()
                t_recv.start()
                t_cam.start()
                t_game.start()

                # Encoder calibration (idk if this is needed tbh)
                COUNTS_PER_REV_OUT = 315

                # Keep connection alive until something stops
                while not stop_evt.is_set():
                    time.sleep(0.1)

                cap.release()
                print("[Disconnected]\n")

if __name__ == "__main__":
    main()

import socket
import threading
import queue
import struct
import json

HOST = "10.227.90.96"
PORT = 12345

TYPE_TEXT = b"T"
TYPE_FRAME = b"F"
TYPE_COMMAND = b"C"
TYPE_POSITION = b"P"
TYPE_ROBOT_DATA = b"R"

_socket = None
_send_lock = threading.Lock()
_recv_queue = queue.Queue()
_latest_frame = None
_frame_lock = threading.Lock()
_latest_position = None
_position_lock = threading.Lock()
_latest_robot_data = None
_robot_data_lock = threading.Lock()

def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed")
        buf += chunk
    return buf

def recv_loop(sock: socket.socket):
    global _latest_frame, _latest_position, _latest_robot_data
    try:
        while True:
            mtype = recv_exact(sock, 1)
            (plen,) = struct.unpack("!I", recv_exact(sock, 4))
            payload = recv_exact(sock, plen)

            if mtype == TYPE_TEXT:
                msg = payload.decode("utf-8", errors="replace")
                _recv_queue.put(f"Server> {msg}")

            elif mtype == TYPE_FRAME:
                # Store latest JPEG frame (bytes). GUI can decode and display.
                with _frame_lock:
                    _latest_frame = payload

            elif mtype == TYPE_POSITION:
                position_data = json.loads(payload.decode("utf-8"))
                with _position_lock:
                    _latest_position = position_data
            
            elif mtype == TYPE_ROBOT_DATA:
                print("New Robot Data Received!")
                robot_data = json.loads(payload.decode("utf-8"))
                with _robot_data_lock:
                    _latest_robot_data = robot_data
    except Exception:
        _recv_queue.put("\n[Server disconnected]")

def connect(ip, port=PORT):
    global _socket
    try:
        _socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        _socket.connect((ip, port))
        _socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        _socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        t = threading.Thread(target=recv_loop, args=(_socket,), daemon=True)
        t.start()

        return "Connection Successful"
    except Exception as e:
        return f"Error: {e}"

def disconnect():
    global _socket
    try:
        if _socket is None:
            return
        _socket.shutdown(socket.SHUT_RDWR)
        _socket.close()
        _socket = None
        _recv_queue.put("[Disconnected]")
    except Exception as e:
        return f"Error: {e}"

def send_packet(mtype: bytes, payload: bytes):
    if _socket is None:
        return "Error: Not connected"
    header = mtype + struct.pack("!I", len(payload))
    with _send_lock:
        _socket.sendall(header)
        _socket.sendall(payload)

def send_command(command, data=None):
    cmd_dict = {
        "command": command,
        "data": data
    }
    payload = json.dumps(cmd_dict).encode("utf-8")
    try:
        send_packet(TYPE_COMMAND, payload)
        _recv_queue.put(f"COMMAND> {command} (data: {data})")
    except Exception as e:
        return e

def send_message(msg: str):
    try:
        send_packet(TYPE_TEXT, msg.encode("utf-8"))
        _recv_queue.put(f"Client> {msg}")
    except Exception as e:
        return e

def get_message():
    messages = []
    while not _recv_queue.empty():
        messages.append(_recv_queue.get_nowait())
    return messages

def get_latest_frame():
    # Returns JPEG bytes or None
    with _frame_lock:
        return _latest_frame

def get_latest_position():
    with _position_lock:
        return _latest_position

def get_latest_robot_data():
    with _robot_data_lock:
        return _latest_robot_data
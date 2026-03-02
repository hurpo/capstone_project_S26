from __future__ import print_function
from dual_g2_hpmd_rpi import motors, MAX_SPEED
import serial
import time

UPLOAD_DATA = 3 
MOTOR_TYPE = 1

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.01  
)

recv_buffer = ""

def send_data(data):
    ser.write(data.encode())
    time.sleep(0.01)

def receive_data():
    global recv_buffer
    if ser.in_waiting > 0:
        recv_buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        messages = recv_buffer.split("#")
        recv_buffer = messages[-1]
        if len(messages) > 1:
            return messages[0] + "#"
    return None

def parse_data(data):
    data = data.strip()
    if data.startswith("$MSPD:"):
        values_str = data[6:-1]
        values = [float(value) if '.' in value else int(value) for value in values_str.split(',')]
        parsed = ', '.join([f"M{i+1}:{value}" for i, value in enumerate(values)])
        return parsed
    elif data.startswith("$MAll:") or data.startswith("$MTEP:"):
        values_str = data[6:-1]
        values = list(map(int, values_str.split(',')))
        parsed = ', '.join([f"M{i+1}:{value}" for i, value in enumerate(values)])
        return parsed
    return None

def send_upload_command(mode):
    if mode == 0:
        send_data("$upload:0,0,0#")
    elif mode == 1:
        send_data("$upload:1,0,0#")
    elif mode == 2:
        send_data("$upload:0,1,0#")
    elif mode == 3:
        send_data("$upload:0,0,1#")

def drive_forward_with_encoder(speed, duration):
    motors.motor1.setSpeed(speed)
    motors.motor2.setSpeed(speed)
    start_time = time.time()
    while time.time() - start_time < duration:
        received_message = receive_data()
        if received_message:
            parsed = parse_data(received_message)
            if parsed:
                print(parsed)
        time.sleep(0.05)  # Small delay to prevent overwhelming the CPU
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)

def turn_left_with_encoder(speed, duration):
    motors.motor1.setSpeed(-speed)
    motors.motor2.setSpeed(speed)
    start_time = time.time()
    while time.time() - start_time < duration:
        received_message = receive_data()
        if received_message:
            parsed = parse_data(received_message)
            if parsed:
                print(parsed)
        time.sleep(0.05)
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)

def turn_right_with_encoder(speed, duration):
    motors.motor1.setSpeed(speed)
    motors.motor2.setSpeed(-speed)
    start_time = time.time()
    while time.time() - start_time < duration:
        received_message = receive_data()
        if received_message:
            parsed = parse_data(received_message)
            if parsed:
                print(parsed)
        time.sleep(0.05)
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)

def stop_all():
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)

if __name__ == "__main__":
    try:
        print("Initializing encoder reader...")
        send_upload_command(UPLOAD_DATA)
        time.sleep(0.5)
        
        print("Turning left...")
        turn_left_with_encoder(100, 3)
        
        print("Driving forward...")
        drive_forward_with_encoder(50, 0.5)
        
        print("Turning right...")
        turn_right_with_encoder(100, 3)
        
        print("Driving forward...")
        drive_forward_with_encoder(-50, 0.5)
        
        print("Turning left...")
        turn_left_with_encoder(100, 1.5)
        
        print("Driving forward...")
        drive_forward_with_encoder(50, 0.5)
        
        print("Turning left...")
        turn_left_with_encoder(100, 1)
        
        print("Stopping...")
        stop_all()

    except KeyboardInterrupt:
        print("\nInterrupted!")
        stop_all()
    finally:
        ser.close()


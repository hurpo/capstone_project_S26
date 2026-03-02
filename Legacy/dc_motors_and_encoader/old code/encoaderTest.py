from dual_g2_hpmd_rpi import motors, MAX_SPEED
from __future__ import print_function
import serial
import time

UPLOAD_DATA = 3  # 0: Do not receive data 1: Receive total encoder data 2: Receive real-time encoder 3: Receive current motor speed mm/s

MOTOR_TYPE = 1  # 1: 520 motor 2: 310 motor 3: speed code disc TT motor 4: TT DC reduction motor 5: L type 520 motor

# Serial port initialization
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Modify it to your serial port device path according to the actual situation
    baudrate=115200,      # Baud rate, must be consistent with the driver board
    parity=serial.PARITY_NONE,  # No check digit
    stopbits=serial.STOPBITS_ONE,  # One stop bit
    bytesize=serial.EIGHTBITS,    # Data bit 8 bits
    timeout=1                     # Timeout (seconds)
)

# Receive Buffer
recv_buffer = ""

# Sending Data
def send_data(data):
    ser.write(data.encode())  # Convert the string to bytes before sending
    time.sleep(0.01)  # Delay to ensure data transmission is completed

# Receiving Data
def receive_data():
    global recv_buffer
    if ser.in_waiting > 0:  # Check if there is data in the serial port buffer
        recv_buffer += ser.read(ser.in_waiting).decode()
        
        # Split the message by the ending character "#"
        messages = recv_buffer.split("#")
        recv_buffer = messages[-1]
        
        if len(messages) > 1:
            return messages[0] + "#"
    return None

# Configure motor type
def set_motor_type(data):
    TYPE = data
    send_data("$mtype:{}#".format(TYPE))

# Configuring Dead Zone
def set_motor_deadzone(data):
    DZ = data
    send_data("$deadzone:{}#".format(DZ))

# Configuring magnetic loop
def set_pluse_line(data):
    LINE = data
    send_data("$mline:{}#".format(LINE))

# Configure the reduction ratio
def set_pluse_phase(data):
    PHASE = data
    send_data("$mphase:{}#".format(PHASE))

# Configuration Diameter
def set_wheel_dis(data):
    WHEEL = data
    send_data("$wdiameter:{}#".format(WHEEL))

# Controlling Speed
def control_speed(m1, m2, m3, m4):
    send_data("$spd:{},{},{},{}#".format(m1, m2, m3, m4))

# Control PWM (for motors without encoder)
def control_pwm(m1, m2, m3, m4):
    send_data("$pwm:{},{},{},{}#".format(m1, m2, m3, m4))

# Parsing received data
def parse_data(data):
    data = data.strip()

    if data.startswith("$MAll:"):
        values_str = data[6:-1]
        values = list(map(int, values_str.split(',')))
        parsed = ', '.join([f"M{i+1}:{value}" for i, value in enumerate(values)])
        return parsed
    elif data.startswith("$MTEP:"):
        values_str = data[6:-1]
        values = list(map(int, values_str.split(',')))
        parsed = ', '.join([f"M{i+1}:{value}" for i, value in enumerate(values)])
        return parsed
    elif data.startswith("$MSPD:"):
        values_str = data[6:-1]
        values = [float(value) if '.' in value else int(value) for value in values_str.split(',')]
        parsed = ', '.join([f"M{i+1}:{value}" for i, value in enumerate(values)])
        return parsed

# Switch that needs to receive data
def send_upload_command(mode):
    if mode == 0:
        send_data("$upload:0,0,0#")
    elif mode == 1:
        send_data("$upload:1,0,0#")
    elif mode == 2:
        send_data("$upload:0,1,0#")
    elif mode == 3:
        send_data("$upload:0,0,1#")

# The following parameters can be configured according to the actual motor you use. You only need to configure it once. The motor driver board has a power-off saving function.
def set_motor_parameter():

    if MOTOR_TYPE == 1:
        set_motor_type(1)
        time.sleep(0.1)
        set_pluse_phase(30)
        time.sleep(0.1)
        set_pluse_line(11)
        time.sleep(0.1)
        set_wheel_dis(67.00)
        time.sleep(0.1)
        set_motor_deadzone(1600)
        time.sleep(0.1)

    elif MOTOR_TYPE == 2:
        set_motor_type(2)
        time.sleep(0.1)
        set_pluse_phase(20)
        time.sleep(0.1)
        set_pluse_line(13)
        time.sleep(0.1)
        set_wheel_dis(48.00)
        time.sleep(0.1)
        set_motor_deadzone(1300)
        time.sleep(0.1)

    elif MOTOR_TYPE == 3:
        set_motor_type(3)
        time.sleep(0.1)
        set_pluse_phase(45)
        time.sleep(0.1)
        set_pluse_line(13)
        time.sleep(0.1)
        set_wheel_dis(68.00)
        time.sleep(0.1)
        set_motor_deadzone(1250)
        time.sleep(0.1)

    elif MOTOR_TYPE == 4:
        set_motor_type(4)
        time.sleep(0.1)
        set_pluse_phase(48)
        time.sleep(0.1)
        set_motor_deadzone(1000)
        time.sleep(0.1)

    elif MOTOR_TYPE == 5:
        set_motor_type(1)
        time.sleep(0.1)
        set_pluse_phase(40)
        time.sleep(0.1)
        set_pluse_line(11)
        time.sleep(0.1)
        set_wheel_dis(67.00)
        time.sleep(0.1)
        set_motor_deadzone(1600)
        time.sleep(0.1)

def drive_forward(speed, duration):
	control_speed(speed, speed, speed, speed)
	time.sleep(duration)

def turn_left(speed, target_angle_time):
	control_speed(0, speed, 0, speed)
	time.sleep(target_angle_time)
	control_speed(0,0,0,0)


def turn_right(speed, target_angle_time):
	control_speed(speed, 0, speed, 0)
	time.sleep(target_angle_time)
	control_speed(0,0,0,0)

def stop_all():
	control_speed(0,0,0,0)

if __name__ == "__main__":
    try:
        t = 0
        print("please wait...")
        send_upload_command(UPLOAD_DATA)
        time.sleep(0.1)
        set_motor_parameter()
        time.sleep(0.5)

        while True:
            received_message = receive_data()
            if received_message:
                parsed = parse_data(received_message)
                if parsed:
                    print(parsed)
		print("Driving forward...")
		drive_forward(500, 3)
		
		print("Turning left")
		turn_left(300, 1.5)
		
		print("Driving forward...")
		drive_forward(500, 2)
		
		print("Turning right")
		turn_left(300, 1.5)
		
		print("Stopping...")
		stop_all()

    except KeyboardInterrupt:
        control_pwm(0, 0, 0, 0)
    finally:
        ser.close()

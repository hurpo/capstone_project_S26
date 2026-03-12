import serial

ser = serial.Serial("/dev/ttyACM0", 1000000, timeout=1)

print("Connected to:", ser.name)

while True:
    data = ser.read(32)
    if data:
        print(data.hex())
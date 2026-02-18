import json
import struct
from HardwareControls.hardware_classes import Magnetometer, LightSensor

TYPE_POSITION = b"P"
TYPE_ROBOT_DATA = b"R"

class Robot():
    def __init__(self, testing=False, sensors_connected=True, cameras_active=True, socket=None, send_lock=None):

        self.socket = socket
        self.send_lock = send_lock

        self.localization = {
            "x": 32.0,
            "y": 6.0,
            "degrees": 90
        }

        self.robot_data = {}

        #* Testing Booleans
        self.testing = testing

        self.sensors_connected = sensors_connected
        self.cameras_active = cameras_active

        print(f"self.sensors_connected {self.sensors_connected}")

        self.setupHardware()

    
    def setupHardware(self):
        print(f"Setting up Hardware...")
        if self.sensors_connected:
            self.LightSensor = LightSensor()
            self.Mag1 = Magnetometer(0x18)
            self.Mag2 = Magnetometer(0x19)
        else:
            print("Skipped for testing!")

    #* Localization and Data Methods

    def updatePosition(self, dx=None, dy=None, degrees=None):

        print(f'Updating POS:\n\tFROM: x={self.localization["x"]} y={self.localization["y"]} degrees={self.localization["degrees"]}\n\tTO: x={dx} y={dy} degrees={degrees}')

        if dx is not None:
            self.localization["x"] = dx
        if dy is not None:
            self.localization["y"] = dy
        if degrees is not None:
            self.localization["degrees"] = degrees
        
        self.send_position()
    
    def updateRobotData(self, datain=None):
        print(f"datain: {datain}")
        if datain is None:
            print(f"Couldn't Update Robot Data, no key or value.")
            return
        
        for key, value in datain.items():
            print(f"In datain: key={key} value={value}")
            if key in self.robot_data:
                if self.robot_data[key] != value:
                    self.robot_data[key] = value
            else:
                self.robot_data[key] = value
        
        self.send_robot_data(datain)

    #* Game State Mathods

    def LEDStart(self, dprint=False):

        print("LED Start ready!")

        if self.sensors_connected:
            while self.LightSensor.returnVisible() <= 6000:
                if dprint:
                    print(self.LightSensor.returnVisible())
            if dprint:
                print(self.LightSensor.returnVisible()) 
        else:
            print("Testing without sensors attached!")
        return True
    
    #* Send to Client Over Socket Methods

    def send_position(self):
        if self.socket is None:
            print("No socket, returning")
            return
        
        print("Sending position data...")
        try:
            payload = json.dumps(self.localization).encode("utf-8")
            header = TYPE_POSITION + struct.pack("!I", len(payload))

            if self.send_lock:
                with self.send_lock:
                    self.socket.sendall(header + payload)
            else: 
                self.socket.sendall(header + payload)
        
        except Exception as e:
            print(f"Error sending POS: {e}")
    
    def send_robot_data(self, datain=None):
        if self.socket is None:
            return
        
        if datain is None:
            return
        
        print("Sending robot data...")
        try:
            payload = json.dumps(datain).encode("utf-8")
            header = TYPE_ROBOT_DATA + struct.pack("!I", len(payload))

            if self.send_lock:
                with self.send_lock:
                    self.socket.sendall(header + payload)
            else:
                self.socket.sendall(header + payload)
        except Exception as e:
            print(f"Error sending POS: {e}")
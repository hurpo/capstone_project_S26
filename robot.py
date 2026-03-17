import json
import struct
import datetime
from HardwareControls.hardware_classes import Magnetometer, LightSensor, Camera
from HardwareControls.Servos.combine import DualContinuousServos
from HardwareControls.Servos.chute import SG90Servo
from HardwareControls.Servos.clawPincher import Servo270Positions
from HardwareControls.Servos.binFloor import Servo270

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
        self.servos_connected = True

        print(f"self.sensors_connected {self.sensors_connected}")

        self.camera = None
        self.setupHardware()

    
    def setupHardware(self):
        print(f"Setting up Hardware...")
        self.camera = Camera(robot=self)
        if self.servos_connected:
            self.Combine = DualContinuousServos()

            self.Claw = Servo270Positions()
            self.ClawBase = Servo270Positions(channel=1)

            self.Chute = SG90Servo()

            self.CameraServo = SG90Servo(channel=15)

            self.BinLift = Servo270Positions(channel=5)
            self.BinFloor = Servo270()
            self.BinDump = Servo270Positions(channel=7)

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
        print(f"datain: {datain} {datetime.datetime.now()}")
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
    
    #* Combine Controls
    def StartIntakeCombine(self, reverse=False):
        if self.servos_connected:
            if reverse:
                self.Combine.a_forward_b_backward = False
            else:
                self.Combine.a_forward_b_backward = True
            self.Combine.run_opposite_full()
        else:
            print("Intaking with Combine - No servos attached")
    
    def StopIntakeCombine(self):
        if self.servos_connected:
            self.Combine.stop_all()
        else:
            print("Stopping Combine - No servos attached")

    #* Chute Controls
    def OpenChute(self):
        if self.servos_connected:
            self.Chute.open()
        else:
            print("Opening Chute - No Servos connected")

    def CloseChute(self):
        if self.servos_connected:
            self.Chute.close()
        else:
            print("Close Chute - No Servos connected")

    #* Claw Controls
    def OpenClaw(self):
        if self.servos_connected:
            self.Claw.open()
        else:
            print("Opening Claw - No Servos connected")
    
    def CenterCloseClaw(self):
        if self.servos_connected:
            self.Claw.center_closed()
        else:
            print("Closing Claw - No Servos connected")
    
    def LatchedClaw(self):
        if self.servos_connected:
            self.Claw.latched()
        else:
            print("Latching Claw - No Servos connected")

    def ExtendClawBase(self):
        self.ClawBase.move_to(target_deg=90)
    
    def RetractClawBase(self):
        self.ClawBase.move_to(target_deg=180)

    #* Bin Controls
    def LiftBin(self):
        self.BinLift.move_to(target_deg=90)
    
    def LowerBin(self):
        self.BinLift.move_to(target_deg=180)

    def OpenFloor(self):
        self.BinFloor.open()
    
    def CloseFloor(self):
        self.BinFloor.close()
    
    def DumpBin(self):
        self.BinDump.set_angle(90)
    
    def UndumpBin(self):
        self.BinDump.set_angle(180)

    #* Camera Servo Controls
    def defaultCameraAngle(self):
        self.CameraServo.set_angle(90)

    # Good angles: 90 (default, straight on), 120 (Slightly angled down)
    def setCameraAngle(self, angle=0):
        self.CameraServo.set_angle(angle_deg=angle)

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
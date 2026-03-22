import json
import struct
from pathlib import Path
import datetime
from HardwareControls.hardware_classes import Magnetometer, LightSensor, Camera
from HardwareControls.Servos.combine import DualContinuousServos
from HardwareControls.Servos.chute import SG90Servo
from HardwareControls.Servos.clawPincher import Servo270Positions
from HardwareControls.Servos.binFloor import Servo270
from HardwareControls.MotorEncoders.MotorController import HiwonderMecanumController

TYPE_POSITION = b"P"
TYPE_ROBOT_DATA = b"R"

class Robot():
    _BASE_DIR = Path(__file__).parent
    print(f"_BASE_DIR: {_BASE_DIR}")

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

        self.drive_train_connected = False
        self.sensors_connected = sensors_connected
        self.cameras_active = cameras_active
        self.servos_connected = False

        print(f"self.sensors_connected {self.sensors_connected}")

        self.camera = None
        self.setupHardware()

    
    def setupHardware(self):
        print(f"Setting up Hardware...")
        self.camera = Camera(robot=self)
        if self.drive_train_connected:
            self.drive_train = HiwonderMecanumController(
                port="/dev/ttyACM2",
                baud=1000000,
                calibration_file=f"{self._BASE_DIR}/HardwareControls/MotorEncoders/robot_calibration.json",
            )
        if self.servos_connected:
            self.Combine = DualContinuousServos()        # In Pinout

            self.Claw = Servo270Positions()              # In Pinout
            self.ClawBase = Servo270Positions(channel=1) # In Pinout

            self.RackPinion = Servo270Positions(channel=8) # In Pinout

            # self.Chute = SG90Servo()                     # In Pinout

            self.CameraServo = SG90Servo(channel=15)     # In Pinout

            # self.BinLift = Servo270Positions(channel=5)  # In Pinout
            # self.BinFloor = Servo270()                   # In Pinout
            # self.BinDump = Servo270Positions(channel=7)  # In Pinout

        if self.sensors_connected:
            pass
            # self.LightSensor = LightSensor()
            # self.Mag1 = Magnetometer(0x18)
            # self.Mag2 = Magnetometer(0x19)
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

    #* Movement Controls
    def open_motors(self):
        if self.drive_train_connected:
            self.drive_train.open()
        else:
            print("open_motors - Drive Train Disconnected")
    
    def close_motors(self):
        if self.drive_train_connected:
            self.drive_train.close()
        else:
            print("close_motors - Drive Train Disconnected")
    
    def stop_all_motors(self):
        if self.drive_train_connected:
            self.drive_train.stop_all()
        else:
            print("stop_all_motors - Drive Train Disconnected")

    def drive_distance(self, inches: float, speed_rev_s: float = 0.4):
        if self.drive_train_connected:
            self.drive_train.drive_distance(inches, speed_rev_s)
        else:
            print("drive_distance - Drive Train Disconnected")
    
    def drive_reverse_distance(self, inches: float, speed_rev_s: float = 0.4):
        if self.drive_train_connected:
            self.drive_train.drive_reverse_distance(inches, speed_rev_s)
        else:
            print("drive_reverse_distance - Drive Train Disconnected")
    
    def strafe_distance_left(self, inches: float, speed_rev_s: float = 0.4):
        if self.drive_train_connected:
            self.drive_train.strafe_distance_left(inches, speed_rev_s)
        else:
            print("strafe_distance_left - Drive Train Disconnected")
    
    def strafe_distance_right(self, inches: float, speed_rev_s: float = 0.4):
        if self.drive_train_connected:
            self.drive_train.strafe_distance_right(inches, speed_rev_s)
        else:
            print("strafe_distance_right - Drive Train Disconnected")
    
    def drive_diagonal(self, inches: float, speed_rev_s: float = 0.4):
        if self.drive_train_connected:
            self.drive_train.drive_diagonal(inches, speed_rev_s)
        else:
            print("drive_diagonal - Drive Train Disconnected")
    
    # continuous motor driving
    def move_forward(self, speed_rev_s: float = 0.5):
        if self.drive_train_connected:
            self.drive_train.move_forward(speed_rev_s)
        else:
            print("move_forward - Drive Train Disconnected")

    def move_reverse(self, speed_rev_s: float = 0.5):
        if self.drive_train_connected:
            self.drive_train.move_reverse(speed_rev_s)
        else:
            print("move_reverse - Drive Train Disconnected")

    def strafe_left(self, speed_rev_s: float = 0.5):
        if self.drive_train_connected:
            self.drive_train.strafe_left(speed_rev_s)
        else:
            print("strafe_left - Drive Train Disconnected")

    def strafe_right(self, speed_rev_s: float = 0.5):
        if self.drive_train_connected:
            self.drive_train.strafe_right(speed_rev_s)
        else:
            print("strafe_right - Drive Train Disconnected")

    def rotate_ccw(self, speed_rev_s: float = 0.4):
        if self.drive_train_connected:
            self.drive_train.rotate_ccw(speed_rev_s)
        else:
            print("rotate_ccw - Drive Train Disconnected")

    def rotate_cw(self, speed_rev_s: float = 0.4):
        if self.drive_train_connected:
            self.drive_train.rotate_cw(speed_rev_s)
        else:
            print("rotate_cw - Drive Train Disconnected")

    def diagonal_forward_left(self, speed_rev_s: float = 0.5):
        if self.drive_train_connected:
            self.drive_train.diagonal_forward_left(speed_rev_s)
        else:
            print("diagonal_forward_left - Drive Train Disconnected")

    def diagonal_forward_right(self, speed_rev_s: float = 0.5):
        if self.drive_train_connected:
            self.drive_train.diagonal_forward_right(speed_rev_s)
        else:
            print("diagonal_forward_right - Drive Train Disconnected")

    def diagonal_reverse_left(self, speed_rev_s: float = 0.5):
        if self.drive_train_connected:
            self.drive_train.diagonal_reverse_left(speed_rev_s)
        else:
            print("diagonal_reverse_left - Drive Train Disconnected")

    def diagonal_reverse_right(self, speed_rev_s: float = 0.5):
        if self.drive_train_connected:
            self.drive_train.diagonal_reverse_right(speed_rev_s)
        else:
            print("diagonal_reverse_right - Drive Train Disconnected")

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
        #TODO Change Angle ????? Unsure
        if self.servos_connected:
            self.Claw.latched()
        else:
            print("Latching Claw - No Servos connected")

    def ExtendClawBase(self):
        #TODO Change Angle
        self.ClawBase.move_to(target_deg=90)
    
    def RetractClawBase(self):
        #TODO Change Angle
        self.ClawBase.move_to(target_deg=180)

    #* Bin Controls
    def LiftBin(self):
        #TODO Change Angle
        self.BinLift.move_to(target_deg=90)
    
    def LowerBin(self):
        #TODO Change Angle
        self.BinLift.move_to(target_deg=180)

    def OpenFloor(self):
        self.BinFloor.open()
    
    def CloseFloor(self):
        self.BinFloor.close()
    
    def DumpBin(self):
        #TODO Change Angle
        self.BinDump.set_angle(90)
    
    def UndumpBin(self):
        #TODO Change Angle
        self.BinDump.set_angle(180)

    #* Rack and Pinion Controls
    def bottomRackPinion(self):
        #TODO Change Angle
        self.RackPinion.set_angle(90)
    
    def topRackPinion(self):
        #TODO Change Angle
        self.RackPinion.set_angle(180)

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
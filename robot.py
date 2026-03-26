import json
import time
import struct
from pathlib import Path
import datetime
from HardwareControls.hardware_classes import Magnetometer, LightSensor, Camera
from HardwareControls.Servos.combine import DualContinuousServos
from HardwareControls.Servos.chute import SG90Servo
from HardwareControls.Servos.clawPincher import Servo270Positions
from HardwareControls.Servos.clawBase import ClawBaseServo
from HardwareControls.Servos.conveyor import ConveyorServo
from HardwareControls.Servos.camera import CameraServo
from HardwareControls.Servos.binDump import BinDumpServo
from HardwareControls.Servos.falseFloor import Servo270
from HardwareControls.MotorEncoders.MotorController import HiwonderMecanumController
from HardwareControls.MotorEncoders.testing.linkVerify import play_hw_error
from StateControllers import State, StateController, ClientController, AutoController

TYPE_POSITION = b"P"
TYPE_ROBOT_DATA = b"R"


class Robot():
    _BASE_DIR = Path(__file__).parent
    print(f"_BASE_DIR: {_BASE_DIR}")

    def __init__(self, controller=None, testing=False, sensors_connected=True, cameras_active=True, socket=None, send_lock=None):

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
        self.controller = controller

        self.drive_train_connected = True
        self.sensors_connected = sensors_connected
        self.cameras_active = cameras_active
        self.servos_connected = True

        print(f"self.sensors_connected {self.sensors_connected}")

        self.camera = None
        self.setupHardware()

    def setupHardware(self):
        print(f"Setting up Hardware...")
        try:
            self.camera = Camera(robot=self)
        except Exception as e:
            print(f"🤬😿Failed to Load Camera: {e}")
            play_hw_error()
        if self.drive_train_connected:
            # if isinstance(self.controller, ClientController):
            #     port = "/dev/ttyACM0"
            # else:
            port = None
            self.drive_train = HiwonderMecanumController(
                port=None,
                baud=1000000,
                calibration_file=f"{self._BASE_DIR}/HardwareControls/MotorEncoders/robot_calibration.json",
            )
        if self.servos_connected:
            try:
                self.Combine = DualContinuousServos()   # Servo A:3 ServoB:4
                self.Chute = SG90Servo()                # 2   
                self.Claw = Servo270Positions()         # 0
                self.Conveyor = ConveyorServo()         # 8
                self.ClawBase = ClawBaseServo()         # 1
                self.CameraServo = CameraServo()        # 15
                self.RackPinion = Servo270Positions()   # 5
                self.BinDump = BinDumpServo()           # 7
                self.FalseFloor = Servo270()            # 6
            except Exception as e:
                print(f"🤬😿Failed to Load Servo: {e}")
                play_hw_error()
        if self.sensors_connected:
            try:
                self.LightSensor = LightSensor()
                self.Mag1 = Magnetometer(0x18)
                self.Mag2 = Magnetometer(0x19)
            except Exception as e:
                print(f"🤬😿Failed to Load Sensor: {e}")
                play_hw_error()
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

    #* Game State Methods

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

    #* Combine + Conveyor Controls
    def StartIntakeCombine(self, reverse=False, speed=1.0):
        if self.servos_connected:
            if reverse:
                self.Combine.a_forward_b_backward = False
            else:
                self.Combine.a_forward_b_backward = True
            self.Combine.run_opposite_full()
            self.Conveyor.run_match_combine(reverse=reverse, speed=speed)
        else:
            print("Intaking with Combine - No servos attached")

    def StopIntakeCombine(self):
        if self.servos_connected:
            self.Combine.stop_all()
            self.Conveyor.stop()
        else:
            print("Stopping Combine - No servos attached")

    def SetConveyorSpeed(self, speed=1.0):
        if self.servos_connected:
            self.Conveyor.set_speed(speed)
        else:
            print("SetConveyorSpeed - No servos attached")

    def ReverseConveyor(self, speed=1.0):
        if self.servos_connected:
            self.Conveyor.reverse(speed)
        else:
            print("ReverseConveyor - No servos attached")

    def ForwardConveyor(self, speed=1.0):
        if self.servos_connected:
            self.Conveyor.forward(speed)
        else:
            print("ForwardConveyor - No servos attached")

    def StopConveyor(self):
        if self.servos_connected:
            self.Conveyor.stop()
        else:
            print("StopConveyor - No servos attached")

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
            print("ClawOpen!")
            self.Claw.open()
        else:
            print("Opening Claw - No Servos connected")

    def CenterCloseClaw(self):
        if self.servos_connected:
            print("CenterClose!!")
            self.Claw.center_closed()
        else:
            print("Closing Claw - No Servos connected")

    def LatchedClaw(self):
        if self.servos_connected:
            print("Latch!")
            self.Claw.latched()
        else:
            print("Latching Claw - No Servos connected")

    def ExtendClawBase(self):
        if self.servos_connected:
            self.ClawBase.extend()
        else:
            print("ExtendClawBase - No Servos connected")

    def RetractClawBase(self):
        if self.servos_connected:
            self.ClawBase.retract()
        else:
            print("RetractClawBase - No Servos connected")

    #* False Floor Controls
    def OpenFalseFloor(self):
        if self.servos_connected:
            self.FalseFloor.open()
        else:
            print("OpenFalseFloor - No Servos connected")

    def CloseFalseFloor(self):
        if self.servos_connected:
            self.FalseFloor.close()
        else:
            print("CloseFalseFloor - No Servos connected")

    # Backward-compatible aliases in case other code still calls floor methods.
    def OpenFloor(self):
        self.OpenFalseFloor()

    def CloseFloor(self):
        self.CloseFalseFloor()

    #* Bin Dump Controls
    def DumpBin(self):
        if self.servos_connected:
            self.BinDump.open()
        else:
            print("DumpBin - No Servos connected")

    def UndumpBin(self):
        if self.servos_connected:
            self.BinDump.close()
        else:
            print("UndumpBin - No Servos connected")

    #* Rack and Pinion Controls
    def bottomRackPinion(self):
        if self.servos_connected:
            self.RackPinion.set_angle(90)
        else:
            print("bottomRackPinion - No Servos connected")

    def topRackPinion(self):
        if self.servos_connected:
            self.RackPinion.set_angle(180)
        else:
            print("topRackPinion - No Servos connected")

    #* Camera Servo Controls
    def defaultCameraAngle(self):
        if self.servos_connected:
            self.CameraServo.look_forward()
        else:
            print("defaultCameraAngle - No Servos connected")

    # Good angles: 90 (default, straight on), 120 (slightly angled down)
    def setCameraAngle(self, angle=0):
        if self.servos_connected:
            self.CameraServo.set_angle(angle_deg=angle)
        else:
            print("setCameraAngle - No Servos connected")

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

#* Import Required Libraries
import time
import datetime
from gpiozero import Button

#* Import Functions etc.
from HardwareControls import xbox
from HardwareControls.CameraControls.USBCam import start_cam, read_april_tag, end_cam
from HardwareControls.MotorEncoders.CopyController.state_replay import run_routine
from robot import Robot
from StateControllers import State, StateController, ClientController, AutoController
from HardwareControls.hardware_classes import Camera
from HardwareControls.MotorEncoders.testing.linkVerify import play_beep
time.sleep(2)
from enum import Enum

ROUTINE_NAME = "routine_000"

ROUTINE_KP_COUNTS = 0.05
ROUTINE_MAX_CORRECTION_REV_S = 0.08
ROUTINE_BLEND = 1.0
ROUTINE_RATE_HZ = 12.0


class StateMachine:
    def __init__(self, controller: StateController, socket=None, send_lock = None, passedCam=None, passedCamLock=None):
        self.side_button = Button(17, pull_up=True, bounce_time=0.1)

        self.controller = controller
        self.socket = socket
        self.send_lock = send_lock
        self.current_state = State.IDLE
        self.prev_state = None

        print(f"passedCam: {passedCam}")

        self.passedCamera = passedCam
        self.passedCameraLock = passedCamLock

        self.rendezvous_pad_location = None
        self.led_started = False
        self.button_started = False
        self.combine_running = False

        #* Testing Booleans
        self.testing = False
        self.sensors_connected = False

        self.robot = Robot(testing=self.testing, controller=self.controller, sensors_connected = self.sensors_connected, socket=self.socket, send_lock=self.send_lock)
        if self.passedCamera:
            self.robot.camera.passed_cam(self.passedCamera, self.passedCameraLock)
        else:
            self.robot.camera.start_cam()

    def combine_should_run(self) -> bool:
        return self.led_started or self.button_started

    def ensure_combine_running(self, reverse: bool = False, speed: float = 1.0):
        if self.combine_should_run() and not self.combine_running:
            print("Starting combine...")
            self.robot.StartIntakeCombine(reverse=reverse, speed=speed)
            self.combine_running = True

    def stop_combine(self):
        if self.combine_running:
            print("Stopping combine...")
            try:
                self.robot.StopIntakeCombine()
            finally:
                self.combine_running = False

    def run(self):
        print(f"State Machine starting in {'AUTONOMOUS' if isinstance(self.controller, AutoController) else 'CLIENT-CONTROLLED'} {'[TESTING]' if self.testing else ''} mode")

        print("Controller in use: ", isinstance(self.controller, ClientController))

        if isinstance(self.controller, ClientController):
            controller_in_use = ClientController
        else:
            controller_in_use = AutoController

        while(isinstance(self.controller, controller_in_use)):

            while not self.controller.should_start():
                time.sleep(0.1)
            self.controller._started = False

            print("State Machine Started!")
            self.current_state = State.INIT

            while self.current_state != State.END:
                print(f'self.controller.should_stop(): {self.controller.should_stop()}')
                if self.controller.should_stop():
                    print("STOP command received - ending state machine")
                    self.stop_combine()
                    break

                while self.controller.should_pause():
                    print("State machine PAUSED")
                    time.sleep(0.1)
                    if self.controller.should_stop():
                        print("STOP command received while paused")
                        self.stop_combine()
                        return

                should_override, new_state = self.controller.get_state_override()
                if should_override:
                    print(f"Overriding state: {self.current_state.name} -> {new_state.name}")
                    self.current_state = new_state

                if self.controller.is_manual_mode():
                    self.handle_manual_mode()
                    continue

                self.execute_state()

                time.sleep(0.1)
            self.robot.updateRobotData({"State": "END"})
            print("State Machine COMPLETE")
            self.robot.camera.stop_pnp_localization()
            self.stop_combine()
            self.robot.stop_all_motors()
            self.robot.close_motors()
            print("GOODBYE!")

    def execute_state(self):
        combine_speed = 1.0
        def play(state, imported_route=ROUTINE_NAME):
            if self.robot.drive_train_connected:
                run_routine(
                    routine_name=imported_route,
                    state=state,
                    controller=self.robot.drive_train,
                    kp_counts=ROUTINE_KP_COUNTS,
                    max_correction_rev_s=ROUTINE_MAX_CORRECTION_REV_S,
                    blend=ROUTINE_BLEND,
                    rate=ROUTINE_RATE_HZ,
                    pre_run_callback=lambda: self.ensure_combine_running(reverse=True, speed=1.0),
                    post_run_callback=self.stop_combine,
                )
            else:
                print(f"[run_routine] Drive train disconnected, skipping {state.name}")

        match self.current_state:
            case State.INIT:
                play_beep()
                self.robot.OpenClaw()
                                  #* Testing booleans applied
                print(f"INIT RobotData")
                self.robot.updateRobotData({
                    "State": "INIT",
                    "LED_Started?": False
                })
                self.robot.send_position()

                self.robot.defaultCameraAngle()


                time.sleep(3)
                self.robot.CenterCloseClaw()

                self.robot.open_motors()
                self.robot.stop_all_motors()
                self.robot.Conveyor.hard_off()
                self.stop_combine()

                self.side_button.when_pressed = self.ButtonStartAlt
                self.ButtonStartTrigger = True

                while self.ButtonStartTrigger: # self.robot.LightSensor.returnVisible() <= 6000 and self.ButtonStartTrigger
                    pass
                print("Yurp")
                # self.transition_to(State.END)
                self.transition_to(State.LED_START) #LED_START

            case State.LED_START:
                self.robot.updateRobotData({"State": "LED_START"})
                self.robot.updateRobotData({"LED_Started?": True})
                self.led_started = True
                # self.ensure_combine_running(reverse=True, speed=combine_speed)
                play(State.LED_START)

                self.transition_to(State.RP_SCAN)

            case State.RP_SCAN: #add failsafe, check if read april tag has it
                self.robot.updateRobotData({"State": "RP_SCAN"})
                # print("Scanning for Rendezvous Pad....")
                self.stop_combine()
                # # rp = self.robot.camera.read_april_tags()
                # # self.rendezvous_pad_location = rp
                # self.robot.updateRobotData({"RP": rp})
                # print(f"Rendezvous Pad Located at {self.rendezvous_pad_location}.")

                try:
                    play(State.RP_SCAN)
                except:
                    pass

                self.transition_to(State.PLACE_BEACON)

            case State.PLACE_BEACON:
                self.robot.updateRobotData({"State": "PLACE_BEACON"})
                # self.ensure_combine_running(reverse=False, speed=1.0)
                try:
                    play(State.PLACE_BEACON)
                except:
                    pass
                self.transition_to(State.ENTER_CAVE)

            case State.ENTER_CAVE:
                self.robot.updateRobotData({"State": "ENTER_CAVE"})
                # self.ensure_combine_running(reverse=False, speed=1.0)
                # self.ensure_combine_running(reverse=True, speed=combine_speed)
                play(State.ENTER_CAVE)

                self.transition_to(State.CAVE_SWEEP)

            case State.CAVE_SWEEP:
                self.robot.updateRobotData({"State": "CAVE_SWEEP"})
                
                play(State.CAVE_SWEEP)

                self.transition_to(State.OUTSIDE_SWEEP)

            case State.OUTSIDE_SWEEP:
                self.robot.updateRobotData({"State": "OUTSIDE_SWEEP"})
                
                self.stop_combine()
                play(State.OUTSIDE_SWEEP)

                self.transition_to(State.MOVE_TO_GEO_CSC)

            case State.MOVE_TO_GEO_CSC:
                self.robot.updateRobotData({"State": "MOVE_TO_GEO_CSC"})
            
                try:
                    play(State.MOVE_TO_GEO_CSC)
                except:
                    pass

                self.transition_to(State.GRAB_GEO_CSC)

            case State.GRAB_GEO_CSC:
                self.robot.updateRobotData({"State": "GRAB_GEO_CSC"})

                try:
                    play(State.GRAB_GEO_CSC)
                except:
                    pass

                self.transition_to(State.MOVE_GEO_TO_RP)

            case State.MOVE_GEO_TO_RP:
                self.robot.updateRobotData({"State": "MOVE_GEO_TO_RP"})
                
                try:
                    play(State.MOVE_GEO_TO_RP)
                except:
                    pass

                self.transition_to(State.DISPENSE_GEO)

            case State.DISPENSE_GEO:
                self.robot.updateRobotData({"State": "DISPENSE_GEO"})
                
                try:
                    play(State.DISPENSE_GEO)
                except:
                    pass

                self.transition_to(State.MOVE_TO_NEB_CSC)

            case State.MOVE_TO_NEB_CSC:
                self.robot.updateRobotData({"State": "MOVE_TO_NEB_CSC"})
                
                try:
                    play(State.MOVE_TO_NEB_CSC)
                except:
                    pass

                self.transition_to(State.MOVE_NEB_TO_RP)

            case State.MOVE_NEB_TO_RP:
                self.robot.updateRobotData({"State": "MOVE_NEB_TO_RP"})
                
                try:
                    play(State.MOVE_NEB_TO_RP)
                except:
                    pass

                self.transition_to(State.DISPENSE_NEB)

            case State.DISPENSE_NEB:
                self.robot.updateRobotData({"State": "DISPENSE_NEB"})
                
                try:
                    play(State.DISPENSE_NEB)
                except:
                    pass

                self.transition_to(State.END)

            case State.END:
                pass

    def ButtonStartAlt(self):
        print("Button pressed!!!!")
        self.button_started = True
        # self.ensure_combine_running(reverse=False, speed=1.0)
        time.sleep(5)
        self.ButtonStartTrigger = False
        return False

    def transition_to(self, new_state):
        print(f"Transitioning: {self.current_state.name} -> {new_state.name}")
        self.prev_state = self.current_state
        self.current_state = new_state

    def handle_manual_mode(self):

        def show(*args):
            for arg in args:
                print(arg, end="")

        def fmtFloat(n):
            return '{:6.3f}'.format(n)


        joy = xbox.Joystick()
        while self.controller.is_manual_mode() and not joy.Back():
            print("lol")
            show("  Left X/Y:", fmtFloat(joy.leftX()), "/", fmtFloat(joy.leftY()))
            show("  Right X/Y:", fmtFloat(joy.rightX()), "/", fmtFloat(joy.rightY()))

            if float(fmtFloat(joy.leftY())) > 0.25:
                self.robot.move_forward(float(fmtFloat(joy.leftY())))
            elif float(fmtFloat(joy.leftY())) < -0.25:
                self.robot.move_reverse(float(fmtFloat(joy.leftY())))
            else:
                self.robot.stop_all_motors()


        print("Manual mode ended...")

    #! Depricated
    def ScanRendezvousPadLocation(self):

        rendezvous_pad_location = None
        autonomous_mode = self.passedCamera is None
        if autonomous_mode:
            start_cam()
            cap = None
            cap_lock = None
        else:
            cap = self.passedCamera
            cap_lock = self.passedCameraLock
        april_id = read_april_tag(cap, cap_lock)
        accepted_values = [0, 1, 2, 3, 4, 5, 6, 7]
        tries = 0
        MAX_TRIES = 3
        looking_for_rp = not (april_id in accepted_values)
        while (april_id not in accepted_values) and tries < MAX_TRIES:
            april_id = read_april_tag(cap, cap_lock)
            print(f"Found {april_id}")
            tries += 1
        if autonomous_mode:
            end_cam()
        if april_id not in accepted_values:
            print("Failed to locate RP")
            return None
        print(april_id)
        return april_id

if __name__ == "__main__":
    # main()
    print("Fart")
    controller = AutoController()
    sm = StateMachine(controller)
    sm.run()

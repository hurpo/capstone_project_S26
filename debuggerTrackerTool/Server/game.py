#* Import Required Libraries
import time

#* Import Functions etc.
from HardwareControls.CameraControls.USBCam import start_cam, read_april_tag, end_cam
from robot import Robot
from StateControllers import State, StateController, ClientController, AutoController

from enum import Enum


class StateMachine:
    def __init__(self, controller: StateController, socket=None, send_lock = None, passedCam=None, passedCamLock=None):
        self.controller = controller
        self.socket = socket
        self.send_lock = send_lock
        self.current_state = State.IDLE
        self.prev_state = None

        print(f"passedCam: {passedCam}")

        self.robot = None
        self.passedCamera = passedCam
        self.passedCameraLock = passedCamLock

        self.rendezvous_pad_location = None

        #* Testing Booleans
        self.testing = True

        self.sensors_connected = False

    
    def run(self):
        print(f"State Machine starting in {'AUTONOMOUS' if isinstance(self.controller, AutoController) else 'CLIENT-CONTROLLED'} {'[TESTING]' if self.testing else ''} mode")

        while not self.controller.should_start():
            time.sleep(0.1)
    
        print("State Machine Started!")
        self.current_state = State.INIT

        while self.current_state != State.END:
            print(f'self.controller.should_stop(): {self.controller.should_stop()}')
            if self.controller.should_stop():
                print("STOP command received - ending state machine")
                break

            while self.controller.should_pause():
                print("State machine PAUSED")
                time.sleep(0.1)
                if self.controller.should_stop():
                    print("STOP command received while paused")
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

    def execute_state(self):
        match self.current_state:
            case State.IDLE:
                pass

            case State.INIT:
                                  #* Testing booleans applied
                self.robot = Robot(testing=self.testing, sensors_connected = self.sensors_connected, socket=self.socket, send_lock=self.send_lock)
                self.robot.updateRobotData({
                    "State": "INIT",
                    "LED_Started?": False
                })
                self.robot.send_position()

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.LED_START) #LED_START

            case State.LED_START:
                self.robot.updateRobotData({"State": "LED_START"})

                if self.testing:
                    time.sleep(0.1)

                print(f"self.robot.testing: {self.robot.testing}")
                self.robot.LEDStart()
                self.robot.updateRobotData({"LED_Started?": True})

                self.robot.updatePosition(dy=24, degrees=180)

                if self.testing:
                    time.sleep(5)
                # self.transition_to(State.END)
                self.transition_to(State.RP_SCAN)

            case State.RP_SCAN:
                self.robot.updateRobotData({"State": "RP_SCAN"})
                print("Scanning for Rendezvous Pad....")
                rp = self.ScanRendezvousPadLocation()
                self.rendezvous_pad_location = rp
                self.robot.updateRobotData({"RP": rp})
                print(f"Rendezvous Pad Located at {self.rendezvous_pad_location}.")
                self.transition_to(State.PLACE_BEACON)

            case State.PLACE_BEACON:
                self.robot.updateRobotData({"State": "PLACE_BEACON"})
                if self.testing:
                    time.sleep(0.1)

                self.robot.updatePosition(dx=6.0)

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.ENTER_CAVE)

            case State.ENTER_CAVE:
                self.robot.updateRobotData({"State": "ENTER_CAVE"})
                if self.testing:
                    time.sleep(0.1)
                
                self.robot.updatePosition(dx=90.0)

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.CAVE_SWEEP)

            case State.CAVE_SWEEP:
                self.robot.updateRobotData({"State": "CAVE_SWEEP"})
                if self.testing:
                    time.sleep(0.1)
                
                self.robot.updatePosition(dx=60.0)

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.OUTSIDE_SWEEP)

            case State.OUTSIDE_SWEEP:
                self.robot.updateRobotData({"State": "OUTSIDE_SWEEP"})
                if self.testing:
                    time.sleep(0.1)
                
                self.robot.updatePosition(dx=40.0)

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.MOVE_TO_GEO_CSC)

            case State.MOVE_TO_GEO_CSC:
                self.robot.updateRobotData({"State": "MOVE_TO_GEO_CSC"})
                if self.testing:
                    time.sleep(0.1)
                
                self.robot.updatePosition(dx=55.0, dy=6.0)

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.GRAB_GEO_CSC)

            case State.GRAB_GEO_CSC:
                self.robot.updateRobotData({"State": "GRAB_GEO_CSC"})
                if self.testing:
                    time.sleep(0.1)
                
                if self.testing:
                    time.sleep(5)

                self.transition_to(State.MOVE_GEO_TO_RP)

            case State.MOVE_GEO_TO_RP:
                self.robot.updateRobotData({"State": "MOVE_GEO_TO_RP"})
                if self.testing:
                    time.sleep(0.1)
                
                self.robot.updatePosition(dx=6.0, dy=24.0)

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.DISPENSE_GEO)

            case State.DISPENSE_GEO:
                self.robot.updateRobotData({"State": "DISPENSE_GEO"})
                if self.testing:
                    time.sleep(0.1)
                
                if self.testing:
                    time.sleep(5)

                self.transition_to(State.MOVE_TO_NEB_CSC)

            case State.MOVE_TO_NEB_CSC:
                self.robot.updateRobotData({"State": "MOVE_TO_NEB_CSC"})
                if self.testing:
                    time.sleep(0.1)
                
                self.robot.updatePosition(dx=40.0, dy= 42.0)

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.MOVE_NEB_TO_RP)

            case State.MOVE_NEB_TO_RP:
                self.robot.updateRobotData({"State": "MOVE_NEB_TO_RP"})
                if self.testing:
                    time.sleep(0.1)
                
                self.robot.updatePosition(dx=6.0, dy=24.0)

                if self.testing:
                    time.sleep(5)

                self.transition_to(State.DISPENSE_NEB)

            case State.DISPENSE_NEB:
                self.robot.updateRobotData({"State": "DISPENSE_NEB"})
                if self.testing:
                    time.sleep(0.1)
                
                if self.testing:
                    time.sleep(5)

                self.transition_to(State.END)

            case State.END:
                print("GOODBYE!")

    def transition_to(self, new_state):
        print(f"Transitioning: {self.current_state.name} -> {new_state.name}")
        self.prev_state = self.current_state
        self.current_state = new_state

    def handle_manual_mode(self):
        print("Manual Mode")
        time.sleep(0.5)
    
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
    controller = AutoController()
    sm = StateMachine(controller)
    sm.run()


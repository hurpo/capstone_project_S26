#* Import Required Libraries
import time

#* Import Functions etc.
from HardwareControls.CameraControls.USBCam import start_cam, read_april_tag, end_cam
from robot import Robot
from StateControllers import State, StateController, ClientController, AutoController

from enum import Enum


class StateMachine:
    def __init__(self, controller: StateController, socket=None, send_lock = None):
        self.controller = controller
        self.socket = socket
        self.send_lock = send_lock
        self.current_state = State.IDLE
        self.prev_state = None

        self.robot = None

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

                self.transition_to(State.RP_SCAN) #LED_START

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
                self.rendezvous_pad_location = self.ScanRendezvousPadLocation()
                self.robot.updateRobotData({"RP": self.rendezvous_pad_location})
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

        start_cam()
        april_id = read_april_tag()

        if april_id == "April Tag Failed":
            print("Failed to locate Rendezvous Pad Location")
        else:
            accepted_values = [0, 1, 2, 3, 4, 5, 6, 7]
            directions = []
            looking_for_rp = not (april_id in accepted_values)

            tries = 0
            MAX_TRIES = 3

            while looking_for_rp and tries < MAX_TRIES:
                april_id = read_april_tag()

                looking_for_rp = not(april_id in accepted_values)

                print(f"Found {april_id}")
                tries += 1

            rendezvous_pad_location = april_id
            print(rendezvous_pad_location)
        end_cam()

        return rendezvous_pad_location

def main(run_on_call=True):
    running = run_on_call
    state = "LED_START"

    rendezvous = None

    #*****************************************#
    #*               Game Plan               *#
    #*****************************************#


    robot = Robot()
    print("Robot Initialized")

    while running:
        match state:

            #* LED Start
            case "LED_START":
                robot.LEDStart()
                state = "END"

            #* Scan Rendezvous Pad Location
            case "RP_SCAN":
                print("Scanning for Rendezvous Pad....")
                rendezvous = ScanRendezvousPadLocation()
                print(f"Rendezvous Pad Located at {rendezvous}.")
                state = "END"

            #* Place Beacon
            case "PLACE_BEACON":
                pass

            #* Enter Cave
            case "ENTER_CAVE":
                pass

            #* Collect Cave Game Pieces
            case "CAVE_SWEEP":
                pass

            #* Collect Outside Game Pieces
            case "OUTSIDE_SWEEP":
                pass

            #* Move to Geodinium CSC
            case "MOVE_TO_GEO_CSC":
                pass

            #* Grab Geodinium CSC
            case "GRAB_GEO_CSC":
                pass

            #* Move Geodinium CSC to Rendezvous Pad
            case "MOVE_GEO_TO_RP":
                pass

            #* Dispense Geodinium
            case "DISPENSE_GEO":
                pass

            #* Move to Nebulite CSC
            case "MOVE_TO_NEB_CSC":
                pass

            #* Move Nebulite CSC to Rendezvous Pad
            case "MOVE_NEB_TO_RP":
                print(f"Moving to RP at ")
                pass

            #* Dispense Nebulite
            case "DISPENSE_NEB":
                pass
            
            #* Graceful End
            case "END":
                print("Ending Run, Goodbye!")
                running = False
                break

            case _:
                state = "LED_START"

if __name__ == "__main__":
    # main()
    controller = AutoController()
    sm = StateMachine(controller)
    sm.run()


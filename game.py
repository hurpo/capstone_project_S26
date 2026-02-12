#* Import Required Libraries
import time

#* Import Functions etc.
from HardwareControls.CameraControls.USBCam import start_cam, read_april_tag, end_cam
from robot import Robot
from StateControllers import State, StateController, ClientController, AutoController

from enum import Enum


class StateMachine:
    def __init__(self, controller: StateController):
        self.controller = controller
        self.current_state = State.IDLE
        self.prev_state = None

        self.robot = None
    
    def run(self):
        print(f"State Machine starting in {'AUTONOMOUS' if isinstance(self.controller, AutoController) else 'CLIENT-CONTROLLED'} mode")

        while not self.controller.should_start():
            time.sleep(0.1)
    
        print("State Machine Started!")
        self.current_state = State.INIT

        while self.current_state != State.END:
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
        print("State Machine COMPLETE")

    def execute_state(self):
        match self.current_state:
            case State.IDLE:
                pass

            case State.INIT:
                self.robot = Robot()
                self.transition_to(State.LED_START)

            case State.LED_START:
                self.robot.LEDStart()
                self.transition_to(State.END)

            case State.RP_SCAN:
                print("Scanning for Rendezvous Pad....")
                rendezvous = ScanRendezvousPadLocation()
                print(f"Rendezvous Pad Located at {rendezvous}.")
                state = "END"

            case State.PLACE_BEACON:
                pass

            case State.ENTER_CAVE:
                pass

            case State.CAVE_SWEEP:
                pass

            case State.OUTSIDE_SWEEP:
                pass

            case State.MOVE_TO_GEO_CSC:
                pass

            case State.GRAB_GEO_CSC:
                pass

            case State.MOVE_GEO_TO_RP:
                pass

            case State.DISPENSE_GEO:
                pass

            case State.MOVE_TO_NEB_CSC:
                pass

            case State.MOVE_NEB_TO_RP:
                pass

            case State.DISPENSE_NEB:
                pass

            case State.END:
                print("GOODBYE!")

    def transition_to(self, new_state):
        print(f"Transitioning: {self.current_state.name} -> {new_state.name}")
        self.prev_state = self.current_state
        self.current_state = new_state

    def handle_manual_mode(self):
        print("Manual Mode")
        time.sleep(0.5)

def ScanRendezvousPadLocation():
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


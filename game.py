#* Import Required Libraries
import time

#* Import Functions etc.
from HardwareControls.CameraControls.USBCam import start_cam, read_april_tag, end_cam

running = True
state = "LED_START"

rendezvous = None


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


#*****************************************#
#*               Game Plan               *#
#*****************************************#

while running:
    match state:

        #* LED Start
        case "LED_START":
            print("LED START!")
            state = "RP_SCAN"

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

        


#* Import Required Libraries
import time

#* Import Functions etc.
from CameraControls.USBCam import start_cam, read_april_tag, end_cam

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

#* LED Start

#* Scan Rendezvous Pad Location
ScanRendezvousPadLocation()

#* Place Beacon

#* Enter Cave

#* Collect Cave Game Pieces

#* Collect Outside Game Pieces

#* Move to Geodinium CSC

#* Grab Geodinium CSC

#* Move Geodinium CSC to Rendezvous Pad

#* Dispense Geodinium

#* Move to Nebulite CSC

#* Dispense Nebulite

#* Move Nebulite CSC to Rendezvous Pad


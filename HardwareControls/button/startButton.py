#!/usr/bin/env python3
from gpiozero import Button
from signal import pause
import time
import sys

# Button wired between GPIO17 and GND
button = Button(17, pull_up=True, bounce_time=0.1)

program_started = False

def my_action():
    global program_started

    # Ignore any extra presses
    if program_started:
        return

    program_started = True

    # Disable any future button press actions
    button.when_pressed = None

    print("Button pressed for the first and only time. Running task...")

    try:
        # Your main code goes here
        for i in range(5):
            print(f"Step {i+1}")
            time.sleep(1)

        print("Program finished. Exiting cleanly.")

    except Exception as e:
        print(f"Program stopped due to error: {e}")

    finally:
        sys.exit(0)

button.when_pressed = my_action

print("Waiting for the first button press only...")
pause()
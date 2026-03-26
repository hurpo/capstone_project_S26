from gpiozero import Button
from gpiozero import Device
from gpiozero.pins.lgpio import LGPIOFactory
from signal import pause
import subprocess
import time

# Button wired between GPIO17 and GND
button = Button(17, pull_up=True, bounce_time=0.1)

running = False




def run_script():
    global running

    if running:
        return
    running = True

    print("Button pressed! Launching script...")
    
    # Optional: debounce safety delay
    time.sleep(0.2)
    
    button.when_pressed = None
    button.close()
    Device.pin_factory.close()

    time.sleep(0.3)

    # Run your second script
    subprocess.run([
        "/home/raspberry/Documents/capstone_project_S26/venv/bin/python",
        "-u",
        "/home/raspberry/Documents/capstone_project_S26/game.py"
    ])

    exit(0)

button.when_pressed = run_script

print("Waiting for button press...")
pause()